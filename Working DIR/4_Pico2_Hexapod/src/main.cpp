/*
 * 4_Pico2_Hexapod
 * RP2350 (Pico2) hexapod brain
 * - Protocol parser over high-speed UART (921600)
 * - Full IK engine (coxa/femur/tibia) with mounting angle compensation
 * - Tripod / Wave / Ripple / Tetrapod gaits
 * - BMI160 IMU via SPI (complementary filter)
 * - PCA9685 servo driver via I2C
 * - Foot-switch adaptive gait
 * - Telemetry sender
 * PlatformIO + Earle Philhower Arduino core
 *
 * FIXES vs v1.0:
 *  [1] Serial USB-CDC: wait for host enumeration before printing
 *  [2] LED blinker: phase 1 properly tracks ON/OFF half-cycles
 *  [3] last_seq dedup: was 8-entry array indexed by nibble — PKT_TELEM(0x81)
 *      collided with PKT_VELOCITY(0x01). Now uses full 256-entry array.
 *  [4] foot_switches_present: was sticky (never cleared). Now re-detected per loop.
 *  [5] SPI: bmi_spi_rw opened/closed transaction per byte (slow + wrong).
 *      Fixed: transaction wraps entire burst.
 *  [6] SPI pins never assigned to SPI object. Added SPI.setRX/TX/SCK/CS.
 *  [7] proto_build hello: length was 7 but "PICO2_OK" is 8 chars.
 *  [8] ERR_OK LED: now does a single slow heartbeat so you always see
 *      something on the LED in every scenario.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "protocol.h"

// ============================
// PIN DEFINITIONS
// ============================
// UART1 from ESP32 (TX=GP8, RX=GP9)
#define UART_TX_PIN 8
#define UART_RX_PIN 9

// I2C0 for PCA9685 (SDA=GP4, SCL=GP5)
#define I2C_SDA 4
#define I2C_SCL 5

// SPI0 for BMI160 (CS=GP13, SCK=GP14, MOSI=GP15, MISO=GP12)
#define BMI_CS   13
#define BMI_SCK  14
#define BMI_MOSI 15
#define BMI_MISO 12

// Foot switches GP16-21 (active low, input_pullup)
#define FOOT0 21
#define FOOT1 20
#define FOOT2 19
#define FOOT3 18
#define FOOT4 17
#define FOOT5 16

// PCA9685 address
#define PCA9685_ADDR_LEFT  0x40
#define PCA9685_ADDR_RIGHT 0x41

// ============================
// DUAL PCA9685 - ONE FOR EACH SIDE
// ============================
// LEFT PCA (0x40) - Left side legs (1,2,3):
//   Leg1(FL)=ch0/1/2, Leg2(ML)=ch5/6/7, Leg3(RL)=ch13/14/15
// RIGHT PCA (0x41) - Right side legs (0,4,5):
//   Leg0(FR)=ch0/1/2, Leg5(MR)=ch13/14/15, Leg4(RR)=ch5/6/7
#define CH_L0_COXA  0   // Leg0 (FR) - Right PCA
#define CH_L0_FEMUR 1
#define CH_L0_TIBIA 2
#define CH_L1_COXA  0   // Leg1 (FL) - Left PCA
#define CH_L1_FEMUR 1
#define CH_L1_TIBIA 2
#define CH_L2_COXA  5   // Leg2 (ML) - Left PCA
#define CH_L2_FEMUR 6
#define CH_L2_TIBIA 7
#define CH_L3_COXA  13  // Leg3 (RL) - Left PCA
#define CH_L3_FEMUR 14
#define CH_L3_TIBIA 15
#define CH_L4_COXA  5   // Leg4 (RR) - Right PCA
#define CH_L4_FEMUR 6
#define CH_L4_TIBIA 7
#define CH_L5_COXA  13  // Leg5 (MR) - Right PCA
#define CH_L5_FEMUR 14
#define CH_L5_TIBIA 15

// ============================
// MOUNTING ANGLES (degrees)
// ============================
static const float mount_deg[6] = {45.0f, 90.0f, 135.0f, 225.0f, 270.0f, 315.0f};

// ============================
// HOME POSITIONS (degrees)
// ============================
static const float home_deg[18] = {
    90.0f, 90.0f, 90.0f,   // Leg0 FR
    90.0f, 90.0f, 90.0f,   // Leg1 FL
    90.0f, 90.0f, 90.0f,   // Leg2 ML
    90.0f, 90.0f, 90.0f,   // Leg3 RL
    90.0f, 90.0f, 90.0f,   // Leg4 RR
    90.0f, 90.0f, 90.0f    // Leg5 MR
};

// Calibration offsets
static float coxa_cal[6]  = {0, 0, 0, 0, 0, 0};
static float femur_cal[6] = {0, 0, 0, 0, 0, 0};
static float tibia_cal[6] = {0, 0, 0, 0, 0, 0};

// ============================
// HEXAPOD GEOMETRY (mm)
// ============================
#define COXA_LEN  30.0f
#define FEMUR_LEN 60.0f
#define TIBIA_LEN 80.0f

static const float leg_x[6] = { 60.0f,  60.0f,   0.0f, -60.0f, -60.0f,   0.0f};
static const float leg_y[6] = {-50.0f,  50.0f,  60.0f,  50.0f, -50.0f, -60.0f};

#define STANCE_W 70.0f
#define STANCE_L 60.0f
#define BODY_H   75.0f

// ============================
// SERVO RANGE
// ============================
#define SERVOMIN    150
#define SERVOMAX    600
#define SERVOCENTER ((SERVOMIN + SERVOMAX) / 2)
#define PWM_FREQ    50.0f

// ============================
// ERROR CODES & LED INDICATOR
// ============================
// Built-in LED on Pico2 is GP25
#define LED_PIN 25

// Error codes — value = number of blinks in the burst:
//   ERR_OK       (0) → heartbeat: 1 slow blink every 2s (solid "alive" signal)
//   ERR_BMI160   (1) → 2 fast blinks, pause
//   ERR_NOCMD    (2) → 3 fast blinks, pause
//   ERR_BMI_NOCMD(3) → 4 fast blinks, pause
enum { ERR_OK = 0, ERR_BMI160 = 1, ERR_NOCMD = 2, ERR_BMI_NOCMD = 3 };
static uint8_t error_code = ERR_OK;

// Non-blocking LED blinker
// FIX [2]: phase 1 now properly tracks LED on/off half-cycles.
// Pattern per error_code:
//   ERR_OK        : slow heartbeat — 200ms on, 1800ms off (always visible)
//   ERR_BMI160    : 2 blinks (150ms on / 150ms off), 1.5s pause
//   ERR_NOCMD     : 3 blinks, 1.5s pause
//   ERR_BMI_NOCMD : 4 blinks, 1.5s pause
static void update_led() {
    static uint32_t last_ms  = 0;
    static uint8_t  phase    = 0;  // 0=pause/idle, 1=burst
    static int      half     = 0;  // half-cycle index within burst (0=first ON, 1=first OFF, ...)
    uint32_t now = millis();

    if (error_code == ERR_OK) {
        // Heartbeat: 200ms on, 1800ms off
        static bool hb_on = false;
        uint32_t interval = hb_on ? 200 : 1800;
        if (now - last_ms >= interval) {
            last_ms = now;
            hb_on = !hb_on;
            digitalWrite(LED_PIN, hb_on ? HIGH : LOW);
        }
        return;
    }

    // Fault blink: n = error_code + 1 blinks
    int n = error_code + 1; // 2, 3, or 4 blinks

    switch (phase) {
        case 0: // idle/pause
            digitalWrite(LED_PIN, LOW);
            if (now - last_ms >= 1500) {
                phase = 1;
                half  = 0;
                last_ms = now;
                digitalWrite(LED_PIN, HIGH); // start first ON immediately
            }
            break;

        case 1: // burst — alternating ON (even half) / OFF (odd half)
            if (now - last_ms >= 150) {
                last_ms = now;
                half++;
                if (half >= n * 2) {
                    // Finished all blinks
                    phase = 0;
                    last_ms = now;
                    digitalWrite(LED_PIN, LOW);
                } else {
                    // Toggle: even half = ON, odd half = OFF
                    digitalWrite(LED_PIN, (half % 2 == 0) ? HIGH : LOW);
                }
            }
            break;
    }
}

// ============================
// GAIT PARAMETERS
// ============================
#define TICK_RATE_HZ 100
#define TICK_US (1000000 / TICK_RATE_HZ)

#define TRIPOD_CYCLE    10
#define WAVE_CYCLE      30
#define RIPPLE_CYCLE    18
#define TETRAPOD_CYCLE  12

// ============================
// GLOBAL STATE
// ============================
static volatile uint32_t tick_counter = 0;
static volatile uint32_t tick_us_last = 0;

static int16_t cmd_vx = 0, cmd_vy = 0, cmd_vr = 0;
static int16_t cmd_body_vz = 0, cmd_body_rx = 0, cmd_body_ry = 0;
static uint8_t cmd_flags = 0, cmd_speed = 100;
static uint8_t gait_mode = 0;
static int8_t  leg_ox[6] = {0}, leg_oy[6] = {0}, leg_oz[6] = {0};

static float n_x = 0.0f, n_y = 0.0f, n_r = 0.0f;
static float n_body_vz = 0.0f, n_body_rx = 0.0f, n_body_ry = 0.0f;

static uint8_t foot_contact = 0;
static bool    foot_early[6] = {false};
// FIX [4]: foot_switches_present is re-evaluated each loop, not sticky.
static bool    foot_switches_present = false;

static bool    imu_ok    = false;
static float   imu_roll  = 0.0f, imu_pitch = 0.0f, imu_yaw = 0.0f;
static int16_t imu_ax, imu_ay, imu_az;
static int16_t imu_gx, imu_gy, imu_gz;

static uint8_t proto_rx_buf[128];
static int     proto_rx_len = 0;
static uint8_t proto_tx_buf[sizeof(PktTelemetry) + 8];
static uint8_t proto_seq = 0;
// FIX [3]: was [8] indexed by nibble — caused collision between
// PKT_VELOCITY(0x01) and PKT_TELEM(0x81) both mapping to index 1.
// Now full 256-entry table, direct index by type byte.
static uint8_t last_seq[256];
static uint32_t last_cmd_ms = 0;

// ============================
// PCA9685 I2C FUNCTIONS
// ============================
static void pca_write(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void pca_set_pwm(uint8_t addr, uint8_t ch, uint16_t off) {
    if (off > 4095) off = 4095;
    uint8_t reg = 0x06 + ch * 4;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(0);
    Wire.write(0);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

static void pca_init(uint8_t addr) {
    pca_write(addr, 0x00, 0x20);  // sleep
    uint8_t prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * PWM_FREQ)) - 1);
    pca_write(addr, 0xFE, prescale);
    pca_write(addr, 0x00, 0x20 | 0x80); // restart
    pca_write(addr, 0x00, 0xA0);  // auto-increment on, normal mode
    delay(5);
}

static void set_servo_pwm(uint8_t addr, uint8_t ch, uint16_t pwm) {
    if (pwm < SERVOMIN) pwm = SERVOMIN;
    if (pwm > SERVOMAX) pwm = SERVOMAX;
    pca_set_pwm(addr, ch, pwm);
}

// ============================
// BMI160 SPI DRIVER
// ============================
// FIX [6]: SPI pins are now assigned to the SPI object.
// FIX [5]: SPI transaction now wraps entire burst, not per-byte.

static void bmi_cs(bool level) {
    digitalWrite(BMI_CS, level ? HIGH : LOW);
}

static bool bmi_init() {
    pinMode(BMI_CS, OUTPUT);
    bmi_cs(true); // deselect

    // FIX [6]: assign SPI pins (Earle Philhower core requires this for non-default pins)
    SPI.setRX(BMI_MISO);
    SPI.setTX(BMI_MOSI);
    SPI.setSCK(BMI_SCK);
    SPI.setCS(BMI_CS);
    SPI.begin(false); // false = don't drive CS automatically; we do it manually

    // Soft reset
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false);
    SPI.transfer(0x7E & 0x7F); // write
    SPI.transfer(0xB6);         // reset cmd
    bmi_cs(true);
    SPI.endTransaction();
    delay(50);

    // Check chip ID
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false);
    SPI.transfer(0x00 | 0x80); // read reg 0x00
    uint8_t id = SPI.transfer(0x00);
    bmi_cs(true);
    SPI.endTransaction();

    if (id != 0xD1 && id != 0xC1) {
        Serial.printf("BMI160 ID fail: 0x%02X\n", id);
        return false;
    }

    // Power up accel
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false); SPI.transfer(0x7E & 0x7F); SPI.transfer(0x11); bmi_cs(true);
    SPI.endTransaction();
    delay(10);

    // Power up gyro
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false); SPI.transfer(0x7E & 0x7F); SPI.transfer(0x15); bmi_cs(true);
    SPI.endTransaction();
    delay(10);

    // Accel config: ±4g, 100Hz
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false); SPI.transfer(0x41 & 0x7F); SPI.transfer(0x03); bmi_cs(true);
    SPI.endTransaction();

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false); SPI.transfer(0x40 & 0x7F); SPI.transfer(0x0A); bmi_cs(true);
    SPI.endTransaction();

    // Gyro config: ±2000°/s, 100Hz
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false); SPI.transfer(0x43 & 0x7F); SPI.transfer(0x00); bmi_cs(true);
    SPI.endTransaction();

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false); SPI.transfer(0x42 & 0x7F); SPI.transfer(0x0A); bmi_cs(true);
    SPI.endTransaction();

    return true;
}

static void bmi_read_all() {
    uint8_t raw[12];

    // FIX [5]: single transaction for the entire burst read
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    bmi_cs(false);
    SPI.transfer(0x0C | 0x80); // burst read from 0x0C: gyro xyz (6) then accel xyz (6)
    for (int i = 0; i < 12; i++) raw[i] = SPI.transfer(0x00);
    bmi_cs(true);
    SPI.endTransaction();

    // BMI160 burst order from 0x0C: GYR_X_L, GYR_X_H, GYR_Y_L, GYR_Y_H, GYR_Z_L, GYR_Z_H,
    //                                ACC_X_L, ACC_X_H, ACC_Y_L, ACC_Y_H, ACC_Z_L, ACC_Z_H
    int16_t gx = (int16_t)(raw[1]  << 8 | raw[0]);
    int16_t gy = (int16_t)(raw[3]  << 8 | raw[2]);
    int16_t gz = (int16_t)(raw[5]  << 8 | raw[4]);
    int16_t ax = (int16_t)(raw[7]  << 8 | raw[6]);
    int16_t ay = (int16_t)(raw[9]  << 8 | raw[8]);
    int16_t az = (int16_t)(raw[11] << 8 | raw[10]);

    imu_ax = ax; imu_ay = ay; imu_az = az;
    imu_gx = gx; imu_gy = gy; imu_gz = gz;

    // Accel scale: ±4g → 8192 LSB/g
    const float a_scale = 1.0f / 8192.0f; // g per LSB
    float ax_g = (float)ax * a_scale;
    float ay_g = (float)ay * a_scale;
    float az_g = (float)az * a_scale;

    // Complementary filter
    const float dt    = 0.01f;
    const float alpha = 0.96f;

    // Gyro scale: ±2000°/s → 16.384 LSB/(°/s)
    const float g_scale = (M_PI / 180.0f) / 16.384f; // rad/s per LSB
    float gx_rs = (float)gx * g_scale;
    float gy_rs = (float)gy * g_scale;
    float gz_rs = (float)gz * g_scale;

    float acc_roll  = atan2f(-ay_g, az_g)                              * 180.0f / M_PI;
    float acc_pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;

    static float roll_f = 0, pitch_f = 0, yaw_f = 0;
    roll_f  = alpha * (roll_f  + gx_rs * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_roll;
    pitch_f = alpha * (pitch_f + gy_rs * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_pitch;
    yaw_f  += gz_rs * dt * 180.0f / M_PI;

    imu_roll = roll_f; imu_pitch = pitch_f; imu_yaw = yaw_f;
}

// ============================
// INVERSE KINEMATICS
// ============================
static void ik_leg(int leg, float fx, float fy, float fz,
                   float* coxa, float* femur, float* tibia) {
    float mount_rad = mount_deg[leg] * M_PI / 180.0f;
    float cos_m = cosf(mount_rad);
    float sin_m = sinf(mount_rad);
    float rx = fx * cos_m - fy * sin_m;
    float ry = fx * sin_m + fy * cos_m;

    float coxa_rad = atan2f(ry, rx);
    float hip_dist = sqrtf(rx * rx + ry * ry) - COXA_LEN;
    if (hip_dist < 1.0f) hip_dist = 1.0f;

    float fz_eff = -fz;
    float d = sqrtf(hip_dist * hip_dist + fz_eff * fz_eff);
    if (d > (FEMUR_LEN + TIBIA_LEN)) d = FEMUR_LEN + TIBIA_LEN;

    float cos_fem = (FEMUR_LEN * FEMUR_LEN + d * d - TIBIA_LEN * TIBIA_LEN)
                  / (2.0f * FEMUR_LEN * d);
    cos_fem = fmaxf(-1.0f, fminf(1.0f, cos_fem));
    float femur_rad = acosf(cos_fem) + atan2f(fz_eff, hip_dist);

    float cos_tib = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - d * d)
                  / (2.0f * FEMUR_LEN * TIBIA_LEN);
    cos_tib = fmaxf(-1.0f, fminf(1.0f, cos_tib));
    float tibia_rad = acosf(cos_tib) - M_PI / 2.0f;

    *coxa  = coxa_rad  * 180.0f / M_PI;
    *femur = femur_rad * 180.0f / M_PI;
    *tibia = tibia_rad * 180.0f / M_PI;
}

// ============================
// BODY TRANSFORM
// ============================
static void body_transform(int leg, float vz, float rx_deg, float ry_deg,
                           float* fx, float* fy, float* fz) {
    float rx_rad = rx_deg * M_PI / 180.0f;
    float ry_rad = ry_deg * M_PI / 180.0f;

    float lx = leg_x[leg];
    float ly = leg_y[leg];
    float lz = vz;

    float y1 = ly * cosf(rx_rad) - lz * sinf(rx_rad);
    float z1 = ly * sinf(rx_rad) + lz * cosf(rx_rad);

    float x2 = lx * cosf(ry_rad) + z1 * sinf(ry_rad);
    float z2 = -lx * sinf(ry_rad) + z1 * cosf(ry_rad);

    *fx = -x2;
    *fy = -y1;
    *fz = -z2 + BODY_H;
}

// ============================
// FOOT TRAJECTORY
// ============================
static void foot_traj(float phase, float stride_x, float stride_y, float step_h,
                      float* fx, float* fy, float* fz) {
    float t = fmodf(phase, 1.0f);
    if (t < 0.5f) {
        // Swing
        float u = t * 2.0f;
        *fx = stride_x * (u - 0.5f);
        *fy = stride_y * (u - 0.5f);
        *fz = step_h * sinf(u * M_PI);
    } else {
        // Stance
        float u = (t - 0.5f) * 2.0f;
        *fx = stride_x * (u - 0.5f);
        *fy = stride_y * (u - 0.5f);
        *fz = 0.0f;
    }
}

// ============================
// GAIT ENGINES
// ============================
static const float tripod_phase[6]   = {0.0f, 0.5f, 0.0f,   0.5f,  0.0f,  0.5f};
static const float wave_phase[6]     = {0.0f, 0.2f, 0.4f,   0.6f,  0.8f,  0.0f};
static const float ripple_phase[6]   = {0.0f, 0.333f, 0.667f, 0.167f, 0.5f, 0.833f};
static const float tetrapod_phase[6] = {0.0f, 0.0f, 0.5f,   0.0f,  0.0f,  0.5f};

static int get_cycle_len() {
    switch (gait_mode) {
        case 1: return WAVE_CYCLE;
        case 2: return RIPPLE_CYCLE;
        case 3: return TETRAPOD_CYCLE;
        default: return TRIPOD_CYCLE;
    }
}

static const float* get_phase_offsets() {
    switch (gait_mode) {
        case 1: return wave_phase;
        case 2: return ripple_phase;
        case 3: return tetrapod_phase;
        default: return tripod_phase;
    }
}

// ============================
// FOOT SWITCH ADAPTIVE GAIT
// ============================
static void read_foot_switches() {
    // FIX [4]: re-detect presence each call so disconnected switches don't
    //          permanently lock foot_switches_present to true.
    foot_contact         = 0;
    foot_switches_present = false;

    if (!digitalRead(FOOT0)) { foot_contact |= (1 << 0); foot_switches_present = true; }
    if (!digitalRead(FOOT1)) { foot_contact |= (1 << 1); foot_switches_present = true; }
    if (!digitalRead(FOOT2)) { foot_contact |= (1 << 2); foot_switches_present = true; }
    if (!digitalRead(FOOT3)) { foot_contact |= (1 << 3); foot_switches_present = true; }
    if (!digitalRead(FOOT4)) { foot_contact |= (1 << 4); foot_switches_present = true; }
    if (!digitalRead(FOOT5)) { foot_contact |= (1 << 5); foot_switches_present = true; }
}

// ============================
// PROTOCOL HANDLING
// ============================
static void process_proto_packet(uint8_t type, uint8_t* payload, uint8_t len, uint8_t seq) {
    // FIX [3]: direct index by full type byte (256 entries), no nibble collision
    if (seq == last_seq[type]) return;
    last_seq[type] = seq;

    switch (type) {
        case PKT_VELOCITY: {
            if (len < sizeof(PktVelocity)) return;
            last_cmd_ms = millis();
            PktVelocity* v = (PktVelocity*)payload;
            cmd_vx       = v->vx;
            cmd_vy       = v->vy;
            cmd_vr       = v->vr;
            cmd_body_vz  = v->body_vz;
            cmd_body_rx  = v->body_rx;
            cmd_body_ry  = v->body_ry;
            cmd_flags    = v->flags;
            cmd_speed    = v->speed;
            gait_mode    = (v->flags >> 1) & 0x03;
            break;
        }
        case PKT_LEG_OFFSET: {
            if (len < sizeof(PktLegOffset)) return;
            PktLegOffset* o = (PktLegOffset*)payload;
            memcpy(leg_ox, o->ox, 6);
            memcpy(leg_oy, o->oy, 6);
            memcpy(leg_oz, o->oz, 6);
            break;
        }
        case PKT_CALIB: {
            if (len < 18) return;
            for (int i = 0; i < 6; i++) {
                coxa_cal[i]  = (int8_t)payload[i];
                femur_cal[i] = (int8_t)payload[6  + i];
                tibia_cal[i] = (int8_t)payload[12 + i];
            }
            break;
        }
        case PKT_MODE: {
            if (len < 1) return;
            gait_mode = payload[0] & 0x03;
            break;
        }
    }
}

static void parse_uart() {
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (proto_rx_len < (int)sizeof(proto_rx_buf)) {
            proto_rx_buf[proto_rx_len++] = b;
        } else {
            proto_rx_len = 0;
        }

        uint8_t type, payload[64], seq;
        int plen = proto_parse(proto_rx_buf, proto_rx_len, &type, payload, &seq);
        if (plen > 0) {
            process_proto_packet(type, payload, (uint8_t)plen, seq);
            proto_rx_len = 0;
        } else if (plen < 0) {
            proto_rx_len = 0;
        }
    }
}

static void send_telemetry() {
    PktTelemetry t;
    t.batt_mv      = 0; // TODO: ADC read
    t.roll         = (int16_t)(imu_roll  * 1000);
    t.pitch        = (int16_t)(imu_pitch * 1000);
    t.yaw          = (int16_t)(imu_yaw   * 1000);
    t.foot_contact = foot_contact;
    t.state        = cmd_flags;
    t.ax = imu_ax; t.ay = imu_ay; t.az = imu_az;
    t.gx = imu_gx; t.gy = imu_gy; t.gz = imu_gz;

    int n = proto_build(proto_tx_buf, PKT_TELEM, (uint8_t*)&t, sizeof(t), proto_seq++);
    Serial1.write(proto_tx_buf, n);
}

// ============================
// MAIN LOOP HELPERS (forward-declared)
// ============================
static uint32_t last_telem_ms = 0;
static uint32_t last_print_ms = 0;
static bool     walk_enabled    = false;
static bool     balance_enabled = false;
static int      prev_cycle_len  = TRIPOD_CYCLE;

static void servo_deg_to_pwm(int leg, float coxa_deg, float femur_deg, float tibia_deg,
                              uint16_t* coxa_pwm, uint16_t* femur_pwm, uint16_t* tibia_pwm) {
    float c = fmaxf(0.0f, fminf(180.0f, coxa_deg  + coxa_cal[leg]));
    float f = fmaxf(0.0f, fminf(180.0f, femur_deg + femur_cal[leg]));
    float t = fmaxf(0.0f, fminf(180.0f, tibia_deg + tibia_cal[leg]));

    *coxa_pwm  = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * c / 180.0f);
    *femur_pwm = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * f / 180.0f);
    *tibia_pwm = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * t / 180.0f);
}

static void get_servo_channels(int leg, uint8_t* addr,
                               uint8_t* ch_coxa, uint8_t* ch_femur, uint8_t* ch_tibia) {
    switch (leg) {
        case 0: *addr = PCA9685_ADDR_RIGHT; *ch_coxa = CH_L0_COXA;  *ch_femur = CH_L0_FEMUR;  *ch_tibia = CH_L0_TIBIA; break;
        case 1: *addr = PCA9685_ADDR_LEFT;  *ch_coxa = CH_L1_COXA;  *ch_femur = CH_L1_FEMUR;  *ch_tibia = CH_L1_TIBIA; break;
        case 2: *addr = PCA9685_ADDR_LEFT;  *ch_coxa = CH_L2_COXA;  *ch_femur = CH_L2_FEMUR;  *ch_tibia = CH_L2_TIBIA; break;
        case 3: *addr = PCA9685_ADDR_LEFT;  *ch_coxa = CH_L3_COXA;  *ch_femur = CH_L3_FEMUR;  *ch_tibia = CH_L3_TIBIA; break;
        case 4: *addr = PCA9685_ADDR_RIGHT; *ch_coxa = CH_L4_COXA;  *ch_femur = CH_L4_FEMUR;  *ch_tibia = CH_L4_TIBIA; break;
        case 5: *addr = PCA9685_ADDR_RIGHT; *ch_coxa = CH_L5_COXA;  *ch_femur = CH_L5_FEMUR;  *ch_tibia = CH_L5_TIBIA; break;
        default: break;
    }
}

void tick_gait() {
    float spd_scale = (float)cmd_speed / 100.0f;
    float stride_vx = (float)cmd_vx * spd_scale / 1000.0f * STANCE_L;
    float stride_vy = (float)cmd_vy * spd_scale / 1000.0f * STANCE_W;
    float stride_vr = (float)cmd_vr * spd_scale / 1000.0f * 30.0f;

    int   cycle   = get_cycle_len();
    float body_vz = (float)cmd_body_vz / 10.0f;
    float body_rx = (float)cmd_body_rx / 10.0f;
    float body_ry = (float)cmd_body_ry / 10.0f;

    if (tick_counter == 0 || cycle != prev_cycle_len) {
        n_x       = stride_vx;
        n_y       = stride_vy;
        n_r       = stride_vr;
        n_body_vz = body_vz;
        n_body_rx = body_rx;
        n_body_ry = body_ry;
        prev_cycle_len = cycle;
    }

    walk_enabled    = (cmd_flags & 0x01) || (cmd_flags & 0x20);
    balance_enabled = foot_switches_present ? (cmd_flags & 0x10) : true;

    const float* phases = get_phase_offsets();

    float balance_rx = 0, balance_ry = 0;
    if (balance_enabled && imu_ok) {
        balance_rx = -imu_roll;
        balance_ry = -imu_pitch;
    }

    for (int leg = 0; leg < 6; leg++) {
        float phase = fmodf((float)tick_counter / (float)cycle + phases[leg], 1.0f);

        float fx, fy, fz;
        if (walk_enabled && (abs(cmd_vx) > 10 || abs(cmd_vy) > 10 || abs(cmd_vr) > 10)) {
            float step_h = 30.0f;
            foot_traj(phase, n_x, n_y, step_h, &fx, &fy, &fz);

            float rot_rad = n_r * M_PI / 180.0f;
            float rx = leg_x[leg];
            float ry = leg_y[leg];
            fx += rx * cosf(rot_rad) - ry * sinf(rot_rad) - rx;
            fy += rx * sinf(rot_rad) + ry * cosf(rot_rad) - ry;
        } else {
            fx = 0; fy = 0; fz = 0;
        }

        fx += (float)leg_ox[leg];
        fy += (float)leg_oy[leg];
        fz += (float)leg_oz[leg];

        float bfx, bfy, bfz;
        body_transform(leg, n_body_vz, n_body_rx + balance_rx, n_body_ry + balance_ry,
                       &bfx, &bfy, &bfz);

        float coxa_deg, femur_deg, tibia_deg;
        ik_leg(leg, fx + bfx, fy + bfy, fz + bfz, &coxa_deg, &femur_deg, &tibia_deg);

        int hi = leg * 3;
        coxa_deg  += home_deg[hi];
        femur_deg += home_deg[hi + 1];
        tibia_deg += home_deg[hi + 2];

        uint16_t coxa_pwm, femur_pwm, tibia_pwm;
        servo_deg_to_pwm(leg, coxa_deg, femur_deg, tibia_deg,
                         &coxa_pwm, &femur_pwm, &tibia_pwm);

        // Adaptive foot switch: detect early ground contact during swing
        if (phase > 0.05f && phase < 0.5f) {
            if (foot_contact & (1 << leg)) foot_early[leg] = true;
        }
        if (phase >= 0.5f && phase < 0.55f) {
            foot_early[leg] = false;
        }

        uint8_t addr, ch_coxa, ch_femur, ch_tibia;
        get_servo_channels(leg, &addr, &ch_coxa, &ch_femur, &ch_tibia);
        set_servo_pwm(addr, ch_coxa,  coxa_pwm);
        set_servo_pwm(addr, ch_femur, femur_pwm);
        set_servo_pwm(addr, ch_tibia, tibia_pwm);
    }
}

// ============================
// SETUP
// ============================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    memset(last_seq, 0xFF, sizeof(last_seq)); // 0xFF so first packet (seq=0) isn't deduped

    // Power-on blink: 3 quick flashes = alive
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW);  delay(100);
    }

    // FIX [1]: USB CDC Serial — must wait for host enumeration or nothing prints.
    //          Timeout after 3s so the bot still runs headless.
    Serial.begin(115200);
    uint32_t usb_wait = millis();
    while (!Serial && (millis() - usb_wait < 3000)) { delay(10); }

    // UART1 (ESP32 link)
    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.begin(921600);

    Serial.println("Pico2 Hexapod Brain v1.1");

    // I2C for PCA9685
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

    pca_init(PCA9685_ADDR_LEFT);
    pca_init(PCA9685_ADDR_RIGHT);

    // Servos to home
    for (int leg = 0; leg < 6; leg++) {
        uint8_t addr, ch_coxa, ch_femur, ch_tibia;
        get_servo_channels(leg, &addr, &ch_coxa, &ch_femur, &ch_tibia);
        set_servo_pwm(addr, ch_coxa,  SERVOCENTER);
        set_servo_pwm(addr, ch_femur, SERVOCENTER);
        set_servo_pwm(addr, ch_tibia, SERVOCENTER);
    }

    // Foot switches
    pinMode(FOOT0, INPUT_PULLUP);
    pinMode(FOOT1, INPUT_PULLUP);
    pinMode(FOOT2, INPUT_PULLUP);
    pinMode(FOOT3, INPUT_PULLUP);
    pinMode(FOOT4, INPUT_PULLUP);
    pinMode(FOOT5, INPUT_PULLUP);

    // BMI160
    imu_ok = bmi_init();
    Serial.println(imu_ok ? "BMI160 OK" : "BMI160 FAILED — balance disabled");

    // Hello packet
    const uint8_t hello[] = "PICO2_OK"; // FIX [7]: length is 8, not 7
    int n = proto_build(proto_tx_buf, PKT_DEBUG, hello, 8, proto_seq++);
    Serial1.write(proto_tx_buf, n);

    Serial.println("Setup complete.");
}

// ============================
// LOOP
// ============================
void loop() {
    uint32_t now = micros();

    parse_uart();
    read_foot_switches();

    static uint32_t last_imu_us = 0;
    if (imu_ok && now - last_imu_us >= 10000) {
        last_imu_us = now;
        bmi_read_all();
    }

    if (now - tick_us_last >= TICK_US) {
        tick_us_last = now;
        tick_counter++;
        tick_gait();
    }

    if (millis() - last_telem_ms >= 50) {
        last_telem_ms = millis();
        send_telemetry();
    }

    if (millis() - last_print_ms >= 500) {
        last_print_ms = millis();
        Serial.printf("roll=%.1f pitch=%.1f yaw=%.1f vx=%d vy=%d vr=%d gait=%d contact=0x%02X err=%d\n",
                      imu_roll, imu_pitch, imu_yaw,
                      cmd_vx, cmd_vy, cmd_vr,
                      gait_mode, foot_contact, error_code);
    }

    // Update error code for LED
    bool cmd_timeout = (millis() - last_cmd_ms > 3000);
    if      (!imu_ok && cmd_timeout) error_code = ERR_BMI_NOCMD;
    else if (!imu_ok)                error_code = ERR_BMI160;
    else if (cmd_timeout)            error_code = ERR_NOCMD;
    else                             error_code = ERR_OK;

    update_led();
}