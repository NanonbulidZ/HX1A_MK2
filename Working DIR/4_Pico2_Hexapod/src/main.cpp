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

// SPI0 for BMI160 (CS=GP17, SCK=GP18, MOSI=GP19, MISO=GP16)
#define BMI_CS   17
#define BMI_SCK  18
#define BMI_MOSI 19
#define BMI_MISO 16

// Foot switches GPIO 22-17 (active low, input_pullup)
#define FOOT0 22
#define FOOT1 21
#define FOOT2 20
#define FOOT3 19
#define FOOT4 18
#define FOOT5 17

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
// Leg0(FR)=+45, Leg1(FL)=+90, Leg2(ML)=+135, Leg3(RL)=+225, Leg4(RR)=+270, Leg5(MR)=+315
static const float mount_deg[6] = {45.0f, 90.0f, 135.0f, 225.0f, 270.0f, 315.0f};

// ============================
// HOME POSITIONS (degrees)
// ============================
// 18 values: L0_coxa,L0_femur,L0_tibia, L1_coxa,... same order as CH_ defines
static const float home_deg[18] = {
    90.0f, 90.0f, 90.0f,   // Leg0 FR
    90.0f, 90.0f, 90.0f,   // Leg1 FL
    90.0f, 90.0f, 90.0f,   // Leg2 ML
    90.0f, 90.0f, 90.0f,   // Leg3 RL
    90.0f, 90.0f, 90.0f,   // Leg4 RR
    90.0f, 90.0f, 90.0f    // Leg5 MR
};

// Calibration offsets (pulled from original hexapod)
static float coxa_cal[6]  = {0, 0, 0, 0, 0, 0};
static float femur_cal[6] = {0, 0, 0, 0, 0, 0};
static float tibia_cal[6] = {0, 0, 0, 0, 0, 0};

// ============================
// HEXAPOD GEOMETRY (mm)
// ============================
#define COXA_LEN  30.0f
#define FEMUR_LEN 60.0f
#define TIBIA_LEN 80.0f

// Leg positions relative to body center (mm)
// Leg0(FR) x=+60,y=-50, Leg1(FL) x=+60,y=+50, Leg2(ML) x=0,y=+60
// Leg3(RL) x=-60,y=+50, Leg4(RR) x=-60,y=-50, Leg5(MR) x=0,y=-60
static const float leg_x[6] = { 60.0f,  60.0f,   0.0f, -60.0f, -60.0f,   0.0f};
static const float leg_y[6] = {-50.0f,  50.0f,  60.0f,  50.0f, -50.0f, -60.0f};

// Default stance width (mm from body center)
#define STANCE_W 70.0f
#define STANCE_L 60.0f
#define BODY_H   75.0f

// ============================
// SERVO RANGE (raw PCA9685 PWM counts, 0-4095 @ 50Hz)
// ============================
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVOCENTER ((SERVOMIN + SERVOMAX) / 2)
#define PWM_FREQ 50.0f

// ============================
// GAIT PARAMETERS
// ============================
#define TICK_RATE_HZ 100   // gait ticks per second
#define TICK_US (1000000 / TICK_RATE_HZ)

// Gait cycle lengths (ticks)
#define TRIPOD_CYCLE    10
#define WAVE_CYCLE      30
#define RIPPLE_CYCLE    18
#define TETRAPOD_CYCLE  12

// ============================
// GLOBAL STATE
// ============================
static volatile uint32_t tick_counter = 0;
static volatile uint32_t tick_us_last = 0;

// Command state
static int16_t cmd_vx = 0, cmd_vy = 0, cmd_vr = 0;
static int16_t cmd_body_vz = 0, cmd_body_rx = 0, cmd_body_ry = 0;
static uint8_t cmd_flags = 0, cmd_speed = 100;
static uint8_t gait_mode = 0; // 0=tripod, 1=wave, 2=ripple, 3=tetrapod
static int8_t leg_ox[6] = {0}, leg_oy[6] = {0}, leg_oz[6] = {0};

// Stride length (locked at tick==0 to prevent mid-cycle changes)
static float n_x = 0.0f, n_y = 0.0f, n_r = 0.0f;
static float n_body_vz = 0.0f, n_body_rx = 0.0f, n_body_ry = 0.0f;

// Foot switch state
static uint8_t foot_contact = 0;
static uint32_t foot_swing_start[6] = {0};
static bool foot_early[6] = {false};
static bool foot_switches_present = false;

// IMU state
static float imu_roll = 0.0f, imu_pitch = 0.0f, imu_yaw = 0.0f;
static int16_t imu_ax, imu_ay, imu_az;
static int16_t imu_gx, imu_gy, imu_gz;

// Protocol state
static uint8_t proto_rx_buf[128];
static int proto_rx_len = 0;
static uint8_t proto_tx_buf[sizeof(PktTelemetry) + 8];
static uint8_t proto_seq = 0;
static uint8_t last_seq[8] = {0}; // dedup per type

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
    pca_write(addr, 0x00, 0x20);
    pca_write(addr, 0xFE, (uint8_t)(roundf(25000000.0f / (4096.0f * PWM_FREQ)) - 1));
    pca_write(addr, 0x00, 0x20 | 0x80);
    pca_write(addr, 0x00, 0xA0);
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
static void bmi_cs(bool level) {
    digitalWrite(BMI_CS, level ? HIGH : LOW);
}

static uint8_t bmi_spi_rw(uint8_t d) {
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    uint8_t r = SPI.transfer(d);
    SPI.endTransaction();
    return r;
}

static void bmi_write(uint8_t reg, uint8_t val) {
    bmi_cs(LOW);
    bmi_spi_rw(reg & 0x7F); // write bit 0
    bmi_spi_rw(val);
    bmi_cs(HIGH);
}

static uint8_t bmi_read(uint8_t reg) {
    bmi_cs(LOW);
    bmi_spi_rw(reg | 0x80); // read bit 1
    uint8_t r = bmi_spi_rw(0);
    bmi_cs(HIGH);
    return r;
}

static void bmi_read_burst(uint8_t reg, uint8_t* buf, int len) {
    bmi_cs(LOW);
    bmi_spi_rw(reg | 0x80);
    for (int i = 0; i < len; i++) buf[i] = bmi_spi_rw(0);
    bmi_cs(HIGH);
}

static bool bmi_init() {
    pinMode(BMI_CS, OUTPUT);
    bmi_cs(HIGH);
    SPI.begin();

    // Soft reset
    bmi_write(0x7E, 0xB6);
    delay(50);

    // Check chip ID
    uint8_t id = bmi_read(0x00);
    if (id != 0xD1 && id != 0xC1) {
        Serial.printf("BMI160 ID fail: 0x%02X\n", id);
        return false;
    }

    // Power up accel (normal mode)
    bmi_write(0x7E, 0x11); // accel normal
    delay(10);
    // Power up gyro (normal mode)
    bmi_write(0x7E, 0x15); // gyro normal
    delay(10);

    // Accel config: ±4g, 100Hz
    bmi_write(0x41, 0x03); // accel range ±4g
    bmi_write(0x40, 0x0A); // accel ODR 100Hz
    // Gyro config: ±2000°/s, 100Hz
    bmi_write(0x43, 0x00); // gyro range ±2000°/s
    bmi_write(0x42, 0x0A); // gyro ODR 100Hz

    return true;
}

static void bmi_read_all() {
    uint8_t raw[12];
    bmi_read_burst(0x0C, raw, 12); // accel xyz (6) + gyro xyz (6)

    int16_t ax = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t ay = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t az = (int16_t)(raw[5] << 8 | raw[4]);
    int16_t gx = (int16_t)(raw[7] << 8 | raw[6]);
    int16_t gy = (int16_t)(raw[9] << 8 | raw[8]);
    int16_t gz = (int16_t)(raw[11] << 8 | raw[10]);

    imu_ax = ax; imu_ay = ay; imu_az = az;
    imu_gx = gx; imu_gy = gy; imu_gz = gz;

    // Accel mg: ±4g = 8192 LSB/g at 16-bit
    float a_scale = 1000.0f / 8192.0f;
    float ax_g = (float)ax * a_scale * 0.001f;
    float ay_g = (float)ay * a_scale * 0.001f;
    float az_g = (float)az * a_scale * 0.001f;

    // Complementary filter: dt ≈ 0.01s @ 100Hz
    float dt = 0.01f;
    float alpha = 0.96f;

    // Gyro rad/s: ±2000°/s = 16.384 LSB/°/s, convert to rad/s
    float g_scale = (M_PI / 180.0f) / 16.384f;
    float gx_rs = (float)gx * g_scale;
    float gy_rs = (float)gy * g_scale;
    float gz_rs = (float)gz * g_scale;

    // Accel-derived angles
    float acc_roll  = atan2f(-ay_g, az_g) * 180.0f / M_PI;
    float acc_pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;

    // Complementary fusion
    static float roll_f = 0, pitch_f = 0, yaw_f = 0;
    roll_f  = alpha * (roll_f  + gx_rs * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_roll;
    pitch_f = alpha * (pitch_f + gy_rs * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_pitch;
    yaw_f  += gz_rs * dt * 180.0f / M_PI;

    imu_roll = roll_f; imu_pitch = pitch_f; imu_yaw = yaw_f;
}

// ============================
// INVERSE KINEMATICS
// ============================
// Returns: coxa_deg, femur_deg, tibia_deg
static void ik_leg(int leg, float fx, float fy, float fz, float* coxa, float* femur, float* tibia) {
    // Mounting rotation
    float mount_rad = mount_deg[leg] * M_PI / 180.0f;
    float cos_m = cosf(mount_rad);
    float sin_m = sinf(mount_rad);
    float rx = fx * cos_m - fy * sin_m;
    float ry = fx * sin_m + fy * cos_m;

    // Coxa angle
    float coxa_rad = atan2f(ry, rx);
    float hip_dist = sqrtf(rx * rx + ry * ry) - COXA_LEN;
    if (hip_dist < 1.0f) hip_dist = 1.0f;

    // Femur-tibia plane
    float fz_eff = -fz; // foot down = negative
    float d = sqrtf(hip_dist * hip_dist + fz_eff * fz_eff);
    if (d > (FEMUR_LEN + TIBIA_LEN)) d = FEMUR_LEN + TIBIA_LEN;

    float cos_fem = (FEMUR_LEN * FEMUR_LEN + d * d - TIBIA_LEN * TIBIA_LEN) / (2.0f * FEMUR_LEN * d);
    if (cos_fem < -1.0f) cos_fem = -1.0f;
    if (cos_fem > 1.0f)  cos_fem = 1.0f;

    float femur_rad = acosf(cos_fem) + atan2f(fz_eff, hip_dist);

    float cos_tib = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - d * d) / (2.0f * FEMUR_LEN * TIBIA_LEN);
    if (cos_tib < -1.0f) cos_tib = -1.0f;
    if (cos_tib > 1.0f)  cos_tib = 1.0f;
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

    // Roll (around x)
    float y1 = ly * cosf(rx_rad) - lz * sinf(rx_rad);
    float z1 = ly * sinf(rx_rad) + lz * cosf(rx_rad);

    // Pitch (around y)
    float x2 = lx * cosf(ry_rad) + z1 * sinf(ry_rad);
    float z2 = -lx * sinf(ry_rad) + z1 * cosf(ry_rad);

    // Foot position relative to coxa
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
        // Swing phase
        float u = t * 2.0f;
        *fx = stride_x * (u - 0.5f);
        *fy = stride_y * (u - 0.5f);
        *fz = step_h * sinf(u * M_PI);
    } else {
        // Stance phase
        float u = (t - 0.5f) * 2.0f;
        *fx = stride_x * (u - 0.5f);
        *fy = stride_y * (u - 0.5f);
        *fz = 0.0f;
    }
}

// ============================
// GAIT ENGINES
// ============================
// Phase offsets per leg for each gait
// Tripod: [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]
// Wave:   [0.0, 0.2, 0.4, 0.6, 0.8, 0.0]  (actually 0,1,2,3,4,5 sequential)
// Ripple: [0.0, 0.333, 0.667, 0.167, 0.5, 0.833]
// Tetrapod: [0.0, 0.5, 0.0, 0.5, 0.0, 0.5] but with 4 legs stance

static const float tripod_phase[6]   = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
static const float wave_phase[6]     = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 0.0f};
static const float ripple_phase[6]   = {0.0f, 0.333f, 0.667f, 0.167f, 0.5f, 0.833f};
static const float tetrapod_phase[6] = {0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.5f};

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
    foot_contact = 0;
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
    // Dedup: skip if we've seen this sequence for this type recently
    if (seq == last_seq[type & 0x0F]) return;
    last_seq[type & 0x0F] = seq;

    switch (type) {
        case PKT_VELOCITY: {
            if (len < sizeof(PktVelocity)) return;
            PktVelocity* v = (PktVelocity*)payload;
            cmd_vx = v->vx;
            cmd_vy = v->vy;
            cmd_vr = v->vr;
            cmd_body_vz = v->body_vz;
            cmd_body_rx = v->body_rx;
            cmd_body_ry = v->body_ry;
            cmd_flags = v->flags;
            cmd_speed = v->speed;

            // Extract gait from flags bits 1-2
            gait_mode = (v->flags >> 1) & 0x03;

            // Lock stride at cycle start (tick==0 detection handled in tick)
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
                femur_cal[i] = (int8_t)payload[6 + i];
                tibia_cal[i] = (int8_t)payload[12 + i];
            }
            break;
        }
        case PKT_MODE: {
            if (len < 2) return;
            gait_mode = payload[0];
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
    t.batt_mv = 0; // TODO: ADC read
    t.roll  = (int16_t)(imu_roll  * 1000);
    t.pitch = (int16_t)(imu_pitch * 1000);
    t.yaw   = (int16_t)(imu_yaw   * 1000);
    t.foot_contact = foot_contact;
    t.state = cmd_flags;
    t.ax = imu_ax; t.ay = imu_ay; t.az = imu_az;
    t.gx = imu_gx; t.gy = imu_gy; t.gz = imu_gz;

    int n = proto_build(proto_tx_buf, PKT_TELEM, (uint8_t*)&t, sizeof(t), proto_seq++);
    Serial1.write(proto_tx_buf, n);
}

// ============================
// MAIN LOOP FUNCTIONS
// ============================
static uint32_t last_telem_ms = 0;
static uint32_t last_print_ms = 0;
static bool walk_enabled = false;
static bool balance_enabled = false;
static int prev_cycle_len = TRIPOD_CYCLE;

static void servo_deg_to_pwm(int leg, float coxa_deg, float femur_deg, float tibia_deg,
                             uint16_t* coxa_pwm, uint16_t* femur_pwm, uint16_t* tibia_pwm) {
    float c = coxa_deg  + coxa_cal[leg];
    float f = femur_deg + femur_cal[leg];
    float t = tibia_deg + tibia_cal[leg];

    if (c < 0) c = 0; if (c > 180) c = 180;
    if (f < 0) f = 0; if (f > 180) f = 180;
    if (t < 0) t = 0; if (t > 180) t = 180;

    *coxa_pwm  = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * c / 180.0f);
    *femur_pwm = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * f / 180.0f);
    *tibia_pwm = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * t / 180.0f);
}

// Per-leg channel lookup
static void get_servo_channels(int leg, uint8_t* addr, uint8_t* ch_coxa, uint8_t* ch_femur, uint8_t* ch_tibia) {
    // Leg0(FR) and Leg4(RR) and Leg5(MR) on RIGHT PCA
    // Leg1(FL), Leg2(ML), Leg3(RL) on LEFT PCA
    switch (leg) {
        case 0: *addr = PCA9685_ADDR_RIGHT; *ch_coxa = CH_L0_COXA;  *ch_femur = CH_L0_FEMUR;  *ch_tibia = CH_L0_TIBIA; break;
        case 1: *addr = PCA9685_ADDR_LEFT;  *ch_coxa = CH_L1_COXA;  *ch_femur = CH_L1_FEMUR;  *ch_tibia = CH_L1_TIBIA; break;
        case 2: *addr = PCA9685_ADDR_LEFT;  *ch_coxa = CH_L2_COXA;  *ch_femur = CH_L2_FEMUR;  *ch_tibia = CH_L2_TIBIA; break;
        case 3: *addr = PCA9685_ADDR_LEFT;  *ch_coxa = CH_L3_COXA;  *ch_femur = CH_L3_FEMUR;  *ch_tibia = CH_L3_TIBIA; break;
        case 4: *addr = PCA9685_ADDR_RIGHT; *ch_coxa = CH_L4_COXA;  *ch_femur = CH_L4_FEMUR;  *ch_tibia = CH_L4_TIBIA; break;
        case 5: *addr = PCA9685_ADDR_RIGHT; *ch_coxa = CH_L5_COXA;  *ch_femur = CH_L5_FEMUR;  *ch_tibia = CH_L5_TIBIA; break;
    }
}

void tick_gait() {
    // Read command speed (0-100) and scale stride
    float spd_scale = (float)cmd_speed / 100.0f;
    float stride_vx = (float)cmd_vx * spd_scale / 1000.0f * STANCE_L; // mm
    float stride_vy = (float)cmd_vy * spd_scale / 1000.0f * STANCE_W;
    float stride_vr = (float)cmd_vr * spd_scale / 1000.0f * 30.0f; // degrees

    int cycle = get_cycle_len();
    float body_vz = (float)cmd_body_vz / 10.0f; // mm
    float body_rx = (float)cmd_body_rx / 10.0f; // degrees
    float body_ry = (float)cmd_body_ry / 10.0f;

    // Lock n values at tick 0 if cycle changed
    if (tick_counter == 0 || cycle != prev_cycle_len) {
        n_x = stride_vx;
        n_y = stride_vy;
        n_r = stride_vr;
        n_body_vz = body_vz;
        n_body_rx = body_rx;
        n_body_ry = body_ry;
        prev_cycle_len = cycle;
    }

    walk_enabled = (cmd_flags & 0x01) || (cmd_flags & 0x20);
    // Auto-balance when foot switches absent; gamepad controls when present
    balance_enabled = foot_switches_present ? (cmd_flags & 0x10) : true;

    const float* phases = get_phase_offsets();

    // Balance: apply roll/pitch from IMU as body compensation
    float balance_rx = 0, balance_ry = 0;
    if (balance_enabled) {
        balance_rx = -imu_roll;
        balance_ry = -imu_pitch;
    }

    for (int leg = 0; leg < 6; leg++) {
        float phase = fmodf((float)tick_counter / (float)cycle + phases[leg], 1.0f);

        float fx, fy, fz;
        if (walk_enabled && (abs(cmd_vx) > 10 || abs(cmd_vy) > 10 || abs(cmd_vr) > 10)) {
            float step_h = 30.0f; // mm
            foot_traj(phase, n_x, n_y, step_h, &fx, &fy, &fz);

            // Rotation contribution
            float rot_rad = n_r * M_PI / 180.0f;
            float rx = leg_x[leg];
            float ry = leg_y[leg];
            float rot_x = rx * cosf(rot_rad) - ry * sinf(rot_rad) - rx;
            float rot_y = rx * sinf(rot_rad) + ry * cosf(rot_rad) - ry;
            fx += rot_x;
            fy += rot_y;
        } else {
            fx = 0; fy = 0; fz = 0;
        }

        // Leg offset
        fx += (float)leg_ox[leg];
        fy += (float)leg_oy[leg];
        fz += (float)leg_oz[leg];

        // Body transform
        float bfx, bfy, bfz;
        body_transform(leg, n_body_vz, n_body_rx + balance_rx, n_body_ry + balance_ry, &bfx, &bfy, &bfz);

        // Final foot pos in leg frame
        float ffx = fx + bfx;
        float ffy = fy + bfy;
        float ffz = fz + bfz;

        // IK
        float coxa_deg, femur_deg, tibia_deg;
        ik_leg(leg, ffx, ffy, ffz, &coxa_deg, &femur_deg, &tibia_deg);

        // Home position offset
        int hi = leg * 3;
        coxa_deg  += home_deg[hi];
        femur_deg += home_deg[hi + 1];
        tibia_deg += home_deg[hi + 2];

        // Convert to servo pulse
        uint16_t coxa_pwm, femur_pwm, tibia_pwm;
        servo_deg_to_pwm(leg, coxa_deg, femur_deg, tibia_deg, &coxa_pwm, &femur_pwm, &tibia_pwm);

        // Detect foot switch early contact for adaptive step height
        if (phase < 0.5f && phase > 0.05f) {
            // Currently in swing
            if (foot_contact & (1 << leg)) {
                // Foot hit ground early - note for next cycle
                foot_early[leg] = true;
            }
        }
        if (phase >= 0.5f && phase < 0.55f) {
            foot_early[leg] = false; // reset for next swing
        }

        // Set servos
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
    Serial.begin(115200);
    delay(100);

    // UART1 (to/from ESP32)
    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.begin(921600);

    Serial.println("Pico2 Hexapod Brain v1.0");

    // I2C for PCA9685
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

    // Init PCA9685s
    pca_init(PCA9685_ADDR_LEFT);
    pca_init(PCA9685_ADDR_RIGHT);

    // Set all servos to home (90°)
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
    if (!bmi_init()) {
        Serial.println("BMI160 init FAILED");
    } else {
        Serial.println("BMI160 OK");
    }

    // Send debug hello
    uint8_t hello[] = "PICO2_OK";
    int n = proto_build(proto_tx_buf, PKT_DEBUG, hello, 7, proto_seq++);
    Serial1.write(proto_tx_buf, n);
}

// ============================
// LOOP
// ============================
void loop() {
    uint32_t now = micros();

    // Parse incoming UART commands
    parse_uart();

    // Read foot switches
    read_foot_switches();

    // Read IMU at ~100Hz
    static uint32_t last_imu_us = 0;
    if (now - last_imu_us >= 10000) {
        last_imu_us = now;
        bmi_read_all();
    }

    // Gait tick at TICK_RATE_HZ
    if (now - tick_us_last >= TICK_US) {
        tick_us_last = now;
        tick_counter++;
        tick_gait();
    }

    // Telemetry at ~20Hz
    if (millis() - last_telem_ms >= 50) {
        last_telem_ms = millis();
        send_telemetry();
    }

    // Debug print at ~2Hz
    if (millis() - last_print_ms >= 500) {
        last_print_ms = millis();
        Serial.printf("roll=%.1f pitch=%.1f yaw=%.1f vx=%d vy=%d vr=%d gait=%d contact=0x%02X\n",
                      imu_roll, imu_pitch, imu_yaw, cmd_vx, cmd_vy, cmd_vr, gait_mode, foot_contact);
    }
}
