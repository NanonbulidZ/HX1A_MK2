/*
 * 8_ESP32_All_In_One
 * Single ESP32 DevKit: Bluepad32 gamepad → IK/gait → PCA9685 servos
 * No Pico2, no UART relay, no ESP-NOW. One board does everything.
 * Arduino IDE - install: Bluepad32, Adafruit PWM Servo Driver Library
 * Board: ESP32 Dev Module (or DOIT ESP32 DEVKIT V1)
 */

#include <Bluepad32.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// ============================
// PIN DEFINITIONS
// ============================
#define LED_PIN          2     // ESP32 built-in LED (active HIGH)
#define I2C_SDA          21
#define I2C_SCL          22

// BMI160 SPI (VSPI on ESP32)
#define BMI_CS           5
#define BMI_SCK          18
#define BMI_MOSI         23
#define BMI_MISO         19

// Foot switches (safe GPIOs on ESP32)
#define FOOT0            26
#define FOOT1            27
#define FOOT2            14
#define FOOT3            32
#define FOOT4            33
#define FOOT5            25

// PCA9685
#define PCA_ADDR_LEFT    0x40
#define PCA_ADDR_RIGHT   0x41

// ============================
// SERVO CHANNEL MAPPING
// ============================
// LEFT PCA (0x40): Leg1(FL)=ch0/1/2, Leg2(ML)=ch5/6/7, Leg3(RL)=ch13/14/15
// RIGHT PCA (0x41): Leg0(FR)=ch0/1/2, Leg5(MR)=ch13/14/15, Leg4(RR)=ch5/6/7
#define CH_L0_COXA  0
#define CH_L0_FEMUR 1
#define CH_L0_TIBIA 2
#define CH_L1_COXA  0
#define CH_L1_FEMUR 1
#define CH_L1_TIBIA 2
#define CH_L2_COXA  5
#define CH_L2_FEMUR 6
#define CH_L2_TIBIA 7
#define CH_L3_COXA  13
#define CH_L3_FEMUR 14
#define CH_L3_TIBIA 15
#define CH_L4_COXA  5
#define CH_L4_FEMUR 6
#define CH_L4_TIBIA 7
#define CH_L5_COXA  13
#define CH_L5_FEMUR 14
#define CH_L5_TIBIA 15

// ============================
// CONSTANTS
// ============================
static const float mount_deg[6] = {45.0f, 90.0f, 135.0f, 225.0f, 270.0f, 315.0f};

static const float home_deg[18] = {
    90.0f, 90.0f, 90.0f,  90.0f, 90.0f, 90.0f,
    90.0f, 90.0f, 90.0f,  90.0f, 90.0f, 90.0f,
    90.0f, 90.0f, 90.0f,  90.0f, 90.0f, 90.0f
};

static float coxa_cal[6]  = {0,0,0,0,0,0};
static float femur_cal[6] = {0,0,0,0,0,0};
static float tibia_cal[6] = {0,0,0,0,0,0};

#define COXA_LEN  30.0f
#define FEMUR_LEN 60.0f
#define TIBIA_LEN 80.0f

static const float leg_x[6] = { 60.0f,  60.0f,   0.0f, -60.0f, -60.0f,   0.0f};
static const float leg_y[6] = {-50.0f,  50.0f,  60.0f,  50.0f, -50.0f, -60.0f};

#define STANCE_W 70.0f
#define STANCE_L 60.0f
#define BODY_H   75.0f

#define SERVOMIN 150
#define SERVOMAX 600
#define SERVOCENTER ((SERVOMIN + SERVOMAX) / 2)
#define PWM_FREQ 50.0f

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
static int8_t leg_ox[6] = {0}, leg_oy[6] = {0}, leg_oz[6] = {0};

static float n_x = 0, n_y = 0, n_r = 0;
static float n_body_vz = 0, n_body_rx = 0, n_body_ry = 0;

static uint8_t foot_contact = 0;
static uint32_t foot_swing_start[6] = {0};
static bool foot_early[6] = {false};
static bool foot_switches_present = false;

static bool imu_ok = false;
static float imu_roll = 0, imu_pitch = 0, imu_yaw = 0;
static int16_t imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz;

GamepadPtr gp = nullptr;

static uint32_t last_cmd_ms = 0;

// ============================
// PCA9685
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
// BMI160 SPI
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
    bmi_spi_rw(reg & 0x7F);
    bmi_spi_rw(val);
    bmi_cs(HIGH);
}

static uint8_t bmi_read(uint8_t reg) {
    bmi_cs(LOW);
    bmi_spi_rw(reg | 0x80);
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
    SPI.begin(BMI_SCK, BMI_MISO, BMI_MOSI, BMI_CS);

    bmi_write(0x7E, 0xB6);
    delay(50);

    uint8_t id = bmi_read(0x00);
    if (id != 0xD1 && id != 0xC1) {
        Serial.printf("BMI160 ID fail: 0x%02X\n", id);
        return false;
    }

    bmi_write(0x7E, 0x11);
    delay(10);
    bmi_write(0x7E, 0x15);
    delay(10);

    bmi_write(0x41, 0x03);
    bmi_write(0x40, 0x0A);
    bmi_write(0x43, 0x00);
    bmi_write(0x42, 0x0A);

    return true;
}

static void bmi_read_all() {
    uint8_t raw[12];
    bmi_read_burst(0x0C, raw, 12);

    int16_t ax = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t ay = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t az = (int16_t)(raw[5] << 8 | raw[4]);
    int16_t gx = (int16_t)(raw[7] << 8 | raw[6]);
    int16_t gy = (int16_t)(raw[9] << 8 | raw[8]);
    int16_t gz = (int16_t)(raw[11] << 8 | raw[10]);

    imu_ax = ax; imu_ay = ay; imu_az = az;
    imu_gx = gx; imu_gy = gy; imu_gz = gz;

    float a_scale = 1000.0f / 8192.0f;
    float ax_g = (float)ax * a_scale * 0.001f;
    float ay_g = (float)ay * a_scale * 0.001f;
    float az_g = (float)az * a_scale * 0.001f;

    float dt = 0.01f;
    float alpha = 0.96f;
    float g_scale = (M_PI / 180.0f) / 16.384f;
    float gx_rs = (float)gx * g_scale;
    float gy_rs = (float)gy * g_scale;
    float gz_rs = (float)gz * g_scale;

    float acc_roll  = atan2f(-ay_g, az_g) * 180.0f / (float)M_PI;
    float acc_pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / (float)M_PI;

    static float roll_f = 0, pitch_f = 0, yaw_f = 0;
    roll_f  = alpha * (roll_f  + gx_rs * dt * 180.0f / (float)M_PI) + (1.0f - alpha) * acc_roll;
    pitch_f = alpha * (pitch_f + gy_rs * dt * 180.0f / (float)M_PI) + (1.0f - alpha) * acc_pitch;
    yaw_f  += gz_rs * dt * 180.0f / (float)M_PI;

    imu_roll = roll_f; imu_pitch = pitch_f; imu_yaw = yaw_f;
}

// ============================
// INVERSE KINEMATICS
// ============================
static void ik_leg(int leg, float fx, float fy, float fz, float* coxa, float* femur, float* tibia) {
    float mount_rad = mount_deg[leg] * (float)M_PI / 180.0f;
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

    float cos_fem = (FEMUR_LEN * FEMUR_LEN + d * d - TIBIA_LEN * TIBIA_LEN) / (2.0f * FEMUR_LEN * d);
    if (cos_fem < -1.0f) cos_fem = -1.0f;
    if (cos_fem > 1.0f)  cos_fem = 1.0f;
    float femur_rad = acosf(cos_fem) + atan2f(fz_eff, hip_dist);

    float cos_tib = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - d * d) / (2.0f * FEMUR_LEN * TIBIA_LEN);
    if (cos_tib < -1.0f) cos_tib = -1.0f;
    if (cos_tib > 1.0f)  cos_tib = 1.0f;
    float tibia_rad = acosf(cos_tib) - (float)M_PI / 2.0f;

    *coxa  = coxa_rad  * 180.0f / (float)M_PI;
    *femur = femur_rad * 180.0f / (float)M_PI;
    *tibia = tibia_rad * 180.0f / (float)M_PI;
}

// ============================
// BODY TRANSFORM
// ============================
static void body_transform(int leg, float vz, float rx_deg, float ry_deg, float* fx, float* fy, float* fz) {
    float rx_rad = rx_deg * (float)M_PI / 180.0f;
    float ry_rad = ry_deg * (float)M_PI / 180.0f;
    float lx = leg_x[leg], ly = leg_y[leg], lz = vz;
    float y1 = ly * cosf(rx_rad) - lz * sinf(rx_rad);
    float z1 = ly * sinf(rx_rad) + lz * cosf(rx_rad);
    float x2 = lx * cosf(ry_rad) + z1 * sinf(ry_rad);
    float z2 = -lx * sinf(ry_rad) + z1 * cosf(ry_rad);
    *fx = -x2; *fy = -y1; *fz = -z2 + BODY_H;
}

// ============================
// GAIT
// ============================
static const float tripod_phase[6]   = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
static const float wave_phase[6]     = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 0.0f};
static const float ripple_phase[6]   = {0.0f, 0.333f, 0.667f, 0.167f, 0.5f, 0.833f};
static const float tetrapod_phase[6] = {0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.5f};

static int get_cycle_len() {
    switch (gait_mode) { case 1: return WAVE_CYCLE; case 2: return RIPPLE_CYCLE; case 3: return TETRAPOD_CYCLE; default: return TRIPOD_CYCLE; }
}

static const float* get_phase_offsets() {
    switch (gait_mode) { case 1: return wave_phase; case 2: return ripple_phase; case 3: return tetrapod_phase; default: return tripod_phase; }
}

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
// SERVO DEG TO PWM
// ============================
static void servo_deg_to_pwm(int leg, float coxa_deg, float femur_deg, float tibia_deg,
                             uint16_t* cp, uint16_t* fp, uint16_t* tp) {
    float c = coxa_deg + coxa_cal[leg], f = femur_deg + femur_cal[leg], t = tibia_deg + tibia_cal[leg];
    if (c < 0) c = 0; if (c > 180) c = 180;
    if (f < 0) f = 0; if (f > 180) f = 180;
    if (t < 0) t = 0; if (t > 180) t = 180;
    *cp = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * c / 180.0f);
    *fp = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * f / 180.0f);
    *tp = (uint16_t)(SERVOMIN + (float)(SERVOMAX - SERVOMIN) * t / 180.0f);
}

static void get_servo_channels(int leg, uint8_t* addr, uint8_t* ch_c, uint8_t* ch_f, uint8_t* ch_t) {
    switch (leg) {
        case 0: *addr=PCA_ADDR_RIGHT; *ch_c=CH_L0_COXA;  *ch_f=CH_L0_FEMUR;  *ch_t=CH_L0_TIBIA; break;
        case 1: *addr=PCA_ADDR_LEFT;  *ch_c=CH_L1_COXA;  *ch_f=CH_L1_FEMUR;  *ch_t=CH_L1_TIBIA; break;
        case 2: *addr=PCA_ADDR_LEFT;  *ch_c=CH_L2_COXA;  *ch_f=CH_L2_FEMUR;  *ch_t=CH_L2_TIBIA; break;
        case 3: *addr=PCA_ADDR_LEFT;  *ch_c=CH_L3_COXA;  *ch_f=CH_L3_FEMUR;  *ch_t=CH_L3_TIBIA; break;
        case 4: *addr=PCA_ADDR_RIGHT; *ch_c=CH_L4_COXA;  *ch_f=CH_L4_FEMUR;  *ch_t=CH_L4_TIBIA; break;
        case 5: *addr=PCA_ADDR_RIGHT; *ch_c=CH_L5_COXA;  *ch_f=CH_L5_FEMUR;  *ch_t=CH_L5_TIBIA; break;
    }
}

// ============================
// GAIT TICK
// ============================
static bool walk_enabled = false;
static bool balance_enabled = false;
static int prev_cycle_len = TRIPOD_CYCLE;

void tick_gait() {
    float spd_scale = (float)cmd_speed / 100.0f;
    float stride_vx = (float)cmd_vx * spd_scale / 1000.0f * STANCE_L;
    float stride_vy = (float)cmd_vy * spd_scale / 1000.0f * STANCE_W;
    float stride_vr = (float)cmd_vr * spd_scale / 1000.0f * 30.0f;

    int cycle = get_cycle_len();
    float body_vz = (float)cmd_body_vz / 10.0f;
    float body_rx = (float)cmd_body_rx / 10.0f;
    float body_ry = (float)cmd_body_ry / 10.0f;

    if (tick_counter == 0 || cycle != prev_cycle_len) {
        n_x = stride_vx; n_y = stride_vy; n_r = stride_vr;
        n_body_vz = body_vz; n_body_rx = body_rx; n_body_ry = body_ry;
        prev_cycle_len = cycle;
    }

    walk_enabled = (cmd_flags & 0x01) || (cmd_flags & 0x20);
    balance_enabled = foot_switches_present ? (cmd_flags & 0x10) : true;

    const float* phases = get_phase_offsets();
    float balance_rx = 0, balance_ry = 0;
    if (balance_enabled && imu_ok) { balance_rx = -imu_roll; balance_ry = -imu_pitch; }

    for (int leg = 0; leg < 6; leg++) {
        float phase = fmodf((float)tick_counter / (float)cycle + phases[leg], 1.0f);
        float fx, fy, fz;

        if (walk_enabled && (abs(cmd_vx) > 10 || abs(cmd_vy) > 10 || abs(cmd_vr) > 10)) {
            float t = fmodf(phase, 1.0f);
            if (t < 0.5f) {
                float u = t * 2.0f;
                fx = n_x * (u - 0.5f); fy = n_y * (u - 0.5f); fz = 30.0f * sinf(u * (float)M_PI);
            } else {
                float u = (t - 0.5f) * 2.0f;
                fx = n_x * (u - 0.5f); fy = n_y * (u - 0.5f); fz = 0;
            }
            float rot_rad = n_r * (float)M_PI / 180.0f;
            float rx = leg_x[leg], ry = leg_y[leg];
            fx += rx * cosf(rot_rad) - ry * sinf(rot_rad) - rx;
            fy += rx * sinf(rot_rad) + ry * cosf(rot_rad) - ry;
        } else { fx = 0; fy = 0; fz = 0; }

        fx += (float)leg_ox[leg]; fy += (float)leg_oy[leg]; fz += (float)leg_oz[leg];

        float bfx, bfy, bfz;
        body_transform(leg, n_body_vz, n_body_rx + balance_rx, n_body_ry + balance_ry, &bfx, &bfy, &bfz);

        float ffx = fx + bfx, ffy = fy + bfy, ffz = fz + bfz;
        float coxa_deg, femur_deg, tibia_deg;
        ik_leg(leg, ffx, ffy, ffz, &coxa_deg, &femur_deg, &tibia_deg);

        int hi = leg * 3;
        coxa_deg += home_deg[hi]; femur_deg += home_deg[hi+1]; tibia_deg += home_deg[hi+2];

        if (phase < 0.5f && phase > 0.05f && (foot_contact & (1 << leg))) foot_early[leg] = true;
        if (phase >= 0.5f && phase < 0.55f) foot_early[leg] = false;

        uint16_t cp, fp, tp;
        servo_deg_to_pwm(leg, coxa_deg, femur_deg, tibia_deg, &cp, &fp, &tp);

        uint8_t addr, ch_c, ch_f, ch_t;
        get_servo_channels(leg, &addr, &ch_c, &ch_f, &ch_t);
        set_servo_pwm(addr, ch_c, cp);
        set_servo_pwm(addr, ch_f, fp);
        set_servo_pwm(addr, ch_t, tp);
    }
}

// ============================
// LED INDICATOR
// ============================
static void update_led() {
    static uint32_t last = 0;
    static bool on = false;
    uint32_t now = millis();
    uint32_t interval;
    bool has_gp = (gp && gp->isConnected());
    if (!has_gp) interval = 2000;
    else if (walk_enabled && (abs(cmd_vx) > 10 || abs(cmd_vy) > 10 || abs(cmd_vr) > 10)) interval = 150;
    else interval = 1000;
    if (now - last >= interval) { last = now; on = !on; digitalWrite(LED_PIN, on); }
}

// ============================
// BLUEPAD32 CALLBACKS
// ============================
void onConnectedGamepad(GamepadPtr g) { gp = g; Serial.println("GP connected"); }
void onDisconnectedGamepad(GamepadPtr g) { if (gp == g) gp = nullptr; Serial.println("GP disconnected"); }

// ============================
// SETUP
// ============================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    for (int i = 0; i < 3; i++) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }

    Serial.begin(115200);
    delay(100);
    Serial.println("ESP32 Hexapod All-In-One v1.0");

    // I2C for PCA9685
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    pca_init(PCA_ADDR_LEFT);
    pca_init(PCA_ADDR_RIGHT);

    for (int leg = 0; leg < 6; leg++) {
        uint8_t addr, ch_c, ch_f, ch_t;
        get_servo_channels(leg, &addr, &ch_c, &ch_f, &ch_t);
        set_servo_pwm(addr, ch_c, SERVOCENTER);
        set_servo_pwm(addr, ch_f, SERVOCENTER);
        set_servo_pwm(addr, ch_t, SERVOCENTER);
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
    Serial.println(imu_ok ? "BMI160 OK" : "BMI160 init FAILED");

    // Bluepad32
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.enableVirtualDevice(false);

    Serial.println("Ready! Connect gamepad.");
    for (int i = 0; i < 2; i++) { digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200); }
}

// ============================
// LOOP
// ============================
static uint32_t last_telem_ms = 0;
static uint32_t last_print_ms = 0;
static uint32_t last_imu_us = 0;

void loop() {
    uint32_t now = micros();
    uint32_t ms = millis();

    BP32.update();

    // Read foot switches
    read_foot_switches();

    // Read IMU
    if (imu_ok && now - last_imu_us >= 10000) { last_imu_us = now; bmi_read_all(); }

    // Gamepad → commands
    if (gp && gp->isConnected()) {
        cmd_vx = constrain(gp->axisRY() * -1000 / 512, -1000, 1000);
        cmd_vy = constrain(gp->axisRX() *  1000 / 512, -1000, 1000);
        cmd_vr = constrain(gp->axisX()  * -1000 / 512, -1000, 1000);
        cmd_body_vz = constrain(gp->axisY() * 500 / 512, -500, 500);
        cmd_body_rx = constrain((gp->brake() - 512) * 100 / 512, -100, 100);
        cmd_body_ry = constrain((gp->throttle() - 512) * 100 / 512, -100, 100);

        cmd_flags = 0;
        if (gp->a()) cmd_flags |= 0x01;
        if (gp->b()) cmd_flags |= 0x08;
        if (gp->x()) cmd_flags |= 0x10;
        if (gp->y()) cmd_flags |= 0x20;

        if (gp->dpad() & 0x02) gait_mode = 1;
        if (gp->dpad() & 0x04) gait_mode = 2;
        if (gp->dpad() & 0x01) gait_mode = 3;
        if (gp->dpad() & 0x08) gait_mode = 4;
        cmd_flags |= (gait_mode << 1);

        cmd_speed = constrain(abs(gp->brake() - 512) * 100 / 512, 0, 100);
        last_cmd_ms = ms;
    }

    // Gait tick
    if (now - tick_us_last >= TICK_US) { tick_us_last = now; tick_counter++; tick_gait(); }

    // LED
    update_led();

    // Debug at ~1Hz
    if (ms - last_print_ms >= 1000) {
        last_print_ms = ms;
        Serial.printf("vx=%d vy=%d vr=%d gait=%d flags=0x%02X spd=%d roll=%.1f pitch=%.1f\n",
                      cmd_vx, cmd_vy, cmd_vr, gait_mode, cmd_flags, cmd_speed, imu_roll, imu_pitch);
    }

    delay(10);
}
