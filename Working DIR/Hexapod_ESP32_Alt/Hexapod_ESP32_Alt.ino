/*
 * Hexapod_ESP32_Alt
 * ESP32 + Bluepad32 + Dual PCA9685
 * Install Bluepad32 + Adafruit PWM Servo via Arduino Library Manager
 *
 * LEFT  PCA9685 (0x40): Leg1(FL) ch 0,1,2 | Leg2(ML) ch 5,6,7 | Leg3(RL) ch 13,14,15
 * RIGHT PCA9685 (0x41): Leg0(FR) ch 0,1,2 | Leg4(RR) ch 5,6,7 | Leg5(MR) ch 13,14,15
 */

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define PCA9685_LEFT_ADDR  0x40
#define PCA9685_RIGHT_ADDR 0x41
#define PCA9685_FREQ 50
#define SERVO_PULSE_MIN 125
#define SERVO_PULSE_MAX 491

struct LegChannels { uint8_t pca_idx; uint8_t coxa_ch; uint8_t femur_ch; uint8_t tibia_ch; };
const LegChannels LEG_CH[6] = {
    {1, 0,  1,  2},   // Leg 0 (FR)  -> RIGHT, ch 0/1/2
    {0, 0,  1,  2},   // Leg 1 (FL)  -> LEFT,  ch 0/1/2
    {0, 5,  6,  7},   // Leg 2 (ML)  -> LEFT,  ch 5/6/7
    {0, 13, 14, 15},  // Leg 3 (RL)  -> LEFT,  ch 13/14/15
    {1, 5,  6,  7},   // Leg 4 (RR)  -> RIGHT, ch 5/6/7
    {1, 13, 14, 15},  // Leg 5 (MR)  -> RIGHT, ch 13/14/15
};

#define I2C_SDA 21
#define I2C_SCL 22
#define BATT_ADC_PIN 36
#define COXA_LENGTH  51
#define FEMUR_LENGTH 65
#define TIBIA_LENGTH 121
#define TRAVEL 30
#define FRAME_TIME_MS 10
#define AXIS_DEADBAND 15

const float HOME_X[6] = {  82.0,   0.0, -82.0, -82.0,    0.0,  82.0};
const float HOME_Y[6] = {  82.0, 116.0,  82.0, -82.0, -116.0, -82.0};
const float HOME_Z[6] = { -80.0, -80.0, -80.0, -80.0,  -80.0, -80.0};
const float BODY_X[6] = { 110.4,   0.0, -110.4, -110.4,    0.0, 110.4};
const float BODY_Y[6] = {  58.4,  90.8,   58.4,  -58.4,  -90.8, -58.4};
const float BODY_Z[6] = {   0.0,   0.0,    0.0,    0.0,    0.0,   0.0};
const int8_t COXA_CAL[6]  = {2, -1, -1, -3, -2, -3};
const int8_t FEMUR_CAL[6] = {4, -2,  0, -1,  0,  0};
const int8_t TIBIA_CAL[6] = {0, -3, -3, -2, -3, -1};

int tripod_case[6]   = {1, 2, 1, 2, 1, 2};
int ripple_case[6]   = {2, 6, 4, 1, 3, 5};
int wave_case[6]     = {1, 2, 3, 4, 5, 6};
int tetrapod_case[6] = {1, 3, 2, 1, 2, 3};

Adafruit_PWMServoDriver pcaLeft(0x40);
Adafruit_PWMServoDriver pcaRight(0x41);
Adafruit_PWMServoDriver* pca[2] = {&pcaLeft, &pcaRight};

GamepadPtr gamepad = nullptr;
uint32_t btn_curr = 0, btn_prev = 0;

#define BTN_A       (1<<0)
#define BTN_B       (1<<1)
#define BTN_X       (1<<2)
#define BTN_Y       (1<<3)
#define BTN_L1      (1<<4)
#define BTN_R1      (1<<5)
#define BTN_L2      (1<<6)
#define BTN_R2      (1<<7)
#define BTN_SELECT  (1<<8)
#define BTN_START   (1<<9)
#define BTN_L3      (1<<10)
#define BTN_R3      (1<<11)
#define BTN_DPAD_UP    (1<<12)
#define BTN_DPAD_DOWN  (1<<13)
#define BTN_DPAD_LEFT  (1<<14)
#define BTN_DPAD_RIGHT (1<<15)

unsigned long currentTime, previousTime;
int mode = 0, gait = 0, gait_speed = 0;
int reset_position = true, capture_offsets = false;
float step_height_multiplier = 1.0;

int batt_voltage, batt_voltage_index, batt_voltage_array[50];
long batt_voltage_sum;

float L0, L3, gamma_femur, phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;
int leg1_IK_control = true, leg6_IK_control = true;
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num, totalX, totalY, totalZ;
int tick, duration = 800;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float strideX, strideY, strideR, sinRotZ, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6] = {0}, offset_Y[6] = {0}, offset_Z[6] = {0};
float current_X[6], current_Y[6], current_Z[6];

void onConnectedGamepad(GamepadPtr gp) { gamepad = gp; Serial.printf("Gamepad connected: %s\n", gp->getModelName().c_str()); }
void onDisconnectedGamepad(GamepadPtr gp) { if (gamepad == gp) { Serial.println("Gamepad disconnected"); gamepad = nullptr; } }

static inline bool btn_pressed(uint32_t m) { return (btn_curr & m) && !(btn_prev & m); }
static inline bool btn_held(uint32_t m) { return (btn_curr & m) != 0; }

void setServo(uint8_t idx, uint8_t ch, float angle) {
    int p = constrain(map((int)angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX), SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    pca[idx]->setPWM(ch, 0, p);
}
void setLeg(int leg, float ca, float fa, float ta) {
    setServo(LEG_CH[leg].pca_idx, LEG_CH[leg].coxa_ch, ca);
    setServo(LEG_CH[leg].pca_idx, LEG_CH[leg].femur_ch, fa);
    setServo(LEG_CH[leg].pca_idx, LEG_CH[leg].tibia_ch, ta);
}

void battery_monitor() {
    batt_voltage_sum -= batt_voltage_array[batt_voltage_index];
    batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_ADC_PIN), 0, 4095, 0, 1497);
    batt_voltage_sum += batt_voltage_array[batt_voltage_index];
    batt_voltage_index = (batt_voltage_index + 1) % 50;
    batt_voltage = batt_voltage_sum / 50;
}

void leg_IK(int leg, float X, float Y, float Z) {
    L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
    L3 = sqrt(sq(L0) + sq(Z));
    if (L3 >= (TIBIA_LENGTH + FEMUR_LENGTH) || L3 <= fabs(TIBIA_LENGTH - FEMUR_LENGTH)) return;
    theta_tibia = constrain(acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH)) * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg], 0.0, 180.0);
    theta_femur = constrain((atan2(Z, L0) + acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3))) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg], 0.0, 180.0);
    theta_coxa  = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg];
    switch (leg) {
        case 0: theta_coxa = constrain(theta_coxa + 45.0, 0.0, 180.0); if (leg1_IK_control) setLeg(leg, theta_coxa, theta_femur, theta_tibia); break;
        case 1: theta_coxa = constrain(theta_coxa + 90.0, 0.0, 180.0); setLeg(leg, theta_coxa, theta_femur, theta_tibia); break;
        case 2: theta_coxa = constrain(theta_coxa + 135.0, 0.0, 180.0); setLeg(leg, theta_coxa, theta_femur, theta_tibia); break;
        case 3: theta_coxa = constrain((theta_coxa < 0) ? theta_coxa + 225.0 : theta_coxa - 135.0, 0.0, 180.0); setLeg(leg, theta_coxa, theta_femur, theta_tibia); break;
        case 4: theta_coxa = constrain((theta_coxa < 0) ? theta_coxa + 270.0 : theta_coxa - 90.0, 0.0, 180.0); setLeg(leg, theta_coxa, theta_femur, theta_tibia); break;
        case 5: theta_coxa = constrain((theta_coxa < 0) ? theta_coxa + 315.0 : theta_coxa - 45.0, 0.0, 180.0); if (leg6_IK_control) setLeg(leg, theta_coxa, theta_femur, theta_tibia); break;
    }
}

void compute_strides() {
    strideX = 90.0 * commandedX / 127.0;
    strideY = 90.0 * commandedY / 127.0;
    strideR = 35.0 * commandedR / 127.0;
    sinRotZ = sin(radians(strideR));
    cosRotZ = cos(radians(strideR));
}

void compute_amplitudes() {
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
    rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;
    amplitudeX = constrain((strideX + rotOffsetX) * 0.5, -50, 50);
    amplitudeY = constrain((strideY + rotOffsetY) * 0.5, -50, 50);
    amplitudeZ = step_height_multiplier * (fabs(strideX + rotOffsetX) > fabs(strideY + rotOffsetY) ? (strideX + rotOffsetX) : (strideY + rotOffsetY)) * 0.35;
}

int calc_duration() {
    if (!gamepad) return 800;
    int cx = constrain(gamepad->axisRY() * -127 / 512, -127, 127);
    int cy = constrain(gamepad->axisRX() *  127 / 512, -127, 127);
    int cr = constrain(gamepad->axisX()  * -127 / 512, -127, 127);
    float mag = constrain(sqrtf(sq(cx) + sq(cy) + sq(cr)), AXIS_DEADBAND, 127);
    return (gait_speed == 0) ? (int)(900 - ((mag - AXIS_DEADBAND) / (127.0 - AXIS_DEADBAND)) * 650) : 1200;
}

void tripod_gait() {
    if (!gamepad) { tick = 0; return; }
    commandedX = constrain(gamepad->axisRY() * -127 / 512, -127, 127);
    commandedY = constrain(gamepad->axisRX() *  127 / 512, -127, 127);
    commandedR = constrain(gamepad->axisX()  * -127 / 512, -127, 127);
    if (sqrtf(sq(commandedX) + sq(commandedY) + sq(commandedR)) > AXIS_DEADBAND || tick > 0) {
        if (tick == 0) duration = calc_duration();
        int n = max(2, duration / FRAME_TIME_MS / 2);
        compute_strides();
        for (leg_num = 0; leg_num < 6; leg_num++) {
            compute_amplitudes();
            float p = (float)tick / n;
            if (tripod_case[leg_num] == 1) {
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * p);
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * p);
                current_Z[leg_num] = HOME_Z[leg_num] + fabs(amplitudeZ) * sinf(M_PI * p);
                if (tick >= n - 1) tripod_case[leg_num] = 2;
            } else {
                current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cosf(M_PI * p);
                current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cosf(M_PI * p);
                current_Z[leg_num] = HOME_Z[leg_num];
                if (tick >= n - 1) tripod_case[leg_num] = 1;
            }
        }
        tick = (tick < n - 1) ? tick + 1 : 0;
    }
}

void wave_gait() {
    if (!gamepad) { tick = 0; return; }
    commandedX = constrain(gamepad->axisRY() * -127 / 512, -127, 127);
    commandedY = constrain(gamepad->axisRX() *  127 / 512, -127, 127);
    commandedR = constrain(gamepad->axisX()  * -127 / 512, -127, 127);
    if (sqrtf(sq(commandedX) + sq(commandedY) + sq(commandedR)) > AXIS_DEADBAND || tick > 0) {
        if (tick == 0) duration = calc_duration();
        int n = max(6, duration / FRAME_TIME_MS / 6);
        compute_strides();
        for (leg_num = 0; leg_num < 6; leg_num++) {
            compute_amplitudes();
            float p = (float)tick / n;
            switch (wave_case[leg_num]) {
                case 1:
                    current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * p);
                    current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * p);
                    current_Z[leg_num] = HOME_Z[leg_num] + fabs(amplitudeZ) * sinf(M_PI * p);
                    if (tick >= n - 1) wave_case[leg_num] = 6; break;
                default:
                    current_X[leg_num] -= amplitudeX / n / 2.5;
                    current_Y[leg_num] -= amplitudeY / n / 2.5;
                    current_Z[leg_num]  = HOME_Z[leg_num];
                    if (tick >= n - 1) wave_case[leg_num]--; break;
            }
        }
        tick = (tick < n - 1) ? tick + 1 : 0;
    }
}

void ripple_gait() {
    if (!gamepad) { tick = 0; return; }
    commandedX = constrain(gamepad->axisRY() * -127 / 512, -127, 127);
    commandedY = constrain(gamepad->axisRX() *  127 / 512, -127, 127);
    commandedR = constrain(gamepad->axisX()  * -127 / 512, -127, 127);
    if (sqrtf(sq(commandedX) + sq(commandedY) + sq(commandedR)) > AXIS_DEADBAND || tick > 0) {
        if (tick == 0) duration = calc_duration();
        int n = max(6, duration / FRAME_TIME_MS / 6);
        compute_strides();
        for (leg_num = 0; leg_num < 6; leg_num++) {
            compute_amplitudes();
            float phase;
            switch (ripple_case[leg_num]) {
                case 1:
                    phase = M_PI * tick / (n * 2);
                    current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(phase);
                    current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(phase);
                    current_Z[leg_num] = HOME_Z[leg_num] + fabs(amplitudeZ) * sinf(phase);
                    if (tick >= n - 1) ripple_case[leg_num] = 2; break;
                case 2:
                    phase = M_PI * (n + tick) / (n * 2);
                    current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(phase);
                    current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(phase);
                    current_Z[leg_num] = HOME_Z[leg_num] + fabs(amplitudeZ) * sinf(phase);
                    if (tick >= n - 1) ripple_case[leg_num] = 3; break;
                default:
                    current_X[leg_num] -= amplitudeX / n / 2.0;
                    current_Y[leg_num] -= amplitudeY / n / 2.0;
                    current_Z[leg_num]  = HOME_Z[leg_num];
                    if (tick >= n - 1) ripple_case[leg_num] = (ripple_case[leg_num] < 6) ? ripple_case[leg_num] + 1 : 1; break;
            }
        }
        tick = (tick < n - 1) ? tick + 1 : 0;
    }
}

void tetrapod_gait() {
    if (!gamepad) { tick = 0; return; }
    commandedX = constrain(gamepad->axisRY() * -127 / 512, -127, 127);
    commandedY = constrain(gamepad->axisRX() *  127 / 512, -127, 127);
    commandedR = constrain(gamepad->axisX()  * -127 / 512, -127, 127);
    if (sqrtf(sq(commandedX) + sq(commandedY) + sq(commandedR)) > AXIS_DEADBAND || tick > 0) {
        if (tick == 0) duration = calc_duration();
        int n = max(3, duration / FRAME_TIME_MS / 3);
        compute_strides();
        for (leg_num = 0; leg_num < 6; leg_num++) {
            compute_amplitudes();
            float p = (float)tick / n;
            switch (tetrapod_case[leg_num]) {
                case 1:
                    current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cosf(M_PI * p);
                    current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cosf(M_PI * p);
                    current_Z[leg_num] = HOME_Z[leg_num] + fabs(amplitudeZ) * sinf(M_PI * p);
                    if (tick >= n - 1) tetrapod_case[leg_num] = 2; break;
                default:
                    current_X[leg_num] -= amplitudeX / n;
                    current_Y[leg_num] -= amplitudeY / n;
                    current_Z[leg_num]  = HOME_Z[leg_num];
                    if (tick >= n - 1) tetrapod_case[leg_num] = (tetrapod_case[leg_num] < 3) ? tetrapod_case[leg_num] + 1 : 1; break;
            }
        }
        tick = (tick < n - 1) ? tick + 1 : 0;
    }
}

void translate_control() {
    if (!gamepad) return;
    translateX = gamepad->axisRY() * -2 * TRAVEL / 512;
    translateY = gamepad->axisRX() *  2 * TRAVEL / 512;
    int ly = gamepad->axisY();
    translateZ = (ly < 0) ? map(ly, -512, 0, TRAVEL, 0) : map(ly, 0, 512, 0, -3 * TRAVEL);
    for (leg_num = 0; leg_num < 6; leg_num++) {
        current_X[leg_num] = HOME_X[leg_num] + translateX;
        current_Y[leg_num] = HOME_Y[leg_num] + translateY;
        current_Z[leg_num] = HOME_Z[leg_num] + translateZ;
    }
    if (capture_offsets) {
        for (leg_num = 0; leg_num < 6; leg_num++) {
            offset_X[leg_num] += translateX; offset_Y[leg_num] += translateY; offset_Z[leg_num] += translateZ;
            current_X[leg_num] = HOME_X[leg_num]; current_Y[leg_num] = HOME_Y[leg_num]; current_Z[leg_num] = HOME_Z[leg_num];
        }
        capture_offsets = false; mode = 0;
    }
}

void rotate_control() {
    if (!gamepad) return;
    float rx = radians(gamepad->axisRX() * -12.0 / 512), ry = radians(gamepad->axisRY() * -12.0 / 512), lx = radians(gamepad->axisX() * 30.0 / 512);
    float sRx = sinf(rx), cRx = cosf(rx), sRy = sinf(ry), cRy = cosf(ry), sRz = sinf(lx), cRz = cosf(lx);
    int ly = gamepad->axisY();
    translateZ = (ly < 0) ? map(ly, -512, 0, TRAVEL, 0) : map(ly, 0, 512, 0, -3 * TRAVEL);
    for (leg_num = 0; leg_num < 6; leg_num++) {
        totalX = HOME_X[leg_num] + BODY_X[leg_num]; totalY = HOME_Y[leg_num] + BODY_Y[leg_num]; totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];
        rotOffsetX = totalX*cRy*cRz + totalY*(sRx*sRy*cRz + cRx*sRz) - totalZ*(cRx*sRy*cRz - sRx*sRz) - totalX;
        rotOffsetY = -totalX*cRy*sRz - totalY*(sRx*sRy*sRz - cRx*cRz) + totalZ*(cRx*sRy*sRz + sRx*cRz) - totalY;
        rotOffsetZ = totalX*sRy - totalY*sRx*cRy + totalZ*cRx*cRy - totalZ;
        current_X[leg_num] = HOME_X[leg_num] + rotOffsetX; current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY; current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;
        if (capture_offsets) {
            offset_X[leg_num] += rotOffsetX; offset_Y[leg_num] += rotOffsetY; offset_Z[leg_num] += rotOffsetZ + translateZ;
            current_X[leg_num] = HOME_X[leg_num]; current_Y[leg_num] = HOME_Y[leg_num]; current_Z[leg_num] = HOME_Z[leg_num];
        }
    }
    if (capture_offsets) { capture_offsets = false; mode = 0; }
}

void one_leg_lift() {
    if (!gamepad) return;
    if (leg1_IK_control) { leg1_coxa = 90 + COXA_CAL[0]; leg1_femur = 90 + FEMUR_CAL[0]; leg1_tibia = 90 + TIBIA_CAL[0]; leg1_IK_control = false; }
    if (leg6_IK_control) { leg6_coxa = 90 + COXA_CAL[5]; leg6_femur = 90 + FEMUR_CAL[5]; leg6_tibia = 90 + TIBIA_CAL[5]; leg6_IK_control = false; }
    int rx = gamepad->axisRX(), ry = gamepad->axisRY(), lx = gamepad->axisX(), ly = gamepad->axisY();
    int ca = map(constrain(rx, -512, 512), -512, 512, 45, -45);
    setLeg(0, constrain(leg1_coxa + ca, 45, 135), leg1_femur, leg1_tibia);
    if (ry < -64) { int lift = map(ry, -512, -64, 24, 0); leg1_femur = constrain(leg1_femur + lift, 0, 170); leg1_tibia = constrain(leg1_tibia + 4 * lift, 0, 170); setLeg(0, constrain(leg1_coxa + ca, 45, 135), leg1_femur, leg1_tibia); }
    else if (ry > 64) { step_height_multiplier = 1.0 + (map(constrain(ry, 64, 512), 64, 512, 1, 8) - 1.0) / 3.0; }
    ca = map(constrain(lx, -512, 512), -512, 512, 45, -45);
    setLeg(5, constrain(leg6_coxa + ca, 45, 135), leg6_femur, leg6_tibia);
    if (ly < -64) { int lift = map(ly, -512, -64, 24, 0); leg6_femur = constrain(leg6_femur + lift, 0, 170); leg6_tibia = constrain(leg6_tibia + 4 * lift, 0, 170); setLeg(5, constrain(leg6_coxa + ca, 45, 135), leg6_femur, leg6_tibia); }
    else if (ly > 64) { step_height_multiplier = 1.0 + (map(constrain(ly, 64, 512), 64, 512, 1, 8) - 1.0) / 3.0; }
}

void set_all_90() { for (int i = 0; i < 6; i++) setLeg(i, 90 + COXA_CAL[i], 90 + FEMUR_CAL[i], 90 + TIBIA_CAL[i]); }

void process_gamepad() {
    if (btn_pressed(BTN_DPAD_DOWN))  { mode = 0; gait = 0; reset_position = true; }
    if (btn_pressed(BTN_DPAD_LEFT))  { mode = 0; gait = 1; reset_position = true; }
    if (btn_pressed(BTN_DPAD_UP))    { mode = 0; gait = 2; reset_position = true; }
    if (btn_pressed(BTN_DPAD_RIGHT)) { mode = 0; gait = 3; reset_position = true; }
    if (btn_pressed(BTN_Y))  { mode = 1; reset_position = true; }
    if (btn_pressed(BTN_X))  { mode = 2; reset_position = true; }
    if (btn_pressed(BTN_B))  { mode = 3; reset_position = true; }
    if (btn_pressed(BTN_A))  { mode = 4; reset_position = true; }
    if (btn_pressed(BTN_START)) gait_speed = !gait_speed;
    if (btn_pressed(BTN_SELECT)) mode = 99;
    if (btn_pressed(BTN_L1) || btn_pressed(BTN_R1)) capture_offsets = true;
    if (btn_pressed(BTN_L2) || btn_pressed(BTN_R2)) {
        for (leg_num = 0; leg_num < 6; leg_num++) { offset_X[leg_num] = 0; offset_Y[leg_num] = 0; offset_Z[leg_num] = 0; }
        leg1_IK_control = true; leg6_IK_control = true; step_height_multiplier = 1.0;
    }
}

void print_debug() { currentTime = millis(); Serial.print(currentTime - previousTime); Serial.print(","); Serial.println(batt_voltage / 100.0); }

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    for (int i = 0; i < 2; i++) { pca[i]->begin(); pca[i]->setPWMFreq(PCA9685_FREQ); }
    analogReadResolution(12); analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);
    pinMode(BATT_ADC_PIN, INPUT);
    for (int i = 0; i < 50; i++) batt_voltage_array[i] = 0;
    batt_voltage_sum = batt_voltage_index = 0;
    for (leg_num = 0; leg_num < 6; leg_num++) offset_X[leg_num] = offset_Y[leg_num] = offset_Z[leg_num] = 0;
    step_height_multiplier = 1.0;
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad); BP32.enableVirtualDevice(false);
    for (leg_num = 0; leg_num < 6; leg_num++) { current_X[leg_num] = HOME_X[leg_num]; current_Y[leg_num] = HOME_Y[leg_num]; current_Z[leg_num] = HOME_Z[leg_num]; }
    reset_position = false;
    previousTime = millis();
    Serial.println("Hexapod ready. Connect a Bluetooth gamepad.");
}

void loop() {
    BP32.update();
    currentTime = millis();
    if (currentTime - previousTime < FRAME_TIME_MS) return;
    previousTime = currentTime;
    btn_prev = btn_curr; btn_curr = 0;
    if (gamepad && gamepad->isConnected()) {
        if (gamepad->a())      btn_curr |= BTN_A;
        if (gamepad->b())      btn_curr |= BTN_B;
        if (gamepad->x())      btn_curr |= BTN_X;
        if (gamepad->y())      btn_curr |= BTN_Y;
        if (gamepad->l1())     btn_curr |= BTN_L1;
        if (gamepad->r1())     btn_curr |= BTN_R1;
        if (gamepad->l2())     btn_curr |= BTN_L2;
        if (gamepad->r2())     btn_curr |= BTN_R2;
        if (gamepad->select()) btn_curr |= BTN_SELECT;
        if (gamepad->start())  btn_curr |= BTN_START;
        if (gamepad->l3())     btn_curr |= BTN_L3;
        if (gamepad->r3())     btn_curr |= BTN_R3;
        uint8_t d = gamepad->dpad();
        if (d & 0x01) btn_curr |= BTN_DPAD_UP;
        if (d & 0x02) btn_curr |= BTN_DPAD_DOWN;
        if (d & 0x04) btn_curr |= BTN_DPAD_LEFT;
        if (d & 0x08) btn_curr |= BTN_DPAD_RIGHT;
        process_gamepad();
    }
    if (reset_position) {
        for (leg_num = 0; leg_num < 6; leg_num++) { current_X[leg_num] = HOME_X[leg_num]; current_Y[leg_num] = HOME_Y[leg_num]; current_Z[leg_num] = HOME_Z[leg_num]; }
        reset_position = false;
    }
    if (mode < 99) { for (leg_num = 0; leg_num < 6; leg_num++) leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]); }
    if (mode != 4) { leg1_IK_control = true; leg6_IK_control = true; }
    battery_monitor(); print_debug();
    if (mode == 1) {
        if (gait == 0) tripod_gait(); else if (gait == 1) wave_gait(); else if (gait == 2) ripple_gait(); else if (gait == 3) tetrapod_gait();
    } else if (mode == 2) { translate_control(); } else if (mode == 3) { rotate_control(); } else if (mode == 4) { one_leg_lift(); } else if (mode == 99) { set_all_90(); }
}
