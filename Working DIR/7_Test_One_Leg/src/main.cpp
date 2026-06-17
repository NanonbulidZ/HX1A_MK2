#include <Arduino.h>
#include <Wire.h>
#include "protocol.h"

#define LED_PIN 25
#define PCA_ADDR 0x40
#define I2C_SDA 4
#define I2C_SCL 5
#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVOCENTER ((SERVOMIN + SERVOMAX) / 2)
#define PWM_FREQ 50.0f

static void pca_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void pca_set_pwm(uint8_t ch, uint16_t off) {
    if (off > 4095) off = 4095;
    uint8_t reg = 0x06 + ch * 4;
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    Wire.write(0);
    Wire.write(0);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
}

static void pca_init() {
    pca_write(0x00, 0x20);
    pca_write(0xFE, (uint8_t)(roundf(25000000.0f / (4096.0f * PWM_FREQ)) - 1));
    pca_write(0x00, 0x20 | 0x80);
    pca_write(0x00, 0xA0);
    delay(5);
}

static void set_servo(uint8_t ch, uint16_t pwm) {
    if (pwm < SERVOMIN) pwm = SERVOMIN;
    if (pwm > SERVOMAX) pwm = SERVOMAX;
    pca_set_pwm(ch, pwm);
}

// Protocol state
static uint8_t rx_buf[128];
static int rx_len = 0;
static uint8_t tx_buf[sizeof(PktTelemetry) + 8];
static uint8_t seq = 0;

static int16_t vx = 0, vy = 0, body_vz = 0;
static uint8_t flags = 0, speed = 100;
static uint32_t last_cmd_ms = 0;

static void process_packet(uint8_t type, uint8_t* payload, uint8_t len) {
    if (type == PKT_VELOCITY && len >= sizeof(PktVelocity)) {
        PktVelocity* p = (PktVelocity*)payload;
        vx = p->vx; vy = p->vy; body_vz = p->body_vz;
        flags = p->flags; speed = p->speed;
        last_cmd_ms = millis();
    }
}

static void parse_uart() {
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (rx_len < (int)sizeof(rx_buf)) rx_buf[rx_len++] = b;
        else rx_len = 0;
        uint8_t type, payload[64], pseq;
        int plen = proto_parse(rx_buf, rx_len, &type, payload, &pseq);
        if (plen > 0) {
            process_packet(type, payload, (uint8_t)plen);
            rx_len = 0;
        } else if (plen < 0) rx_len = 0;
    }
}

static void send_telemetry() {
    PktTelemetry t;
    memset(&t, 0, sizeof(t));
    int n = proto_build(tx_buf, PKT_TELEM, (uint8_t*)&t, sizeof(t), seq++);
    Serial1.write(tx_buf, n);
}

// LED states: 0=waiting for ESP, 1=idle (linked), 2=servo moving
static uint8_t led_state = 0;

static void update_led() {
    static uint32_t last = 0;
    static bool on = false;
    uint32_t now = millis();
    uint32_t interval;
    if (led_state == 0) interval = 3000;       // slow blink = waiting for ESP
    else if (led_state == 2) interval = 150;   // fast blink = moving
    else interval = 1500;                      // normal blink = linked OK
    if (now - last >= interval) { last = now; on = !on; digitalWrite(LED_PIN, on); }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    for (int i = 0; i < 3; i++) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }

    Serial.begin(115200);

    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.begin(921600);

    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

    pca_init();

    for (int ch = 0; ch < 16; ch++) set_servo(ch, SERVOCENTER);

    uint8_t hello[] = "LEG_TEST_OK";
    int n = proto_build(tx_buf, PKT_DEBUG, hello, 11, seq++);
    Serial1.write(tx_buf, n);
}

void loop() {
    parse_uart();

    // Map joystick axes to Leg1 (FL) servo angles: ch0=coxa, ch1=femur, ch2=tibia
    // vx → coxa (±30°), vy → femur (±30°), body_vz → tibia (±30°)
    bool enabled = flags & 0x01;
    float spd = speed / 100.0f;

    if (enabled && (abs(vx) > 10 || abs(vy) > 10 || abs(body_vz) > 10)) {
        float c = 90.0f + (float)vx * spd * 30.0f / 1000.0f;
        float f = 90.0f + (float)vy * spd * 30.0f / 1000.0f;
        float t = 90.0f + (float)body_vz * spd * 30.0f / 1000.0f;
        if (c < 0) c = 0; if (c > 180) c = 180;
        if (f < 0) f = 0; if (f > 180) f = 180;
        if (t < 0) t = 0; if (t > 180) t = 180;
        uint16_t cp = SERVOMIN + (uint16_t)((float)(SERVOMAX - SERVOMIN) * c / 180.0f);
        uint16_t fp = SERVOMIN + (uint16_t)((float)(SERVOMAX - SERVOMIN) * f / 180.0f);
        uint16_t tp = SERVOMIN + (uint16_t)((float)(SERVOMAX - SERVOMIN) * t / 180.0f);
        set_servo(0, cp);
        set_servo(1, fp);
        set_servo(2, tp);
    } else {
        set_servo(0, SERVOCENTER);
        set_servo(1, SERVOCENTER);
        set_servo(2, SERVOCENTER);
    }

    // Telemetry at ~20Hz
    static uint32_t last_telem = 0;
    if (millis() - last_telem >= 50) { last_telem = millis(); send_telemetry(); }

    // LED: slow=waiting, normal=linked idle, fast=servo moving
    bool moving = (abs(vx) > 10 || abs(vy) > 10 || abs(body_vz) > 10);
    if (millis() - last_cmd_ms > 3000) led_state = 0;
    else if (moving && (flags & 0x01)) led_state = 2;
    else led_state = 1;
    update_led();

    // Heartbeat at ~1Hz
    static uint32_t last_hb = 0;
    static uint32_t hb_count = 0;
    if (millis() - last_hb >= 1000) {
        last_hb = millis(); hb_count++;
        bool linked = (millis() - last_cmd_ms < 3000);
        Serial.printf("[%lu] ♥ PICO2 OK | ESP link=%s | servo ch0=%d ch1=%d ch2=%d\n",
                      hb_count, linked ? "YES" : "NO",
                      SERVOCENTER + (int)((float)vx * speed / 1000.0f * 30.0f / 180.0f * (SERVOMAX - SERVOMIN)),
                      SERVOCENTER + (int)((float)vy * speed / 1000.0f * 30.0f / 180.0f * (SERVOMAX - SERVOMIN)),
                      SERVOCENTER + (int)((float)body_vz * speed / 1000.0f * 30.0f / 180.0f * (SERVOMAX - SERVOMIN)));
    }

    delay(10);
}
