/*
 * 5_Direct_Test
 * Direct Bluepad32 → UART → Pico2 (no ESP-NOW)
 * For bench testing the hexapod brain without wireless relay
 * Arduino IDE - install Bluepad32 library
 */
#include <Bluepad32.h>
#include "protocol.h"

// Direct UART to Pico2 (Serial2: TX=17, RX=16)
#define UART_BAUD 921600

GamepadPtr gp = nullptr;

void onConnectedGamepad(GamepadPtr g) { gp = g; Serial.println("GP connected"); }
void onDisconnectedGamepad(GamepadPtr g) { if (gp == g) gp = nullptr; Serial.println("GP disconnected"); }

uint8_t tx_buf[32];
uint8_t seq = 0;

void send_velocity(int16_t vx, int16_t vy, int16_t vr, int16_t body_vz, int16_t body_rx, int16_t body_ry, uint8_t flags, uint8_t spd) {
    PktVelocity p;
    p.vx = vx; p.vy = vy; p.vr = vr;
    p.body_vz = body_vz; p.body_rx = body_rx; p.body_ry = body_ry;
    p.flags = flags; p.speed = spd;
    int n = proto_build(tx_buf, PKT_VELOCITY, (uint8_t*)&p, sizeof(p), seq++);
    Serial2.write(tx_buf, n);
}

void send_leg_offsets(const int8_t ox[6], const int8_t oy[6], const int8_t oz[6]) {
    PktLegOffset p;
    memcpy(p.ox, ox, 6); memcpy(p.oy, oy, 6); memcpy(p.oz, oz, 6);
    int n = proto_build(tx_buf, PKT_LEG_OFFSET, (uint8_t*)&p, sizeof(p), seq++);
    Serial2.write(tx_buf, n);
}

void send_mode(uint8_t mode, uint8_t gait) {
    uint8_t m[2] = {mode, gait};
    int n = proto_build(tx_buf, PKT_MODE, m, 2, seq++);
    Serial2.write(tx_buf, n);
}

// Telemetry receive buffer
uint8_t telem_buf[128];
int telem_len = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, 16, 17);
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.enableVirtualDevice(false);
    Serial.println("Direct Test ready");
}

void loop() {
    BP32.update();

    // Read back telemetry from Pico2 and print
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (telem_len < (int)sizeof(telem_buf)) {
            telem_buf[telem_len++] = b;
        } else {
            telem_len = 0;
        }
        uint8_t type, payload[64], pseq;
        int plen = proto_parse(telem_buf, telem_len, &type, payload, &pseq);
        if (plen > 0) {
            if (type == PKT_TELEM && plen >= (int)sizeof(PktTelemetry)) {
                PktTelemetry* t = (PktTelemetry*)payload;
                Serial.printf("TELEM: batt=%d roll=%d pitch=%d contact=0x%02X state=0x%02X\n",
                              t->batt_mv, t->roll, t->pitch, t->foot_contact, t->state);
            } else if (type == PKT_DEBUG) {
                payload[plen] = 0;
                Serial.printf("DEBUG: %s\n", (char*)payload);
            }
            telem_len = 0;
        } else if (plen < 0) {
            telem_len = 0;
        }
    }

    if (!gp || !gp->isConnected()) return;

    // Same gamepad mapping as Code 1
    int16_t vx = constrain(gp->axisRY() * -1000 / 512, -1000, 1000);
    int16_t vy = constrain(gp->axisRX() *  1000 / 512, -1000, 1000);
    int16_t vr = constrain(gp->axisX()  * -1000 / 512, -1000, 1000);
    int16_t body_vz = constrain(gp->axisY() * 500 / 512, -500, 500);

    int16_t body_rx = constrain((gp->brake() - 512) * 100 / 512, -100, 100);
    int16_t body_ry = constrain((gp->throttle() - 512) * 100 / 512, -100, 100);

    uint8_t flags = 0;
    if (gp->a())      flags |= 0x01;
    if (gp->b())      flags |= 0x08;
    if (gp->x())      flags |= 0x10;
    if (gp->y())      flags |= 0x20;

    uint8_t gait = 0;
    if (gp->dpad() & 0x02) gait = 1;
    if (gp->dpad() & 0x04) gait = 2;
    if (gp->dpad() & 0x01) gait = 3;
    if (gp->dpad() & 0x08) gait = 4;
    flags |= (gait << 1);

    uint8_t spd = constrain(abs(gp->brake() - 512) * 100 / 512, 0, 100);

    send_velocity(vx, vy, vr, body_vz, body_rx, body_ry, flags, spd);
    delay(10);
}
