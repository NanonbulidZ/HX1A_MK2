#ifndef HEXAPOD_PROTOCOL_H
#define HEXAPOD_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// ─── Frame format ───────────────────────────────────────────────
//  SOF(0xAA) | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC(1)
//  CRC = XOR of TYPE + LEN + all payload bytes

#define PKT_SOF         0xAA
#define PKT_HEADER_BYTES 3
#define PKT_CRC_BYTES    1
#define PKT_MAX_PAYLOAD  64

// ─── Packet types ───────────────────────────────────────────────
#define PKT_CMD_MOTION    0x01
#define PKT_CMD_LEG_OFF   0x02
#define PKT_CMD_SYSTEM    0x03
#define PKT_CMD_LIGHTS    0x04
#define PKT_TELEM_STATUS  0x05
#define PKT_TELEM_DEBUG   0x06

// ─── CMD_MOTION (len=16) ────────────────────────────────────────
// Sent every frame from ground to robot
typedef struct __attribute__((packed)) {
    int16_t vx;          // forward velocity mm/s * 10
    int16_t vy;          // lateral velocity mm/s * 10
    int16_t vz;          // vertical velocity mm/s * 10
    int16_t rx;          // roll rate cdeg/s
    int16_t ry;          // pitch rate cdeg/s
    int16_t rz;          // yaw rate cdeg/s
    int8_t  body_height; // height offset mm (-50..50)
    uint8_t flags;       // see FLAG_*
    uint16_t buttons;    // see BTN_*
} PktCmdMotion;

// ─── CMD_LEG_OFF (len=7) ────────────────────────────────────────
// Set individual leg foot offset
typedef struct __attribute__((packed)) {
    uint8_t leg_idx;     // 0-5
    int16_t ox;          // mm * 10
    int16_t oy;          // mm * 10
    int16_t oz;          // mm * 10
} PktCmdLegOff;

// ─── CMD_SYSTEM (len=2) ─────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t command;     // CMD_*
    uint8_t param;       // command-specific
} PktCmdSystem;

// ─── CMD_LIGHTS (len=4) ─────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t led_mask;
    uint8_t r, g, b;
} PktCmdLights;

// ─── TELEM_STATUS (len=8) ───────────────────────────────────────
// Sent from robot back to ground
typedef struct __attribute__((packed)) {
    uint16_t batt_mv;     // mV
    uint8_t  foot_contact; // bit 0=leg0 .. bit5=leg5
    int16_t  roll;         // cdeg
    int16_t  pitch;        // cdeg
    uint8_t  mode_actual;
} PktTelemStatus;

// ─── TELEM_DEBUG (len=18) ──────────────────────────────────────
typedef struct __attribute__((packed)) {
    int16_t  current_x[6];
    int16_t  current_y[6];
    int16_t  current_z[6];
} PktTelemDebug;

// ─── Flag defines ──────────────────────────────────────────────
#define FLAG_GAIT_MASK    0x03
#define FLAG_GAIT_TRIPOD  0x00
#define FLAG_GAIT_WAVE    0x01
#define FLAG_GAIT_RIPPLE  0x02
#define FLAG_GAIT_TETRA   0x03
#define FLAG_SPEED_SLOW   0x04
#define FLAG_MODE_SHIFT   3
#define FLAG_MODE_MASK    0x18
#define FLAG_MODE_IDLE    0x00
#define FLAG_MODE_WALK    0x08
#define FLAG_MODE_TRANS   0x10
#define FLAG_MODE_ROTATE  0x18

// ─── System commands ────────────────────────────────────────────
#define CMD_IDLE      0
#define CMD_CALIBRATE 1
#define CMD_HOME      2
#define CMD_STOP      3
#define CMD_REBOOT    4

// ─── Button bits (for PktCmdMotion.buttons) ────────────────────
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
#define BTN_DPAD_U  (1<<12)
#define BTN_DPAD_D  (1<<13)
#define BTN_DPAD_L  (1<<14)
#define BTN_DPAD_R  (1<<15)

// ─── CRC ────────────────────────────────────────────────────────
static inline uint8_t pkt_calc_crc(uint8_t type, uint8_t len, const uint8_t* pay) {
    uint8_t c = type ^ len;
    for (uint8_t i = 0; i < len; i++) c ^= pay[i];
    return c;
}

// ─── Receiver state machine ─────────────────────────────────────
typedef struct {
    uint8_t state;       // 0=sof,1=type,2=len,3=pay,4=crc
    uint8_t type;
    uint8_t len;
    uint8_t idx;
    uint8_t crc;
    uint8_t payload[PKT_MAX_PAYLOAD];
} PktParser;

static inline void pkt_parser_init(PktParser* p) { p->state = 0; }

// Call with each byte. Returns packet type when complete, 0 otherwise.
static inline uint8_t pkt_parser_feed(PktParser* p, uint8_t byte) {
    switch (p->state) {
        case 0: if (byte == PKT_SOF) p->state = 1; break;
        case 1: p->type = byte; p->state = 2; break;
        case 2:
            p->len = byte;
            if (p->len > PKT_MAX_PAYLOAD) { p->state = 0; break; }
            p->idx = 0; p->crc = p->type ^ p->len; p->state = 3;
            break;
        case 3:
            if (p->idx < p->len) {
                p->payload[p->idx++] = byte;
                p->crc ^= byte;
                if (p->idx >= p->len) p->state = 4;
            }
            break;
        case 4:
            p->state = 0;
            if (byte == p->crc) return p->type;
            break;
    }
    return 0;
}

// ─── Sender helper ──────────────────────────────────────────────
// Usage: pkt_send(&Serial1, type, len, payload);
// Where T has a write(uint8_t) method (Stream, Print, etc.)
template<typename T>
static inline void pkt_send(T* stream, uint8_t type, uint8_t len, const uint8_t* pay) {
    stream->write(PKT_SOF);
    stream->write(type);
    stream->write(len);
    uint8_t crc = type ^ len;
    for (uint8_t i = 0; i < len; i++) {
        stream->write(pay[i]);
        crc ^= pay[i];
    }
    stream->write(crc);
}

#endif // HEXAPOD_PROTOCOL_H
