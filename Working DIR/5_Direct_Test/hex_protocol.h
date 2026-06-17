#ifndef HEXAPOD_PROTOCOL_H
#define HEXAPOD_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define PROTO_START    0xAA
#define PROTO_VER      0x01

#define PKT_VELOCITY   0x01
#define PKT_LEG_OFFSET 0x02
#define PKT_MODE       0x03
#define PKT_LIGHTS     0x04
#define PKT_CALIB      0x05

#define PKT_TELEM      0x81
#define PKT_STATUS     0x82
#define PKT_DEBUG      0x83

typedef struct __attribute__((packed)) {
    int16_t vx;      int16_t vy;      int16_t vr;
    int16_t body_vz; int16_t body_rx; int16_t body_ry;
    uint8_t flags;   uint8_t speed;
} PktVelocity;

typedef struct __attribute__((packed)) {
    int8_t ox[6]; int8_t oy[6]; int8_t oz[6];
} PktLegOffset;

typedef struct __attribute__((packed)) {
    uint16_t batt_mv; int16_t roll;  int16_t pitch; int16_t yaw;
    uint8_t foot_contact; uint8_t state;
    int16_t ax; int16_t ay; int16_t az;
    int16_t gx; int16_t gy; int16_t gz;
} PktTelemetry;

static inline uint8_t proto_crc(const uint8_t* d, int l) {
    uint8_t c = 0; while (l--) c ^= *d++; return c;
}

static inline int proto_build(uint8_t* buf, uint8_t type, const uint8_t* payload, uint8_t len, uint8_t seq) {
    buf[0] = PROTO_START; buf[1] = PROTO_VER; buf[2] = type;
    buf[3] = len; buf[4] = seq;
    if (len && payload) memcpy(buf + 5, payload, len);
    buf[5 + len] = proto_crc(buf, 5 + len);
    return 6 + len;
}

static inline int proto_parse(const uint8_t* buf, int avail, uint8_t* type, uint8_t* payload, uint8_t* seq) {
    if (avail < 6) return 0;
    if (buf[0] != PROTO_START || buf[1] != PROTO_VER) return -1;
    uint8_t len = buf[3]; int total = 6 + len;
    if (avail < total) return 0;
    if (proto_crc(buf, total - 1) != buf[total - 1]) return -1;
    *type = buf[2]; *seq = buf[4];
    if (len) memcpy(payload, buf + 5, len);
    return len;
}

#endif
