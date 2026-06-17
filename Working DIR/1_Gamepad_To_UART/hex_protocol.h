#ifndef HEXAPOD_PROTOCOL_H
#define HEXAPOD_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define PROTO_START    0xAA
#define PROTO_VER      0x01

// Packet types (ground → robot)
#define PKT_VELOCITY   0x01
#define PKT_LEG_OFFSET 0x02
#define PKT_MODE       0x03
#define PKT_LIGHTS     0x04
#define PKT_CALIB      0x05

// Packet types (robot → ground)
#define PKT_TELEM      0x81
#define PKT_STATUS     0x82
#define PKT_DEBUG      0x83

#define PROTO_HDR_LEN  5
#define PROTO_MAX_LEN  64
#define PROTO_CRC_LEN  1

// Velocity command: 10 bytes
typedef struct __attribute__((packed)) {
    int16_t vx;      // mm/s * 10
    int16_t vy;
    int16_t vr;      // rotation °/s * 10
    int16_t body_vz; // body height offset mm * 10
    int16_t body_rx; // body roll °/10
    int16_t body_ry; // body pitch °/10
    uint8_t flags;   // bit0=walk_enable, bit1-2=gait, bit3=reset, bit4=enable_balance
    uint8_t speed;   // 0-100%
} __attribute__((packed)) PktVelocity;

// Leg offset: 18 bytes (6 legs × 3 axes × 1 byte)
typedef struct __attribute__((packed)) {
    int8_t ox[6];
    int8_t oy[6];
    int8_t oz[6];
} __attribute__((packed)) PktLegOffset;

// Telemetry: 20 bytes
typedef struct __attribute__((packed)) {
    uint16_t batt_mv;
    int16_t roll;     // millidegrees
    int16_t pitch;
    int16_t yaw;
    uint8_t foot_contact; // bit0-5 = leg0-5
    uint8_t state;
    int16_t accel_x;  // mg
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;   // mdps
    int16_t gyro_y;
    int16_t gyro_z;
} __attribute__((packed)) PktTelemetry;

// Packet builder
static inline uint8_t proto_crc(const uint8_t* d, int l) {
    uint8_t c = 0;
    while (l--) c ^= *d++;
    return c;
}

static inline int proto_build(uint8_t* buf, uint8_t type, const uint8_t* payload, uint8_t len, uint8_t seq) {
    buf[0] = PROTO_START;
    buf[1] = PROTO_VER;
    buf[2] = type;
    buf[3] = len;
    buf[4] = seq;
    if (len && payload) memcpy(buf + 5, payload, len);
    uint8_t crc = proto_crc(buf, 5 + len);
    buf[5 + len] = crc;
    return 6 + len;
}

// Returns payload length or -1 on error
static inline int proto_parse(const uint8_t* buf, int avail, uint8_t* type, uint8_t* payload, uint8_t* seq) {
    if (avail < 6) return 0;
    if (buf[0] != PROTO_START || buf[1] != PROTO_VER) return -1;
    uint8_t len = buf[3];
    int total = 6 + len;
    if (avail < total) return 0;
    uint8_t crc = proto_crc(buf, total - 1);
    if (crc != buf[total - 1]) return -1;
    *type = buf[2];
    *seq = buf[4];
    if (len) memcpy(payload, buf + 5, len);
    return len;
}

#endif
