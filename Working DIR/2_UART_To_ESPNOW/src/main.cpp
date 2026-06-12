/*
 * 2_UART_To_ESPNOW
 * ESP32 relay: receives protocol via UART → forwards via ESP-NOW to robot
 * Also receives telemetry back via ESP-NOW → forwards via UART to ground
 * PlatformIO
 */
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"

// UART from gamepad ESP (Serial2: RX=16, TX=17)
#define UART_BAUD 921600
#define UART_RX_BUF 128

// Robot ESP MAC address (set this!)
uint8_t robot_mac[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

uint8_t rx_buf[UART_RX_BUF];
int rx_len = 0;

uint8_t tx_buf[sizeof(PktTelemetry) + 8];
uint8_t seq = 0;

esp_now_peer_info_t peer;
bool robot_ready = false;

// Called when ESP-NOW send completes
void on_data_sent(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS)
        Serial.println("ESPNOW send fail");
}

// Called when ESP-NOW receives telemetry from robot
void on_data_recv(const uint8_t* mac, const uint8_t* data, int len) {
    // Forward telemetry back via UART to ground station
    Serial2.write(data, len);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, 16, 17);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESPNOW init fail");
        return;
    }
    esp_now_register_send_cb(on_data_sent);
    esp_now_register_recv_cb(on_data_recv);

    memcpy(peer.peer_addr, robot_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        robot_ready = true;
        Serial.println("ESPNOW peer added");
    }
}

void forward_to_robot(const uint8_t* data, int len) {
    if (!robot_ready) return;
    esp_now_send(robot_mac, data, len);
}

void loop() {
    // Read from UART (gamepad ESP), forward to robot via ESP-NOW
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (rx_len < UART_RX_BUF) {
            rx_buf[rx_len++] = b;
        } else {
            rx_len = 0; // overflow, reset
        }

        // Try to parse
        uint8_t type, payload[64], pseq;
        int plen = proto_parse(rx_buf, rx_len, &type, payload, &pseq);
        if (plen > 0) {
            // Forward complete packet via ESP-NOW
            forward_to_robot(rx_buf, rx_len);
            rx_len = 0;
        } else if (plen < 0) {
            rx_len = 0; // bad packet, reset
        }
    }
}
