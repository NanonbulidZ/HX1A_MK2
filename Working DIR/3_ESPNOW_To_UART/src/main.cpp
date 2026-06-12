/*
 * 3_ESPNOW_To_UART
 * Robot ESP32: receives ESP-NOW from relay → forwards via UART to Pico2
 * Also receives telemetry from Pico2 via UART → sends back via ESP-NOW
 * PlatformIO
 */
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"

// UART to Pico2 (Serial2: TX=17, RX=16)
#define UART_BAUD 921600
#define UART_RX_BUF 256

// Relay ESP MAC (for telemetry backhaul)
uint8_t relay_mac[] = {0x24, 0x6F, 0x28, 0x11, 0x22, 0x33};

uint8_t rx_buf[UART_RX_BUF];
int rx_len = 0;

esp_now_peer_info_t peer;
bool relay_ready = false;

void on_data_sent(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS)
        Serial.println("Telem send fail");
}

void on_data_recv(const uint8_t* mac, const uint8_t* data, int len) {
    // Forward commands from relay to Pico2 via UART
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

    memcpy(peer.peer_addr, relay_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        relay_ready = true;
        Serial.println("Relay peer added");
    }
}

void send_back_to_relay(const uint8_t* data, int len) {
    if (!relay_ready) return;
    esp_now_send(relay_mac, data, len);
}

void loop() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (rx_len < UART_RX_BUF) {
            rx_buf[rx_len++] = b;
        } else {
            rx_len = 0;
        }

        uint8_t type, payload[128], pseq;
        int plen = proto_parse(rx_buf, rx_len, &type, payload, &pseq);
        if (plen > 0) {
            // Telemetry or status from Pico2 → relay back to ground
            if (type == PKT_TELEM || type == PKT_STATUS || type == PKT_DEBUG) {
                send_back_to_relay(rx_buf, rx_len);
            }
            rx_len = 0;
        } else if (plen < 0) {
            rx_len = 0;
        }
    }
}
