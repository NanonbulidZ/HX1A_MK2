/*
 * 6_Test_Servo_Sweep (Modified)
 * Pico2 + PCA9685 + SSD1306: 3 servos (ch0-2)
 * Channel 0 is locked at 90°, Channel 1 & 2 sweep independently with different speeds.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define I2C_SDA 4
#define I2C_SCL 5

#define PCA_ADDR 0x40
#define OLED_ADDR 0x3C

// New 12-bit tick constraints requested by user
const uint16_t SERVOMIN = 150;
const uint16_t SERVOMAX = 600;
const float PWM_FREQ = 50.0; // Hz

#define SERVOS 3

// Custom individual speeds (cycle duration in milliseconds)
#define CYCLE_MS_CH1 4000  // Channel 1 takes 4 seconds per full loop
#define CYCLE_MS_CH2 2500  // Channel 2 moves faster (2.5 seconds per full loop)

#define SCREEN_W 128
#define SCREEN_H 64

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA_ADDR);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// Updated function to write ticks directly
void set_servo_deg(uint8_t ch, float deg) {
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    
    // Linearly map 0-180 degrees directly to 150-600 ticks
    uint16_t pwm = SERVOMIN + (uint16_t)(deg / 180.0f * (SERVOMAX - SERVOMIN));
    pca.setPWM(ch, 0, pwm);
}

float ease_sine(float t) {
    return 0.5f * (1.0f - cosf(t * M_PI));
}

void draw_bar(int x, int y, int w, int h, float frac, uint16_t color) {
    int fw = (int)(frac * w);
    if (fw < 0) fw = 0; if (fw > w) fw = w;
    display.drawRect(x, y, w, h, color);
    if (fw > 0) display.fillRect(x, y, fw, h, color);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

    pca.begin();
    pca.setPWMFreq(PWM_FREQ);
    delay(10);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 not found");
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Servo System");
        display.display();
        delay(500);
    }

    // Initialize all to center
    for (int i = 0; i < SERVOS; i++) set_servo_deg(i, 90.0f);
    delay(500);

    Serial.println("Servo test ready (CH0 = 90 deg fixed, CH1/CH2 dynamic speeds)");
}

void loop() {
    uint32_t ms = millis();
    float degs[SERVOS];

    // --- Channel 0: Fixed at 90 degrees ---
    degs[0] = 90.0f;
    set_servo_deg(0, degs[0]);

    // --- Channel 1: Fixed Dynamic Sweep with Symmetry ---
    float t1 = fmodf((float)ms / CYCLE_MS_CH1, 1.0f);
    // Triangle wave: goes 0 -> 1 in first half, then 1 -> 0 in second half
    float poff1 = (t1 < 0.5f) ? (t1 * 2.0f) : (2.0f - (t1 * 2.0f)); 
    float eoff1 = ease_sine(poff1);
    degs[1] = eoff1 * 180.0f; // Naturally handles up and down smoothly
    set_servo_deg(1, degs[1]);

    // --- Channel 2: Fixed Dynamic Sweep with Symmetry ---
    float t2 = fmodf((float)ms / CYCLE_MS_CH2, 1.0f);
    // Triangle wave: goes 0 -> 1 in first half, then 1 -> 0 in second half
    float poff2 = (t2 < 0.5f) ? (t2 * 2.0f) : (2.0f - (t2 * 2.0f)); 
    float eoff2 = ease_sine(poff2);
    degs[2] = eoff2 * 180.0f; 
    set_servo_deg(2, degs[2]);

    // OLED Update (~15 fps)
    static uint32_t last_disp = 0;
    if (ms - last_disp > 66) {
        last_disp = ms;
        display.clearDisplay();

        // Title info (Tracking CH1's raw cycle ratio)
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.printf("Sys Running...");

        // Render Bar graphs
        for (int ch = 0; ch < SERVOS; ch++) {
            int y = 18 + ch * 15;
            float frac = degs[ch] / 180.0f;
            display.setCursor(0, y);
            display.printf("CH%d", ch);
            draw_bar(24, y + 1, 80, 8, frac, SSD1306_WHITE);
            display.setCursor(108, y);
            display.printf("%3.0f", degs[ch]);
        }
        display.display();
    }

    // Serial Logging
    static uint32_t last_print = 0;
    if (ms - last_print > 100) {
        last_print = ms;
        Serial.printf("Positions -> CH0: %.0f | CH1: %.0f | CH2: %.0f\n", degs[0], degs[1], degs[2]);
    }

    delay(10);
}