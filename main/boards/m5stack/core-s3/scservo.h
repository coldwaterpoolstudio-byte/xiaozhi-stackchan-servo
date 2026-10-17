#pragma once
#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"
#include "driver/gpio.h"

// Lightweight Feetech SCS0009 serial bus servo driver.
// Protocol matches official M5Stack StackChan hal_servo.cpp (SCSCL library):
//   - UART1, 1Mbps, GPIO6 TX / GPIO7 RX
//   - big-endian (high byte first, SCSCL End=1)
//   - raw range 0~1000, 0.32 raw per 0.1 degree (16/5/10)
//   - yaw center=460, pitch center=620
class ScServo {
public:
    ScServo(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int servo_id, int center_raw = 512);
    ~ScServo();

    void init();
    void move(int position_01deg);
    void moveWithSpeed(int position_01deg, int speed_0_1000);
    int getCurrentAngle();
    void setTorque(bool enabled);
    void ensurePositionMode();
    int readLoad();  // Read present load (0x3C), returns -1 on failure
    bool stallProtect(int target_angle_01deg, int timeout_ms=1000);  // Release torque if stalled

private:
    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    int servo_id_;
    int center_raw_;
    int last_target_01deg_ = 0;       // last target for stall detection
    TaskHandle_t stall_task_ = nullptr; // async stall protection task
    static void stallTaskFunc(void* arg);

    static constexpr float kRawPer01Deg = 16.0f / 5.0f / 10.0f;  // 0.32, official StackChan
    static constexpr int kRawMin = 0;
    static constexpr int kRawMax = 1000;

    uint8_t checksum(const uint8_t* data, size_t len);
    void sendPacket(const uint8_t* data, size_t len);
    void writeRegister(uint8_t reg, const uint8_t* data, size_t len);
    bool readRegister(uint8_t reg, uint8_t* data, size_t len, int timeout_ms = 50);
    bool readRegisterInternal(uint8_t reg, uint8_t* data, size_t len, int timeout_ms);
    int angleToRaw(int angle_01deg);
    int rawToAngle(int raw);
};
