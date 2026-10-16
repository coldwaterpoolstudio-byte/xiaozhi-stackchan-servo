#ifndef _SCSERVO_H_
#define _SCSERVO_H_

#include <driver/uart.h>
#include <driver/gpio.h>

// Feetech SCS0009 serial bus servo driver
// Half-duplex UART, 1Mbps, 8N1
// StackChan wiring: UART1 TX=GPIO6, RX=GPIO7 (single-bus, TX/RX tied)
class ScServo {
public:
    ScServo(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int servo_id);
    ~ScServo();

    void init();

    // Move to target position. Position unit: 0.1 degree (same as StackChan motion API)
    // yaw range: -1280~1280 (-128~128 deg), pitch range: 0~900 (0~90 deg)
    void move(int position_01deg);
    void moveWithSpeed(int position_01deg, int speed_0_1000);

    // Read current position in 0.1 degree
    int getCurrentAngle();

    // Enable/disable torque
    void setTorque(bool enabled);

    int getId() const { return servo_id_; }

private:
    void writeRegister(uint8_t reg, const uint8_t* data, size_t len);
    bool readRegister(uint8_t reg, uint8_t* data, size_t len, int timeout_ms = 50);
    void sendPacket(const uint8_t* data, size_t len);
    uint8_t checksum(const uint8_t* data, size_t len);

    // Convert 0.1-degree position to SCS0009 raw position (0~1023)
    int angleToRaw(int angle_01deg);
    // Convert SCS0009 raw position to 0.1-degree
    int rawToAngle(int raw);

    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    int servo_id_;

    // Calibration: raw position at 0 degree, and raw units per 0.1 degree
    // SCS0009: 0~1023 maps to 0~300 deg => 1023/3000 ≈ 0.341 raw per 0.1deg
    // StackChan yaw: center ~512, range ±128deg => ±1280 (0.1deg)
    // StackChan pitch: 0deg ~ some offset, range 0~90deg => 0~900 (0.1deg)
    static constexpr int kRawCenter = 512;
    static constexpr float kRawPer01Deg = 1023.0f / 3000.0f;  // ~0.341
};

#endif  // _SCSERVO_H_
