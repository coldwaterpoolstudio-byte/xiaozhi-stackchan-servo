#include "scservo.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "ScServo";

// SCS0009 instruction set
static constexpr uint8_t kScsInstWrite = 0x03;
static constexpr uint8_t kScsInstRead = 0x02;

// SCS0009 registers
static constexpr uint8_t kRegGoalPosition = 0x2A;  // 2 bytes, little-endian
static constexpr uint8_t kRegGoalSpeed = 0x2E;      // 2 bytes, little-endian
static constexpr uint8_t kRegTorqueEnable = 0x30;   // 1 byte
static constexpr uint8_t kRegCurrentPosition = 0x3A; // 2 bytes, little-endian, read-only

ScServo::ScServo(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int servo_id)
    : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), servo_id_(servo_id) {}

ScServo::~ScServo() {
    uart_driver_delete(uart_num_);
}

void ScServo::init() {
    // Shared UART bus: install driver only once (yaw + pitch both use UART1)
    static bool driver_installed = false;
    if (!driver_installed) {
        uart_driver_install(uart_num_, 256, 256, 0, nullptr, 0);
        driver_installed = true;
    }
    uart_config_t config = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(uart_num_, &config);
    uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "Servo %d initialized on UART%d", servo_id_, uart_num_);
}

uint8_t ScServo::checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 2; i < len; i++) {  // skip 0xFF 0xFF header
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

void ScServo::sendPacket(const uint8_t* data, size_t len) {
    // Flush RX buffer before sending (half-duplex loopback)
    uart_flush_input(uart_num_);
    uart_write_bytes(uart_num_, data, len);
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(50));
    // Small delay for servo to process
    vTaskDelay(pdMS_TO_TICKS(2));
}

void ScServo::writeRegister(uint8_t reg, const uint8_t* data, size_t len) {
    uint8_t packet[16];
    size_t pkt_len = 7 + len;  // header(2) + id(1) + length(1) + inst(1) + reg(1) + data(len) + checksum(1)
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = (uint8_t)servo_id_;
    packet[3] = (uint8_t)(len + 3);  // length = inst + reg + data + checksum
    packet[4] = kScsInstWrite;
    packet[5] = reg;
    memcpy(&packet[6], data, len);
    packet[6 + len] = checksum(packet, pkt_len);
    sendPacket(packet, pkt_len);
}

bool ScServo::readRegister(uint8_t reg, uint8_t* data, size_t len, int timeout_ms) {
    uint8_t packet[8];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = (uint8_t)servo_id_;
    packet[3] = 0x04;  // length = inst + reg + count + checksum
    packet[4] = kScsInstRead;
    packet[5] = reg;
    packet[6] = (uint8_t)len;
    packet[7] = checksum(packet, 8);
    sendPacket(packet, 8);

    // Read response: 0xFF 0xFF ID Length Error [data...] Checksum
    uint8_t resp[16];
    size_t expected = 6 + len;  // header(2) + id(1) + length(1) + error(1) + data(len) + checksum(1)
    int rx_len = uart_read_bytes(uart_num_, resp, expected, pdMS_TO_TICKS(timeout_ms));
    if (rx_len < expected) {
        return false;
    }
    if (resp[0] != 0xFF || resp[1] != 0xFF || resp[2] != servo_id_) {
        return false;
    }
    memcpy(data, &resp[5], len);
    return true;
}

int ScServo::angleToRaw(int angle_01deg) {
    // Map 0.1-degree angle to SCS0009 raw position (0~1023)
    // Center (0 deg) = 512, positive = clockwise
    int raw = kRawCenter + (int)(angle_01deg * kRawPer01Deg);
    if (raw < 0) raw = 0;
    if (raw > 1023) raw = 1023;
    return raw;
}

int ScServo::rawToAngle(int raw) {
    return (int)((raw - kRawCenter) / kRawPer01Deg);
}

void ScServo::move(int position_01deg) {
    moveWithSpeed(position_01deg, 500);
}

void ScServo::moveWithSpeed(int position_01deg, int speed_0_1000) {
    int raw_pos = angleToRaw(position_01deg);
    uint8_t pos_data[2] = {(uint8_t)(raw_pos & 0xFF), (uint8_t)((raw_pos >> 8) & 0xFF)};
    writeRegister(kRegGoalPosition, pos_data, 2);

    // Speed: map 0~1000 to 0~1023
    int raw_speed = speed_0_1000;
    if (raw_speed < 0) raw_speed = 0;
    if (raw_speed > 1023) raw_speed = 1023;
    uint8_t speed_data[2] = {(uint8_t)(raw_speed & 0xFF), (uint8_t)((raw_speed >> 8) & 0xFF)};
    writeRegister(kRegGoalSpeed, speed_data, 2);
}

int ScServo::getCurrentAngle() {
    uint8_t data[2] = {0, 0};
    if (!readRegister(kRegCurrentPosition, data, 2)) {
        return 0;
    }
    int raw = data[0] | (data[1] << 8);
    return rawToAngle(raw);
}

void ScServo::setTorque(bool enabled) {
    uint8_t data[1] = {(uint8_t)(enabled ? 1 : 0)};
    writeRegister(kRegTorqueEnable, data, 1);
}
