#include "scservo.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "ScServo";

// SCS0009 instruction set
static constexpr uint8_t kScsInstWrite = 0x03;
static constexpr uint8_t kScsInstRead = 0x02;

// SCS0009 registers (SCS series: big-endian, high byte first)
static constexpr uint8_t kRegGoalPosition = 0x2A;  // 2 bytes, big-endian
static constexpr uint8_t kRegGoalSpeed = 0x2E;      // 2 bytes, big-endian
static constexpr uint8_t kRegTorqueEnable = 0x28;   // 1 byte
static constexpr uint8_t kRegCurrentPosition = 0x38; // 2 bytes, big-endian, read-only

ScServo::ScServo(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int servo_id, int center_raw)
    : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), servo_id_(servo_id), center_raw_(center_raw) {}

ScServo::~ScServo() {
    uart_driver_delete(uart_num_);
}

void ScServo::init() {
    // Shared UART bus: install driver only once (yaw + pitch both use UART1)
    static bool driver_installed = false;
    if (!driver_installed) {
        esp_err_t ret = uart_driver_install(uart_num_, 256, 256, 0, nullptr, 0);
        ESP_LOGI(TAG, "uart_driver_install ret=%d", ret);
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
    esp_err_t ret = uart_param_config(uart_num_, &config);
    ESP_LOGI(TAG, "uart_param_config ret=%d", ret);
    ret = uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "uart_set_pin ret=%d tx=%d rx=%d", ret, tx_pin_, rx_pin_);
    // Keep TX as UART output for write operations (servo does not reply to WRITE).
    // Only release TX during READ operations.
    ESP_LOGI(TAG, "Servo %d initialized on UART%d (tx=%d rx=%d)", servo_id_, uart_num_, tx_pin_, rx_pin_);
}

uint8_t ScServo::checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 2; i < len; i++) {  // skip 0xFF 0xFF header
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

void ScServo::sendPacket(const uint8_t* data, size_t len) {
    // TX stays as UART output (write-only path). Flush RX loopback first.
    uart_flush_input(uart_num_);
    int written = uart_write_bytes(uart_num_, data, len);
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(50));
    // Small delay for servo to process the command
    vTaskDelay(pdMS_TO_TICKS(3));
    if (written != (int)len) {
        ESP_LOGW(TAG, "sendPacket: wrote %d/%d bytes for servo %d", written, len, servo_id_);
    }
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

    // Half-duplex: the sent command loops back into RX before the servo response.
    // Search for frame header 0xFF 0xFF ID, then use Length byte to distinguish
    // loopback (Length=0x04 for READ cmd) from real response (Length=len+2).
    uint8_t byte;
    int state = 0;  // 0=wait 0xFF, 1=wait 0xFF, 2=wait ID
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        if (uart_read_bytes(uart_num_, &byte, 1, pdMS_TO_TICKS(10)) != 1) continue;
        if (state == 0 && byte == 0xFF) { state = 1; continue; }
        if (state == 1 && byte == 0xFF) { state = 2; continue; }
        if (state == 2 && byte == servo_id_) {
            // Peek Length byte
            uint8_t length_byte;
            if (uart_read_bytes(uart_num_, &length_byte, 1, pdMS_TO_TICKS(50)) != 1) {
                state = 0; continue;
            }
            if (length_byte == 0x04) {
                // Loopback READ command: skip remaining 4 bytes (inst, reg, count, checksum)
                uint8_t discard[4];
                uart_read_bytes(uart_num_, discard, 4, pdMS_TO_TICKS(50));
                state = 0;
                continue;
            }
            // Real response: Length=len+2, followed by Error(1), Data(len), Checksum(1)
            uint8_t error_byte;
            if (uart_read_bytes(uart_num_, &error_byte, 1, pdMS_TO_TICKS(50)) != 1) return false;
            if (uart_read_bytes(uart_num_, data, len, pdMS_TO_TICKS(timeout_ms)) != (int)len) return false;
            uint8_t checksum_byte;
            uart_read_bytes(uart_num_, &checksum_byte, 1, pdMS_TO_TICKS(50));
            return true;
        }
        state = 0;
    }
    return false;
}

int ScServo::angleToRaw(int angle_01deg) {
    // Official StackChan: 0.32 raw per 0.1 degree (16/5/10), raw range 0~1000
    int raw = center_raw_ + (int)(angle_01deg * kRawPer01Deg);
    if (raw < kRawMin) raw = kRawMin;
    if (raw > kRawMax) raw = kRawMax;
    return raw;
}

int ScServo::rawToAngle(int raw) {
    return (int)((raw - center_raw_) / kRawPer01Deg);
}

void ScServo::move(int position_01deg) {
    moveWithSpeed(position_01deg, 500);
}

void ScServo::moveWithSpeed(int position_01deg, int speed_0_1000) {
    int raw_pos = angleToRaw(position_01deg);
    ESP_LOGI(TAG, "Servo %d move: angle=%d raw=%d speed=%d", servo_id_, position_01deg, raw_pos, speed_0_1000);

    // Write speed FIRST, then position — otherwise servo starts moving with
    // previous (possibly zero) speed and the new speed never triggers.
    int raw_speed = speed_0_1000;
    if (raw_speed < 0) raw_speed = 0;
    if (raw_speed > 1023) raw_speed = 1023;
    uint8_t speed_data[2] = {(uint8_t)((raw_speed >> 8) & 0xFF), (uint8_t)(raw_speed & 0xFF)};
    writeRegister(kRegGoalSpeed, speed_data, 2);

    // SCS series: big-endian (high byte first)
    uint8_t pos_data[2] = {(uint8_t)((raw_pos >> 8) & 0xFF), (uint8_t)(raw_pos & 0xFF)};
    writeRegister(kRegGoalPosition, pos_data, 2);
}

int ScServo::getCurrentAngle() {
    uint8_t data[2] = {0, 0};
    if (!readRegister(kRegCurrentPosition, data, 2)) {
        return 0;
    }
    // SCS series: big-endian (high byte first)
    int raw = (data[0] << 8) | data[1];
    return rawToAngle(raw);
}

void ScServo::setTorque(bool enabled) {
    uint8_t data[1] = {(uint8_t)(enabled ? 1 : 0)};
    writeRegister(kRegTorqueEnable, data, 1);
}
