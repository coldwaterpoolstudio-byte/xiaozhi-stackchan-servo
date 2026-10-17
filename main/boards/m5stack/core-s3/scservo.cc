#include "scservo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "ScServo";

// SCS0009 instruction set
static constexpr uint8_t kScsInstWrite = 0x03;
static constexpr uint8_t kScsInstRead = 0x02;

// SCS0009 registers (SCS series: BIG-ENDIAN, high byte first (matches official SCSCL End=1))
static constexpr uint8_t kRegGoalPosition = 0x2A;  // 2 bytes, big-endian
static constexpr uint8_t kRegGoalTime = 0x2C;       // 2 bytes, big-endian; effective when speed=0
static constexpr uint8_t kRegGoalSpeed = 0x2E;      // 2 bytes, big-endian
static constexpr uint8_t kRegTorqueEnable = 0x28;   // 1 byte
static constexpr uint8_t kRegCurrentPosition = 0x38; // 2 bytes, big-endian, read-only
static constexpr uint8_t kRegPresentLoad = 0x3C;    // 2 bytes, big-endian, read-only (bit15=direction, bits14-0=load)

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
        .source_clk = UART_SCLK_APB,
    };
    esp_err_t ret = uart_param_config(uart_num_, &config);
    ESP_LOGI(TAG, "uart_param_config ret=%d", ret);
    ret = uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "uart_set_pin ret=%d tx=%d rx=%d", ret, tx_pin_, rx_pin_);
    // Keep TX as UART output for write operations (servo does not reply to WRITE).
    // Only release TX during READ operations.
    ESP_LOGI(TAG, "Servo %d initialized on UART%d (tx=%d rx=%d)", servo_id_, uart_num_, tx_pin_, rx_pin_);
    // Ensure servo is in position mode (factory firmware may leave yaw in motor/PWM mode)
    ensurePositionMode();
}

uint8_t ScServo::checksum(const uint8_t* data, size_t len) {
    // SCS protocol: checksum = ~(ID + Length + Instruction + Address + Data...)
    // Must NOT include the checksum byte itself (at data[len-1], still uninitialized).
    uint16_t sum = 0;
    for (size_t i = 2; i < len - 1; i++) {  // skip header AND checksum byte
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

void ScServo::sendPacket(const uint8_t* data, size_t len) {
    // TX stays as UART output (write-only path). Flush RX loopback first.
    uart_flush_input(uart_num_);
    int written = uart_write_bytes(uart_num_, data, len);
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(50));
    // Always log: confirm data actually went out on the wire
    char hexbuf[64];
    int pos = 0;
    for (size_t i = 0; i < len && pos < (int)sizeof(hexbuf) - 4; i++) {
        pos += snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "TX servo=%d len=%d wrote=%d: %s", servo_id_, len, written, hexbuf);
    // Small delay for servo to process the command
    vTaskDelay(pdMS_TO_TICKS(3));
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
    // Retry up to 3 times (half-duplex bus can miss a response)
    for (int attempt = 0; attempt < 3; attempt++) {
        if (readRegisterInternal(reg, data, len, timeout_ms)) return true;
        if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "Servo %d readRegister 0x%02X failed after 3 retries", servo_id_, reg);
    return false;
}

bool ScServo::readRegisterInternal(uint8_t reg, uint8_t* data, size_t len, int timeout_ms) {
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

void ScServo::stallTaskFunc(void* arg) {
    ScServo* servo = static_cast<ScServo*>(arg);
    int target = servo->last_target_01deg_;
    int target_raw = servo->angleToRaw(target);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    int last_pos = -9999;
    int stable_count = 0;
    while (xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
        int cur = servo->getCurrentAngle();
        int cur_raw = servo->angleToRaw(cur);
        if (abs(cur_raw - target_raw) < 15) {
            servo->stall_task_ = nullptr;
            vTaskDelete(nullptr);
            return;
        }
        if (cur_raw == last_pos) {
            stable_count++;
            if (stable_count >= 8) {  // 400ms no movement = stalled
                ESP_LOGW("ScServo", "Servo %d stalled! Releasing torque.", servo->servo_id_);
                servo->setTorque(false);
                vTaskDelay(pdMS_TO_TICKS(200));
                servo->setTorque(true);
                servo->stall_task_ = nullptr;
                vTaskDelete(nullptr);
                return;
            }
        } else {
            stable_count = 0;
        }
        last_pos = cur_raw;
    }
    servo->stall_task_ = nullptr;
    vTaskDelete(nullptr);
}

void ScServo::moveWithSpeed(int position_01deg, int speed_0_1000) {
    int raw_pos = angleToRaw(position_01deg);
    ESP_LOGI(TAG, "Servo %d move: angle=%d raw=%d speed=%d", servo_id_, position_01deg, raw_pos, speed_0_1000);

    // SCS0009 requires CONTIGUOUS write from 0x2A: GoalPos(2) + GoalTime(2) + GoalSpeed(2)
    // Writing them as separate register writes can leave servo at default speed=0 (stopped).
    // Reference: Qiita Ninagawa123 SCS0009 direct drive example.
    uint8_t data[6];
    // Goal Position (BIG-ENDIAN: high byte first, matches official SCSCL End=1 Host2SCS)
    data[0] = (uint8_t)((raw_pos >> 8) & 0xFF);
    data[1] = (uint8_t)(raw_pos & 0xFF);
    // Goal Time = 10000/speed (unit=10ms). speed=500 -> Time=20 (official baseline).
    // speed=300 (default, smoother) -> Time=33 (330ms). speed=100 -> Time=100 (1s, very slow).
    int goal_time = 10000 / (speed_0_1000 > 0 ? speed_0_1000 : 1);
    if (goal_time < 5) goal_time = 5;      // min 50ms
    if (goal_time > 200) goal_time = 200;   // max 2000ms
    data[2] = (uint8_t)((goal_time >> 8) & 0xFF);
    data[3] = (uint8_t)(goal_time & 0xFF);
    // Goal Speed = 0 (big-endian; matches official StackChan WritePos Speed=0 = max speed, Time controls duration)
    data[4] = 0x00;
    data[5] = 0x00;
    writeRegister(kRegGoalPosition, data, 6);

    // Async stall protection: start one-shot task (does not block main thread)
    last_target_01deg_ = position_01deg;
    if (stall_task_ != nullptr) {
        vTaskDelete(stall_task_);  // cancel previous stale task
    }
    xTaskCreate(stallTaskFunc, "servo_stall", 2048, this, 5, &stall_task_);
}

int ScServo::getCurrentAngle() {
    uint8_t data[2] = {0, 0};
    if (!readRegister(kRegCurrentPosition, data, 2)) {
        return 0;
    }
    // SCS series: BIG-ENDIAN (high byte first, matches official SCSCL End=1)
    int raw = (data[0] << 8) | data[1];
    return rawToAngle(raw);
}

int ScServo::readLoad() {
    uint8_t data[2] = {0, 0};
    if (!readRegister(kRegPresentLoad, data, 2)) {
        return -1;
    }
    int raw = (data[0] << 8) | data[1];
    // bit15 = direction (0=CCW, 1=CW), bits14-0 = load value
    int load = raw & 0x7FFF;
    if (raw & 0x8000) load = -load;
    return load;
}

bool ScServo::stallProtect(int target_angle_01deg, int timeout_ms) {
    // Wait for servo to reach target, release torque if stalled
    int target_raw = angleToRaw(target_angle_01deg);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    int last_pos = -9999;
    int stable_count = 0;
    while (xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
        int cur = getCurrentAngle();
        int cur_raw = angleToRaw(cur);
        if (abs(cur_raw - target_raw) < 10) {
            return true;  // reached target
        }
        if (cur_raw == last_pos) {
            stable_count++;
            if (stable_count >= 6) {  // 300ms no movement = stalled
                ESP_LOGW(TAG, "Servo %d stalled! Releasing torque.", servo_id_);
                setTorque(false);
                vTaskDelay(pdMS_TO_TICKS(100));
                setTorque(true);
                return false;
            }
        } else {
            stable_count = 0;
        }
        last_pos = cur_raw;
    }
    return false;  // timeout
}

void ScServo::setTorque(bool enabled) {
    uint8_t data[1] = {(uint8_t)(enabled ? 1 : 0)};
    writeRegister(kRegTorqueEnable, data, 1);
}

void ScServo::ensurePositionMode() {
    // SCS0009 enters motor mode when BOTH angle limits are set to 0.
    // Factory firmware enables PWM mode on yaw; ensure we restore position mode.
    // EEPROM area (0x09-0x0C) uses BIG-ENDIAN (H byte first), unlike SRAM area.
    // EEPROM write requires unlock (reg 0x30=0), then lock (0x30=1) after.
    ESP_LOGI(TAG, "Servo %d ensuring position mode...", servo_id_);

    // Unlock EEPROM
    uint8_t unlock = 0;
    writeRegister(0x30, &unlock, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Min angle limit = 0 (addr 0x09, big-endian H=0x00 L=0x00)
    uint8_t min_limit[2] = {0x00, 0x00};
    writeRegister(0x09, min_limit, 2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Max angle limit = 1023 = 0x03FF (addr 0x0B, big-endian H=0x03 L=0xFF)
    uint8_t max_limit[2] = {0x03, 0xFF};
    writeRegister(0x0B, max_limit, 2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Lock EEPROM
    uint8_t lock = 1;
    writeRegister(0x30, &lock, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Servo %d position mode ensured (angle limit 0~1023)", servo_id_);
}
