#include "wifi_board.h"
#include "cores3_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "config.h"
#include "power_save_timer.h"
#include "i2c_device.h"
#include "axp2101.h"
#include "servo_mcp_tool.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9341.h>
#include <esp_timer.h>
#include "esp_video.h"

#define TAG "M5StackCoreS3Board"

class Pmic : public Axp2101 {
public:
    // Power Init
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        uint8_t data = ReadReg(0x90);
        data |= 0b10110100;
        WriteReg(0x90, data);
        WriteReg(0x99, (0b11110 - 5));
        WriteReg(0x97, (0b11110 - 2));
        WriteReg(0x69, 0b00110101);
        WriteReg(0x30, 0b111111);
        WriteReg(0x90, 0xBF);
        WriteReg(0x94, 33 - 5);
        WriteReg(0x95, 33 - 5);
    }

    void SetBrightness(uint8_t brightness) {
        brightness = ((brightness + 641) >> 5);
        WriteReg(0x99, brightness);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(Pmic *pmic) : pmic_(pmic) {}

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->SetBrightness(target_brightness_);
        brightness_ = target_brightness_;
    }

private:
    Pmic *pmic_;
};

class Aw9523 : public I2cDevice {
public:
    // Exanpd IO Init
    Aw9523(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x02, 0b00000111);  // P0
        WriteReg(0x03, 0b10001111);  // P1
        WriteReg(0x04, 0b00011000);  // CONFIG_P0
        WriteReg(0x05, 0b00001100);  // CONFIG_P1
        WriteReg(0x11, 0b00010000);  // GCR P0 port is Push-Pull mode.
        WriteReg(0x12, 0b11111111);  // LEDMODE_P0
        WriteReg(0x13, 0b11111111);  // LEDMODE_P1
    }

    void ResetAw88298() {
        ESP_LOGI(TAG, "Reset AW88298");
        WriteReg(0x02, 0b00000011);
        vTaskDelay(pdMS_TO_TICKS(10));
        WriteReg(0x02, 0b00000111);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void ResetIli9342() {
        ESP_LOGI(TAG, "Reset IlI9342");
        WriteReg(0x03, 0b10000001);
        vTaskDelay(pdMS_TO_TICKS(20));
        WriteReg(0x03, 0b10000011);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    
    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

// PY32L020 IO expander on StackChan body: controls servo power (pin 0 = VM_EN)
// and RGB LED strip (pin 13). I2C addr 0x6F, 100kHz.
// Reference: M5Stack official hal_io_expander.cpp + PY32IOExpander_Class driver
class Py32L020 : public I2cDevice {
public:
    Py32L020(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr), addr_(addr) {}

    // Safe Begin: caller must probe via i2c_master_probe first.
    // I2cDevice::ReadReg uses ESP_ERROR_CHECK and would crash on missing device.
    bool Begin() {
        ESP_LOGI(TAG, "PY32L020 initialized at 0x%02X", addr_);
        return true;
    }

    // Enable servo power (VM_EN on pin 0)
    void EnableServoPower(bool enable) {
        // Set pin 0 as output (REG_GPIO_M_L = 0x03, bit0)
        uint8_t mode = ReadReg(0x03);
        if (enable) mode |= 0x01; else mode &= ~0x01;
        WriteReg(0x03, mode);

        // Set pin 0 pull-up (REG_GPIO_PU_L = 0x09, bit0)
        uint8_t pu = ReadReg(0x09);
        if (enable) pu |= 0x01; else pu &= ~0x01;
        WriteReg(0x09, pu);

        // Set pin 0 output level (REG_GPIO_O_L = 0x05, bit0)
        uint8_t out = ReadReg(0x05);
        if (enable) out |= 0x01; else out &= ~0x01;
        WriteReg(0x05, out);

        ESP_LOGI(TAG, "PY32L020 servo power (VM_EN pin0): %s", enable ? "ON" : "OFF");
    }

    // === RGB LED strip control (12x WS2812C on pin 13) ===
    // PY32 generates WS2812 timing internally; ESP32 only sends color data via I2C.
    void SetLedCount(uint8_t count) {
        if (count > 32) count = 32;
        WriteReg(0x24, count & 0x3F);  // REG_LED_CFG
    }

    // Primary: RGB888 input (r,g,b each 0-255)
    void SetLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (index >= 32) return;
        uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        uint8_t data[2] = {(uint8_t)(color565 & 0xFF), (uint8_t)((color565 >> 8) & 0xFF)};
        WriteRegs(0x30 + index * 2, data, 2);  // REG_LED_RAM_START
    }

    // Overload: accept pre-converted RGB565 (e.g. read directly from display framebuffer)
    void SetLedColor(uint8_t index, uint16_t color565) {
        if (index >= 32) return;
        uint8_t data[2] = {(uint8_t)(color565 & 0xFF), (uint8_t)((color565 >> 8) & 0xFF)};
        WriteRegs(0x30 + index * 2, data, 2);
    }

    // Overload: accept 0xRRGGBB packed integer (web color format)
    void SetLedColor(uint8_t index, uint32_t color) {
        SetLedColor(index,
                    (uint8_t)((color >> 16) & 0xFF),
                    (uint8_t)((color >> 8) & 0xFF),
                    (uint8_t)(color & 0xFF));
    }

    // Bulk write: raw RGB565 pairs for all LEDs in a single I2C transaction.
    // data = 2 bytes per LED (little-endian RGB565), len = count*2.
    // Much faster than per-LED writes when animating (breathing/rainbow/etc).
    void SetLedData(const uint8_t* data, size_t len) {
        if (!data || len == 0) return;
        if (len > 64) len = 64;  // max 32 LEDs * 2 bytes
        WriteRegs(0x30, data, len);  // REG_LED_RAM_START
    }

    void RefreshLeds() {
        uint8_t val = ReadReg(0x24);
        WriteReg(0x24, val | (1 << 6));  // trigger refresh bit
    }

    void ShowAllLeds(uint8_t r, uint8_t g, uint8_t b) {
        for (int i = 0; i < 12; i++) {
            SetLedColor(i, r, g, b);
        }
        RefreshLeds();
    }

    void InitLedStrip() {
        // Pin 13 = RGB data output (in high byte registers: bit 5)
        uint8_t mode_h = ReadReg(0x04);   // GPIO_M_H
        mode_h |= 0x20;                    // pin13 output
        WriteReg(0x04, mode_h);

        uint8_t pu_h = ReadReg(0x0A);     // GPIO_PU_H
        pu_h |= 0x20;                      // pin13 pull-up
        WriteReg(0x0A, pu_h);

        uint8_t drv_h = ReadReg(0x14);    // GPIO_DRV_H
        drv_h &= ~0x20;                    // pin13 push-pull (0=push-pull)
        WriteReg(0x14, drv_h);

        SetLedCount(12);
        vTaskDelay(pdMS_TO_TICKS(50));
        ShowAllLeds(0, 0, 0);  // all off by default
        ESP_LOGI(TAG, "PY32L020 LED strip (12x WS2812 on pin13) initialized");
    }

private:
    uint8_t addr_;
};

class M5StackCoreS3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_;
    Aw9523* aw9523_;
    Py32L020* py32_;
    Ft6336* ft6336_;
    LcdDisplay* display_;
    EspVideo* camera_;
    esp_timer_handle_t touchpad_timer_;
    PowerSaveTimer* power_save_timer_;
    ServoMcpTool* servo_mcp_;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 120, 1200);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(10);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            if (pmic_->IsVbusPresent()) {
                ESP_LOGI(TAG, "VBUS present, skip power off (stay in sleep mode)");
                return;
            }
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeAw9523() {
        ESP_LOGI(TAG, "Init AW9523");
        aw9523_ = new Aw9523(i2c_bus_, 0x58);
        // Constructor sets: 0x02=0x07(BUS_OUT_EN=P0_1 high), 0x03=0x8F(BOOST_EN=P1_7 high),
        // 0x04=0x18, 0x05=0x0C, 0x11=0x10(P0 push-pull). Base 5V is enabled by default.
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void InitializePy32L020() {
        ESP_LOGI(TAG, "Init PY32L020");
        py32_ = nullptr;
        // Probe FIRST — I2cDevice::ReadReg uses ESP_ERROR_CHECK and would crash on missing device.
        // PY32L020 default addr = 0x6F (ADD_SEL low); alt = 0x71 (ADD_SEL high).
        uint8_t addrs[] = {0x6F, 0x71};
        uint8_t found = 0;
        for (int retry = 0; retry < 5 && found == 0; retry++) {
            for (uint8_t a : addrs) {
                if (i2c_master_probe(i2c_bus_, a, pdMS_TO_TICKS(100)) == ESP_OK) {
                    found = a;
                    ESP_LOGI(TAG, "PY32L020 probed at 0x%02X (retry %d)", a, retry);
                    break;
                }
            }
            if (found == 0) vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (found == 0) {
            ESP_LOGW(TAG, "PY32L020 not found at 0x6F or 0x71 — servos unpowered, LEDs off");
            return;
        }
        py32_ = new Py32L020(i2c_bus_, found);
        py32_->Begin();
        py32_->EnableServoPower(true);
        vTaskDelay(pdMS_TO_TICKS(200));
        py32_->InitLedStrip();
    }

    void RegisterLedMcpTool() {
        if (!py32_) return;
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(
            "self.robot.set_led",
            "Set one of the 12 base RGB LEDs by index (0..11). Channels 0..255. Updates immediately. Use for visual feedback: green when listening, blue when thinking, red on error, warm white during conversation. index=99 means all LEDs.",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, 0, 99),
                Property("r", kPropertyTypeInteger, 0, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 0, 255),
            }),
            [this](const PropertyList& properties) -> ToolResult {
                int index = properties["index"].value<int>();
                int r = properties["r"].value<int>();
                int g = properties["g"].value<int>();
                int b = properties["b"].value<int>();
                if (!py32_) {
                    return ReturnValue(std::string("{\"error\": \"PY32 not initialized\"}"));
                }
                if (index == 99) {
                    py32_->ShowAllLeds(r, g, b);
                } else if (index >= 0 && index < 12) {
                    py32_->SetLedColor(index, r, g, b);
                    py32_->RefreshLeds();
                }
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"index\": %d, \"rgb\": [%d,%d,%d]}", index, r, g, b);
                return ReturnValue(std::string(buf));
            });
        ESP_LOGI(TAG, "LED MCP tool registered (self.robot.set_led)");
    }

    void PollTouchpad() {
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        const int64_t TOUCH_THRESHOLD_MS = 500;  // 触摸时长阈值，超过500ms视为长按
        
        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        
        // 检测触摸开始
        if (touch_point.num > 0 && !was_touched) {
            was_touched = true;
            touch_start_time = esp_timer_get_time() / 1000; // 转换为毫秒
        } 
        // 检测触摸释放
        else if (touch_point.num == 0 && was_touched) {
            was_touched = false;
            int64_t touch_duration = (esp_timer_get_time() / 1000) - touch_start_time;
            
            // 只有短触才触发
            if (touch_duration < TOUCH_THRESHOLD_MS) {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting) {
                    EnterWifiConfigMode();
                    return;
                }
                app.ToggleChatState();
            }
        }
    }

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);
        
        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                M5StackCoreS3Board* board = (M5StackCoreS3Board*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeIli9342Display() {
        ESP_LOGI(TAG, "Init IlI9342");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_3;
        io_config.dc_gpio_num = GPIO_NUM_35;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        aw9523_->ResetIli9342();

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

     void InitializeCamera() {
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_bus_,
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);
    }

public:
    M5StackCoreS3Board() {
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAxp2101();
        InitializeAw9523();
        InitializePy32L020();
        I2cDetect();
        InitializeSpi();
        InitializeIli9342Display();
        InitializeCamera();
        InitializeFt6336TouchPad();
        // StackChan servo MCP: enable voice-controlled head movement via cloud AI
        servo_mcp_ = new ServoMcpTool();
        servo_mcp_->Initialize();
        // LED MCP tool: voice-controlled RGB light strip
        RegisterLedMcpTool();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CoreS3AudioCodec audio_codec(i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_AW88298_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        // Keep power-save timer enabled on both USB and battery:
        // USB supply still gets 120s screen dimming; shutdown is gated by
        // the VBUS check in OnShutdownRequest so it won't power-cycle.
        power_save_timer_->SetEnabled(true);
        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    virtual Backlight *GetBacklight() override {
        static CustomBacklight backlight(pmic_);
        return &backlight;
    }
};

DECLARE_BOARD(M5StackCoreS3Board);
