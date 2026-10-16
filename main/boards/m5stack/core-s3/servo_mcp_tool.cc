#include "servo_mcp_tool.h"
#include <esp_log.h>
#include <cstdio>

static const char* TAG = "ServoMcp";

// StackChan servo IDs: yaw=1, pitch=2
static constexpr int kYawServoId = 1;
static constexpr int kPitchServoId = 2;

ServoMcpTool::ServoMcpTool() {}

ServoMcpTool::~ServoMcpTool() {}

void ServoMcpTool::Initialize() {
    // UART1: TX=GPIO6, RX=GPIO7, 1Mbps (StackChan servo bus)
    yaw_servo_ = std::make_unique<ScServo>(UART_NUM_1, GPIO_NUM_6, GPIO_NUM_7, kYawServoId);
    pitch_servo_ = std::make_unique<ScServo>(UART_NUM_1, GPIO_NUM_6, GPIO_NUM_7, kPitchServoId);
    yaw_servo_->init();
    pitch_servo_->init();
    yaw_servo_->setTorque(true);
    pitch_servo_->setTorque(true);

    // Move to home position
    yaw_servo_->moveWithSpeed(0, 300);
    pitch_servo_->moveWithSpeed(450, 300);  // pitch center ~45deg

    auto& mcp_server = McpServer::GetInstance();

    // get_head_angles
    mcp_server.AddTool(
        "self.robot.get_head_angles",
        "Returns current yaw/pitch in degrees. Neutral position is {yaw:0, pitch:45}. "
        "Yaw: negative=left, positive=right. Pitch: 0=down, 90=up.",
        std::vector<Property>{},
        [this](const PropertyList& properties) -> ReturnValue {
            int yaw = yaw_servo_->getCurrentAngle() / 10;
            int pitch = pitch_servo_->getCurrentAngle() / 10;
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"yaw\": %d, \"pitch\": %d}", yaw, pitch);
            ESP_LOGI(TAG, "get_head_angles: %s", buf);
            return std::string(buf);
        });

    // set_head_angles
    mcp_server.AddTool(
        "self.robot.set_head_angles",
        "Move the robot's head. GUIDELINES: "
        "1. For natural interaction, stay within yaw +/-45, pitch 20~70. "
        "2. Only use extreme values if user explicitly asks to look far away/behind. "
        "3. Ranges: yaw(-128 to 128, negative=your left), pitch(0 to 90, 90=up). "
        "4. speed(100-1000, 150=natural, 500=fast). "
        "5. Use -9999 for yaw or pitch to leave that axis unchanged.",
        PropertyList({
            Property("yaw", kPropertyTypeInteger, -9999, -9999, 128),
            Property("pitch", kPropertyTypeInteger, -9999, -9999, 90),
            Property("speed", kPropertyTypeInteger, 150, 100, 1000),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int yaw = properties["yaw"].value<int>();
            int pitch = properties["pitch"].value<int>();
            int speed = properties["speed"].value<int>();

            ESP_LOGI(TAG, "set_head_angles: yaw=%d pitch=%d speed=%d", yaw, pitch, speed);

            if (yaw != -9999) {
                yaw_servo_->moveWithSpeed(yaw * 10, speed);
            }
            if (pitch != -9999) {
                pitch_servo_->moveWithSpeed(pitch * 10, speed);
            }
            return true;
        });

    ESP_LOGI(TAG, "Servo MCP tools registered");
}

ToolResult ServoMcpTool::HandleGetHeadAngles(const PropertyList& properties) {
    return std::unexpected(std::string("not used"));
}

ToolResult ServoMcpTool::HandleSetHeadAngles(const PropertyList& properties) {
    return std::unexpected(std::string("not used"));
}
