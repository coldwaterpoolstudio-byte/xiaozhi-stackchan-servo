#include "servo_mcp_tool.h"
#include "scservo.h"
#include "mcp_server.h"
#include <esp_log.h>
#include <memory>

static const char* TAG = "ServoMcp";

ServoMcpTool::ServoMcpTool() {}

ServoMcpTool::~ServoMcpTool() {}

void ServoMcpTool::Initialize() {
    ESP_LOGI(TAG, "Initializing SCS0009 servos on UART1 (GPIO6 TX, GPIO7 RX)");

    // Official StackChan zero positions: yaw=460, pitch=620
    yaw_servo_ = std::make_unique<ScServo>(UART_NUM_1, GPIO_NUM_6, GPIO_NUM_7, 1, 460);
    pitch_servo_ = std::make_unique<ScServo>(UART_NUM_1, GPIO_NUM_6, GPIO_NUM_7, 2, 620);

    yaw_servo_->init();
    pitch_servo_->init();

    yaw_servo_->setTorque(true);
    pitch_servo_->setTorque(true);

    // Move to home (center) position
    vTaskDelay(pdMS_TO_TICKS(100));
    yaw_servo_->moveWithSpeed(0, 300);   // yaw center
    pitch_servo_->moveWithSpeed(0, 300); // pitch center

    // Register MCP tools (same names as official StackChan firmware)
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.robot.get_head_angles",
        "Returns current yaw/pitch in degrees. yaw(-128 to 128, negative=your left), pitch(0 to 90, 90=up).",
        PropertyList(),
        [this](const PropertyList& properties) -> ToolResult {
            (void)properties;
            int yaw = yaw_servo_ ? yaw_servo_->getCurrentAngle() / 10 : 0;
            int pitch = pitch_servo_ ? pitch_servo_->getCurrentAngle() / 10 : 0;
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"yaw\": %d, \"pitch\": %d}", yaw, pitch);
            return ReturnValue(std::string(buf));
        });

    mcp_server.AddTool(
        "self.robot.set_head_angles",
        "Move head to target angles. Use for natural conversation: small yaw shifts while listening, slight pitch up when thinking, return to center when idle. Extreme values only when user explicitly says 'look up/down/left/right'. yaw(-128 to 128, negative=your left), pitch(0 to 90, 90=up), speed(100-1000, default 150). Pass -9999 to leave an axis unchanged.",
        PropertyList({
            Property("yaw", kPropertyTypeInteger, -9999, -9999, 128),
            Property("pitch", kPropertyTypeInteger, -9999, -9999, 90),
            Property("speed", kPropertyTypeInteger, 150, 100, 1000),
        }),
        [this](const PropertyList& properties) -> ToolResult {
            int yaw = properties["yaw"].value<int>();
            int pitch = properties["pitch"].value<int>();
            int speed = properties["speed"].value<int>();
            if (yaw != -9999 && yaw_servo_) {
                yaw_servo_->moveWithSpeed(yaw * 10, speed);
            }
            if (pitch != -9999 && pitch_servo_) {
                pitch_servo_->moveWithSpeed(pitch * 10, speed);
            }
            return ReturnValue(true);
        });

    ESP_LOGI(TAG, "Servo MCP tools registered");
}

ToolResult ServoMcpTool::HandleGetHeadAngles(const PropertyList& properties) {
    return std::unexpected(std::string("not used"));
}

ToolResult ServoMcpTool::HandleSetHeadAngles(const PropertyList& properties) {
    return std::unexpected(std::string("not used"));
}
