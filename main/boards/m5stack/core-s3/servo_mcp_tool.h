#ifndef _SERVO_MCP_TOOL_H_
#define _SERVO_MCP_TOOL_H_

#include "mcp_server.h"
#include "scservo.h"
#include <memory>

// Servo MCP tools for StackChan: expose head movement to cloud AI via MCP
class ServoMcpTool {
public:
    ServoMcpTool();
    ~ServoMcpTool();

    // Initialize servos and register MCP tools
    void Initialize();

    // Access servos for board-level init
    ScServo* yawServo() { return yaw_servo_.get(); }
    ScServo* pitchServo() { return pitch_servo_.get(); }

private:
    ToolResult HandleGetHeadAngles(const PropertyList& properties);
    ToolResult HandleSetHeadAngles(const PropertyList& properties);

    std::unique_ptr<ScServo> yaw_servo_;
    std::unique_ptr<ScServo> pitch_servo_;
};

#endif  // _SERVO_MCP_TOOL_H_
