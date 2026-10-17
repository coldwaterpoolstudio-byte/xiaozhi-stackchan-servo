#pragma once
#include <memory>
#include "mcp_server.h"

class ScServo;

class ServoMcpTool {
public:
    ServoMcpTool();
    ~ServoMcpTool();
    void Initialize();
    ScServo* GetYawServo() { return yaw_servo_.get(); }
    ScServo* GetPitchServo() { return pitch_servo_.get(); }

private:
    std::unique_ptr<ScServo> yaw_servo_;
    std::unique_ptr<ScServo> pitch_servo_;

    ToolResult HandleGetHeadAngles(const PropertyList& properties);
    ToolResult HandleSetHeadAngles(const PropertyList& properties);
};
