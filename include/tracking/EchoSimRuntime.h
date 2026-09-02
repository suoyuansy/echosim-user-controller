#pragma once

#include "common/PathTypes.h"
#include "app/TaskConfig.h"

// 封装 EchoSim 消息系统初始化、车辆状态订阅、控制发布和停车收尾。
class EchoSimRuntime
{
public:
    // 在地形规划前初始化 EchoSim 消息系统，保持旧版运行顺序。
    void initialize() const;

    // 运行路径跟踪控制循环，失败时抛出 std::runtime_error。
    int run(const TaskConfig& config, const RoutePlan& route) const;
};

