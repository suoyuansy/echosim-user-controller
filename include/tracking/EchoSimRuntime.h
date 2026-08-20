#pragma once

#include "common/PathTypes.h"
#include "app/TaskConfig.h"

// 封装 EchoSim 消息系统初始化、车辆状态订阅、控制发布和停车收尾。
//
// 职责（tracking 模块的运行时层）：
//   1. initialize(): 调用 sim_msg::Initialize 建立与 EchoSim 消息总线的连接，
//      必须在发布/订阅之前完成；
//   2. run(): 主控制循环——订阅 ego 状态主题 echo.vehicle.states.Ego，
//      逐帧调用 PathTracker 计算转角与限速，经控制主题
//      echo.controller.userdefined 下发（距离/速度模式或 LQR 加速度模式），
//      同时驱动可视化、日志，并在到点后发布停车指令退出循环。
// 单位约定：算法内部全部使用米/弧度/米每秒，与 EchoSim 的度、km/h 换算
// 只在本模块（EchoSimRuntime.cpp）的收发边界处进行。
class EchoSimRuntime
{
public:
    // 在地形规划前初始化 EchoSim 消息系统，保持旧版运行顺序。
    // 失败（总线未就绪、端口被占等）时抛出 std::runtime_error。
    void initialize() const;

    // 运行路径跟踪控制循环，失败时抛出 std::runtime_error。
    // 输入：任务配置（含跟踪参数、起终点、挡位、日志间隔）与参考路径
    // （世界系，米/弧度）。循环以 0.1 s 周期运行：等状态 -> 读状态 ->
    // 跟踪计算 -> 限速门控 -> 构造控制消息 -> 发布 -> 可视化 -> 日志 ->
    // 到点判定/停车。正常到点停车返回 0。
    int run(const TaskConfig& config, const Path& path) const;
};
