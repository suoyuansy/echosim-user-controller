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
//      echo.controller.userdefined 统一以 TARGET_ACC_CONTROL 加速度模式下发，
//      同时驱动可视化、日志；进入终点圈后持续制动，圈内停稳保持达到配置
//      时长后退出循环，满足任务完成证据的连续保持要求。
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
    // 跟踪计算 -> 终点接近限速 -> 构造控制消息 -> 发布 -> 可视化 -> 日志 ->
    // 终点捕获/持续制动。正常在圈内停稳并保持完成后返回 0。
    int run(const TaskConfig& config, const Path& path) const;
};
