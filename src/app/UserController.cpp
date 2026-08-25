// 构造默认配置，编排规划流程和 EchoSim 运行时，并统一处理异常。
// 文件功能：控制器程序入口（main）。按固定顺序串联四个阶段：
//   1) makeDefaultTaskConfig()  生成任务配置（起终点/地形根/各模块可调参数）；
//   2) EchoSimRuntime::initialize()  连接 EchoSim 消息总线（订阅 ego 状态 + 准备控制发布）；
//   3) PlanningPipeline::buildPath()  代价地图加载或构建 -> 全局规划 -> 路径优化；
//   4) EchoSimRuntime::run()  跟踪控制循环，到达终点后持续驻车，直到仿真结束。
// 任何阶段抛出的异常在 main 末尾统一捕获，打印 [error] 后以 -1 退出。
#include "tracking/EchoSimRuntime.h"
#include "app/PlanningPipeline.h"
#include "app/TaskConfig.h"

#include <iomanip>
#include <iostream>

namespace
{
// 输出任务、调试模式和控制日志周期，便于确认当前运行配置。
// 输入：TaskConfig（只读）；输出：固定 3 位小数的任务摘要到 stdout，包括
// 起点/终点（米、弧度）、地形根、输出目录、调试开关和控制日志周期。无返回值。
// 在连接消息总线之前调用，起终点或地形根配置错误可第一时间发现。
void printTaskConfiguration(const TaskConfig& config)
{
    std::cout << std::fixed << std::setprecision(3)
              << "[task] configuration loaded" << std::endl
              << "[task] start=(x=" << config.start.x
              << ",y=" << config.start.y
              << ",yaw_rad=" << config.start.yaw << ')' << std::endl
              << "[task] goal=(x=" << config.goal.x
              << ",y=" << config.goal.y
              << ",z=" << config.goal_z
              << ",yaw_rad=" << config.goal.yaw << ')' << std::endl
              << "[task] terrain_root=" << config.terrain_root.string() << std::endl
              << "[task] output_directory=" << config.output_directory.string() << std::endl
              << "[task] debug_output="
              << (config.enable_debug_output ? "enabled" : "disabled")
              << " tracker_log_interval=" << config.control_log_interval
              << std::endl;
}
} // namespace

int main()
{
    try
    {
        // 创建任务配置
        // 阶段 1/4：makeDefaultTaskConfig() 是所有可调参数的唯一入口，内部完成
        // 项目根（从工作目录向上最多 8 层）与 Moon2 地形根的自动发现，并给出
        // 代价地图/规划器/优化器/跟踪器的全部默认参数。
        const TaskConfig config = makeDefaultTaskConfig();
        // 先打印配置摘要再进入仿真交互，配置错误可立即暴露。
        printTaskConfiguration(config);

        // 初始化 EchoSim 消息系统，随后等待车辆状态并发布控制量。
        // 阶段 2/4：初始化消息总线（sim_msg 初始化 + ego 状态话题订阅）。
        // 控制器应在 EchoSim 启动测试之前运行，保证规划完成时能立即收到车辆状态。
        EchoSimRuntime runtime;
        runtime.initialize();

        // 构建或读取地形代价地图，并规划全局路径。
        // 阶段 3/4：规划流水线 = 读取磁盘缓存或按 1 m 分辨率构建代价地图
        // -> 双向 A* 全局规划 -> 视线捷径/平滑/尖角圆弧化优化 -> 调试文件输出。
        // 返回的 Path 为世界坐标（米/弧度），作为跟踪阶段的参考路径。
        PlanningPipeline pipeline;
        const Path path = pipeline.buildPath(config);

        // 进入车辆控制循环，直到任务完成或运行时返回错误。
        // 阶段 4/4：run() 阻塞执行跟踪控制循环（默认 Pure Pursuit + 速度门控），
        // 周期性发布控制指令，直到在终点容差内停车或运行时错误；返回值作为进程退出码。
        return runtime.run(config, path);
    }
    catch (const std::exception& error)
    {
        // 统一异常兜底：各阶段失败（消息总线初始化、地形根发现、建图、规划、
        // 跟踪）都以 std::exception 抛出；打印到 stderr 后返回 -1，便于外部脚本判定失败。
        std::cerr << "[error] " << error.what() << std::endl;
        return -1;
    }
}
