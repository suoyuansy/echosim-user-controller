// 构造默认配置，编排规划流程和 EchoSim 运行时，并统一处理异常。
#include "EchoSimRuntime.h"
#include "PlanningPipeline.h"
#include "TaskConfig.h"

#include <iomanip>
#include <iostream>

namespace
{
// 输出任务、调试模式和控制日志周期，便于确认当前运行配置。
void printTaskConfiguration(const TaskConfig& config)
{
    std::cout << std::fixed << std::setprecision(3)
              << "[task] configuration loaded" << std::endl
              << "[task] start=(x=" << config.start.x
              << ",y=" << config.start.y
              << ",yaw_rad=" << config.start.yaw << ')' << std::endl
              << "[task] goal=(x=" << config.goal.x
              << ",y=" << config.goal.y
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
        const TaskConfig config = makeDefaultTaskConfig();
        printTaskConfiguration(config);

        // 初始化 EchoSim 消息系统，随后等待车辆状态并发布控制量。
        EchoSimRuntime runtime;
        runtime.initialize();

        // 构建或读取地形代价地图，并规划全局路径。
        PlanningPipeline pipeline;
        const Path path = pipeline.buildPath(config);

        // 进入车辆控制循环，直到任务完成或运行时返回错误。
        return runtime.run(config, path);
    }
    catch (const std::exception& error)
    {
        std::cerr << "[error] " << error.what() << std::endl;
        return -1;
    }
}
