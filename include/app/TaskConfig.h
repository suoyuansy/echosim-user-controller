#pragma once

#include "planning/GlobalPlanner.h"
#include "tracking/PathTracker.h"

#include <cstddef>
#include <filesystem>
#include <vector>

// 文件功能：集中定义 EchoSim 任务、地形、规划、跟踪和输出配置。
// 人工修改任务参数时，只修改 makeDefaultTaskConfig()。
struct TaskConfig
{
    int test_number = 4;
    Pose2D start; // 任务起点坐标和航向角。
    Pose2D goal; // 任务终点坐标和航向角。
    double goal_z = -91.662;
    std::vector<Pose2D> waypoints;
    std::filesystem::path terrain_root; // Moon 地形切片根目录。
    std::filesystem::path output_directory; // 缓存和结果输出目录。
    TerrainBounds scan_bounds; // 固定地形查询边界。
    TerrainCostmapConfig terrain; // 地形代价地图配置。
    std::vector<HardObstacleCircle> hard_obstacle_circles; // 人工禁行区域。
    GlobalPlannerConfig planner; // 全局规划器配置。
    TrackingConfig tracking; // 路径跟踪器和车辆配置。
    double start_position_tolerance_m = 2.0; // 起点位置校验容差。
    double start_yaw_tolerance_rad = 15.0 * 3.14159265358979323846 / 180.0; // 起点航向校验容差。
    bool enable_debug_output = true; // 调试模式开关；比赛模式设为 false 时不输出任何文件。
    bool enable_speed_visualization = true; // 速度调试开关；同时显示窗口并输出 TXT。
    double visualization_sample_interval_sec = 1.0; // 仿真时间采样间隔。
    std::size_t control_log_interval = 10; // 有效车辆状态接收日志周期。
    int gear_mode = 4; // EchoSim DRIVE 前进挡枚举值。
};

// 创建默认任务配置，人工任务参数的唯一入口。
TaskConfig makeDefaultTaskConfig();

// 构造指定预置任务；当前支持 Test2、Test4、Test5、短距离 Test6 和 Test7。
TaskConfig makeTaskConfigForTest(int test_number);
