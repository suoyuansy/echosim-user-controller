#pragma once

#include "GlobalPlanner.h"
#include "PathTracker.h"

#include <cstddef>
#include <filesystem>

// 文件功能：集中定义 EchoSim 任务、地形、规划、跟踪和输出配置。
// 人工修改任务参数时，只修改 makeDefaultTaskConfig()。
struct TaskConfig
{
    Pose2D start; // 任务起点坐标和航向角。
    Pose2D goal; // 任务终点坐标和航向角。
    std::filesystem::path terrain_root; // Moon 地形切片根目录。
    std::filesystem::path output_directory; // 缓存和结果输出目录。
    TerrainBounds scan_bounds; // 固定地形查询边界。
    TerrainCostmapConfig terrain; // 地形代价地图配置。
    GlobalPlannerConfig planner; // 全局规划器配置。
    TrackingConfig tracking; // 路径跟踪器和车辆配置。
    double start_position_tolerance_m = 2.0; // 起点位置校验容差。
    double start_yaw_tolerance_rad = 15.0 * 3.14159265358979323846 / 180.0; // 起点航向校验容差。
    bool enable_debug_output = true; // 调试模式开关；比赛模式设为 false 时不输出任何文件。
    double visualization_sample_interval_sec = 2.0; // 实际轨迹采样间隔。
    std::size_t control_log_interval = 200; // 成功发布控制后的日志周期。
    int gear_mode = 4; // EchoSim 前进挡枚举值。
};

// 创建默认任务配置，人工任务参数的唯一入口。
TaskConfig makeDefaultTaskConfig();
