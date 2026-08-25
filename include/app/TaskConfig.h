#pragma once

#include "optimization/PathOptimizer.h"
#include "planning/GlobalPlanner.h"
#include "tracking/PathTracker.h"

#include <cstddef>
#include <filesystem>
#include <vector>

// 文件功能：集中定义任务级配置与三个模块配置的聚合结构。
// 参数分层（模块隔离）约定：
//   - 模块参数的默认值与调参入口在各模块自己的头文件（TerrainCostmap.h /
//     GlobalPlanner.h / PathOptimizer.h / PathTracker.h），由模块所有者维护；
//   - 本结构聚合四个模块配置子结构（terrain / planner / optimizer / tracking），
//     makeDefaultTaskConfig() 中不再逐项赋模块参数，仅当某次任务需要覆盖
//     个别模块参数时才显式赋值；
//   - 任务级参数（起终点、地形根、输出目录、扫描走廊、运行期开关）仍在
//     makeDefaultTaskConfig() 中设置，是任务参数的唯一入口。
// 模块间只通过 common/PathTypes.h 交换数据；本结构只含配置，不包含任何运行期状态。
struct TaskConfig
{
    // 起终点平面位姿为仿真世界坐标：x/y 单位米，yaw 单位弧度；目标 Z
    // 单独保存，用于复现评测器 distance_to_goal 的三维欧氏距离。
    // 默认值对应 Moon2 场景 Test1；切换测试时需同步修改（Test3/Test4 还有必经途经点）。
    Pose2D start; // 任务起点坐标和航向角。
    Pose2D goal; // 任务终点坐标和航向角。
    double goal_z = 0.0; // 任务契约中的目标世界坐标 Z，单位为米。
    std::vector<Pose2D> waypoints; // 按任务定义顺序必须经过的中间点。
    // 合法地形根必须包含 level_1m / level_10m / level_100m / level_1000m
    // 切片目录；可被 USER_CONTROLLER_TERRAIN_ROOT / ECHOSIM_TERRAIN_ROOT 环境变量覆盖。
    std::filesystem::path terrain_root; // Moon 地形切片根目录。
    // 默认为 <项目根>/output：存放代价地图磁盘缓存、global_path*.txt、
    // optimized_path*.txt、terrain_preview.png 等调试产物（比赛模式不写文件）。
    std::filesystem::path output_directory; // 缓存和结果输出目录。
    // 代价地图构建范围（世界坐标，米），成员为 min_x/min_y/max_x/max_y。
    // 取"覆盖 Test1–Test5 全部起终点与途经点 + 约 300 m 余量"的走廊，
    // 避免以 1 m 分辨率构建全图（约 10 km）导致启动过慢。
    TerrainBounds scan_bounds; // 固定地形查询边界。
    // 以下四个子结构分别归三个模块所有（terrain/planner 归模块 1 规划，
    // optimizer 归模块 2 优化，tracking 归模块 3 跟踪），此处仅做聚合。
    // 各字段含义与默认取值依据见对应模块头文件；跨模块一致性约束：
    // optimizer 的 wheelbase_m / max_front_wheel_angle_rad 与
    // tracking.geometry 的同名字段必须同源（设计文档要求规划与控制共用
    // 同一套车辆参数），PlanningPipeline 启动时有不一致告警兜底。
    TerrainCostmapConfig terrain; // 地形代价地图配置。
    GlobalPlannerConfig planner; // 全局规划器配置。
    PathOptimizerConfig optimizer; // 路径优化器配置。
    TrackingConfig tracking; // 路径跟踪器和车辆配置。
    // 运行时用 ego 实际位姿与 config.start 对比做起点校验（单位分别为米/弧度），
    // 超差说明任务配置与场景不匹配，提前暴露配置错误。
    double start_position_tolerance_m = 2.0; // 起点位置校验容差。
    double start_path_snap_radius_m = 32.0; // 起点落在膨胀障碍区时寻找安全路径入口的最大半径。
    double start_yaw_tolerance_rad = 15.0 * 3.14159265358979323846 / 180.0; // 起点航向校验容差。
    bool enable_debug_output = true; // 调试模式开关；比赛模式设为 false 时不输出任何文件。
    // 实际轨迹的采样间隔（秒）：TrajectoryVisualizer 按此间隔从控制循环记录
    // 一个实际位姿点，用于 actual_path.txt 和最终对比图；过小则轨迹点过密。
    double visualization_sample_interval_sec = 2.0; // 实际轨迹采样间隔。
    // 每成功发布 N 次控制指令打印一条跟踪日志（stdout），用于粗略观察跟踪误差。
    std::size_t control_log_interval = 10; // 成功发布控制后的日志周期。
    // 对应 sim_msg::Control_GEAR_MODE 枚举值（4 = DRIVE 前进挡），与 SDK 枚举保持一致。
    int gear_mode = 4; // EchoSim 前进挡枚举值。
};

// 创建默认任务配置，任务级参数的唯一入口。
// 返回值：完整 TaskConfig（含自动发现的项目根/地形根）；坐标单位米、角度单位弧度。
// 需要改任务行为（起终点、扫描走廊、输出开关等）时只改 TaskConfig.cpp 中该
// 函数的实现；各模块算法参数（阈值、增益、速度门控等）在模块自己的头文件里
// 调，不要在本函数或其他模块散落硬编码。
TaskConfig makeDefaultTaskConfig();

// 按启动菜单选择的 Test 编号加载预置路线表（起点、终点和必经点）。
// 编号无效时抛出异常。
TaskConfig makeTaskConfigForTest(int test_number);
