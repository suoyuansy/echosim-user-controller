#include "app/TaskConfig.h"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace
{

// 从当前工作目录向上最多 8 层查找项目根目录，返回其绝对路径。
// 判定标记：CMake 工程根 = 同时存在 CMakeLists.txt 和 src/app/TaskConfig.cpp；
// 兼容旧单文件工程根 = 存在 UserControllerTest.vcxproj。这样无论从
// Cplusplus_CMake/ 内哪个子目录启动 UserController.exe 都能定位 output/ 与地形。
// 找不到标记时退回当前工作目录绝对路径（后续建图/输出大概率失败，便于暴露问题）。
std::filesystem::path findProjectRoot_()
{
    // 以当前工作目录的绝对路径作为起始候选目录。
    std::filesystem::path candidate = std::filesystem::absolute(
        std::filesystem::current_path());
    // 逐层向上（parent_path），最多 8 层且候选非空，防止越过磁盘根后异常。
    for (int level = 0; level < 8 && !candidate.empty(); ++level)
    {
        // 命中 CMake 根标记（双文件同时存在）或旧工程标记即认为找到项目根。
        if ((std::filesystem::exists(candidate / "CMakeLists.txt")
             && std::filesystem::exists(candidate / "src" / "app" / "TaskConfig.cpp"))
            || std::filesystem::exists(candidate / "UserControllerTest.vcxproj"))
        {
            return candidate;
        }
        // 未命中则退到父目录继续查找。
        candidate = candidate.parent_path();
    }
    // 兜底：找不到任何标记时返回当前目录绝对路径。
    return std::filesystem::absolute(std::filesystem::current_path());
}

// 读取环境变量并转为路径；变量未设置或为空字符串时返回空路径（表示未覆盖）。
std::filesystem::path environmentPath_(const char* name)
{
    const char* value = std::getenv(name);
    // 只有非空取值才视为有效覆盖，避免把空串当成合法地形根。
    if (value != nullptr && value[0] != '\0')
        return std::filesystem::path(value);
    return {};
}

// 判断目录是否像地形切片根：至少含一个 level_* 分辨率切片目录
// （level_1m / level_10m / level_100m / level_1000m）即认为有效。
bool looksLikeTerrainRoot_(const std::filesystem::path& path)
{
    // 任一分辨率目录存在即可通过；1 m 建图主要使用 level_1m / level_10m。
    return std::filesystem::exists(path / "level_1m")
        || std::filesystem::exists(path / "level_10m")
        || std::filesystem::exists(path / "level_100m")
        || std::filesystem::exists(path / "level_1000m");
}

// 自动发现 Moon2 地形切片根目录（环境变量覆盖 + 候选路径探测）。
// 输入：项目根目录；输出：地形根绝对路径。查找优先级：
// 1) USER_CONTROLLER_TERRAIN_ROOT / ECHOSIM_TERRAIN_ROOT 环境变量（前者优先）；
// 2) 按常见工作区布局逐个探测 4 个候选路径，第一个通过 looksLikeTerrainRoot_
//    校验的生效；
// 3) 全部落空时返回标准布局候选（项目根兄弟目录下的
//    EchoSim/data/project/Onsite8_B/Heightmap/Moon2），由后续 TerrainService
//    初始化报错来暴露路径问题。
std::filesystem::path findTerrainRoot_(const std::filesystem::path& project_root)
{
    // 第 1 优先级：环境变量覆盖（控制器专用变量优先于通用 ECHOSIM 变量）。
    for (const char* variable :
         {"USER_CONTROLLER_TERRAIN_ROOT", "ECHOSIM_TERRAIN_ROOT"})
    {
        const std::filesystem::path override_path = environmentPath_(variable);
        // 覆盖值不做 level_* 内容校验，用户显式指定即信任该路径。
        if (!override_path.empty())
            return std::filesystem::absolute(override_path);
    }

    // 第 2 优先级：候选路径探测。project_root 可能是 Cplusplus_CMake/
    // （此时地形在兄弟目录 EchoSim/ 下），也可能是仓库根，两种布局各给两条候选。
    const std::filesystem::path project_parent = project_root.parent_path();
    const std::array<std::filesystem::path, 4> candidates = {
        project_root / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
        project_root / "EchoSim" / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
        project_parent / "EchoSim" / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
        project_parent / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
    };

    // 依次校验候选路径，返回第一个含 level_* 切片目录的候选。
    for (const std::filesystem::path& candidate : candidates)
    {
        if (looksLikeTerrainRoot_(candidate))
            return std::filesystem::absolute(candidate);
    }

    // 兜底：返回标准工作区布局（Cplusplus_CMake 的兄弟 EchoSim/ 目录）。
    return std::filesystem::absolute(candidates[2]);
}

} // namespace

// 创建默认任务配置（任务级参数唯一入口）。返回完整 TaskConfig：坐标为世界
// 坐标（米），角度为弧度。
// 参数分层约定（模块隔离）：各模块参数的默认值与调参入口在模块自己的头文件
// （TerrainCostmap.h / GlobalPlanner.h / PathOptimizer.h / PathTracker.h），
// 本函数不再逐项赋值；仅当某次任务需要覆盖个别模块参数时，才在这里对
// config.terrain / config.planner / config.optimizer / config.tracking 的
// 对应字段显式赋值。本函数只负责任务级配置：起终点、地形根、输出目录、
// 扫描走廊与运行期开关。
TaskConfig makeDefaultTaskConfig()
{
    // 先定位项目根：它决定 output/ 调试产物目录和地形根的候选路径。
    const std::filesystem::path project_root = findProjectRoot_();

    TaskConfig config;
    // 起点、终点、目标高度与必经点由启动菜单选中的预置路线覆盖。
    // 地形根：环境变量优先，其次候选路径自动发现；输出固定在项目根下 output/。
    config.terrain_root = findTerrainRoot_(project_root);
    config.output_directory = project_root / "output";
    // 扫描走廊：覆盖 Test1–Test5 全部起终点与必经途经点（x ∈ [-3304, 1173]、
    // y ∈ [-3016, 3424]）并各留约 300 m 余量，避免 1 m 分辨率下全图 10 km 构建过慢。
    config.scan_bounds = {-3600.0, -3350.0, 1500.0, 3750.0};

    // 以下均为任务/运行期参数；模块参数默认值见各模块头文件，不在此重复。
    // 实际轨迹采样间隔 2 秒、每 10 次成功控制发布打一条日志（含义见头文件注释）。
    config.visualization_sample_interval_sec = 2.0;
    config.control_log_interval = 10;
    return config;
}

TaskConfig makeTaskConfigForTest(int test_number)
{
    if (test_number < 1 || test_number > 6)
        throw std::runtime_error("test number must be between 1 and 6");

    TaskConfig config = makeDefaultTaskConfig();
    switch (test_number)
    {
    case 1:
        config.start = {-901.787, -3016.257, 0.0};
        config.goal = {-465.065, -1269.836, 0.0};
        config.goal_z = -73.944;
        break;
    case 2:
        config.start = {-2102.775, -69.172, 0.0};
        config.goal = {-574.245, -505.777, 0.0};
        config.goal_z = -94.517;
        break;
    case 3:
        config.start = {-3085.401, 1568.098, 0.0};
        config.goal = {-1884.414, 2441.308, 0.0};
        config.goal_z = -84.211;
        config.waypoints = {{-2430.317, 2004.703, 0.0}};
        // 降低软地形代价对路径长度的放大，避免 A* 为绕开非障碍高代价格
        // 在必经点引导走廊末端立即折返；硬障碍与两格全局余量不变。
        config.planner.cost_weight = 2.0;
        // Test3 的可行走廊在首段转角处较窄。全局搜索仍保留两格余量；
        // 优化阶段由一格中心线余量叠加完整车体矩形碰撞检查，避免重复膨胀
        // 阻止本来具有车体净空的曲率连续圆弧。
        config.optimizer.clearance_margin_cells = 1;
        break;
    case 4:
        config.start = {517.561, 3314.519, 0.0};
        config.goal = {299.200, 2659.611, 0.0};
        config.goal_z = -91.662;
        config.waypoints = {
            {954.284, 3423.670, 0.0},
            {1172.645, 2987.065, 0.0},
            {954.284, 2550.460, 0.0},
        };
        break;
    case 5:
        config.start = {-3303.762, -2907.106, 0.0};
        config.goal = {-2211.956, -1051.533, 0.0};
        config.goal_z = -83.539;
        break;
    case 6:
        config.start = {-541.400, -1398.900, 0.0};
        config.goal = {-465.065, -1269.836, 0.0};
        config.goal_z = -73.944;
        break;
    default:
        break;
    }
    return config;
}
