#include "app/TaskConfig.h"

#include <algorithm>
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
    const std::array<std::filesystem::path, 5> candidates = {
        project_root / "third_party" / "echosim" / "Moon2",
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

    // 兜底：优先返回项目内的 third_party 地形路径，错误时信息更贴近当前工程。
    return std::filesystem::absolute(candidates[0]);
}

// 为没有途经点的单段任务生成与起终点相匹配的地形扫描范围。按坐标跨度
// 外扩 30%，避免 A* 因只允许贴直线搜索而错过安全绕行通道。
void setDirectRouteScanBounds_(TaskConfig& config)
{
    const double min_x = std::min(config.start.x, config.goal.x);
    const double max_x = std::max(config.start.x, config.goal.x);
    const double min_y = std::min(config.start.y, config.goal.y);
    const double max_y = std::max(config.start.y, config.goal.y);
    const double margin_x = std::max(1.0, (max_x - min_x) * 0.30);
    const double margin_y = std::max(1.0, (max_y - min_y) * 0.30);
    config.scan_bounds = {min_x - margin_x, min_y - margin_y,
                          max_x + margin_x, max_y + margin_y};
}

} // namespace

// 创建默认任务配置（任务级参数唯一入口）。返回完整 TaskConfig：坐标为世界
// 坐标（米），角度为弧度。
// 参数分层约定（模块隔离）：各模块参数的默认值与调参入口在模块自己的头文件
// （TerrainCostmap.h / GlobalPlanner.h / PathTracker.h），
// 本函数不再逐项赋值；仅当某次任务需要覆盖个别模块参数时，才在这里对
// config.terrain / config.planner / config.tracking 的
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
    config.output_directory = project_root / "output" / "test4";
    // Test4 点集包围盒整体扩大 30%，栅格分辨率固定为 1 m。
    config.start = {517.561, 3314.519, 0.0};//517.561, 3314.519
    config.goal = {299.200, 2659.611, 0.0};
    config.goal_z = -91.662;
    config.waypoints = {{954.284, 3423.670, 0.0},
                        {1172.645, 2987.065, 0.0},
                        {954.284, 2550.460, 0.0}};
    config.scan_bounds = {161.8, 2458.5, 1309.5, 3515.7};
    // 日志表明车辆在此附近失去正常地面接触后的制动/转向响应。该圆不改变
    // 原始代价值，只作为人工硬障碍，迫使 A* 从安全侧绕行。
    config.hard_obstacle_circles = {{876.0, 3244.0, 30.0}};
    // 当前 Test4/Test6 的软代价只用于避开高坡度区域；粗糙度仍只作为
    // 独立的物理硬障碍条件，不参与可通行栅格之间的路径排序。
    config.terrain.slope_weight = 1.0;
    config.terrain.roughness_weight = 0.0;

    // 以下均为任务/运行期参数；模块参数默认值见各模块头文件，不在此重复。
    // 实际轨迹按仿真时间每 1 秒采样、每 10 次成功状态接收打一条日志
    // （含义见头文件注释）。
    config.visualization_sample_interval_sec = 1.0;
    config.control_log_interval = 10;
    return config;
}

TaskConfig makeTaskConfigForTest(int test_number)
{
    if (test_number != 2 && test_number != 4 && test_number != 5
        && test_number != 6 && test_number != 7)
    {
        throw std::runtime_error(
            "this sy build supports Test2, Test4, Test5, Test6 and Test7 only");
    }

    TaskConfig config = makeDefaultTaskConfig();
    config.test_number = test_number;
    config.output_directory = config.output_directory.parent_path()
        / ("test" + std::to_string(test_number));
    if (test_number == 2)
    {
        // Test2.tst：无途经点，直接从任务起点驶向调试终点。
        // 任务要求的原始起点已恢复；终点暂设为此前日志中大转向前
        // 仍能正常贴踪、坡度约 7.95° 的安全路径点（原路径序号约 1634）：
        //   safe goal = (-712.334, -427.759), z ≈ -80.037 m
        // 原任务终点保留在这里，完整任务恢复时将 config.goal 改回该值：
        //   original goal = (-574.245, -505.777), z = -94.517 m
        config.start = {-2102.775, -69.172, 0.0};
        config.goal = { -574.245, -505.777, 0.0};
        config.goal_z = -94.517;
        config.waypoints.clear();
        config.hard_obstacle_circles.clear();
        // Test2 的目标位于约 40° 坡面：本任务允许坡度低于 40° 的
        // 栅格参与规划，并把坡度按 0°~40° 映射为软代价；达到 40°
        // 或更高时仍作为硬障碍，避免车辆驶上更陡区域。
        config.terrain.slope_limit_deg = 40.0;
        config.terrain.hard_slope_limit_deg = 40.0;
        setDirectRouteScanBounds_(config);
    }
    else if (test_number == 5)
    {
        // Test5.tst：无途经点，直接从起点驶向终点。
        config.start = {-3303.762, -2907.106, 0.0};
        config.goal = {-2211.956, -1051.533, 0.0};
        config.goal_z = -83.539;
        config.waypoints.clear();
        config.hard_obstacle_circles.clear();
        setDirectRouteScanBounds_(config);
    }
    else if (test_number == 6)
    {
        // lks 分支定义的短距离回归任务；这里只移植任务点，不移植
        // lks 的 PathOptimizer 或距离-速度控制逻辑。
        config.start = {-541.400, -1398.900, 0.0};
        config.goal = {-465.065, -1269.836, 0.0};
        config.goal_z = -73.944;
        config.waypoints.clear();
        config.hard_obstacle_circles.clear();

        const double min_x = std::min(config.start.x, config.goal.x);
        const double max_x = std::max(config.start.x, config.goal.x);
        const double min_y = std::min(config.start.y, config.goal.y);
        const double max_y = std::max(config.start.y, config.goal.y);
        const double margin_x = std::max(1.0, (max_x - min_x) * 0.15);
        const double margin_y = std::max(1.0, (max_y - min_y) * 0.15);
        config.scan_bounds = {min_x - margin_x, min_y - margin_y,
                              max_x + margin_x, max_y + margin_y};
    }
    else if (test_number == 7)
    {
        // 从失控点前约 50 m 的已验证正常路段起跑，直接驶向原 Test4 的 wp2。
        // 保留 Test4 的 wp1/wp2/wp3 配置，不修改整体任务链路。
        config.start = {877.0, 3294.0, -3.14159265358979323846 / 2.0};
        config.goal = {1172.645, 2987.065, 0.0};
        config.goal_z = -64.807;
        config.waypoints.clear();

        const double min_x = std::min(config.start.x, config.goal.x);
        const double max_x = std::max(config.start.x, config.goal.x);
        const double min_y = std::min(config.start.y, config.goal.y);
        const double max_y = std::max(config.start.y, config.goal.y);
        const double margin_x = std::max(60.0, (max_x - min_x) * 0.15);
        const double margin_y = std::max(60.0, (max_y - min_y) * 0.15);
        config.scan_bounds = {min_x - margin_x, min_y - margin_y,
                              max_x + margin_x, max_y + margin_y};
    }
    return config;
}
