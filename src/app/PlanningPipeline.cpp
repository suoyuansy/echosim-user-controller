// 实现地形缓存加载、TerrainService 建图、全局规划、路径优化和路径文件/图片输出。
// 本文件是 buildPath() 流水线的实现，完整流程：
//   1) 创建 output 目录；
//   2) 有磁盘缓存 -> 加载并校验一致性；无缓存 -> 初始化 TerrainService 在
//      扫描走廊内构建 1 m 代价地图，成功后写回缓存（仅调试模式写）；
//   3) 双向 A* 全局规划得到原始栅格路径；
//   4) PathOptimizer 视线捷径 + 平滑 + 尖角圆弧化得到可执行路径；
//   5) 调试模式下输出 global_path*.txt / optimized_path*.txt /
//      terrain_preview.png / global_path_on_costmap.png。
#include "app/PlanningPipeline.h"

#include "optimization/PathOptimizer.h"
#include "planning/GlobalPlanner.h"
#include "planning/TerrainCostmap.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace
{
// 写出不含航向角的世界坐标路径。
// 格式：每行 "x y"（米，17 位有效数字避免精度损失）；文件打开失败返回 false。
bool save_path_(const std::filesystem::path& file, const Path& path)
{
    std::ofstream output(file);
    // 打开失败（目录不存在/权限问题）直接返回 false，由调用方决定报错方式。
    if (!output)
        return false;
    // 17 位有效数字覆盖 double 全部精度，回读时不引入累积误差。
    output << std::setprecision(17);
    for (const PathPoint& point : path)
        output << point.x << ' ' << point.y << '\n';
    return output.good();
}

// 写出含航向角的世界坐标路径。
// 格式：每行 "x y yaw"（米/弧度），用于离线复现跟踪或绘图分析。
bool save_path_with_yaw_(const std::filesystem::path& file, const Path& path)
{
    std::ofstream output(file);
    // 同 save_path_：打开失败返回 false，17 位精度写出。
    if (!output)
        return false;
    output << std::setprecision(17);
    for (const PathPoint& point : path)
        output << point.x << ' ' << point.y << ' ' << point.yaw << '\n';
    return output.good();
}

// 将代价地图和全局路径绘制为灰度底图加红色路径图。
// 坐标换算：栅格 (row, col) 直接映射为像素 (col, image_row)，其中
// image_row = rows-1-row 做上下翻转（图像 y 轴向下、世界 y 轴向上），
// 使图像"上方"对应世界 y 更大的方向；1 个栅格单元 = 1 个像素。
// 颜色含义：白色 = 障碍/越界单元；灰度 = 通行代价（越亮代价越高）；
// 红色折线 = 优化后路径；蓝点 = 起点；绿点 = 终点。未编译 OpenCV 时返回 false。
bool save_path_image_(const std::filesystem::path& file,
                    const TerrainGrid& grid,
                    const Path& path,
                    const Pose2D& start,
                    const Pose2D& goal)
{
#ifdef ECHOSIM_USE_OPENCV
    // 空网格（建图未成功）无法绘制。
    if (grid.empty())
        return false;
    // 第一遍：逐单元生成灰度底图。
    cv::Mat gray(grid.rows, grid.cols, CV_8UC1);
    for (int row = 0; row < grid.rows; ++row)
    {
        const int image_row = grid.rows - 1 - row;
        for (int col = 0; col < grid.cols; ++col)
        {
            const std::size_t cell_index = grid.index(row, col);
            const bool obstacle = grid.valid[cell_index] == 0
                || grid.cost[cell_index] >= 1.0F;
            gray.at<unsigned char>(image_row, col) = obstacle
                ? 255
                : static_cast<unsigned char>(std::clamp(
                    static_cast<int>(std::lround(255.0 * grid.cost[cell_index])),
                    0, 255));
        }
    }
    cv::Mat image;
    // 转三通道以便绘制彩色路径与标记。
    cv::cvtColor(gray, image, cv::COLOR_GRAY2BGR);
    // 世界坐标（米）-> 像素：先 grid.worldToGrid 转栅格坐标，再按
    // "列 = col、行 = rows-1-row"翻转到图像坐标系（像素 y 向下）。
    auto to_image_point = [&grid](double x, double y, cv::Point& point) {
        int col = 0;
        int row = 0;
        if (!grid.worldToGrid(x, y, col, row))
            return false;
        point = {col, grid.rows - 1 - row};
        return true;
    };
    // 第二遍：路径点逐个换算为像素并连线（红色，线宽 2）。
    std::vector<cv::Point> pixels;
    for (const PathPoint& path_point : path)
    {
        cv::Point pixel;
        // 落在栅格范围外的点直接丢弃，不参与绘制。
        if (to_image_point(path_point.x, path_point.y, pixel))
            pixels.push_back(pixel);
    }
    if (pixels.size() > 1)
        cv::polylines(image, pixels, false, cv::Scalar(0, 0, 255), 2);
    // 第三遍：标注起点（蓝色实心圆 + 文本）与终点（绿色实心圆 + 文本），
    // 文本偏移 (6,-6) 像素避免压住圆点。
    cv::Point start_pixel;
    cv::Point goal_pixel;
    if (to_image_point(start.x, start.y, start_pixel))
    {
        cv::circle(image, start_pixel, 5, cv::Scalar(255, 0, 0), -1);
        cv::putText(image, "start", start_pixel + cv::Point(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 0), 1);
    }
    if (to_image_point(goal.x, goal.y, goal_pixel))
    {
        cv::circle(image, goal_pixel, 5, cv::Scalar(0, 255, 0), -1);
        cv::putText(image, "goal", goal_pixel + cv::Point(6, -6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
    }
    return cv::imwrite(file.string(), image);
#else
    (void)file; (void)grid; (void)path; (void)start; (void)goal;
    return false;
#endif
}

// 检查缓存分辨率和覆盖范围，防止误用其他任务地图。
// 逐项校验：分辨率/坡度阈值/粗糙度阈值必须与当前配置完全一致（1e-9 容差）；
// 缓存宽高与当前扫描走廊一致（允许 1 个栅格单元误差）；起终点都落在缓存
// 覆盖范围内。任一项不满足都说明缓存来自旧配置，必须作废重建。
bool cache_matches_(const TerrainCostmap& costmap, const TaskConfig& config)
{
    const TerrainGrid& grid = costmap.grid();
    // 当前配置请求的扫描走廊宽高（米）。
    const double requested_width = config.scan_bounds.max_x - config.scan_bounds.min_x;
    const double requested_height = config.scan_bounds.max_y - config.scan_bounds.min_y;
    // 建图参数逐项比对：浮点相等比较用 1e-9 容差，宽高允许 1 个单元误差。
    return std::abs(grid.resolution_m - config.terrain.resolution_m) < 1e-9
        && std::abs(grid.slope_limit_deg - config.terrain.slope_limit_deg) < 1e-9
        && std::abs(grid.roughness_limit - config.terrain.roughness_limit) < 1e-9
        && std::abs(grid.requested_width_m - requested_width)
            <= config.terrain.resolution_m
        && std::abs(grid.requested_height_m - requested_height)
            <= config.terrain.resolution_m
        && costmap.covers(config.start.x, config.start.y)
        && costmap.covers(config.goal.x, config.goal.y);
}
} // namespace

// 构建完整规划流程，代价地图只在本函数作用域内占用内存。
// 失败语义：每个阶段失败均抛出 std::runtime_error，由 main 统一打印为
// [error] 并退出，保证不会带着坏地图/坏路径进入跟踪阶段。
Path PlanningPipeline::buildPath(const TaskConfig& config) const
{
    // 模块隔离的一致性兜底：优化与跟踪两模块的车辆运动学参数独立维护
    // （分别在 PathOptimizerConfig 与 TrackingConfig::geometry），设计文档
    // 要求规划与控制共用同一套车辆参数；不一致只告警不阻断，避免阻断
    // 调参实验，但必须先修正再用于正式跑分。
    if (std::abs(config.optimizer.wheelbase_m - config.tracking.geometry.wheelbase_m) > 1e-9
        || std::abs(config.optimizer.max_front_wheel_angle_rad
                    - config.tracking.geometry.max_front_wheel_angle_rad) > 1e-9)
    {
        std::cerr << "[warning] optimizer 与 tracking 的车辆运动学参数不一致"
                  << "（wheelbase_m/max_front_wheel_angle_rad），规划曲率上限与"
                  << "跟踪转向能力将不匹配，请同步两处配置" << std::endl;
    }
#ifdef ECHOSIM_USE_OPENCV
    // 关闭 OpenCV 可选并行插件加载的 INFO 提示，避免干扰控制终端日志。
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
#endif
    // 阶段 0：确保输出目录存在（调试文件与代价地图缓存都写在这里）。
    std::error_code error;
    std::filesystem::create_directories(config.output_directory, error);
    if (error)
        throw std::runtime_error("无法创建 output 目录");
    std::cout << "[terrain] output directory ready: "
              << config.output_directory.string() << std::endl;

    // 阶段 1：获取代价地图。优先尝试 output 目录下的磁盘缓存（cost/valid 数据），
    // 缓存未命中才初始化 TerrainService 现场建图，避免每次启动重复查询地形。
    TerrainCostmap costmap;
#ifdef ECHOSIM_USE_ECHOSIM_SDK
    if (TerrainCostmap::hasCache(config.output_directory.string()))
    {
        // 缓存命中分支：加载后必须通过 cache_matches_ 一致性校验，
        // 防止 scan_bounds 或阈值修改后误用旧地图（直接抛错让用户重建）。
        std::cout << "[terrain] local cache found; loading cost/valid data" << std::endl;
        if (!costmap.load(config.output_directory.string(), false)
            || !cache_matches_(costmap, config))
        {
            throw std::runtime_error("terrain cache does not match configured scan bounds");
        }
        std::cout << "[terrain] cache loaded rows=" << costmap.grid().rows
                  << " cols=" << costmap.grid().cols
                  << " resolution_m=" << costmap.grid().resolution_m << std::endl;
    }
    else
    {
        // 缓存未命中分支：初始化 TerrainService 并在扫描走廊内现场建图。
        terrain::TerrainQueryConfig terrain_config;
        // 地形切片根：必须包含 level_1m/level_10m/level_100m/level_1000m 目录。
        terrain_config.tiledMapRootDir = config.terrain_root.string();
        // 高程查询与轮地接触均优先 1 m 分辨率切片，与代价地图分辨率一致。
        terrain_config.preferredQueryResolution = 1.0;
        terrain_config.preferredWheelResolution = 1.0;
        // 月球场景无道路层不优先道路；启用预取与粗糙度计算。
        terrain_config.enableRoadFirst = false;
        terrain_config.enablePrefetch = true;
        terrain_config.enableRoughness = true;
        terrain::TerrainQueryService service;
        // 初始化失败最常见原因是地形根路径错误（缺 level_* 切片目录）。
        if (!service.Initialize(terrain_config, nullptr, {}))
            throw std::runtime_error("TerrainService initialization failed");
        std::cout << "[terrain] TerrainService initialized" << std::endl;
        std::cout << "[terrain] building " << config.terrain.resolution_m
                  << "m costmap in configured scan bounds" << std::endl;
        // 在扫描走廊内批量查询地形（每次 64x64 单元）构建 1 m 代价地图。
        if (!costmap.build(service, config.scan_bounds, config.terrain))
        {
            // 建图失败：先释放 TerrainService 再抛错，保证异常路径也清理资源。
            service.Clear();
            throw std::runtime_error("terrain costmap build failed");
        }
        std::cout << "[terrain] costmap build completed rows=" << costmap.grid().rows
                  << " cols=" << costmap.grid().cols << std::endl;
        // 建图完成即释放 TerrainService：后续阶段只用栅格数据，不再查询地形。
        service.Clear();
        // 缓存只在调试模式下写盘：下次启动直接加载、跳过耗时建图；
        // 比赛模式（enable_debug_output=false）不写任何文件，代价是下次仍需建图。
        if (config.enable_debug_output
            && !costmap.save(config.output_directory.string(), false))
        {
            throw std::runtime_error("terrain cache save failed");
        }
        if (config.enable_debug_output)
            std::cout << "[terrain] costmap cache saved" << std::endl;
    }
#else
    throw std::runtime_error("PlanningPipeline requires EchoSim SDK");
#endif

    // 阶段 1.5（仅调试模式）：输出地形代价预览图 terrain_preview.png；
    // 失败仅告警不中断（可视化是辅助功能，不影响规划正确性）。
    if (config.enable_debug_output)
    {
        if (!costmap.savePreviewImage(config.output_directory.string()))
            std::cerr << "[visualization] terrain_preview.png was not written" << std::endl;
        else
            std::cout << "[visualization] terrain preview saved" << std::endl;
    }

    // 阶段 2：全局规划。在刚获取的代价地图上从起点到终点搜索原始路径
    // （默认双向 A*，方法见 config.planner.method；失败内部回退普通 A*）。
    GlobalPlanner planner;
    std::cout << "[planner] starting global path search" << std::endl;
    const Path path = planner.plan(costmap.grid(), config.start, config.goal,
                                   config.planner);
    // 规划结果为空（无可行路径）直接抛错，绝不带着空路径进入跟踪阶段。
    if (path.empty())
        throw std::runtime_error("global path planning failed");
    // 调试模式下写出原始路径 txt（含/不含航向两份）；写失败视为硬错误：
    // 连文本文件都写不出说明磁盘/目录有实质问题。
    if (config.enable_debug_output
        && (!save_path_(config.output_directory / "global_path.txt", path)
            || !save_path_with_yaw_(config.output_directory / "global_path_with_yaw.txt", path)))
    {
        throw std::runtime_error("global path text output failed");
    }

    // 路径优化：输入原始栅格路径 + 代价地图，输出优化后路径（当前透传）。
    // 阶段 3：视线捷径 + 迭代平滑 + 尖角圆弧化，全程按代价地图校验障碍
    // 余量并受阿克曼曲率上限约束（参数取值依据见 TaskConfig.cpp optimizer 段）。
    PathOptimizer optimizer;
    const Path optimized = optimizer.optimize(path, costmap.grid(), config.optimizer);
    // 优化结果为空（如候选捷径全被障碍否决）同样视为硬失败。
    if (optimized.empty())
        throw std::runtime_error("path optimization failed");
    // 阶段 4（仅调试模式）：写出优化后路径 txt 与"路径叠加代价地图"PNG
    // （global_path_on_costmap.png）；txt 写失败是硬错误，PNG 失败仅告警。
    if (config.enable_debug_output
        && (!save_path_(config.output_directory / "optimized_path.txt", optimized)
            || !save_path_with_yaw_(config.output_directory / "optimized_path_with_yaw.txt", optimized)))
    {
        throw std::runtime_error("optimized path text output failed");
    }
    if (config.enable_debug_output
        && !save_path_image_(config.output_directory / "global_path_on_costmap.png",
                           costmap.grid(), optimized, config.start, config.goal))
    {
        std::cerr << "[visualization] global_path_on_costmap.png was not written" << std::endl;
    }
    if (config.enable_debug_output)
        std::cout << "[visualization] global path image and text saved" << std::endl;
    // 打印优化前后的点数对比（实测 Moon2：914 点 -> 11 点），直观反映压缩效果。
    std::cout << "[planner] global path points=" << path.size()
              << " optimized points=" << optimized.size() << std::endl;
    // 返回优化后路径；costmap 在此离开作用域，随即释放大块栅格内存。
    return optimized;
}
