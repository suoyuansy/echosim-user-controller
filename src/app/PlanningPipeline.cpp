// 实现地形缓存加载、TerrainService 建图、全局规划和路径文件/图片输出。
#include "app/PlanningPipeline.h"
#include "app/SegmentPlanningPolicy.h"

#include "planning/GlobalPlanner.h"
#include "planning/TerrainCostmap.h"
#include "tracking/TerrainSurfaceQuery.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace
{
constexpr int kGlobalPathCacheFormatVersion = 2;

bool path_cache_format_matches_(const std::filesystem::path& output_directory)
{
    std::ifstream input(output_directory / "global_path_format_version.txt");
    int version = 0;
    return input >> version && version == kGlobalPathCacheFormatVersion;
}

bool save_path_cache_format_version_(const std::filesystem::path& output_directory)
{
    std::ofstream output(output_directory / "global_path_format_version.txt");
    if (!output)
        return false;
    output << kGlobalPathCacheFormatVersion << '\n';
    return output.good();
}

// 写出不含航向角的世界坐标路径。
bool save_path_(const std::filesystem::path& file, const Path& path)
{
    std::ofstream output(file);
    if (!output)
        return false;
    output << std::setprecision(17);
    for (const PathPoint& point : path)
        output << point.x << ' ' << point.y << '\n';
    return output.good();
}

// 读取仅含 x y 的全局折线路径。路径文件存在时，运行阶段直接复用它，
// 不再调用 TerrainService 或重新执行 A*。
bool load_path_(const std::filesystem::path& file, Path& path)
{
    std::ifstream input(file);
    if (!input)
        return false;
    path.clear();
    std::string line;
    while (std::getline(input, line))
    {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos)
            continue;
        std::istringstream values(line);
        PathPoint point;
        if (!(values >> point.x >> point.y))
        {
            path.clear();
            return false;
        }
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
        {
            path.clear();
            return false;
        }
        path.push_back(point);
    }
    return input.eof() && !path.empty();
}

// 为路径文件补齐几何航向。PP 主要使用坐标折线，但统一生成 yaw 可以保证
// 日志和跟踪器看到的是一致的数据；终点完成判定只使用位置容差。
void assign_headings_(Path& path, const Pose2D& start, const Pose2D& goal)
{
    if (path.empty())
        return;
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const std::size_t previous = index == 0 ? index : index - 1;
        const std::size_t next = index + 1 < path.size() ? index + 1 : index;
        const double dx = path[next].x - path[previous].x;
        const double dy = path[next].y - path[previous].y;
        if (std::hypot(dx, dy) > 1e-9)
            path[index].yaw = std::atan2(dy, dx);
    }
    path.front().yaw = start.yaw;
}

std::size_t nearest_path_index_(const Path& path, const Pose2D& pose,
                                std::size_t begin_index = 0)
{
    if (path.empty())
        return 0;
    begin_index = std::min(begin_index, path.size() - 1);
    std::size_t nearest = begin_index;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = begin_index; index < path.size(); ++index)
    {
        const double distance = std::hypot(path[index].x - pose.x,
                                           path[index].y - pose.y);
        if (distance < nearest_distance)
        {
            nearest = index;
            nearest_distance = distance;
        }
    }
    return nearest;
}

// output/test6 可能由旧版任务坐标生成。只有路径首尾与当前任务一致时，
// 才允许直接复用，避免车辆首帧被错误吸附到旧路径末端。
bool path_matches_task_(const Path& path, const TaskConfig& config)
{
    if (path.size() < 2)
        return false;
    constexpr double kEndpointToleranceM = 2.0;
    return std::hypot(path.front().x - config.start.x,
                      path.front().y - config.start.y) <= kEndpointToleranceM
        && std::hypot(path.back().x - config.goal.x,
                      path.back().y - config.goal.y) <= kEndpointToleranceM;
}

// 计算最终拼接折线路径的总长度，单位为米。
double path_length_(const Path& path)
{
    double length_m = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index)
        length_m += std::hypot(path[index].x - path[index - 1].x,
                               path[index].y - path[index - 1].y);
    return length_m;
}

// 判断一条路径是否进入任一人工禁行圆。既检查路径点，也检查相邻点之间的
// 线段，避免低采样折线从圆内穿过却没有恰好落点的情况。
bool path_intersects_hard_obstacle_circle_(
    const Path& path, const std::vector<HardObstacleCircle>& circles)
{
    if (path.empty() || circles.empty())
        return false;
    for (const HardObstacleCircle& circle : circles)
    {
        if (!std::isfinite(circle.x) || !std::isfinite(circle.y)
            || !std::isfinite(circle.radius_m) || circle.radius_m <= 0.0)
        {
            continue;
        }
        const double radius_squared = circle.radius_m * circle.radius_m;
        for (std::size_t index = 0; index < path.size(); ++index)
        {
            const PathPoint& first = path[index];
            const double first_dx = first.x - circle.x;
            const double first_dy = first.y - circle.y;
            if (first_dx * first_dx + first_dy * first_dy <= radius_squared)
                return true;
            if (index == 0)
                continue;

            const PathPoint& previous = path[index - 1];
            const double segment_x = first.x - previous.x;
            const double segment_y = first.y - previous.y;
            const double segment_length_squared = segment_x * segment_x
                + segment_y * segment_y;
            if (segment_length_squared <= 1e-12)
                continue;
            const double projection = std::clamp(
                ((circle.x - previous.x) * segment_x
                    + (circle.y - previous.y) * segment_y)
                    / segment_length_squared,
                0.0, 1.0);
            const double nearest_x = previous.x + projection * segment_x;
            const double nearest_y = previous.y + projection * segment_y;
            const double dx = nearest_x - circle.x;
            const double dy = nearest_y - circle.y;
            if (dx * dx + dy * dy <= radius_squared)
                return true;
        }
    }
    return false;
}

double heading_change_(const PathPoint& previous,
                       const PathPoint& corner,
                       const PathPoint& next)
{
    const double incoming_x = corner.x - previous.x;
    const double incoming_y = corner.y - previous.y;
    const double outgoing_x = next.x - corner.x;
    const double outgoing_y = next.y - corner.y;
    const double incoming_length = std::hypot(incoming_x, incoming_y);
    const double outgoing_length = std::hypot(outgoing_x, outgoing_y);
    if (incoming_length <= 1e-9 || outgoing_length <= 1e-9)
        return 0.0;
    const double cosine = std::clamp(
        (incoming_x * outgoing_x + incoming_y * outgoing_y)
            / (incoming_length * outgoing_length), -1.0, 1.0);
    return std::acos(cosine);
}

void report_waypoint_transitions_(const Path& path,
                                  const std::vector<std::size_t>& stop_indices,
                                  double opposite_turn_limit_rad)
{
    for (std::size_t waypoint_index = 0;
         waypoint_index < stop_indices.size(); ++waypoint_index)
    {
        const std::size_t stop_index = stop_indices[waypoint_index];
        if (stop_index == 0 || stop_index + 1 >= path.size())
        {
            std::cout << "[planner] waypoint " << waypoint_index + 1
                      << " heading transition unavailable at path boundary"
                      << std::endl;
            continue;
        }
        const double transition = heading_change_(
            path[stop_index - 1], path[stop_index], path[stop_index + 1]);
        std::cout << std::fixed << std::setprecision(1)
                  << "[planner] waypoint " << waypoint_index + 1
                  << " heading transition angle_deg="
                  << transition * 180.0 / 3.14159265358979323846;
        if (transition > std::clamp(opposite_turn_limit_rad, 0.0,
                                    3.14159265358979323846))
            std::cout << " warning=immediate_opposite_turn";
        std::cout << std::endl;
    }
}

// 将代价地图和全局路径绘制为灰度底图加红色路径图。
bool save_path_image_(const std::filesystem::path& file,
                    const TerrainGrid& grid,
                    const Path& path,
                    const Pose2D& start,
                    const Pose2D& goal,
                    const std::vector<Pose2D>& waypoints,
                    const std::vector<HardObstacleCircle>& hard_obstacle_circles)
{
#ifdef ECHOSIM_USE_OPENCV
    if (grid.empty())
        return false;
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
    cv::cvtColor(gray, image, cv::COLOR_GRAY2BGR);
    auto to_image_point = [&grid](double x, double y, cv::Point& point) {
        int col = 0;
        int row = 0;
        if (!grid.worldToGrid(x, y, col, row))
            return false;
        point = {col, grid.rows - 1 - row};
        return true;
    };
    std::vector<cv::Point> pixels;
    for (const PathPoint& path_point : path)
    {
        cv::Point pixel;
        if (to_image_point(path_point.x, path_point.y, pixel))
            pixels.push_back(pixel);
    }
    if (pixels.size() > 1)
        cv::polylines(image, pixels, false, cv::Scalar(0, 0, 255), 2);

    // 人工禁行圆不修改灰度 cost 底图；仅以洋红色圆周和叉号叠加标出，
    // 便于核对规划绕行范围以及以后复查失控位置。
    for (const HardObstacleCircle& circle : hard_obstacle_circles)
    {
        if (!std::isfinite(circle.x) || !std::isfinite(circle.y)
            || !std::isfinite(circle.radius_m) || circle.radius_m <= 0.0)
        {
            continue;
        }
        cv::Point center;
        if (!to_image_point(circle.x, circle.y, center))
            continue;
        const int radius_pixels = std::max(1, static_cast<int>(std::lround(
            circle.radius_m / grid.resolution_m)));
        const cv::Scalar color(255, 0, 255);
        cv::circle(image, center, radius_pixels, color, 2, cv::LINE_AA);
        cv::line(image, center + cv::Point(-6, -6), center + cv::Point(6, 6),
                 color, 2, cv::LINE_AA);
        cv::line(image, center + cv::Point(-6, 6), center + cv::Point(6, -6),
                 color, 2, cv::LINE_AA);
        cv::putText(image, "no-go", center + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
    }

    // 途经点单独叠加在全局路径之上，避免被路径线或底图遮住。
    for (std::size_t index = 0; index < waypoints.size(); ++index)
    {
        cv::Point waypoint_pixel;
        if (!to_image_point(waypoints[index].x, waypoints[index].y,
                            waypoint_pixel))
            continue;
        cv::circle(image, waypoint_pixel, 7, cv::Scalar(0, 165, 255), -1);
        cv::circle(image, waypoint_pixel, 9, cv::Scalar(0, 0, 0), 1);
        cv::putText(image, "wp" + std::to_string(index + 1),
                    waypoint_pixel + cv::Point(9, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 165, 255), 2);
    }

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
    (void)waypoints; (void)hard_obstacle_circles;
    return false;
#endif
}

// 检查缓存分辨率和覆盖范围，防止误用其他任务地图。
bool cache_matches_(const TerrainCostmap& costmap, const TaskConfig& config)
{
    const TerrainGrid& grid = costmap.grid();
    const double requested_width = config.scan_bounds.max_x - config.scan_bounds.min_x;
    const double requested_height = config.scan_bounds.max_y - config.scan_bounds.min_y;
    return std::abs(grid.resolution_m - config.terrain.resolution_m) < 1e-9
        && std::abs(grid.slope_limit_deg - config.terrain.slope_limit_deg) < 1e-9
        && std::abs(grid.roughness_limit - config.terrain.roughness_limit) < 1e-9
        && std::abs(grid.hard_slope_limit_deg - config.terrain.hard_slope_limit_deg) < 1e-9
        && std::abs(grid.hard_roughness_limit - config.terrain.hard_roughness_limit) < 1e-9
        && std::abs(grid.requested_width_m - requested_width)
            <= config.terrain.resolution_m
        && std::abs(grid.requested_height_m - requested_height)
            <= config.terrain.resolution_m
        && costmap.covers(config.start.x, config.start.y)
        && costmap.covers(config.goal.x, config.goal.y)
        && std::all_of(config.waypoints.begin(), config.waypoints.end(),
                       [&costmap](const Pose2D& waypoint) {
                           return costmap.covers(waypoint.x, waypoint.y);
                       });
}

std::shared_ptr<const TerrainGrid> load_matching_terrain_cache_(
    const TaskConfig& config)
{
    TerrainCostmap costmap;
    if (!TerrainCostmap::hasCache(config.output_directory.string())
        || !costmap.load(config.output_directory.string(), true)
        || !cache_matches_(costmap, config))
        return {};
    TerrainCostmap::applyHardObstacleCircles(costmap.grid(),
                                             config.hard_obstacle_circles);
    std::cout << "[terrain] cached surface data loaded rows="
              << costmap.grid().rows << " cols=" << costmap.grid().cols
              << " resolution_m=" << costmap.grid().resolution_m << std::endl;
    return std::make_shared<const TerrainGrid>(costmap.grid());
}

std::shared_ptr<const TerrainSurfaceQuery> create_surface_query_(
    const TaskConfig& config)
{
    auto query = std::make_shared<TerrainSurfaceQuery>();
    if (!query->initialize(config.terrain_root))
    {
        std::cerr << "[surface] lightweight query initialization failed; terrain_root="
                  << config.terrain_root.string() << std::endl;
        return {};
    }
    std::cout << "[surface] lightweight terrain query initialized from "
              << config.terrain_root.string() << std::endl;
    return query;
}

void print_surface_(const TerrainGrid& grid, const std::string& label,
                    double x, double y)
{
    int col = 0;
    int row = 0;
    if (!grid.worldToGrid(x, y, col, row))
    {
        std::cout << "[surface] " << label << " outside_map" << std::endl;
        return;
    }
    const std::size_t index = grid.index(row, col);
    if (index >= grid.slope_deg.size() || index >= grid.roughness.size())
    {
        std::cout << "[surface] " << label << " unavailable" << std::endl;
        return;
    }
    std::cout << std::fixed << std::setprecision(3)
              << "[surface] " << label << "=(x=" << x << ",y=" << y
              << ",slope_deg=" << grid.slope_deg[index]
              << ",roughness=" << grid.roughness[index] << ')'
              << std::endl;
}
} // namespace

// 构建完整规划流程，代价地图只在本函数作用域内占用内存。
RoutePlan PlanningPipeline::buildPath(const TaskConfig& config) const
{
#ifdef ECHOSIM_USE_OPENCV
    // 关闭 OpenCV 可选并行插件加载的 INFO 提示，避免干扰控制终端日志。
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
#endif
    std::error_code error;
    std::filesystem::create_directories(config.output_directory, error);
    if (error)
        throw std::runtime_error("无法创建 output 目录");
    std::cout << "[terrain] output directory ready: "
              << config.output_directory.string() << std::endl;

    const std::filesystem::path path_file =
        config.output_directory / "global_path.txt";
    RoutePlan route;
    bool reuse_existing_path = false;
    if (std::filesystem::exists(path_file)
        && path_cache_format_matches_(config.output_directory))
    {
        if (!load_path_(path_file, route.path))
            throw std::runtime_error("global_path.txt exists but cannot be read");
        reuse_existing_path = path_matches_task_(route.path, config);
        if (reuse_existing_path && path_intersects_hard_obstacle_circle_(
                route.path, config.hard_obstacle_circles))
        {
            reuse_existing_path = false;
            std::cout << "[planner] existing global path enters an artificial "
                         "no-go circle; rebuilding it" << std::endl;
        }
        if (!reuse_existing_path)
        {
            std::cout << "[planner] existing global path does not match current "
                         "task endpoints; rebuilding it" << std::endl;
            route.path.clear();
        }
    }
    else if (std::filesystem::exists(path_file))
    {
        std::cout << "[planner] existing global path uses an older planning "
                     "format; rebuilding it" << std::endl;
    }
    if (reuse_existing_path)
    {
        assign_headings_(route.path, config.start, config.goal);
        route.terrain_grid = load_matching_terrain_cache_(config);
        // 轻量查询与代价地图是两条独立链路。即使任务缓存匹配，实际车辆
        // 也可能暂时驶出本次小范围栅格；此时仍需使用完整 Moon2 数据查询。
        route.surface_query = create_surface_query_(config);
        if (!route.terrain_grid)
            std::cout << "[surface] matching terrain cache unavailable; "
                         "using lightweight terrain queries without replanning"
                      << std::endl;
        std::cout << "[planner] existing global path loaded from "
                  << path_file.string() << ", points=" << route.path.size()
                  << std::endl;
    }
    else
    {
        TerrainCostmap costmap;
#ifdef ECHOSIM_USE_ECHOSIM_SDK
        // 没有全局路径文件时强制重新建图；路径文件本身是是否重规划的
        // 唯一判断条件，不再用旧 terrain_* 缓存替代本次规划。
        terrain::TerrainQueryConfig terrain_config;
        terrain_config.tiledMapRootDir = config.terrain_root.string();
        terrain_config.preferredQueryResolution = 1.0;
        terrain_config.preferredWheelResolution = 1.0;
        terrain_config.enableRoadFirst = false;
        terrain_config.enablePrefetch = true;
        terrain_config.enableRoughness = true;
        terrain::TerrainQueryService service;
        if (!service.Initialize(terrain_config, nullptr, {}))
            throw std::runtime_error("TerrainService initialization failed");
        std::cout << "[terrain] TerrainService initialized" << std::endl;
        std::cout << "[terrain] building " << config.terrain.resolution_m
                  << "m costmap in configured scan bounds" << std::endl;
        if (!costmap.build(service, config.scan_bounds, config.terrain))
        {
            service.Clear();
            throw std::runtime_error("terrain costmap build failed");
        }
        std::cout << "[terrain] costmap build completed rows="
                  << costmap.grid().rows << " cols=" << costmap.grid().cols
                  << std::endl;
        service.Clear();
#else
        throw std::runtime_error("PlanningPipeline requires EchoSim SDK");
#endif

        TerrainCostmap::applyHardObstacleCircles(costmap.grid(),
                                                 config.hard_obstacle_circles);

        if (config.enable_debug_output && !costmap.save(
                config.output_directory.string(), true))
        {
            throw std::runtime_error("terrain costmap cache save failed");
        }
        std::cout << "[terrain] costmap cache saved" << std::endl;

        if (config.enable_debug_output)
            std::cout << "[visualization] terrain preview saved" << std::endl;
        GlobalPlanner planner;
        std::vector<Pose2D> anchors{config.start};
        anchors.insert(anchors.end(), config.waypoints.begin(), config.waypoints.end());
        anchors.push_back(config.goal);
        std::vector<Path> raw_segments;
        raw_segments.reserve(anchors.size() - 1);
        for (std::size_t segment = 0;
             segment + 1 < anchors.size(); ++segment)
        {
            Path raw = planner.plan(costmap.grid(), anchors[segment],
                                    anchors[segment + 1], config.planner);
            if (raw.empty())
                throw std::runtime_error(
                    "global path planning failed at segment "
                    + std::to_string(segment + 1));
            raw_segments.push_back(std::move(raw));
        }

        // 途经点的航向不作为固定目标角。每一段完成后，从“最终输出的
        // 上一段”末端提取进入方向，而不是从未经衔接的 raw A* 段提取；
        // 这样下一段才能真正沿车辆抵达途经点时的方向离开。
        double incoming_heading = config.start.yaw;
        for (std::size_t segment = 0;
             segment + 1 < anchors.size(); ++segment)
        {
            Pose2D segment_start = anchors[segment];
            Pose2D segment_goal = anchors[segment + 1];
            if (segment > 0)
                segment_start.yaw = incoming_heading;
            if (segment + 1 < anchors.size() - 1)
            {
                const Path& raw_segment = raw_segments[segment];
                if (raw_segment.size() >= 2)
                {
                    const PathPoint& previous =
                        raw_segment[raw_segment.size() - 2];
                    const PathPoint& last = raw_segment.back();
                    segment_goal.yaw = std::atan2(
                        last.y - previous.y, last.x - previous.x);
                }
                else
                    segment_goal.yaw = incoming_heading;
            }
            // 只有已停车的途经点后才需要沿抵达方向重新起步。任务初始
            // 出生点不是途经点，首段必须保留原始 A* 的离开方向，避免
            // 在 A* 位于车尾时先插入前向短线、随后形成折返。
            const bool align_segment_start =
                shouldAlignSegmentStartHeading(segment,
                                               config.waypoints.size());
            const bool is_before_waypoint = segment + 1 < anchors.size() - 1;
            const bool preserve_goal_approach = segment + 1 == anchors.size() - 1;
            // 只处理 wp1 停车后驶向下一目标的急转；其余路径段保持原有
            // A* 与端点航向衔接行为，避免把局部修正扩散到 wp2、wp3 或终点。
            const bool split_after_wp1 = segment == 1;
            Path segment_path = planner.planWithHeadingConstraints(
                costmap.grid(), segment_start, segment_goal, config.planner,
                align_segment_start, is_before_waypoint,
                preserve_goal_approach, split_after_wp1);
            if (segment_path.empty())
                throw std::runtime_error("global path planning failed at segment "
                                         + std::to_string(segment + 1));
            if (!route.path.empty())
                segment_path.erase(segment_path.begin());
            route.path.insert(route.path.end(), segment_path.begin(), segment_path.end());
            if (segment + 1 < anchors.size() - 1)
                route.stop_indices.push_back(route.path.size() - 1);
            if (segment_path.size() >= 2)
            {
                const PathPoint& previous = segment_path[segment_path.size() - 2];
                const PathPoint& last = segment_path.back();
                if (std::hypot(last.x - previous.x, last.y - previous.y) > 1e-9)
                    incoming_heading = std::atan2(
                        last.y - previous.y, last.x - previous.x);
            }
            std::cout << "[planner] segment " << segment + 1
                      << " planned, points=" << segment_path.size() << std::endl;
        }
        route.terrain_grid = std::make_shared<const TerrainGrid>(costmap.grid());
        // 建图服务已释放；运行期若驶出小范围栅格，仍可查询完整地形。
        route.surface_query = create_surface_query_(config);
    }

    if (route.terrain_grid)
    {
        print_surface_(*route.terrain_grid, "start", config.start.x, config.start.y);
        for (std::size_t index = 0; index < config.waypoints.size(); ++index)
            print_surface_(*route.terrain_grid, "wp" + std::to_string(index + 1),
                           config.waypoints[index].x,
                           config.waypoints[index].y);
        print_surface_(*route.terrain_grid, "goal", config.goal.x, config.goal.y);
        if (config.enable_debug_output
            && !route.terrain_grid->empty())
        {
            // 缓存路径运行时只加载地形网格；预览文件若已存在则保留，
            // 不在此分支重新初始化 TerrainService。
            std::cout << "[terrain] surface cache available for tracking logs"
                      << std::endl;
        }
    }
    else if (route.surface_query)
    {
        std::cout << "[surface] using lightweight terrain query for surface logs"
                  << std::endl;
        auto print_query_surface = [&route](const std::string& label,
                                             double x, double y) {
            double slope_deg = 0.0;
            double roughness = 0.0;
            if (route.surface_query->query(x, y, slope_deg, roughness))
            {
                std::cout << std::fixed << std::setprecision(3)
                          << "[surface] " << label << "=(x=" << x
                          << ",y=" << y << ",slope_deg=" << slope_deg
                          << ",roughness=" << roughness << ')' << std::endl;
            }
            else
            {
                std::cout << "[surface] " << label << " unavailable"
                          << std::endl;
            }
        };
        print_query_surface("start", config.start.x, config.start.y);
        for (std::size_t index = 0; index < config.waypoints.size(); ++index)
            print_query_surface("wp" + std::to_string(index + 1),
                                config.waypoints[index].x,
                                config.waypoints[index].y);
        print_query_surface("goal", config.goal.x, config.goal.y);
    }

    const Path& path = route.path;
    if (path.empty())
        throw std::runtime_error("global path planning produced an empty path");
    if (config.enable_debug_output
        && !save_path_(config.output_directory / "global_path.txt", path))
    {
        throw std::runtime_error("global path text output failed");
    }
    if (config.enable_debug_output
        && !save_path_cache_format_version_(config.output_directory))
    {
        throw std::runtime_error("global path cache format output failed");
    }
    if (config.enable_debug_output && route.terrain_grid
        && !save_path_image_(config.output_directory / "global_path_on_costmap.png",
                           *route.terrain_grid, path, config.start, config.goal,
                           config.waypoints, config.hard_obstacle_circles))
    {
        std::cerr << "[visualization] global_path_on_costmap.png was not written" << std::endl;
    }
    if (config.enable_debug_output)
        std::cout << "[visualization] global path image and text saved" << std::endl;
    std::cout << std::fixed << std::setprecision(3)
              << "[planner] global path points=" << path.size()
              << " total_length_m=" << path_length_(path) << std::endl;
    if (route.stop_indices.empty() && !config.waypoints.empty())
    {
        std::size_t begin_index = 0;
        for (const Pose2D& waypoint : config.waypoints)
        {
            const std::size_t index = nearest_path_index_(
                path, waypoint, begin_index);
            route.stop_indices.push_back(index);
            begin_index = index;
        }
    }
    if (!config.waypoints.empty())
    {
        report_waypoint_transitions_(
            path, route.stop_indices,
            config.planner.initial_opposite_turn_limit_rad);
    }
    return route;
}
