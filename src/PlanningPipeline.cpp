// 实现地形缓存加载、TerrainService 建图、全局规划和路径文件/图片输出。
#include "PlanningPipeline.h"

#include "GlobalPlanner.h"
#include "TerrainCostmap.h"

#include <algorithm>
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

// 写出含航向角的世界坐标路径。
bool save_path_with_yaw_(const std::filesystem::path& file, const Path& path)
{
    std::ofstream output(file);
    if (!output)
        return false;
    output << std::setprecision(17);
    for (const PathPoint& point : path)
        output << point.x << ' ' << point.y << ' ' << point.yaw << '\n';
    return output.good();
}

// 将代价地图和全局路径绘制为灰度底图加红色路径图。
bool save_path_image_(const std::filesystem::path& file,
                    const TerrainGrid& grid,
                    const Path& path,
                    const Pose2D& start,
                    const Pose2D& goal)
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
bool cache_matches_(const TerrainCostmap& costmap, const TaskConfig& config)
{
    const TerrainGrid& grid = costmap.grid();
    const double requested_width = config.scan_bounds.max_x - config.scan_bounds.min_x;
    const double requested_height = config.scan_bounds.max_y - config.scan_bounds.min_y;
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
Path PlanningPipeline::buildPath(const TaskConfig& config) const
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

    TerrainCostmap costmap;
#ifdef ECHOSIM_USE_ECHOSIM_SDK
    if (TerrainCostmap::hasCache(config.output_directory.string()))
    {
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
        std::cout << "[terrain] costmap build completed rows=" << costmap.grid().rows
                  << " cols=" << costmap.grid().cols << std::endl;
        service.Clear();
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

    if (config.enable_debug_output)
    {
        if (!costmap.savePreviewImage(config.output_directory.string()))
            std::cerr << "[visualization] terrain_preview.png was not written" << std::endl;
        else
            std::cout << "[visualization] terrain preview saved" << std::endl;
    }

    GlobalPlanner planner;
    std::cout << "[planner] starting global path search" << std::endl;
    const Path path = planner.plan(costmap.grid(), config.start, config.goal,
                                   config.planner);
    if (path.empty())
        throw std::runtime_error("global path planning failed");
    if (config.enable_debug_output
        && (!save_path_(config.output_directory / "global_path.txt", path)
            || !save_path_with_yaw_(config.output_directory / "global_path_with_yaw.txt", path)))
    {
        throw std::runtime_error("global path text output failed");
    }
    if (config.enable_debug_output
        && !save_path_image_(config.output_directory / "global_path_on_costmap.png",
                           costmap.grid(), path, config.start, config.goal))
    {
        std::cerr << "[visualization] global_path_on_costmap.png was not written" << std::endl;
    }
    if (config.enable_debug_output)
        std::cout << "[visualization] global path image and text saved" << std::endl;
    std::cout << "[planner] global path points=" << path.size() << std::endl;
    return path;
}
