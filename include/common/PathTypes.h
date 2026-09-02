#pragma once

#include <cstddef>
#include <memory>
#include <vector>

struct TerrainGrid;
class TerrainSurfaceQuery;

// 定义路径规划、路径跟踪和可视化共用的二维位姿与路径点类型。
struct Pose2D
{
    double x = 0.0;   // 世界坐标 X，单位为米。
    double y = 0.0;   // 世界坐标 Y，单位为米。
    double yaw = 0.0; // 航向角，单位为弧度。
};

struct PathPoint
{
    double x = 0.0;   // 路径点世界坐标 X，单位为米。
    double y = 0.0;   // 路径点世界坐标 Y，单位为米。
    double yaw = 0.0; // 路径点航向角，单位为弧度。
};

using Path = std::vector<PathPoint>;

// 完整路线以及需要停车的途经点在路径中的索引。
struct RoutePlan
{
    Path path;
    std::vector<std::size_t> stop_indices;
    std::shared_ptr<const TerrainGrid> terrain_grid;
    std::shared_ptr<const TerrainSurfaceQuery> surface_query;
};
