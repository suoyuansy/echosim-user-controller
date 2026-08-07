#pragma once

#include "PathTypes.h"
#include "TerrainCostmap.h"

#include <vector>

// 文件功能：在 TerrainGrid 上执行八邻域 A* 和双向 A* 全局路径规划。
enum class GlobalPlannerMethod
{
    AStar = 0, // 普通八邻域 A*。
    BidirectionalAStar = 1 // 双向八邻域 A*，默认方法。
};

struct GlobalPlannerConfig
{
    GlobalPlannerMethod method = GlobalPlannerMethod::BidirectionalAStar; // 全局规划方法。
    double cost_weight = 10.0; // 地形代价对搜索边代价的权重。
};

class GlobalPlanner
{
public:
    // 根据配置规划路径，双向 A* 失败时回退到普通 A*。
    Path plan(const TerrainGrid& grid,
              const Pose2D& start,
              const Pose2D& goal,
              const GlobalPlannerConfig& config) const;

    // 执行普通八邻域 A*。
    Path planAStar(const TerrainGrid& grid,
                   const Pose2D& start,
                   const Pose2D& goal,
                   double cost_weight) const;

    // 执行双向八邻域 A*。
    Path planBidirectionalAStar(const TerrainGrid& grid,
                                const Pose2D& start,
                                const Pose2D& goal,
                                double cost_weight) const;

private:
    struct GridPoint
    {
        int row = 0; // 栅格行号。
        int col = 0; // 栅格列号。
    };

    static bool to_grid_point_(const TerrainGrid& grid,
                            const Pose2D& pose,
                            GridPoint& point);
    static Path build_world_path_(const TerrainGrid& grid,
                               const std::vector<GridPoint>& grid_path,
                               const Pose2D& start,
                               const Pose2D& goal);
};
