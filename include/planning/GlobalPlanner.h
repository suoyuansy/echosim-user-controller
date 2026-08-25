#pragma once

#include "common/PathTypes.h"
#include "planning/TerrainCostmap.h"

#include <vector>

// 文件功能：在 TerrainGrid 上执行八邻域 A* 和双向 A* 全局路径规划。
enum class GlobalPlannerMethod
{
    AStar = 0, // 普通八邻域 A*。
    BidirectionalAStar = 1 // 双向八邻域 A*，默认方法。
};

// 全局规划配置：默认值即本模块实测调出的参数，模块所有者直接在此调参；
// app 层 TaskConfig 聚合本结构，仅任务级覆盖时才在 makeDefaultTaskConfig()
// 显式赋值。默认双向 A*（从起终点同时搜索加速长距离规划，失败时规划器
// 内部回退普通 A*）。
struct GlobalPlannerConfig
{
    GlobalPlannerMethod method = GlobalPlannerMethod::BidirectionalAStar; // 全局规划方法。
    // 地形代价对搜索边代价的权重：边代价 = 段长 * (1 + cost_weight * 两端
    // 平均代价)。默认 10 表示满代价栅格的等效边长放大到 11 倍，使 A* 明显
    // 绕开高代价（陡峭/崎岖）区域而不是走捷径穿越陡坡；设 0 则退化为纯
    // 最短路径搜索。
    double cost_weight = 10.0; // 地形代价对搜索边代价的权重。
    int clearance_margin_cells = 2; // 为约 1 m 半车宽和转弯圆弧预留两格硬障碍余量。
    bool allow_zero_margin_fallback = false; // 禁止产生车体包络可能覆盖硬障碍的路径。
};

class GlobalPlanner
{
public:
    // 根据配置规划路径，双向 A* 失败时回退到普通 A*。
    // 输入 start/goal 为世界坐标位姿（米/弧度），grid 为地形代价栅格；
    // 输出为世界坐标 Path（米/弧度），空 Path 表示无可行路径。
    Path plan(const TerrainGrid& grid,
              const Pose2D& start,
              const Pose2D& goal,
              const GlobalPlannerConfig& config) const;

    // 执行普通八邻域 A*。
    // 输入单位同 plan()；cost_weight 为地形代价边权。搜索保证在八邻域
    // 度量下最优（启发式可采纳），但可能贴近障碍边缘。
    Path planAStar(const TerrainGrid& grid,
                   const Pose2D& start,
                   const Pose2D& goal,
                   double cost_weight,
                   int clearance_margin_cells = 1) const;

    // 执行双向八邻域 A*。
    // 从起点/终点两个方向同时做 Dijkstra 式扩展，两边前沿相遇后拼接路径；
    // 在大地图上比单向 A* 扩展更少节点，但代价是启发式不再参与定向。
    Path planBidirectionalAStar(const TerrainGrid& grid,
                                const Pose2D& start,
                                const Pose2D& goal,
                                double cost_weight,
                                int clearance_margin_cells = 1) const;

private:
    // 栅格坐标点：row 沿世界 +Y、col 沿世界 +X，均为无量纲整数格数。
    struct GridPoint
    {
        int row = 0; // 栅格行号。
        int col = 0; // 栅格列号。
    };

    // 将世界坐标位姿四舍五入到最近栅格；越界或地图为空返回 false。
    static bool to_grid_point_(const TerrainGrid& grid,
                            const Pose2D& pose,
                            GridPoint& point);
    // 将栅格折线转换为世界坐标路径：首末端点用精确的 start/goal 坐标替换，
    // 中间点航向角取前后点连线方向（弧度）。
    static Path build_world_path_(const TerrainGrid& grid,
                               const std::vector<GridPoint>& grid_path,
                               const Pose2D& start,
                               const Pose2D& goal);
};
