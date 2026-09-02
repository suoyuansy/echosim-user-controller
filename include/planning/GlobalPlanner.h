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
    // 平均代价)。默认 10000 使满代价栅格的等效边长显著放大，使 A* 明显
    // 绕开高代价（陡峭/崎岖）区域而不是走捷径穿越陡坡；设 0 则退化为纯
    // 最短路径搜索。当前使用 10000，使达到软风险基准的地形代价远大于
    // 普通绕行距离，仅在没有低风险连通路线时才穿过软风险区。
    double cost_weight = 10000.0; // 超大软代价权重：近似优先最小化地形风险，再考虑路径长度。
    int clearance_margin_cells = 1; // 全局路径与硬障碍保持一格安全余量。
    bool allow_zero_margin_fallback = true; // 禁止产生车体包络可能覆盖硬障碍的路径。
    double heading_alignment_max_distance_m = 15.0;
    double heading_alignment_min_distance_m = 5.0;
    double initial_opposite_turn_limit_rad =
        120.0 * 3.14159265358979323846 / 180.0;
    double large_turn_split_threshold_rad =
        100.0 * 3.14159265358979323846 / 180.0;
    // wp1 后的大转角优先插入 15 m 中间方向段；只接受不少于 10 m 的
    // 安全候选，避免退化为难以跟踪的短段。无候选时保留原始 A* 路径。
    double large_turn_split_max_distance_m = 15.0;
    double large_turn_split_min_distance_m = 10.0;
};

class GlobalPlanner
{
public:
    // 栅格坐标点：row 沿世界 +Y、col 沿世界 +X，均为无量纲整数格数。
    // 该类型仅用于规划器内部的轻量航向衔接辅助函数，但需要在实现文件
    // 的匿名命名空间中传递，因此保持为 public 类型而不是暴露规划状态。
    struct GridPoint
    {
        int row = 0; // 栅格行号。
        int col = 0; // 栅格列号。
    };

    // 判断栅格自身可通行且满足指定安全余量。必达点只需调用
    // TerrainGrid::isTraversable()；普通路径和航向衔接段使用本接口。
    static bool isTraversableWithClearance(const TerrainGrid& grid,
                                           int row,
                                           int col,
                                           int clearance_margin_cells = 1);

    // 在 A* 折线路径基础上做轻量端点航向衔接：尝试在起点/终点附近加入
    // 与输入航向一致的短直线段；任何新增线段都必须通过安全余量检查。
    // 对障碍敏感时自动缩短，无法安全加入时保留原始 A* 路径。
    // enable_large_turn_split 由任务编排层限定到需要处理的途经点后路径段。
    Path planWithHeadingConstraints(
        const TerrainGrid& grid,
        const Pose2D& start,
        const Pose2D& goal,
        const GlobalPlannerConfig& config,
        bool align_start_heading,
        bool align_goal_heading,
        bool preserve_goal_approach_heading = false,
        bool enable_large_turn_split = false) const;

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
