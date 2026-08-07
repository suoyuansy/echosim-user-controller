// 文件功能：实现八邻域 A*、双向 A*、世界坐标转换和路径航向角生成。
#include "GlobalPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace
{
constexpr double kSqrtTwo = 1.4142135623730950488;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct SearchNode
{
    int index = -1;
    double g = 0.0;
    double f = 0.0;
};

struct SearchNodeCompare
{
    bool operator()(const SearchNode& left, const SearchNode& right) const
    {
        if (left.f != right.f)
            return left.f > right.f;
        return left.g > right.g;
    }
};

struct Neighbor
{
    int row = 0;
    int col = 0;
    double distance_factor = 1.0;
};

constexpr Neighbor kNeighbors[] = {
    {-1, -1, kSqrtTwo}, {-1, 0, 1.0}, {-1, 1, kSqrtTwo},
    {0, -1, 1.0},                         {0, 1, 1.0},
    {1, -1, kSqrtTwo},  {1, 0, 1.0},     {1, 1, kSqrtTwo}};

// 计算八邻域网格距离启发式代价。
double heuristic(int row, int col, int target_row, int target_col,
                 double resolution_m)
{
    const double row_delta = std::abs(target_row - row);
    const double col_delta = std::abs(target_col - col);
    const double diagonal = std::min(row_delta, col_delta);
    const double straight = std::max(row_delta, col_delta) - diagonal;
    return (diagonal * kSqrtTwo + straight) * resolution_m;
}

// 检查当前栅格及其八邻域，保证路径按八邻域安全规则通过。
bool is_clear_with_eight_neighbors_(const TerrainGrid& grid, int row, int col)
{
    for (int row_delta = -1; row_delta <= 1; ++row_delta)
    {
        for (int col_delta = -1; col_delta <= 1; ++col_delta)
        {
            const int neighbor_row = row + row_delta;
            const int neighbor_col = col + col_delta;
            if (!grid.inBounds(neighbor_row, neighbor_col)
                || !grid.isTraversable(neighbor_row, neighbor_col))
            {
                return false;
            }
        }
    }
    return true;
}

// 计算经过相邻栅格的地形代价边权。
double edge_cost_(const TerrainGrid& grid, int current_index, int next_index,
                double distance, double cost_weight)
{
    const double current_cost = std::clamp(
        static_cast<double>(grid.cost[current_index]), 0.0, 1.0);
    const double next_cost = std::clamp(
        static_cast<double>(grid.cost[next_index]), 0.0, 1.0);
    return distance * (1.0 + std::max(0.0, cost_weight)
        * 0.5 * (current_cost + next_cost));
}

// 按父节点数组恢复单向搜索结果。
std::vector<int> reconstruct_path_(int start_index, int goal_index,
                                  const std::vector<int>& parent,
                                  int cols)
{
    std::vector<int> indices;
    for (int index = goal_index; index >= 0; index = parent[index])
    {
        indices.push_back(index);
        if (index == start_index)
            break;
    }
    if (indices.empty() || indices.back() != start_index)
        return {};
    std::reverse(indices.begin(), indices.end());
    (void)cols;
    return indices;
}

// 清理优先队列中的过期节点。
void discard_stale_(std::priority_queue<SearchNode,
                                      std::vector<SearchNode>,
                                      SearchNodeCompare>& open,
                  const std::vector<double>& costs)
{
    while (!open.empty())
    {
        const SearchNode& node = open.top();
        if (node.index >= 0 && node.index < static_cast<int>(costs.size())
            && node.g <= costs[node.index] + 1e-9)
        {
            return;
        }
        open.pop();
    }
}
} // namespace

// 按配置执行全局规划，双向 A* 失败时回退到普通 A*。
Path GlobalPlanner::plan(const TerrainGrid& grid, const Pose2D& start,
                         const Pose2D& goal,
                         const GlobalPlannerConfig& config) const
{
    if (config.method == GlobalPlannerMethod::AStar)
        return planAStar(grid, start, goal, config.cost_weight);

    Path path = planBidirectionalAStar(grid, start, goal, config.cost_weight);
    if (path.empty())
        path = planAStar(grid, start, goal, config.cost_weight);
    return path;
}

// 执行带地形代价和八邻域安全约束的普通 A*。
Path GlobalPlanner::planAStar(const TerrainGrid& grid, const Pose2D& start,
                              const Pose2D& goal, double cost_weight) const
{
    GridPoint start_point;
    GridPoint goal_point;
    if (!to_grid_point_(grid, start, start_point)
        || !to_grid_point_(grid, goal, goal_point)
        || !is_clear_with_eight_neighbors_(grid, start_point.row, start_point.col)
        || !is_clear_with_eight_neighbors_(grid, goal_point.row, goal_point.col))
    {
        return {};
    }

    const int cell_count = grid.rows * grid.cols;
    const int start_index = start_point.row * grid.cols + start_point.col;
    const int goal_index = goal_point.row * grid.cols + goal_point.col;
    std::vector<double> costs(cell_count, kInfinity);
    std::vector<int> parent(cell_count, -1);
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> open;
    costs[start_index] = 0.0;
    open.push({start_index, 0.0, heuristic(start_point.row, start_point.col,
                                            goal_point.row, goal_point.col,
                                            grid.resolution_m)});

    while (!open.empty())
    {
        const SearchNode current = open.top();
        open.pop();
        if (current.g > costs[current.index] + 1e-9)
            continue;
        if (current.index == goal_index)
            break;

        const int row = current.index / grid.cols;
        const int col = current.index % grid.cols;
        for (const Neighbor& neighbor : kNeighbors)
        {
            const int next_row = row + neighbor.row;
            const int next_col = col + neighbor.col;
            if (!is_clear_with_eight_neighbors_(grid, next_row, next_col))
                continue;
            const int next_index = next_row * grid.cols + next_col;
            const double distance = neighbor.distance_factor * grid.resolution_m;
            const double tentative = current.g + edge_cost_(
                grid, current.index, next_index, distance, cost_weight);
            if (tentative >= costs[next_index])
                continue;
            costs[next_index] = tentative;
            parent[next_index] = current.index;
            open.push({next_index, tentative,
                       tentative + heuristic(next_row, next_col,
                                             goal_point.row, goal_point.col,
                                             grid.resolution_m)});
        }
    }

    const std::vector<int> indices = reconstruct_path_(
        start_index, goal_index, parent, grid.cols);
    std::vector<GridPoint> grid_path;
    grid_path.reserve(indices.size());
    for (const int index : indices)
        grid_path.push_back({index / grid.cols, index % grid.cols});
    return build_world_path_(grid, grid_path, start, goal);
}

// 执行从起点和终点同时扩展的双向 A*。
Path GlobalPlanner::planBidirectionalAStar(const TerrainGrid& grid,
                                           const Pose2D& start,
                                           const Pose2D& goal,
                                           double cost_weight) const
{
    GridPoint start_point;
    GridPoint goal_point;
    if (!to_grid_point_(grid, start, start_point)
        || !to_grid_point_(grid, goal, goal_point)
        || !is_clear_with_eight_neighbors_(grid, start_point.row, start_point.col)
        || !is_clear_with_eight_neighbors_(grid, goal_point.row, goal_point.col))
    {
        return {};
    }

    const int cell_count = grid.rows * grid.cols;
    const int start_index = start_point.row * grid.cols + start_point.col;
    const int goal_index = goal_point.row * grid.cols + goal_point.col;
    if (start_index == goal_index)
        return build_world_path_(grid, {{start_point.row, start_point.col}}, start, goal);

    std::vector<double> forward_cost(cell_count, kInfinity);
    std::vector<double> backward_cost(cell_count, kInfinity);
    std::vector<int> forward_parent(cell_count, -1);
    std::vector<int> backward_parent(cell_count, -1);
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> forward_open;
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> backward_open;
    forward_cost[start_index] = 0.0;
    backward_cost[goal_index] = 0.0;
    forward_open.push({start_index, 0.0, 0.0});
    backward_open.push({goal_index, 0.0, 0.0});

    double best_cost = kInfinity;
    int meeting_index = -1;
    auto update_meeting = [&](int index) {
        if (forward_cost[index] != kInfinity && backward_cost[index] != kInfinity)
        {
            const double candidate = forward_cost[index] + backward_cost[index];
            if (candidate < best_cost)
            {
                best_cost = candidate;
                meeting_index = index;
            }
        }
    };

    while (!forward_open.empty() && !backward_open.empty())
    {
        discard_stale_(forward_open, forward_cost);
        discard_stale_(backward_open, backward_cost);
        if (forward_open.empty() || backward_open.empty())
            break;
        const bool expand_forward = forward_open.size() <= backward_open.size();
        auto& open = expand_forward ? forward_open : backward_open;
        auto& own_cost = expand_forward ? forward_cost : backward_cost;
        auto& own_parent = expand_forward ? forward_parent : backward_parent;
        const SearchNode current = open.top();
        open.pop();
        update_meeting(current.index);
        const int row = current.index / grid.cols;
        const int col = current.index % grid.cols;
        for (const Neighbor& neighbor : kNeighbors)
        {
            const int next_row = row + neighbor.row;
            const int next_col = col + neighbor.col;
            if (!is_clear_with_eight_neighbors_(grid, next_row, next_col))
                continue;
            const int next_index = next_row * grid.cols + next_col;
            const double tentative = current.g + edge_cost_(
                grid, current.index, next_index,
                neighbor.distance_factor * grid.resolution_m, cost_weight);
            if (tentative >= own_cost[next_index])
                continue;
            own_cost[next_index] = tentative;
            own_parent[next_index] = current.index;
            open.push({next_index, tentative, tentative});
            update_meeting(next_index);
        }
        if (meeting_index >= 0 && !forward_open.empty() && !backward_open.empty()
            && forward_open.top().f + backward_open.top().f >= best_cost)
            break;
    }

    if (meeting_index < 0)
        return {};

    std::vector<int> indices;
    for (int index = meeting_index; index >= 0; index = forward_parent[index])
    {
        indices.push_back(index);
        if (index == start_index)
            break;
    }
    if (indices.empty() || indices.back() != start_index)
        return {};
    std::reverse(indices.begin(), indices.end());
    for (int index = backward_parent[meeting_index]; index >= 0;
         index = backward_parent[index])
    {
        indices.push_back(index);
        if (index == goal_index)
            break;
    }
    if (indices.back() != goal_index)
        return {};

    std::vector<GridPoint> grid_path;
    grid_path.reserve(indices.size());
    for (const int index : indices)
        grid_path.push_back({index / grid.cols, index % grid.cols});
    return build_world_path_(grid, grid_path, start, goal);
}

// 将世界位姿转换为栅格点。
bool GlobalPlanner::to_grid_point_(const TerrainGrid& grid, const Pose2D& pose,
                                GridPoint& point)
{
    if (!grid.worldToGrid(pose.x, pose.y, point.col, point.row))
        return false;
    return grid.inBounds(point.row, point.col);
}

// 将栅格折线转换为世界坐标，并生成中间点航向角。
Path GlobalPlanner::build_world_path_(const TerrainGrid& grid,
                                    const std::vector<GridPoint>& grid_path,
                                    const Pose2D& start,
                                    const Pose2D& goal)
{
    if (grid_path.empty())
        return {};
    Path path;
    path.reserve(grid_path.size() + 1);
    for (const GridPoint& point : grid_path)
    {
        double x = 0.0;
        double y = 0.0;
        grid.gridToWorld(point.row, point.col, x, y);
        path.push_back({x, y, 0.0});
    }
    path.front().x = start.x;
    path.front().y = start.y;
    if (path.size() == 1)
    {
        if (std::hypot(goal.x - start.x, goal.y - start.y) > 1e-9
            || std::abs(goal.yaw - start.yaw) > 1e-9)
            path.push_back({goal.x, goal.y, goal.yaw});
    }
    else
    {
        path.back().x = goal.x;
        path.back().y = goal.y;
    }
    for (std::size_t index = 1; index + 1 < path.size(); ++index)
    {
        const double dx = path[index + 1].x - path[index - 1].x;
        const double dy = path[index + 1].y - path[index - 1].y;
        path[index].yaw = std::hypot(dx, dy) > 1e-9
            ? std::atan2(dy, dx) : path[index - 1].yaw;
    }
    path.front().yaw = start.yaw;
    path.back().yaw = goal.yaw;
    return path;
}
