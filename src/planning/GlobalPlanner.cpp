// 文件功能：实现八邻域 A*、双向 A*、世界坐标转换和路径航向角生成。
#include "planning/GlobalPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace
{
constexpr double kSqrtTwo = 1.4142135623730950488; // 对角线边长系数 sqrt(2)。
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// 搜索节点：优先队列的基本元素。
struct SearchNode
{
    int index = -1; // 栅格的一维下标 row*cols+col；-1 表示尚未绑定栅格。
    double g = 0.0;  // 从该方向搜索源点到当前栅格的累计边代价（含地形权重，米量纲）。
    double f = 0.0;  // 排序键：单向 A* 中为 g+启发式；双向搜索中恒等于 g。
};

// 优先队列比较器。std::priority_queue 是最大堆，返回 left.f > right.f
// 使堆顶取最小 f；f 相同时取 g 较小者（同等估计代价下偏向离目标更近的节点），
// 这也保证相遇终止判定中堆顶 f 是各方向的下界。
struct SearchNodeCompare
{
    bool operator()(const SearchNode& left, const SearchNode& right) const
    {
        if (left.f != right.f)
            return left.f > right.f;
        return left.g > right.g;
    }
};

// 邻居偏移量：dr/dc 为相对当前栅格的行列增量，distance_factor 为该边的
// 长度系数（直线边 1、对角边 sqrt(2)），乘以分辨率即得到边长（米）。
struct Neighbor
{
    int row = 0;
    int col = 0;
    double distance_factor = 1.0;
};

// 八邻域展开表：上/下/左/右四个直线邻居和四个对角邻居。
constexpr Neighbor kNeighbors[] = {
    {-1, -1, kSqrtTwo}, {-1, 0, 1.0}, {-1, 1, kSqrtTwo},
    {0, -1, 1.0},                         {0, 1, 1.0},
    {1, -1, kSqrtTwo},  {1, 0, 1.0},     {1, 1, kSqrtTwo}};

// 计算八邻域网格距离启发式代价。
// 八邻域（octile）距离：先走对角线到行列对齐，再走直线；这是八连通网格上
// 两点间的真实最短几何距离，因此该启发式可采纳（不高估），保证 A* 最优性。
// 输入为栅格行列，输出乘分辨率换算为米。
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
// 要求中心格及周围 8 格全部可通行，等效于给每个路径点留出 1 格（分辨率米）
// 安全余量：既防止车体擦过障碍，也杜绝相邻两步斜穿两个障碍夹角的情况。
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
// 边代价 = 段长(米) * (1 + cost_weight * 两端栅格代价的平均值)。
// 取两端平均使代价在边的两个端点间平滑过渡；cost_weight<=0 时被钳到 0，
// 边代价退化为纯距离。输入下标为一维栅格下标，distance 单位为米。
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
// 从 goal 沿 parent 链回溯到 start，再反转得到正向路径；若链在中途断裂
// （goal 从未被扩展到）则返回空。cols 当前未使用，保留以兼容旧签名。
std::vector<int> reconstruct_path_(int start_index, int goal_index,
                                  const std::vector<int>& parent,
                                  int cols)
{
    std::vector<int> indices;
    // 逆序回溯：goal -> ... -> start。
    for (int index = goal_index; index >= 0; index = parent[index])
    {
        indices.push_back(index);
        if (index == start_index)
            break;
    }
    // 链头不是 start 说明 goal 与 start 不连通。
    if (indices.empty() || indices.back() != start_index)
        return {};
    std::reverse(indices.begin(), indices.end());
    (void)cols;
    return indices;
}

// 清理优先队列中的过期节点。
// 惰性删除策略：队列中可能残留同一点的旧副本（g 更大），只有当
// node.g 与当前最优 costs[index] 一致（含 1e-9 容差）时才是有效条目，
// 其余在弹出前先剔除，保证堆顶总是每个栅格的最新最优节点。
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
// 输入均为世界坐标（米/弧度）。双向搜索在"起终点位于狭窄死区/被安全
// 规则切断"等场景下可能失败，此时用单向 A* 再试一次以提高鲁棒性。
Path GlobalPlanner::plan(const TerrainGrid& grid, const Pose2D& start,
                         const Pose2D& goal,
                         const GlobalPlannerConfig& config) const
{
    if (config.method == GlobalPlannerMethod::AStar)
        return planAStar(grid, start, goal, config.cost_weight);

    // 双向 A* 失败回退到普通 A*：两者可通行性判定一致，回退只在搜索
    // 策略层面兜底。
    Path path = planBidirectionalAStar(grid, start, goal, config.cost_weight);
    if (path.empty())
        path = planAStar(grid, start, goal, config.cost_weight);
    return path;
}

// 执行带地形代价和八邻域安全约束的普通 A*。
// 输入 start/goal 为世界坐标（米/弧度），返回世界坐标路径；空路径表示
// 起终点无效（越界或自身不满足八邻域安全规则）或不存在连通路径。
Path GlobalPlanner::planAStar(const TerrainGrid& grid, const Pose2D& start,
                              const Pose2D& goal, double cost_weight) const
{
    // 起终点栅格化后先做八邻域安全检查：任一端自身处在障碍贴邻位置
    // 即判为不可解，避免搜索出终点紧贴障碍的路径。
    GridPoint start_point;
    GridPoint goal_point;
    if (!to_grid_point_(grid, start, start_point)
        || !to_grid_point_(grid, goal, goal_point)
        || !is_clear_with_eight_neighbors_(grid, start_point.row, start_point.col)
        || !is_clear_with_eight_neighbors_(grid, goal_point.row, goal_point.col))
    {
        return {};
    }

    // 搜索状态：costs 即 g 值数组（也隐式充当 close 集合--被扩展过的
    // 节点 g 有限），parent 记录前驱用于回溯，open 为惰性删除的最小堆。
    const int cell_count = grid.rows * grid.cols;
    const int start_index = start_point.row * grid.cols + start_point.col;
    const int goal_index = goal_point.row * grid.cols + goal_point.col;
    std::vector<double> costs(cell_count, kInfinity);
    std::vector<int> parent(cell_count, -1);
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> open;
    costs[start_index] = 0.0;
    // 起点入堆，f = g + 启发式（八邻域距离，米）。
    open.push({start_index, 0.0, heuristic(start_point.row, start_point.col,
                                            goal_point.row, goal_point.col,
                                            grid.resolution_m)});

    while (!open.empty())
    {
        const SearchNode current = open.top();
        open.pop();
        // 惰性删除：堆顶是过期副本（已有更小 g）则跳过。
        if (current.g > costs[current.index] + 1e-9)
            continue;
        // 堆顶即目标：启发式可采纳保证此时 g(goal) 已是最优，提前终止。
        if (current.index == goal_index)
            break;

        // 一维下标反解行列号，供八邻域展开使用。
        const int row = current.index / grid.cols;
        const int col = current.index % grid.cols;
        // 邻居展开：8 个方向逐一检查安全规则、松弛边、更新 g 与父节点。
        for (const Neighbor& neighbor : kNeighbors)
        {
            const int next_row = row + neighbor.row;
            const int next_col = col + neighbor.col;
            // 邻居自身及其八邻域必须全部可通行（含安全余量与防斜穿夹角）。
            if (!is_clear_with_eight_neighbors_(grid, next_row, next_col))
                continue;
            const int next_index = next_row * grid.cols + next_col;
            // 边长 = 长度系数(1 或 sqrt(2)) * 分辨率（米）。
            const double distance = neighbor.distance_factor * grid.resolution_m;
            // 松弛：经当前点到 next 的代价更小才更新。
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

    // 回溯栅格路径并转换为世界坐标路径（首末端点用精确起终点替换）。
    const std::vector<int> indices = reconstruct_path_(
        start_index, goal_index, parent, grid.cols);
    std::vector<GridPoint> grid_path;
    grid_path.reserve(indices.size());
    for (const int index : indices)
        grid_path.push_back({index / grid.cols, index % grid.cols});
    return build_world_path_(grid, grid_path, start, goal);
}

// 执行从起点和终点同时扩展的双向 A*。
// 两个方向均按 Dijkstra 方式扩展（f = g，不用启发式，因为反向搜索的
// 启发式方向相反），通过平衡两边的开放集规模交替扩展；相遇判定基于
// "两方向都到达过的栅格"维护全局最优拼接代价，终止条件是两个堆顶的
// f 值之和不再小于已知最优相遇代价。输入输出单位与 planAStar 相同。
Path GlobalPlanner::planBidirectionalAStar(const TerrainGrid& grid,
                                           const Pose2D& start,
                                           const Pose2D& goal,
                                           double cost_weight) const
{
    // 与单向 A* 相同的预处理：栅格化 + 起终点八邻域安全检查。
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
    // 退化情形：起终点落进同一栅格，直接输出单点路径（build_world_path_
    // 内部会用精确起终点坐标补齐为两点）。
    if (start_index == goal_index)
        return build_world_path_(grid, {{start_point.row, start_point.col}}, start, goal);

    // 正向搜索状态：从 start 出发；backward_* 为反向：从 goal 出发。
    // 两组 g 值/父节点数组完全独立，等价于并行跑两个 Dijkstra。
    std::vector<double> forward_cost(cell_count, kInfinity);
    std::vector<double> backward_cost(cell_count, kInfinity);
    std::vector<int> forward_parent(cell_count, -1);
    std::vector<int> backward_parent(cell_count, -1);
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> forward_open;
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> backward_open;
    forward_cost[start_index] = 0.0;
    backward_cost[goal_index] = 0.0;
    // 双向搜索的 f 恒等于 g（Dijkstra 式扩展），两堆初始各放搜索源点。
    forward_open.push({start_index, 0.0, 0.0});
    backward_open.push({goal_index, 0.0, 0.0});

    // 相遇判定核心状态：best_cost 记录目前最优的"起点->相遇点->终点"
    // 拼接代价（米），meeting_index 为对应相遇栅格。
    double best_cost = kInfinity;
    int meeting_index = -1;
    // 相遇判定：某栅格同时被正反两个方向到达（两侧 g 均有限）即为候选
    // 相遇点，拼接代价 = forward_cost + backward_cost；取历史最小者。
    // 注意这会持续更新，即使之后还有更优相遇点也不会漏掉。
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
        // 每轮先剔除两个堆中的过期节点，保证堆顶 f 是真实的当前最优下界。
        discard_stale_(forward_open, forward_cost);
        discard_stale_(backward_open, backward_cost);
        if (forward_open.empty() || backward_open.empty())
            break;
        // 扩展策略：交替扩展开放集较小的一侧，保持两边搜索波前平衡，
        // 避免一侧先铺满整张地图。
        const bool expand_forward = forward_open.size() <= backward_open.size();
        auto& open = expand_forward ? forward_open : backward_open;
        auto& own_cost = expand_forward ? forward_cost : backward_cost;
        auto& own_parent = expand_forward ? forward_parent : backward_parent;
        const SearchNode current = open.top();
        open.pop();
        // 弹出节点本身可能是新的相遇点（对侧此前已到达过它）。
        update_meeting(current.index);
        const int row = current.index / grid.cols;
        const int col = current.index % grid.cols;
        // 与单向 A* 相同的邻居展开/松弛，只是作用于当前方向的代价与父数组。
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
            // f = g：Dijkstra 式扩展，保证本侧 g 值按非降序弹出。
            open.push({next_index, tentative, tentative});
            // 新松弛的邻居也可能与对侧相遇，立即参与相遇判定。
            update_meeting(next_index);
        }
        // 最优性终止条件：两个堆顶 f 分别是两侧"再扩展任意节点"的代价
        // 下界，其和 >= best_cost 说明不可能再出现更优的相遇路径，
        // 可以安全停止（保证拼接路径最优）。
        if (meeting_index >= 0 && !forward_open.empty() && !backward_open.empty()
            && forward_open.top().f + backward_open.top().f >= best_cost)
            break;
    }

    // 两方向从未到达同一栅格：无可行路径。
    if (meeting_index < 0)
        return {};

    // 路径拼接第一段：从相遇点沿正向父链回溯到起点，反转得到
    // start -> ... -> meeting 的正向序列。
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
    // 第二段：从相遇点的反向父链回溯，追加 meeting -> ... -> goal。
    for (int index = backward_parent[meeting_index]; index >= 0;
         index = backward_parent[index])
    {
        indices.push_back(index);
        if (index == goal_index)
            break;
    }
    // 反向链没有连到 goal（理论不应发生）则视为搜索失败。
    if (indices.back() != goal_index)
        return {};

    // 一维下标转行列号，再交给 build_world_path_ 输出世界坐标路径。
    std::vector<GridPoint> grid_path;
    grid_path.reserve(indices.size());
    for (const int index : indices)
        grid_path.push_back({index / grid.cols, index % grid.cols});
    return build_world_path_(grid, grid_path, start, goal);
}

// 将世界位姿转换为栅格点。
// 输入 pose 为世界坐标（米）；worldToGrid 内部按四舍五入取最近栅格，
// 再显式做一次边界检查（空地图时 worldToGrid 返回 false）。
bool GlobalPlanner::to_grid_point_(const TerrainGrid& grid, const Pose2D& pose,
                                GridPoint& point)
{
    if (!grid.worldToGrid(pose.x, pose.y, point.col, point.row))
        return false;
    return grid.inBounds(point.row, point.col);
}

// 将栅格折线转换为世界坐标，并生成中间点航向角。
// 步骤：栅格中心 -> 世界坐标（米） -> 首末端点替换为精确的 start/goal
// 位姿（消除栅格化误差） -> 中间点航向角取前后相邻点的连线方向（弧度）。
// 输出 Path 供优化/跟踪模块使用，首末航向角分别沿用输入位姿的 yaw。
Path GlobalPlanner::build_world_path_(const TerrainGrid& grid,
                                    const std::vector<GridPoint>& grid_path,
                                    const Pose2D& start,
                                    const Pose2D& goal)
{
    if (grid_path.empty())
        return {};
    Path path;
    path.reserve(grid_path.size() + 1);
    // 第一步：每个栅格中心转为世界坐标，yaw 暂置 0，稍后统一填充。
    for (const GridPoint& point : grid_path)
    {
        double x = 0.0;
        double y = 0.0;
        grid.gridToWorld(point.row, point.col, x, y);
        path.push_back({x, y, 0.0});
    }
    // 第二步：首点替换为精确起点，消除"起点被吸附到栅格中心"的半格误差。
    path.front().x = start.x;
    path.front().y = start.y;
    if (path.size() == 1)
    {
        // 单点退化：起终点同栅格时补一个精确终点点，保证跟踪模块
        // 拿到的是真实的起终点而不是同一个栅格中心。
        if (std::hypot(goal.x - start.x, goal.y - start.y) > 1e-9
            || std::abs(goal.yaw - start.yaw) > 1e-9)
            path.push_back({goal.x, goal.y, goal.yaw});
    }
    else
    {
        // 末点同样替换为精确终点坐标。
        path.back().x = goal.x;
        path.back().y = goal.y;
    }
    // 第三步：中间点航向角 = 前一点到后一点连线的方位角 atan2(dy, dx)
    // （弧度，X 轴正向为 0，逆时针为正）；两点几乎重合时沿用前一点航向，
    // 避免 atan2 数值噪声。
    for (std::size_t index = 1; index + 1 < path.size(); ++index)
    {
        const double dx = path[index + 1].x - path[index - 1].x;
        const double dy = path[index + 1].y - path[index - 1].y;
        path[index].yaw = std::hypot(dx, dy) > 1e-9
            ? std::atan2(dy, dx) : path[index - 1].yaw;
    }
    // 首末航向角分别使用输入位姿的精确 yaw（弧度）。
    path.front().yaw = start.yaw;
    path.back().yaw = goal.yaw;
    return path;
}
