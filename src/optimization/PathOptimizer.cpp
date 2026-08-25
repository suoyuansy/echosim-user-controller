// 文件功能：实现视线捷径（Theta* 式）、迭代平滑与尖角圆弧化，全程用代价地图
// 做碰撞/余量校验，并以阿克曼运动学曲率上限约束每个折角，保证输出路径可执行。
#include "optimization/PathOptimizer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
// 圆周率常量（仅用于最大前轮角 < 90 度的合法性判断等）。
constexpr double kPi = 3.14159265358979323846;

// 两点间欧氏距离。
// 输入：世界坐标下两个路径点（米）；返回：距离（米），非负。
double distance_(const PathPoint& from, const PathPoint& to)
{
    return std::hypot(to.x - from.x, to.y - from.y);
}

// 检查某栅格及其 margin 圈邻域是否都可通行（与规划器八邻域安全规则一致）。
// 输入：栅格行列号与余量圈数（格）；返回：true 表示该 (2*margin+1)^2 方形
// 邻域内全部可通行。等价于把障碍层向外膨胀 margin 圈后再查单点。
bool cell_is_clear_(const TerrainGrid& grid, int row, int col, int margin)
{
    // 行方向偏移 dr、列方向偏移 dc 各取 [-margin, margin]，覆盖方形邻域。
    for (int dr = -margin; dr <= margin; ++dr)
    {
        for (int dc = -margin; dc <= margin; ++dc)
        {
            // 任一邻域栅格不可通行（障碍或越出地图范围）即整体判不安全。
            if (!grid.isTraversable(row + dr, col + dc))
                return false;
        }
    }
    return true;
}

bool footprint_is_clear_at_(const TerrainGrid& grid, double x, double y,
                            double yaw, double length_m, double width_m,
                            double sample_step_m)
{
    const double step = std::clamp(sample_step_m, 0.2, grid.resolution_m);
    const int length_steps = std::max(1, static_cast<int>(std::ceil(
        length_m / step)));
    const int width_steps = std::max(1, static_cast<int>(std::ceil(
        width_m / step)));
    const double half_length = 0.5 * length_m;
    const double half_width = 0.5 * width_m;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (int length_index = 0; length_index <= length_steps; ++length_index)
    {
        const double forward = -half_length + length_m * length_index
            / static_cast<double>(length_steps);
        for (int width_index = 0; width_index <= width_steps; ++width_index)
        {
            const double side = -half_width + width_m * width_index
                / static_cast<double>(width_steps);
            const double sample_x = x + forward * cos_yaw - side * sin_yaw;
            const double sample_y = y + forward * sin_yaw + side * cos_yaw;
            int col = 0;
            int row = 0;
            if (!grid.worldToGrid(sample_x, sample_y, col, row)
                || !grid.isTraversable(row, col))
                return false;
        }
    }
    return true;
}

// 判断两点间直线是否全程满足余量要求，采用定步长采样（端点本身已由路径保证）。
// 输入：起终点为世界坐标（米），sample_step 为采样步长（米），margin 为
// 栅格余量圈数；返回：true 表示线段上每个采样点连同其 margin 圈邻域均安全。
bool line_is_clear_(const TerrainGrid& grid, const PathPoint& from,
                    const PathPoint& to, double sample_step, int margin,
                    double vehicle_length_m, double vehicle_width_m,
                    double footprint_step_m)
{
    // 线段长度（米）与按采样步长向上取整折算的等分段数；至少 1 段，
    // 保证极短线段也能进入采样循环且除法不失效。
    const double segment_length = distance_(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(
        segment_length / sample_step)));
    // 只采样内部等分点 k = 1..steps-1：两个端点已在既有路径上，
    // 由外层逐段校验保证安全，无需重复检查。
    for (int k = 1; k < steps; ++k)
    {
        // 等分参数 t 属于 (0, 1)，线性插值得到采样点世界坐标（米）。
        const double t = static_cast<double>(k) / static_cast<double>(steps);
        const double x = from.x + t * (to.x - from.x);
        const double y = from.y + t * (to.y - from.y);
        int col = 0;
        int row = 0;
        // 世界坐标落到代价地图范围之外：直接判不可通行（不许贴边出图）。
        if (!grid.worldToGrid(x, y, col, row))
            return false;
        // 采样点所在栅格连同 margin 圈邻域必须全部可通行。
        if (!cell_is_clear_(grid, row, col, margin))
            return false;
        const double yaw = std::atan2(to.y - from.y, to.x - from.x);
        if (!footprint_is_clear_at_(grid, x, y, yaw, vehicle_length_m,
                                    vehicle_width_m, footprint_step_m))
            return false;
    }
    return true;
}

// 三点外接圆曲率（Menger 曲率）；共线或点重合时返回 0。
// 输入：世界坐标下相邻三个路径点（米）；返回：曲率（1/米），等于外接圆
// 半径的倒数。公式 kappa = 2*S/(|AB|*|BC|*|CA|)，其中 S 为三角形 ABC
// 叉积面积的两倍；三点共线时面积为 0，曲率自然为 0。
double menger_curvature_(const PathPoint& a, const PathPoint& b, const PathPoint& c)
{
    // 三条边长（米）。
    const double ab = distance_(a, b);
    const double bc = distance_(b, c);
    const double ca = distance_(c, a);
    // 退化保护：任一边短于 1e-9 米视为点重合，曲率无定义，返回 0。
    if (ab < 1e-9 || bc < 1e-9 || ca < 1e-9)
        return 0.0;
    // 叉积 = 平行四边形有向面积 = 三角形面积的两倍；取绝对值不分转向，
    // 曲率只取大小，方向信息由别处（圆弧 sweep 符号）处理。
    const double doubled_area = std::abs((b.x - a.x) * (c.y - a.y)
                                         - (b.y - a.y) * (c.x - a.x));
    // kappa = 2*叉积面积/(三边乘积) = 1/外接圆半径，单位 1/米。
    return 2.0 * doubled_area / (ab * bc * ca);
}

// 顶点折角处的切向圆弧：用半径 radius 的圆弧替换 prev->vertex->next 折角，
// 两侧各从顶点回退切线长度 t = radius*tan(turn/2)。tangent_fraction 是切线
// 长度占相邻段长的最大比例（取 0.5 保证相邻折角的圆弧互不重叠）；段太短或
// 转角接近 180 度放不下圆弧时 valid 为 false，即该折角超出车辆转向能力。
struct CornerArc_
{
    bool valid = false;        // 几何上能否用该半径的圆弧替换折角。
    double turn = 0.0;         // 外转角（来向与去向方向夹角，弧度）。
    PathPoint entry;           // 来向直段上的切入点。
    PathPoint exit;            // 去向直段上的切出点。
    double center_x = 0.0;     // 圆弧圆心。
    double center_y = 0.0;
    double radius = 0.0;       // 圆弧半径。
    double start_angle = 0.0;  // 圆心到切入点向量的方位角。
    double sweep = 0.0;        // 带符号扫掠角（逆时针为正）。
};

// 构造顶点折角处的切向圆弧（纯几何，不含障碍校验，后者由 arc_is_clear_ 负责）。
// 几何推导：设来向单位方向 u1 = (vertex-prev)/|.|，去向单位方向 u2 =
// (next-vertex)/|.|，外转角 turn = acos(u1·u2)（弧度，[0, pi]）。
// 半径 radius 的圆弧与两段直线相切，切点距顶点的切线长度
// t = radius * tan(turn/2)（直角三角形 vertex-entry-center 的关系）。
// 切入点 entry = vertex - t*u1，切出点 exit = vertex + t*u2；
// 圆心在切入点处沿 u1 的法向偏移一个半径（偏向转向一侧）。
// 输入：vertex 为折角顶点，prev/next 为前后路径点（世界坐标，米），
// radius 为圆弧半径（米），tangent_fraction 为切线长度占相邻段长的上限比例。
// 返回：CornerArc_，几何放不下时 valid 为 false。
CornerArc_ build_corner_arc_(const PathPoint& vertex, const PathPoint& prev,
                             const PathPoint& next, double radius,
                             double tangent_fraction)
{
    CornerArc_ arc;
    // 半径非正（未启用曲率约束）时无法构造圆弧，直接返回无效结果。
    if (radius <= 0.0)
        return arc;
    // 来向/去向位移向量（米）。
    double d1x = vertex.x - prev.x;
    double d1y = vertex.y - prev.y;
    double d2x = next.x - vertex.x;
    double d2y = next.y - vertex.y;
    // 两侧段长（米）。
    const double len1 = std::hypot(d1x, d1y);
    const double len2 = std::hypot(d2x, d2y);
    // 退化保护：任一侧段长不足 1e-9 米（点重合）无方向可言。
    if (len1 < 1e-9 || len2 < 1e-9)
        return arc;
    // 归一化为单位方向向量 u1、u2。
    d1x /= len1;
    d1y /= len1;
    d2x /= len2;
    d2y /= len2;
    // 叉积符号给出转向一侧（左转为正），点积给出转角余弦。
    const double cross = d1x * d2y - d1y * d2x;
    const double dot = d1x * d2x + d1y * d2y;
    // 外转角（弧度）；clamp 到 [-1, 1] 防浮点误差导致 acos 定义域越界。
    arc.turn = std::acos(std::clamp(dot, -1.0, 1.0));
    arc.radius = radius;
    if (arc.turn < 1e-6)
    {
        // 近似直线：无需圆弧，视为可行（sweep 保持 0，圆弧化阶段直接保留顶点）。
        arc.valid = true;
        return arc;
    }
    // 切线长度 t = R*tan(turn/2)（米）；超过相邻段长的 tangent_fraction 倍
    // （调用方传 0.5，保证相邻折角的圆弧互不重叠、不越过段中点）则
    // 几何上放不下，返回无效。
    const double tangent = radius * std::tan(0.5 * arc.turn);
    if (tangent > tangent_fraction * len1 || tangent > tangent_fraction * len2)
        return arc;
    // side：+1 表示左转（圆心在行进方向左侧），-1 表示右转。
    const double side = cross >= 0.0 ? 1.0 : -1.0; // 转向一侧（左转为正）。
    // 切入点：沿来向从顶点回退切线长度 t；切出点：沿去向从顶点前伸 t。
    arc.entry.x = vertex.x - tangent * d1x;
    arc.entry.y = vertex.y - tangent * d1y;
    arc.exit.x = vertex.x + tangent * d2x;
    arc.exit.y = vertex.y + tangent * d2y;
    // 圆心位于切入点法向（转向一侧）：entry + side*radius*(-d1y, d1x)。
    // （u1 旋转 90 度得法向，乘 side 选出指向圆心的一侧。）
    arc.center_x = arc.entry.x - side * radius * d1y;
    arc.center_y = arc.entry.y + side * radius * d1x;
    // 起始角：圆心指向切入点的方位角（弧度，atan2 约定）。
    arc.start_angle = std::atan2(arc.entry.y - arc.center_y,
                                 arc.entry.x - arc.center_x);
    // 扫掠角：大小等于外转角（切线垂直于半径，圆心角与转角互补相等），
    // 符号随转向侧，逆时针（左转）为正。
    arc.sweep = side * arc.turn;
    arc.valid = true;
    return arc;
}

// 判断圆弧全程（含切入/切出点）是否满足障碍余量要求。
// 输入：arc 为已构造的切向圆弧，sample_step 为采样步长（米），margin 为
// 栅格余量圈数；返回：true 表示圆弧上每个采样点均安全。
bool arc_is_clear_(const TerrainGrid& grid, const CornerArc_& arc,
                   double sample_step, int margin, double vehicle_length_m,
                   double vehicle_width_m, double footprint_step_m)
{
    // 圆弧弧长 = |sweep|*radius（米），按采样步长向上取整折算等分角数。
    // 与直线检查不同，这里连端点（切入/切出点）也一并校验（k = 0..steps），
    // 因为这两个点是新引入路径的点，未经上游保证。
    const double sweep_abs = std::abs(arc.sweep);
    const int steps = std::max(1, static_cast<int>(std::ceil(
        sweep_abs * arc.radius / sample_step)));
    for (int k = 0; k <= steps; ++k)
    {
        // 从起始角沿带符号扫掠方向等分插值圆心角（弧度）。
        const double angle = arc.start_angle
            + arc.sweep * (static_cast<double>(k) / static_cast<double>(steps));
        // 圆心角参数化反算圆弧上的世界坐标（米）。
        const double x = arc.center_x + arc.radius * std::cos(angle);
        const double y = arc.center_y + arc.radius * std::sin(angle);
        int col = 0;
        int row = 0;
        // 落到地图范围之外直接判不可通行。
        if (!grid.worldToGrid(x, y, col, row))
            return false;
        // 采样点所在栅格连同 margin 圈邻域必须全部可通行。
        if (!cell_is_clear_(grid, row, col, margin))
            return false;
        const double tangent_yaw = angle
            + (arc.sweep >= 0.0 ? 0.5 * kPi : -0.5 * kPi);
        if (!footprint_is_clear_at_(grid, x, y, tangent_yaw,
                                    vehicle_length_m, vehicle_width_m,
                                    footprint_step_m))
            return false;
    }
    return true;
}

// 捷径角点可行性：锚点处由来向直段与候选直段构成的折角，必须能用半径
// corner_radius 的切向圆弧替换（几何放得下，且切角圆弧满足障碍余量——
// 圆弧向转角内侧切削，处于两条已校验直段之间的区域，必须单独校验）。
// 不可行则该候选跨度不可接受，回退到更近的候选点。
bool shortcut_corner_acceptable_(const Path& result, const Path& raw,
                                 std::size_t anchor, std::size_t farthest,
                                 const TerrainGrid& grid, double corner_radius,
                                 double sample_step, int margin,
                                 double vehicle_length_m, double vehicle_width_m,
                                 double footprint_step_m)
{
    // 未启用曲率约束（corner_radius <= 0），或起点尚无来向直段
    // （result 只有首点）时，无需折角校验，直接接受。
    if (corner_radius <= 0.0 || result.size() < 2)
        return true; // 未启用曲率约束，或起点无来向直段。
    // 锚点处的折角由来向直段（result 末两点方向）与候选直段（锚点->farthest）
    // 构成：vertex 取 raw[anchor]，prev 取已确认路径的倒数第二个点，
    // next 取候选直达点 raw[farthest]，重建 corner_radius 切向圆弧。
    const CornerArc_ arc = build_corner_arc_(raw[anchor],
                                             result[result.size() - 2],
                                             raw[farthest], corner_radius, 0.5);
    // 几何放不下（切线超长/转角过钝）即该折角超出车辆转向能力，拒绝。
    if (!arc.valid)
        return false;
    // 折角近似直线（扫掠角近 0）时无需圆弧，也无需障碍校验。
    if (std::abs(arc.sweep) < 1e-6)
        return true;
    // 圆弧向转角内侧切削，落在两条已校验直段之间的未校验区域，必须单独校验。
    return arc_is_clear_(grid, arc, sample_step, margin, vehicle_length_m,
                         vehicle_width_m, footprint_step_m);
}

// 视线捷径：从锚点向后扫描最远的可直线直达且角点运动学可行的点，
// 跳过中间栅格折线。
// 输入：raw 为原始栅格路径（世界坐标，米），max_distance_m 为单段最大
// 跨度（米），sample_step/margin 为直线采样与余量参数，corner_radius 为
// 折角圆弧半径（米，0 表示不做折角校验）；返回：精简后的路径（首尾点必保留）。
Path shortcut_(const Path& raw, const TerrainGrid& grid, double max_distance_m,
               double sample_step, int margin, double corner_radius,
               double vehicle_length_m, double vehicle_width_m,
               double footprint_step_m)
{
    Path result;
    result.reserve(raw.size());
    result.push_back(raw.front()); // 起点固定保留。
    // 单次捷径搜索的最大点数跨度：最大距离（米）/ 栅格分辨率（米/格）折算，
    // 限制最坏情况下的视线检查次数（每候选点一次全段采样扫描）。
    const std::size_t max_span_points = static_cast<std::size_t>(
        std::ceil(max_distance_m / grid.resolution_m)) + 1;
    std::size_t anchor = 0;
    // 贪心扫描：anchor 为已确认的输出末点，直到覆盖到原始路径末点为止。
    while (anchor + 1 < raw.size())
    {
        // 候选直达点上界：锚点后 max_span_points 个点与路径末点取较小者。
        std::size_t farthest = std::min(raw.size() - 1, anchor + max_span_points);
        // 从远到近逐个回退：候选直线的余量校验（line_is_clear_）与锚点处
        // 折角的圆弧化可行性（shortcut_corner_acceptable_）任一不过就退一格；
        // 最坏退到 anchor+1，即保留原始相邻段（原折线必可行）。
        while (farthest > anchor + 1
               && (!line_is_clear_(grid, raw[anchor], raw[farthest],
                                   sample_step, margin, vehicle_length_m,
                                   vehicle_width_m, footprint_step_m)
                   || !shortcut_corner_acceptable_(result, raw, anchor, farthest,
                                                   grid, corner_radius,
                                                   sample_step, margin,
                                                   vehicle_length_m,
                                                   vehicle_width_m,
                                                   footprint_step_m)))
        {
            --farthest;
        }
        // 确认该点为下一个捷径航点，并以其为新锚点继续向后扫描。
        result.push_back(raw[farthest]);
        anchor = farthest;
    }
    return result;
}

// 平滑候选点移动后，受影响顶点（i-1、i、i+1）的折角仍须可被 corner_radius
// 圆弧替换。稀疏航点尺度上这是"折角车辆能否执行"的正确度量：外接圆曲率在
// 数百米段长下几乎恒为零（两段 200 m 直线夹 90 度折角的外接圆半径约 141 m），
// 不能反映折角锐度；能否放下 R_plan 切向圆弧才能。
bool smoothing_keeps_corners_roundable_(const Path& path, std::size_t i,
                                        const PathPoint& candidate,
                                        double corner_radius)
{
    // 未启用曲率约束时跳过门控，允许任意平滑移动。
    if (corner_radius <= 0.0)
        return true;
    // 候选点只牵动相邻三个折角（i-1、i、i+1 处），逐一用旧路径上的
    // 未动点 + 候选点重建 corner_radius 切向圆弧，验证仍放得下。
    // i-1 处折角：path[i-1] 为顶点，来向 path[i-2]，去向为候选点。
    if (i >= 2
        && !build_corner_arc_(path[i - 1], path[i - 2], candidate,
                              corner_radius, 0.5).valid)
        return false;
    // i 处折角：候选点为顶点，来向 path[i-1]，去向 path[i+1]。
    if (!build_corner_arc_(candidate, path[i - 1], path[i + 1],
                           corner_radius, 0.5).valid)
        return false;
    // i+1 处折角：path[i+1] 为顶点，来向为候选点，去向 path[i+2]。
    if (i + 2 < path.size()
        && !build_corner_arc_(path[i + 1], candidate, path[i + 2],
                              corner_radius, 0.5).valid)
        return false;
    return true;
}

// 迭代平滑：每个内部点向两侧邻点的加权平均移动，移动后重新校验直线余量
// 与折角可圆弧化，任一校验不过则保留原位置（避免把点推进障碍或制造
// 超出车辆转向能力的折角）。
// 输入/输出：path 原地更新（首尾点不变）；weight 为平滑权重 (0, 0.5]；
// 收敛方式为固定轮数的定点迭代，每轮"读旧写新"，未收敛即停。
void smooth_(Path& path, const TerrainGrid& grid, int iterations,
             double weight, double sample_step, int margin, double corner_radius,
             double vehicle_length_m, double vehicle_width_m,
             double footprint_step_m)
{
    // 外层：平滑迭代轮数，由配置指定（smoothing_iterations）。
    // 每轮基于上一轮结果整条重建，校验失败的点原地保留、下一轮仍可继续移动。
    for (int iteration = 0; iteration < iterations; ++iteration)
    {
        Path smoothed;
        smoothed.reserve(path.size());
        // 首点（起点）不参与平滑，保证端点约束。
        smoothed.push_back(path.front());
        // 内部点 i = 1..size-2：逐点生成候选位置并校验。
        for (std::size_t i = 1; i + 1 < path.size(); ++i)
        {
            const PathPoint& prev = path[i - 1];
            const PathPoint& current = path[i];
            const PathPoint& next = path[i + 1];
            PathPoint candidate;
            // 候选位置 = 邻域加权平均：w*prev + (1-2w)*current + w*next，
            // w 属于 (0, 0.5]；w 越大点被拉向邻点越狠，转角越圆，
            // w=0.5 时退化为两邻点中点（最大平滑力度）。
            candidate.x = weight * prev.x + (1.0 - 2.0 * weight) * current.x
                + weight * next.x;
            candidate.y = weight * prev.y + (1.0 - 2.0 * weight) * current.y
                + weight * next.y;
            // 航向暂不重算（平滑结束后由 recompute_yaw_ 统一差分）。
            candidate.yaw = current.yaw;
            // 三重安全校验：prev->candidate 与 candidate->next 两段直线余量，
            // 加上受影响的相邻折角仍可被 corner_radius 圆弧替换；
            // 全部通过才接受移动，否则该点本轮保持原位。
            if (line_is_clear_(grid, prev, candidate, sample_step, margin,
                               vehicle_length_m, vehicle_width_m,
                               footprint_step_m)
                && line_is_clear_(grid, candidate, next, sample_step, margin,
                                  vehicle_length_m, vehicle_width_m,
                                  footprint_step_m)
                && smoothing_keeps_corners_roundable_(path, i, candidate,
                                                      corner_radius))
            {
                smoothed.push_back(candidate);
            }
            else
            {
                smoothed.push_back(current);
            }
        }
        // 尾点（终点）同样不参与平滑。
        smoothed.push_back(path.back());
        // 用本轮结果覆盖输入，供下一轮迭代（或下游圆弧化）使用。
        path.swap(smoothed);
    }
}

// 尖角圆弧化：平滑后仍存在的折角，用半径 radius 的切向圆弧替换（删除顶点，
// 两侧各回退切线长度，圆弧按采样步长离散插入）。切线放不下或圆弧不满足
// 障碍余量时保留原折点，由跟踪端弯道限速兜底；unrounded_count 统计
// 因运动学或余量原因未能圆弧化的真实折角数。
Path round_corners_(const Path& path, const TerrainGrid& grid, double radius,
                    double sample_step, int margin, int& unrounded_count,
                    double vehicle_length_m, double vehicle_width_m,
                    double footprint_step_m)
{
    unrounded_count = 0;
    // 路径不足三点（无内部折角可圆化）或未启用曲率约束时原样返回。
    if (path.size() < 3 || radius <= 0.0)
        return path;
    Path result;
    result.reserve(path.size() * 2); // 圆弧化后点数最多约为原路径两倍。
    result.push_back(path.front()); // 首点（起点）固定保留。
    // 逐个处理内部顶点：可行则以"切入点 + 圆弧离散点 + 切出点"替换原顶点，
    // 不可行则保留原折点并计数（由跟踪端弯道限速兜底）。
    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        // 重建该折角的切向圆弧：切线比例 0.5，保证相邻折角的圆弧互不重叠。
        const CornerArc_ arc = build_corner_arc_(path[i], path[i - 1], path[i + 1],
                                                 radius, 0.5);
        // 生效条件：几何可行 + 折角非近似直线（|sweep| >= 1e-6 弧度）
        // + 圆弧全程满足障碍余量。
        if (arc.valid && std::abs(arc.sweep) >= 1e-6
            && arc_is_clear_(grid, arc, sample_step, margin,
                             vehicle_length_m, vehicle_width_m,
                             footprint_step_m))
        {
            // 先放入切入点（来向直段与圆弧的切点）。
            result.push_back(arc.entry);
            // 圆弧弧长 = |sweep|*radius（米），按采样步长向上取整等分为
            // steps 份，逐点从圆心角参数化反算世界坐标插入；
            // k = steps 时恰好落在切出点上，切出点无需单独添加。
            const double sweep_abs = std::abs(arc.sweep);
            const int steps = std::max(1, static_cast<int>(std::ceil(
                sweep_abs * arc.radius / sample_step)));
            for (int k = 1; k <= steps; ++k)
            {
                const double angle = arc.start_angle
                    + arc.sweep * (static_cast<double>(k)
                                   / static_cast<double>(steps));
                PathPoint point;
                point.x = arc.center_x + arc.radius * std::cos(angle);
                point.y = arc.center_y + arc.radius * std::sin(angle);
                result.push_back(point);
            }
        }
        else
        {
            // 不可圆弧化（切线放不下或圆弧撞障碍）：保留原折点。
            // 转角非零（turn >= 1e-6 弧度）时计入未圆弧化折角数，
            // 提示存在依赖跟踪端限速兜底的残余尖角。
            result.push_back(path[i]);
            if (arc.turn >= 1e-6)
                ++unrounded_count;
        }
    }
    result.push_back(path.back()); // 尾点（终点）固定保留。
    return result;
}

// 在平滑后的稀疏航点间按固定步长重采样，恢复稠密参考点。
// 跟踪模块的纯跟踪依赖最近点随车辆前进而推进（移动的"胡萝卜"）；稀疏航点
// 会让前视目标长时间固定在一个远端点上，起步大航向偏差时围绕固定点绕圈。
Path resample_(const Path& path, double step_m)
{
    // 路径不足两点或步长非正（0 表示禁用重采样）时原样返回。
    if (path.size() < 2 || step_m <= 0.0)
        return path;
    Path result;
    result.push_back(path.front()); // 首点直接保留。
    // 逐段处理：段长（米）/ 步长（米）向上取整为等分段数，保证任意段长
    // 下相邻输出点间距不超过 step_m。
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const PathPoint& from = path[i - 1];
        const PathPoint& to = path[i];
        const double segment_length = distance_(from, to);
        const int steps = std::max(1, static_cast<int>(std::ceil(
            segment_length / step_m)));
        // k = 1..steps 线性插值：t = k/steps，k = steps 时插值点与段终点 to
        // 重合，因此段尾点由下一段的插值（或路径末点）覆盖，不重复入队。
        for (int k = 1; k <= steps; ++k)
        {
            const double t = static_cast<double>(k) / static_cast<double>(steps);
            PathPoint point;
            point.x = from.x + t * (to.x - from.x);
            point.y = from.y + t * (to.y - from.y);
            // 插值点航向清零占位，统一由 recompute_yaw_ 差分重算。
            point.yaw = 0.0;
            result.push_back(point);
        }
    }
    return result;
}

void remove_near_duplicate_points_(Path& path, double minimum_spacing_m)
{
    if (path.size() < 3 || minimum_spacing_m <= 0.0)
        return;

    Path filtered;
    filtered.reserve(path.size());
    filtered.push_back(path.front());
    const double minimum_spacing_squared =
        minimum_spacing_m * minimum_spacing_m;

    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        const double dx = path[i].x - filtered.back().x;
        const double dy = path[i].y - filtered.back().y;
        if (dx * dx + dy * dy >= minimum_spacing_squared)
            filtered.push_back(path[i]);
    }
    filtered.push_back(path.back());
    path.swap(filtered);
}

// 按相邻点差分重算内部点切线方向；首尾点航向保持不变。
// 航向为弧度，atan2(dy, dx) 约定（X 轴为零、逆时针为正）；内部点 i 取
// 前后邻点连线方向（中央差分），比单侧差分更平滑、对采样间距不敏感。
void recompute_yaw_(Path& path)
{
    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        // 中央差分：i-1 -> i+1 连线方向即 i 点的近似切向。
        const double dx = path[i + 1].x - path[i - 1].x;
        const double dy = path[i + 1].y - path[i - 1].y;
        // 前后邻点几乎重合（位移 < 1e-9 米）时无方向可取，保留原航向。
        if (std::hypot(dx, dy) > 1e-9)
            path[i].yaw = std::atan2(dy, dx);
    }
}

double minimum_clearance_m_(const Path& path, const TerrainGrid& grid)
{
    if (path.empty() || grid.empty() || grid.resolution_m <= 0.0)
        return 0.0;

    constexpr double kSearchRadiusM = 32.0;
    const int search_radius_cells = static_cast<int>(std::ceil(
        kSearchRadiusM / grid.resolution_m));
    const double half_cell = 0.5 * grid.resolution_m;
    double minimum_clearance = std::numeric_limits<double>::infinity();

    for (const PathPoint& point : path)
    {
        int col = 0;
        int row = 0;
        if (!grid.worldToGrid(point.x, point.y, col, row))
            return 0.0;

        const int first_row = std::max(0, row - search_radius_cells);
        const int last_row = std::min(grid.rows - 1, row + search_radius_cells);
        const int first_col = std::max(0, col - search_radius_cells);
        const int last_col = std::min(grid.cols - 1, col + search_radius_cells);

        for (int candidate_row = first_row;
             candidate_row <= last_row; ++candidate_row)
        {
            for (int candidate_col = first_col;
                 candidate_col <= last_col; ++candidate_col)
            {
                if (grid.isTraversable(candidate_row, candidate_col))
                    continue;

                double obstacle_x = 0.0;
                double obstacle_y = 0.0;
                grid.gridToWorld(candidate_row, candidate_col,
                                 obstacle_x, obstacle_y);
                const double dx = std::max(
                    std::abs(point.x - obstacle_x) - half_cell, 0.0);
                const double dy = std::max(
                    std::abs(point.y - obstacle_y) - half_cell, 0.0);
                minimum_clearance = std::min(
                    minimum_clearance, std::hypot(dx, dy));
            }
        }
    }

    return std::isfinite(minimum_clearance) ? minimum_clearance : -1.0;
}

double footprint_clearance_m_(const Path& path, const TerrainGrid& grid,
                              double length_m, double width_m,
                              double sample_step_m)
{
    if (path.empty() || grid.empty() || length_m <= 0.0 || width_m <= 0.0)
        return -1.0;

    const double step = std::clamp(sample_step_m, 0.2, grid.resolution_m);
    const int longitudinal_steps = std::max(1, static_cast<int>(std::ceil(
        length_m / step)));
    const int lateral_steps = std::max(1, static_cast<int>(std::ceil(
        width_m / step)));
    const double half_length = 0.5 * length_m;
    const double half_width = 0.5 * width_m;
    const double half_cell = 0.5 * grid.resolution_m;
    double minimum_clearance = std::numeric_limits<double>::infinity();

    for (const PathPoint& point : path)
    {
        const double cos_yaw = std::cos(point.yaw);
        const double sin_yaw = std::sin(point.yaw);
        for (int longitudinal = 0; longitudinal <= longitudinal_steps;
             ++longitudinal)
        {
            const double forward = -half_length + length_m
                * static_cast<double>(longitudinal)
                / static_cast<double>(longitudinal_steps);
            for (int lateral = 0; lateral <= lateral_steps; ++lateral)
            {
                const double side = -half_width + width_m
                    * static_cast<double>(lateral)
                    / static_cast<double>(lateral_steps);
                const double sample_x = point.x + forward * cos_yaw
                    - side * sin_yaw;
                const double sample_y = point.y + forward * sin_yaw
                    + side * cos_yaw;
                int col = 0;
                int row = 0;
                if (!grid.worldToGrid(sample_x, sample_y, col, row))
                    return 0.0;
                if (!grid.isTraversable(row, col))
                    return 0.0;

                const int search_radius_cells = static_cast<int>(std::ceil(
                    32.0 / grid.resolution_m));
                const int first_row = std::max(0, row - search_radius_cells);
                const int last_row = std::min(grid.rows - 1,
                                              row + search_radius_cells);
                const int first_col = std::max(0, col - search_radius_cells);
                const int last_col = std::min(grid.cols - 1,
                                              col + search_radius_cells);
                for (int obstacle_row = first_row; obstacle_row <= last_row;
                     ++obstacle_row)
                {
                    for (int obstacle_col = first_col;
                         obstacle_col <= last_col; ++obstacle_col)
                    {
                        if (grid.isTraversable(obstacle_row, obstacle_col))
                            continue;
                        double obstacle_x = 0.0;
                        double obstacle_y = 0.0;
                        grid.gridToWorld(obstacle_row, obstacle_col,
                                         obstacle_x, obstacle_y);
                        const double dx = std::max(
                            std::abs(sample_x - obstacle_x) - half_cell, 0.0);
                        const double dy = std::max(
                            std::abs(sample_y - obstacle_y) - half_cell, 0.0);
                        minimum_clearance = std::min(
                            minimum_clearance, std::hypot(dx, dy));
                    }
                }
            }
        }
    }
    return std::isfinite(minimum_clearance) ? minimum_clearance : -1.0;
}
} // namespace

Path PathOptimizer::optimize(const Path& raw_path, const TerrainGrid& grid,
                             const PathOptimizerConfig& config) const
{
    // 点数过少或地图为空时无法优化，原样返回。
    if (raw_path.size() < 3 || grid.empty())
        return raw_path;

    // 直线/圆弧校验采样步长（米）：显式配置优先，并夹紧到
    // [0.2 米, 栅格分辨率]（0.2 为最小采样密度下限，防止过密采样拖慢校验；
    // 上限取栅格分辨率保证不漏检整格障碍）；未配置时取栅格分辨率的一半。
    const double sample_step = config.clearance_sample_step_m > 0.0
        ? std::clamp(config.clearance_sample_step_m, 0.2, grid.resolution_m)
        : grid.resolution_m * 0.5;
    // 障碍余量圈数（格），下限 0 表示不膨胀。
    const int margin = std::max(0, config.clearance_margin_cells);
    // 重采样步长（米）：显式配置优先，未配置时取栅格分辨率。
    const double resample_step = config.resample_step_m > 0.0
        ? config.resample_step_m
        : grid.resolution_m;

    // 运动学参数：最小转弯半径显式配置优先，否则由轴距和最大前轮角推导
    // R = L/tan(delta)；规划曲率上限 kappa_plan = 安全系数 / R（圆弧半径取
    // R/安全系数，给跟踪控制留余量）。参数非法或安全系数为 0 时
    // corner_radius <= 0，退回纯几何优化（向后兼容）。
    double min_radius = config.min_turning_radius_m;
    if (min_radius <= 0.0 && config.wheelbase_m > 0.0
        && config.max_front_wheel_angle_rad > 0.0
        && config.max_front_wheel_angle_rad < 0.5 * kPi)
    {
        // 阿克曼运动学：R = L/tan(delta)；前轮角必须严格小于 90 度
        // （tan 在 90 度处发散），否则公式无意义。
        min_radius = config.wheelbase_m
            / std::tan(config.max_front_wheel_angle_rad);
    }
    // 圆弧倒圆半径 R_plan = R_min/安全系数：比物理最小转弯半径更保守，
    // 对应曲率上限 kappa_plan = 安全系数/R_min < 1/R_min，给跟踪控制留余量。
    const double corner_radius = (config.curvature_safety_factor > 0.0
                                  && std::isfinite(min_radius)
                                  && min_radius > 0.0)
        ? min_radius / config.curvature_safety_factor
        : 0.0;

    // 优化流水线（顺序固定）：视线捷径删冗余折点 -> 迭代平滑 -> 尖角圆弧化
    // -> 固定步长重采样 -> 差分重算航向。每步以上步输出为输入，
    // 碰撞/余量校验贯穿前三步。
    Path optimized = shortcut_(raw_path, grid, config.shortcut_max_distance_m,
                               sample_step, margin, corner_radius,
                               config.vehicle_overall_length_m,
                               config.vehicle_overall_width_m,
                               config.vehicle_footprint_sample_step_m);
    smooth_(optimized, grid, config.smoothing_iterations,
            config.smoothing_weight, sample_step, margin, corner_radius,
            config.vehicle_overall_length_m,
            config.vehicle_overall_width_m,
            config.vehicle_footprint_sample_step_m);
    int unrounded_corners = 0;
    optimized = round_corners_(optimized, grid, corner_radius, sample_step,
                               margin, unrounded_corners,
                               config.vehicle_overall_length_m,
                               config.vehicle_overall_width_m,
                               config.vehicle_footprint_sample_step_m);
    optimized = resample_(optimized, resample_step);
    remove_near_duplicate_points_(optimized, 0.1);
    recompute_yaw_(optimized);

    // 输出运动学校验信息：重采样后路径的最大离散曲率应不超过 1/corner_radius；
    // 未圆弧化折角数 > 0 时（栅格尺度密集折角、圆弧被障碍挡住等）说明仍有
    // 依赖跟踪端限速兜底的折角，需要关注。
    const double kappa_plan = corner_radius > 0.0 ? 1.0 / corner_radius : 0.0;
    // 逐内部点计算 Menger 离散曲率取全路径最大值（1/米），用于与
    // kappa_plan 对比的自检输出；离散曲率受采样间距影响，仅作参考量。
    double max_curvature = 0.0;
    for (std::size_t i = 1; i + 1 < optimized.size(); ++i)
    {
        max_curvature = std::max(max_curvature,
            menger_curvature_(optimized[i - 1], optimized[i], optimized[i + 1]));
    }
    const double minimum_clearance = minimum_clearance_m_(optimized, grid);
    const double footprint_clearance = footprint_clearance_m_(
        optimized, grid, config.vehicle_overall_length_m,
        config.vehicle_overall_width_m,
        config.vehicle_footprint_sample_step_m);
    std::cout << "[optimizer] kinematic check: kappa_plan=" << kappa_plan
              << " 1/m (R=" << corner_radius << " m)"
              << ", max discrete curvature=" << max_curvature << " 1/m"
              << ", unrounded corners=" << unrounded_corners
              << ", minimum clearance=" << minimum_clearance << " m"
              << ", footprint clearance=" << footprint_clearance << " m"
              << std::endl;
    return optimized;
}
