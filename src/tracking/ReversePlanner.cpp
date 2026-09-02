#include "tracking/ReversePlanner.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle_(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}
} // namespace

bool reverseRequired(const Path& path, std::size_t progress_index,
                     double vehicle_yaw_rad,
                     const ReverseDetectionConfig& config)
{
    if (path.size() < 2 || progress_index >= path.size() - 1)
        return false;

    const double threshold = std::clamp(
        config.opposite_heading_threshold_rad,
        0.5 * kPi, kPi);
    const double minimum_path_turn = std::clamp(
        config.minimum_path_turn_for_reverse_rad,
        0.0, kPi);
    const std::size_t required_segments = std::max<std::size_t>(
        config.minimum_reverse_segments, 2);
    const double required_distance = std::max(
        config.minimum_reverse_distance_m, 0.0);

    double accumulated_distance = 0.0;
    std::size_t reverse_segments = 0;
    const std::size_t begin = progress_index;
    // 倒车几何必须从当前进度点立即开始。普通路径可能在很远处
    // 折返；如果跳过前面的正向段继续累计，就会把普通急转弯误判为
    // 倒车请求，导致车辆无故进入 PARK/REVERSE 换挡状态。
    for (std::size_t index = begin; index + 1 < path.size(); ++index)
    {
        const double dx = path[index + 1].x - path[index].x;
        const double dy = path[index + 1].y - path[index].y;
        const double segment_distance = std::hypot(dx, dy);
        if (segment_distance < 1e-6)
            continue;

        const double segment_heading = std::atan2(dy, dx);
        const double heading_error = std::abs(normalizeAngle_(
            segment_heading - vehicle_yaw_rad));
        if (heading_error >= threshold)
        {
            // 从路径内部第一次进入候选反向段时，路径自身必须先
            // 发生明显掉头。仅仅因为车辆在普通弯道中转过车头，
            // 不能把一段连续的普通弯道误判成倒车路径。
            if (index > 0 && reverse_segments == 0)
            {
                const double previous_dx = path[index].x - path[index - 1].x;
                const double previous_dy = path[index].y - path[index - 1].y;
                if (std::hypot(previous_dx, previous_dy) < 1e-6)
                    break;
                const double previous_heading = std::atan2(
                    previous_dy, previous_dx);
                const double path_turn = std::abs(normalizeAngle_(
                    segment_heading - previous_heading));
                if (path_turn < minimum_path_turn)
                    break;
            }
            ++reverse_segments;
            accumulated_distance += segment_distance;
            if (reverse_segments >= required_segments
                && accumulated_distance >= required_distance)
                return true;
        }
        else
        {
            break;
        }

        // 只观察车辆前方有限距离，避免把很远处的路线方向变化
        // 提前解释成当前需要倒车。
        if (index - begin >= 20)
            break;
    }

    return false;
}
