#pragma once

#include "common/PathTypes.h"

#include <cstddef>

// 倒车只在前方存在一段连续、明确朝向车尾的路径时触发。
struct ReverseDetectionConfig
{
    double opposite_heading_threshold_rad =
        160.0 * 3.14159265358979323846 / 180.0;
    // 从已有路径内部进入倒车候选时，路径自身必须先发生明显掉头。
    // 这可避免车辆在普通弯道中因车身航向变化而误触发倒车。
    double minimum_path_turn_for_reverse_rad =
        135.0 * 3.14159265358979323846 / 180.0;
    std::size_t minimum_reverse_segments = 3;
    double minimum_reverse_distance_m = 2.5;
};

// 判断从 progress_index 开始的前方路径是否明确位于车辆后方。
// 单个急转弯不会满足连续段和累计距离条件。
bool reverseRequired(const Path& path, std::size_t progress_index,
                     double vehicle_yaw_rad,
                     const ReverseDetectionConfig& config = {});
