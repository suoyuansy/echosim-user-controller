#pragma once

#include "common/PathTypes.h"

#include <cstddef>

// 只有已完成停车的途经点才需要按抵达航向重新起步。无途经点任务（如
// Test2/Test5/Test6）和任务初始出生点均必须保留原始 A* 的离开几何。
bool shouldAlignSegmentStartHeading(std::size_t segment_index,
                                    std::size_t waypoint_count);
