#include "app/SegmentPlanningPolicy.h"

bool shouldAlignSegmentStartHeading(std::size_t segment_index,
                                    std::size_t waypoint_count)
{
    // segment 0 从仿真出生点出发，不存在“刚在途经点停车后再次起步”的
    // 语义。waypoint_count 为零时，所有段均属于无途经点直达任务，绝不
    // 施加途经点重起步的航向约束。
    return waypoint_count > 0 && segment_index > 0;
}
