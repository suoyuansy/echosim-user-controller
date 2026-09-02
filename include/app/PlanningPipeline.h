#pragma once

#include "common/PathTypes.h"
#include "app/TaskConfig.h"

#include <stdexcept>

// 文件功能：编排地形缓存、全局规划和规划结果输出，并控制代价地图生命周期。
class PlanningPipeline
{
public:
    // 加载或构建代价地图，规划并输出路径；失败时抛出 std::runtime_error。
    RoutePlan buildPath(const TaskConfig& config) const;
};
