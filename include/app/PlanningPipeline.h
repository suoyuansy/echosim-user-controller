#pragma once

#include "common/PathTypes.h"
#include "app/TaskConfig.h"

#include <stdexcept>

// 文件功能：编排地形缓存、全局规划、路径优化和规划结果输出，并控制代价地图生命周期。
// PlanningPipeline 是 app 集成层的编排器：本身无状态、无成员变量，按固定
// 顺序聚合三个模块--代价地图（模块 1 TerrainCostmap）-> 全局规划（模块 1
// GlobalPlanner）-> 路径优化（模块 2 PathOptimizer），并把调试产物写入
// config.output_directory。
class PlanningPipeline
{
public:
    // 加载或构建代价地图，规划、优化并输出路径；失败时抛出 std::runtime_error。
    // 输入：TaskConfig（扫描走廊、建图/规划/优化参数、起终点、调试开关）。
    // 输出：优化后的全局路径 Path（世界坐标，米/弧度），直接交给跟踪模块使用。
    // 执行顺序：创建 output 目录 -> 读缓存或建图（含缓存一致性校验）
    // -> 全局规划 -> 路径优化 -> 调试文件输出（txt/png，仅调试模式）。
    // 代价地图仅在本函数栈上存在，函数返回后即释放大块栅格内存。
    Path buildPath(const TaskConfig& config) const;
};
