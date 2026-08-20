#pragma once

#include "common/PathTypes.h"
#include "planning/TerrainCostmap.h"

// 文件功能：定义路径优化模块的配置与接口。输入全局规划产出的原始栅格路径
// 和代价地图（只读，用于直线/圆弧可行性/碰撞检查），输出优化后的路径。
// 优化分为三步：视线捷径（Theta* 式）删除被障碍遮挡而保留的冗余折点，
// 随后迭代平滑圆化转角，最后对残余尖角做切向圆弧替换；三步都逐采样校验
// 与障碍的余量，且曲率处处不超过阿克曼运动学上限（curvature_safety_factor
// 乘以最小转弯半径倒数），保证不穿越障碍、不超出车辆转向能力。

// 路径优化配置：全部为纯参数（米/弧度/无量纲），无运行期状态。
// 默认值即本模块在 Moon2 实测调出的整组参数（914 点/2234 m 原始路径 ->
// 11 点/1964 m，比起点终点直线 1800 m 仅多 9%），模块所有者直接在此调参；
// app 层 TaskConfig 聚合本结构，仅任务级覆盖时才在 makeDefaultTaskConfig()
// 显式赋值。优化器只读这些参数，不回写。
struct PathOptimizerConfig
{
    double shortcut_max_distance_m = 200.0; // 视线捷径单段最大跨度（米），限制最坏搜索量；跨度再大也不合并，防止一次直线校验覆盖过长线段。
    double clearance_sample_step_m = 1.0;   // 直线可行性检查的采样步长（米）；0 表示取栅格分辨率的一半。实际值会被夹紧到 [0.2, 栅格分辨率] 区间。
    int clearance_margin_cells = 1;         // 直线与障碍保持的栅格余量圈数（1 = 八邻域空旷，与规划器一致）；等价于把障碍层膨胀 margin 圈后再查询单点。0 表示只查采样点所在栅格。
    int smoothing_iterations = 3;           // 平滑迭代次数；0 表示只做捷径不平滑。每轮迭代对全部内部点各生成一次候选移动，收敛速度与轮数成正比。
    double smoothing_weight = 0.25;         // 平滑位移权重，取值 (0, 0.5]，越大转角越圆；候选点 = w*prev + (1-2w)*current + w*next 的加权平均，w=0.5 时退化为两邻点中点。
    double resample_step_m = 2.0;           // 平滑后重采样步长（米），恢复稠密参考点供跟踪；0 表示取栅格分辨率。跟踪的最近点搜索依赖稠密点随车推进。
    // ===== 运动学参数（与跟踪模块 TrackingConfig::geometry 同源）=====
    // 设计文档要求规划与控制共用同一套车辆参数；两处独立维护，修改任一处
    // 必须同步另一处（PlanningPipeline 启动时有不一致告警兜底）。
    double wheelbase_m = 2.76;              // 车辆轴距（米），用于推导最小转弯半径（R = L/tan(delta)）。
    double max_front_wheel_angle_rad = 23.0 * (3.14159265358979323846 / 180.0); // 最大前轮角（弧度），默认 23 度换算而来；用于推导最小转弯半径。由轴距 2.76 m / 23 度推导 R_min 约 6.5 m。
    double min_turning_radius_m = 0.0;      // 最小转弯半径（米）；0 表示由轴距和最大前轮角推导 R = L/tan(delta)。显式配置时优先生效。
    double curvature_safety_factor = 0.8;   // 规划曲率上限系数：kappa_plan = 系数 / R_min，给跟踪控制留余量；0 表示不启用曲率约束。圆弧倒圆半径 R_plan = R_min/系数（比物理最小半径更保守），即 6.5/0.8 约 8.1 m（kappa_plan 约 0.123 1/m）。
};

// 路径优化器：消费模块一（全局路径规划）的原始 Path，输出给模块三（轨迹跟踪）的 Path。
// 本类无成员状态、无副作用，optimize() 可重复调用（const 成员函数）。
class PathOptimizer
{
public:
    // 输入原始栅格路径与代价地图，输出优化后路径。
    // 保证：首尾点不变、每段直线/圆弧满足 clearance_margin_cells 圈余量、
    // 离散曲率不超过 kappa_plan（curvature_safety_factor = 0 时不启用）、总长不增。
    // 路径点数过少或地图为空时原样返回。
    // 参数单位：raw_path 各点为世界坐标（米/弧度）；grid 为代价地图栅格
    // （只读，仅用于直线/圆弧的碰撞与余量校验）；返回路径同为单位的世界坐标。
    Path optimize(const Path& raw_path,
                  const TerrainGrid& grid,
                  const PathOptimizerConfig& config) const;
};
