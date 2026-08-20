#pragma once

#include "common/PathTypes.h"

#include <cstddef>

// 文件功能：定义车辆状态、跟踪配置、三种横向跟踪算法和纵向 P 控制接口。
// 补充说明：三种横向算法为 Pure Pursuit / Stanley / 简化 LQR。本模块内所有
// 角度均为弧度、距离均为米、速度均为米每秒，与 EchoSim 的度/km/h 换算只在
// EchoSimRuntime 边界处进行；车辆状态中 x/y/yaw 为世界系，vx/vy/yaw_rate
// 为车体系分量（x 前向、y 左正）。
struct VehicleState2D
{
    double x = 0.0; // 车辆世界坐标 X，单位为米。
    double y = 0.0; // 车辆世界坐标 Y，单位为米。
    double yaw = 0.0; // 车辆航向角，单位为弧度。
    double vx = 0.0; // 车体系前向速度，单位为米每秒。
    double vy = 0.0; // 车体系横向速度（左正），单位为米每秒。
    double yaw_rate = 0.0; // 横摆角速度，单位为弧度每秒（左转/逆时针为正）。
};

// 车辆几何参数：轴距决定最小转弯半径 R_min = L / tan(δ_max)，
// 纯跟踪以后轴为基准点，几何参数单位均为米。
// 注意：wheelbase_m 与 max_front_wheel_angle_rad 与优化模块的
// PathOptimizerConfig 同源（设计文档要求规划与控制共用同一套车辆参数），
// 两处独立维护，修改任一处必须同步另一处（PlanningPipeline 启动时有
// 不一致告警兜底）。
struct VehicleGeometry
{
    double wheelbase_m = 2.76; // 车辆轴距，单位为米。
    double front_axle_offset_m = 1.41; // 质心到前轴距离，当前版本保留未参与计算。
    double rear_axle_offset_m = 1.35; // 质心到后轴距离，作为纯跟踪基准点。
    double max_front_wheel_angle_rad = 23.0 * (3.14159265358979323846 / 180.0); // 最大前轮角。
};

// 横向跟踪算法选择枚举：PurePursuit 为默认推荐配置（月壤低侧向抓地下
// 参数已整体调谐，切换算法需重调速度门控相关参数）。
enum class TrackerMethod
{
    PurePursuit = 0, // 纯跟踪法。
    Stanley = 1, // Stanley 法。
    Lqr = 2 // 简化 LQR 法。
};

// 跟踪配置：包含算法选择、车辆几何、各算法增益以及速度门控阈值。
// 默认值即本模块在 Moon2 + Test1–Test5 实测调出的整组配套参数，模块所有者
// 直接在此调参；app 层 TaskConfig 聚合本结构，仅任务级覆盖时才显式赋值。
// 月壤侧向抓地低是多数限速参数的设计根源：带误差/带侧滑高速修正会触发
// 后轴侧滑甩尾，因此速度门控（turn/bend/curve/slide）需与跟踪增益一起调。
struct TrackingConfig
{
    TrackerMethod method = TrackerMethod::PurePursuit; // 当前使用的跟踪算法。
    VehicleGeometry geometry; // 车辆几何参数。
    // ===== 前视与弯道门控（2026-08-20 全程时限优先调整，整组联动验证）=====
    // 路径优化已保证曲率 <= kappa_plan（R >= 8.1 m），4 m/s 过弯的运动学指标
    // 均在评分限内（ay = 16/8.1 = 1.98 < 3.5 m/s^2，yaw rate = 0.49 < 0.61 rad/s），
    // 因此弯道门控不再需要为"不可执行尖角"兜底，放宽以保全程时限（1964 m / 600 s）。
    // - 前视 8.0 -> 5.0、增益 0.8 -> 0.6：巡航时 Ld = 5 + 0.6*4 = 7.4 m < 弧半径
    //   8.1 m。此前 Ld = 11.2 m > R 时前视目标绕过弯道远端，进弧瞬间转向指令
    //   仅 5-6 度（弧需要约 19 度），欠转向后猛修，实测横向加速度尖峰
    //   7.55 m/s^2（红线 3.5）并触发急刹（Run 20260820_002054 t=42-44）。
    // - 弯道限速 2.5 -> 3.5、限幅/未收敛限速 1.5 -> 2.5：路径曲率已有保证，
    //   过弯不需要掉到巡航一半以下；侧偏角 >5 度仍限速 1.0 兜底月壤侧滑。
    // - 若整组调整后出现转向振荡，优先回退前视（5.0/0.6 -> 8.0/0.8）而不是
    //   单独调速度项。网格路径含短距离方向突变，单独调某一项易引发转向振荡。
    // 前视基值 5.0 m 与速度增益 0.6：巡航 4 m/s 时动态前视 Ld = 5.0 + 0.6*4
    // = 7.4 m，小于规划最小弧半径 8.1 m，保证前视目标不越过弯道远端、
    // 进弧瞬间转向指令即接近所需值。
    double pure_pursuit_lookahead_m = 5.0; // 纯跟踪基础前视距离（米）。
    double pure_pursuit_lookahead_gain = 0.6; // 纯跟踪前向速度前视增益（无量纲）。
    double pure_pursuit_turn_radius_margin = 1.15; // 大航向偏差限幅用的转弯半径系数：R_cap = 系数×最小转弯半径；α 被钳到 asin(Ld/(2·R_cap))，弧线必达目标点且转角不打满（满舵时轮胎刮擦阻力会让车辆近乎停滞）。
    // 大航向偏差（转弯不可执行/未收敛）时的限速 2.5 m/s：路径曲率上限已有
    // 保证，不再需要压到巡航一半以下，为保 600 s 全程时限放宽（原 1.5）。
    double pure_pursuit_turn_speed_mps = 2.5; // 目标方位角被限幅或未收敛时的限速值；月壤侧向抓地低，带误差高速修正会后轴侧滑甩尾。
    double pure_pursuit_converged_cross_track_m = 1.5; // 判定已收敛的横向误差阈值（米），超过则限速到 turn_speed；航向对齐由 α 限幅门控。
    double pure_pursuit_bend_alpha_rad = 12.0 * (3.14159265358979323846 / 180.0); // 弯道预限速的 α 触发阈值；直道栅格噪声下 α 约 ±9°，取 12° 隔开，前视目标进入弯道（α 15°~20°）时提前压速。
    // 弯道限速 3.5 m/s：曲率上限已由优化模块保证，过弯运动学指标在评分限内
    // （见上方 2026-08-20 注释），为保全程时限从 2.5 放宽。
    double pure_pursuit_bend_speed_mps = 3.5; // 弯道预限速值（米/秒）。
    double curve_lookahead_m = 40.0; // 曲率预判减速的前瞻距离（米）：高速下前方出现显著转向时提前按最大减速度限速。
    double curve_heading_threshold_rad = 20.0 * 3.14159265358979323846 / 180.0; // 判定为弯的累计航向变化阈值。
    double pure_pursuit_slide_sideslip_rad = 5.0 * 3.14159265358979323846 / 180.0; // 侧滑检测阈值：车体系侧偏角超过该值说明后轴开始滑动（评分上限 8°，留余量）。
    double pure_pursuit_slide_speed_mps = 1.0; // 检测到侧滑时的限速值，等残余滑动衰减后再提速。
    double stanley_gain = 1.0; // Stanley 横向误差增益。
    double stanley_min_speed_mps = 0.5; // Stanley 最小计算速度。
    // 巡航速度 4 m/s：这是低抓地后驱车在月壤上能稳定巡航的速度，而不是动力
    // 极限（电机理论极速 14.4 m/s）。实测把上限提到 14.4 后，车因离目标太远
    // 持续踩油门，加速 + 转向使后轴失去横向附着力反复甩尾（侧偏角 22°），
    // 速度反而在 1~4 m/s 震荡。目标必须"可达"才能进入匀速稳态。大航向偏差
    // 场景由转弯半径安全系数限幅 + 侧滑速度门控兜底；速度指令另有上升斜率
    // 限制（见 EchoSimRuntime），防弯中门控瞬时全开。
    double base_speed_mps = 4.0; // Pure Pursuit/Stanley 的基础 SDK 速度上限（低抓地月壤上可稳定巡航的速度）。
    double max_speed_mps = 4.0; // Pure Pursuit/Stanley 的最大速度上限（取值依据同 base_speed_mps）。
    double lqr_speed_weight = 1.0; // 简化 LQR 速度误差权重。
    double lqr_lateral_weight = 3.0; // 简化 LQR 横向误差权重。
    double lqr_heading_weight = 2.0; // 简化 LQR 航向误差权重。
    double lqr_steer_weight = 1.0; // 简化 LQR 转向输入权重。
    double lqr_speed_input_weight = 1.0; // 简化 LQR 速度输入权重。
    double lqr_longitudinal_kp = 0.8; // LQR 纵向速度 P 控制增益。
    double max_acceleration_mps2 = 1.0; // 最大加速度。
    double max_deceleration_mps2 = 1.5; // 最大减速度绝对值。
    double goal_position_tolerance_m = 1.0; // 终点位置容差。
    double goal_yaw_tolerance_rad = 10.0 * 3.14159265358979323846 / 180.0; // 终点航向容差。
};

// 单帧跟踪输出：前轮角与速度上限为控制量（单位：弧度、米每秒），
// 其余字段用于进度统计、可视化与到点判定。
struct TrackingCommand
{
    double front_wheel_angle_rad = 0.0; // 目标前轮角，单位为弧度。
    double target_speed_mps = 0.0; // LQR 使用的目标速度，其他算法不修改 SDK 速度目标。
    double remaining_path_distance_m = 0.0; // 当前参考点到终点剩余折线距离。
    std::size_t nearest_path_index = 0; // 用于计算进度的最近路径点。
    std::size_t target_path_index = 0; // 用于控制的路径目标点。
    bool reached_goal = false; // 是否满足终点位置和航向角要求。
};

class PathTracker
{
public:
    // 构造函数：保存 TrackingConfig 副本，跟踪器本身无内部状态（const 方法），
    // 可在多帧间安全复用。
    explicit PathTracker(TrackingConfig config = TrackingConfig());

    // 按配置选择 Pure Pursuit、Stanley 或简化 LQR。
    // 输入：参考路径（世界系，米/弧度）、车辆状态（世界系位姿 + 车体系速度）、
    // 路径进度索引（避免最近点搜索回退过远）。
    // 输出：TrackingCommand，前轮角单位为弧度，速度单位为米每秒。
    TrackingCommand calculate(const Path& path,
                              const VehicleState2D& state,
                              std::size_t progress_index = 0) const;

    // 计算纯跟踪前轮角。核心公式 δ = atan2(2·L·sin(α), Ld)：
    // 以后轴（由质心沿航向反推 rear_axle_offset_m）为基准，选取沿路径累计
    // 弧长恰为前视距离的插值点，α 为目标点相对车体航向的方位角（弧度）。
    // 内含可达性 α 限幅与四级速度门控。
    TrackingCommand calculatePurePursuit(const Path& path,
                                         const VehicleState2D& state,
                                         std::size_t progress_index = 0) const;

    // 计算 Stanley 前轮角。公式 δ = heading_error + atan2(k·e, v)：
    // 航向项直接消除朝向偏差，横向误差项按速度归一（低速时误差修正增强），
    // v 取平面合成速度并设下限，防止低速时增益发散。
    TrackingCommand calculateStanley(const Path& path,
                                     const VehicleState2D& state,
                                     std::size_t progress_index = 0) const;

    // 计算简化 LQR 前轮角和目标速度修正。非完整 Riccati 求解，而是将
    // 权重比近似为反馈增益：δ = (w_lat/w_steer/v)·e_lat + (w_head/w_steer)·e_yaw，
    // 纵向按速度误差 P 修正目标速度，横摆由前轮角间接抑制。
    TrackingCommand calculateLqr(const Path& path,
                                 const VehicleState2D& state,
                                 std::size_t progress_index = 0) const;

    // 返回车辆对应的最近参考路径点序号：从 progress_index 向后限窗 30 点、
    // 向前全量搜索，用于外部维护单调推进的路径进度。
    std::size_t findNearestPathPoint(const Path& path,
                                     const VehicleState2D& state,
                                     std::size_t progress_index = 0) const;

private:
    // 生成所有横向控制器共用的路径状态基座：最近点、剩余距离、到点判定和
    // 距离刹车限速后的目标速度（尚未叠加各算法自身的速度门控）。
    TrackingCommand make_base_command_(const Path& path,
                                       const VehicleState2D& state,
                                       std::size_t progress_index) const;
    // 最近点搜索实现：向后限窗防止进度回退，向前全量保证目标可达。
    std::size_t find_nearest_index_(const Path& path, double x, double y,
                                    std::size_t progress_index) const;
    // 计算从第 index 个路径点到路径终点的折线累计长度（米）。
    double remaining_distance_(const Path& path, std::size_t index) const;
    // 将前轮角钳位到 ±max_front_wheel_angle_rad（弧度）。
    double clamp_steer_(double angle_rad) const;

    TrackingConfig config_;
};

// 纵向控制器：静态工具类，提供速度误差 P 控制和剩余距离刹车限速。
class LongitudinalController
{
public:
    // 计算带上下限的速度误差 P 控制加速度。
    // 输入：目标/实际速度（米每秒）、增益 kp、加速度上下限（米每二次方秒，
    // 允许 min > max 的任意顺序传入，内部会归一）。
    // 输出：a = clamp(kp·(v_target - v_actual), a_min, a_max)。
    static double speedP(double target_speed_mps,
                         double actual_speed_mps,
                         double kp,
                         double min_acceleration_mps2,
                         double max_acceleration_mps2);

    // 根据剩余路径距离计算满足减速约束的速度上限。
    // 物理模型 v = sqrt(2·a_max·d)：从当前速度按最大减速度刹到 0 恰好走完
    // 剩余距离 d。输出再与 base_speed 取小，保证不超过巡航速度上限。
    static double distanceSpeedLimit(double remaining_distance_m,
                                     double base_speed_mps,
                                     double max_deceleration_mps2);
};
