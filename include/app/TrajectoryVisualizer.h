#pragma once

#include "tracking/PathTracker.h"

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

// 在可视化开关打开时异步绘制局部跟踪窗口，并保存最终轨迹结果。
// 线程模型：控制主线程负责 update()（采样 + 提交帧）、pumpWindow()（HighGUI
// 窗口消息与显示）和 saveFinal()；后台 worker 线程只做 renderFrame_ 图像生成，
// 双方通过互斥锁 + 条件变量交接帧/图像，渲染耗时不阻塞控制循环。
// 可视化关闭（enable_visualization=false）时所有方法立即返回，不建线程不写文件。
class TrajectoryVisualizer
{
public:
    // 析构时停止并 join 后台渲染线程，防止线程访问已销毁的成员。
    ~TrajectoryVisualizer();

    // 初始化路径、输出目录和可选后台渲染线程。
    // 输入：参考路径（世界坐标，米）、起点、终点、输出目录、可视化开关、
    // 实际轨迹采样间隔（秒，下限 0.1）。输出：成功返回 true；路径为空或
    // 输出目录创建失败返回 false。可视化关闭时只保存参数，不创建线程和窗口。
    bool initialize(const Path& path,
                    const Pose2D& start,
                    const Pose2D& goal,
                    const std::string& output_directory,
                    bool enable_visualization,
                    double sample_interval_sec);

    // 接收控制循环的最新状态，按时间间隔保存实际轨迹采样点。
    // 输入：车辆状态（世界坐标）、当前参考点序号、仿真时间戳（秒）。
    // 行为：距上次采样超过采样间隔则记录一个实际点；车辆走出当前局部窗口
    // 且最近参考点变化时，把窗口重新居中并提交一帧给后台线程渲染。
    void update(const VehicleState2D& state,
                std::size_t reference_index,
                double timestamp_sec);

    // 在控制主线程显示后台线程准备完成的最新局部图像。
    // 每个控制周期调用一次：取走最新渲染结果并 imshow，同时 waitKey(1)
    // 泵窗口消息（窗口拖动/关闭的响应依赖这一步）。
    void pumpWindow();

    // 停止渲染线程并统一写出实际路径与最终图片。
    // 输出三个文件到 output_directory：actual_path.txt（实际轨迹，含航向）、
    // tracking_local_final.png（最后一帧局部窗口）、global_and_actual_final.png
    // （全局路径与实际轨迹的最终对比图）。任务结束时由运行时调用一次。
    void saveFinal();

private:
    struct Frame
    {
        VehicleState2D state; // 当前车辆状态。
        std::size_t reference_index = 0; // 当前参考点序号。
        std::size_t window_anchor_index = 0; // 当前局部窗口中心点序号。
        std::vector<Pose2D> actual_points; // 当前窗口范围内的历史实际点。
        std::size_t actual_total = 0; // 截至当前帧已保存的实际轨迹点总数。
        bool valid = false; // 帧是否有效。
    };

    // 请求后台线程退出并 join（析构与 saveFinal 复用）。
    void stopWorker_();
    // 后台渲染线程主循环：条件变量等待新帧，只渲染最新帧并交付主线程。
    void workerLoop_();
#ifdef ECHOSIM_USE_OPENCV
    // 渲染单帧局部跟踪窗口图像（20m x 20m，200x200 像素），颜色约定见 .cpp。
    cv::Mat renderFrame_(const Frame& frame) const;
#endif
    // 保存全局路径与实际轨迹的最终对比图（1 米/像素），成功返回 true。
    bool renderGlobalComparison_(const std::vector<Pose2D>& actual_points) const;
    // 保存实际轨迹文本 actual_path.txt（每行 "x y yaw"，米/弧度），成功返回 true。
    bool saveActualPath_(const std::vector<Pose2D>& actual_points) const;

    Path path_; // 全局规划路径。
    Pose2D start_; // 任务起点。
    Pose2D goal_; // 任务终点。
    std::string output_directory_; // 可视化输出目录。
    bool enable_visualization_ = false; // 是否启用可视化。
    double sample_interval_sec_ = 2.0; // 实际轨迹采样间隔。
    std::vector<Pose2D> actual_points_; // 完整历史实际轨迹。
    double last_sample_time_sec_ = 0.0; // 最近一次采样时间。
    bool has_sample_time_ = false; // 是否已经采样过。
    std::size_t window_anchor_index_ = 0; // 当前局部窗口中心参考点。
    bool has_window_anchor_ = false; // 是否已建立局部窗口中心。
    mutable std::mutex mutex_; // 保护轨迹和待渲染帧。
    std::condition_variable condition_; // 通知后台线程刷新或退出。
    std::thread worker_; // 后台渲染线程。
    Frame pending_frame_; // 最新待渲染帧。
    bool frame_pending_ = false; // 是否有待渲染帧。
#ifdef ECHOSIM_USE_OPENCV
    cv::Mat latest_image_; // 后台线程生成、等待主线程显示的最新图像。
    bool image_pending_ = false; // 是否存在等待主线程显示的新图像。
#endif
    bool stop_requested_ = false; // 是否请求线程退出。
};
