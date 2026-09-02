#pragma once

#include "tracking/PathTracker.h"

#include <condition_variable>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

// 在可视化开关打开时异步绘制局部跟踪窗口，并保存最终轨迹结果。
class TrajectoryVisualizer
{
public:
    ~TrajectoryVisualizer();

    // 初始化路径、输出目录和可选后台渲染线程。
    bool initialize(const Path& path,
                    const Pose2D& start,
                    const Pose2D& goal,
                    const std::vector<Pose2D>& waypoints,
                    const std::string& output_directory,
                    bool enable_visualization,
                    double sample_interval_sec);

    // 接收控制循环的最新状态，按时间间隔保存实际轨迹采样点。
    void update(const VehicleState2D& state,
                std::size_t reference_index,
                double timestamp_sec);

    // 在控制主线程显示后台线程准备完成的最新局部图像。
    void pumpWindow();

    static bool isFrameCurrent_(std::uint64_t frame_sequence,
                                std::uint64_t latest_sequence)
    {
        return frame_sequence == latest_sequence;
    }

    // 停止渲染线程并统一写出实际路径与最终图片。
    void saveFinal();

private:
    struct Frame
    {
        std::uint64_t sequence = 0; // 帧序号，防止旧渲染结果覆盖新结果。
        VehicleState2D state; // 当前车辆状态。
        std::size_t reference_index = 0; // 当前参考点序号。
        std::size_t window_anchor_index = 0; // 当前局部窗口中心点序号。
        std::vector<Pose2D> actual_points; // 当前窗口范围内的历史实际点。
        std::size_t actual_total = 0; // 截至当前帧已保存的实际轨迹点总数。
        bool valid = false; // 帧是否有效。
    };

    void stopWorker_();
    void workerLoop_();
    // 返回 pose 在完整全局路径中的最近路径点序号；坐标存在栅格吸附误差时，
    // 途经点和终点标签仍可显示与控制日志一致的 path_index。
    static std::size_t nearestPathIndex_(const Path& path, const Pose2D& pose)
    {
        if (path.empty())
            return 0;
        std::size_t nearest = 0;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < path.size(); ++index)
        {
            const double distance = std::hypot(path[index].x - pose.x,
                                               path[index].y - pose.y);
            if (distance < nearest_distance)
            {
                nearest_distance = distance;
                nearest = index;
            }
        }
        return nearest;
    }

    // 生成显示在左上角的必达点路径序号，避免把较长的序号标签绘制在地图点旁。
    static std::vector<std::string> pathIndexStatusLines_(
        const Path& path, const std::vector<Pose2D>& waypoints,
        const Pose2D& goal)
    {
        std::vector<std::string> lines;
        lines.reserve(waypoints.size() + 1);
        for (std::size_t index = 0; index < waypoints.size(); ++index)
        {
            lines.push_back("wp" + std::to_string(index + 1)
                            + "_path_index="
                            + std::to_string(nearestPathIndex_(path,
                                                               waypoints[index])));
        }
        lines.push_back("goal_path_index="
                        + std::to_string(nearestPathIndex_(path, goal)));
        return lines;
    }
#ifdef ECHOSIM_USE_OPENCV
    cv::Mat renderFrame_(const Frame& frame) const;
#endif
    bool renderGlobalComparison_(const std::vector<Pose2D>& actual_points) const;
    bool saveActualPath_(const std::vector<Pose2D>& actual_points) const;

    Path path_; // 全局规划路径。
    Pose2D start_; // 任务起点。
    Pose2D goal_; // 任务终点。
    std::vector<Pose2D> waypoints_; // 必须经过的途经点。
    std::string output_directory_; // 可视化输出目录。
    bool enable_visualization_ = false; // 是否启用可视化。
    double sample_interval_sec_ = 1.0; // 实际轨迹采样间隔，单位为仿真秒。
    std::vector<Pose2D> actual_points_; // 完整历史实际轨迹。
    double last_sample_time_sec_ = 0.0; // 最近一次采样时间。
    bool has_sample_time_ = false; // 是否已经采样过。
    std::size_t window_anchor_index_ = 0; // 当前局部窗口中心参考点。
    bool has_window_anchor_ = false; // 是否已建立局部窗口中心。
    std::uint64_t next_frame_sequence_ = 0;
    std::uint64_t latest_requested_sequence_ = 0;
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
