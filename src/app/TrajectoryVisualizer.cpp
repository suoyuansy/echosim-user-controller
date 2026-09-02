// 文件功能：实现异步局部窗口绘制、实际轨迹保存和最终路径对比图。
#include "app/TrajectoryVisualizer.h"

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace
{
constexpr double kWindowHalfSizeM = 10.0;
constexpr double kLocalResolutionM = 0.1;
constexpr int kLocalImageSize = 200;
constexpr double kVehicleHeadingLengthM = 1.0; // 实际航向线段长度，单位为米。

// 将窗口世界坐标转换为图像坐标。
#ifdef ECHOSIM_USE_OPENCV
cv::Point toLocalPixel(double x, double y, double center_x, double center_y)
{
    const int col = static_cast<int>(std::lround(
        (x - center_x + kWindowHalfSizeM) / kLocalResolutionM));
    const int row = kLocalImageSize - 1 - static_cast<int>(std::lround(
        (y - center_y + kWindowHalfSizeM) / kLocalResolutionM));
    return {col, row};
}

bool inImage(const cv::Point& point)
{
    return point.x >= 0 && point.x < kLocalImageSize
        && point.y >= 0 && point.y < kLocalImageSize;
}
#endif
} // namespace

TrajectoryVisualizer::~TrajectoryVisualizer()
{
    stopWorker_();
}

// 初始化可视化对象，关闭可视化时不创建线程也不产生文件。
bool TrajectoryVisualizer::initialize(const Path& path,
                                      const Pose2D& start,
                                      const Pose2D& goal,
                                      const std::vector<Pose2D>& waypoints,
                                      const std::string& output_directory,
                                      bool enable_visualization,
                                      double sample_interval_sec)
{
    stopWorker_();
    if (path.empty())
        return false;
    path_ = path;
    start_ = start;
    goal_ = goal;
    waypoints_ = waypoints;
    output_directory_ = output_directory;
    enable_visualization_ = enable_visualization;
    sample_interval_sec_ = std::max(0.1, sample_interval_sec);
    actual_points_.clear();
    has_sample_time_ = false;
    has_window_anchor_ = false;
    next_frame_sequence_ = 0;
    latest_requested_sequence_ = 0;
    frame_pending_ = false;
#ifdef ECHOSIM_USE_OPENCV
    latest_image_.release();
    image_pending_ = false;
#endif
    stop_requested_ = false;
    if (!enable_visualization_)
        return true;
    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error)
        return false;
    // 清理上一次调试运行的最终结果，避免运行中误把旧文件当成当前结果。
    for (const char* file_name : {"actual_path.txt",
                                  "tracking_local_final.png",
                                  "global_and_actual_final.png"})
    {
        std::filesystem::remove(std::filesystem::path(output_directory_) / file_name,
                                error);
    }
    worker_ = std::thread(&TrajectoryVisualizer::workerLoop_, this);
    // 初始化后立即提交一帧起点窗口，保持旧版本启动即显示实时窗口的行为。
    {
        std::lock_guard<std::mutex> lock(mutex_);
        window_anchor_index_ = 0;
        has_window_anchor_ = true;
        pending_frame_.sequence = ++next_frame_sequence_;
        pending_frame_.state = {start_.x, start_.y, start_.yaw, 0.0, 0.0};
        pending_frame_.reference_index = 0;
        pending_frame_.window_anchor_index = 0;
        pending_frame_.actual_points.clear();
        pending_frame_.actual_total = 0;
        pending_frame_.valid = true;
        latest_requested_sequence_ = pending_frame_.sequence;
        frame_pending_ = true;
    }
#ifdef ECHOSIM_USE_OPENCV
    // HighGUI 窗口必须由控制主线程创建，后台线程只生成图像数据。
    cv::namedWindow("tracking_local", cv::WINDOW_AUTOSIZE);
#endif
    condition_.notify_one();
    return true;
}

// 保存按时间采样的实际点，并在窗口外且参考中心变化时提交局部刷新。
void TrajectoryVisualizer::update(const VehicleState2D& state,
                                  std::size_t reference_index,
                                  double timestamp_sec)
{
    if (!enable_visualization_ || path_.empty())
        return;
    const std::size_t safe_index = std::min(reference_index, path_.size() - 1);
    std::lock_guard<std::mutex> lock(mutex_);
    bool sampled = false;
    if (!has_sample_time_ || timestamp_sec - last_sample_time_sec_ >= sample_interval_sec_)
    {
        actual_points_.push_back({state.x, state.y, state.yaw});
        last_sample_time_sec_ = timestamp_sec;
        has_sample_time_ = true;
        sampled = true;
    }
    const bool outside = !has_window_anchor_
        || std::abs(state.x - path_[window_anchor_index_].x) > kWindowHalfSizeM
        || std::abs(state.y - path_[window_anchor_index_].y) > kWindowHalfSizeM;
    const bool recenter = outside && (!has_window_anchor_
                                     || safe_index != window_anchor_index_);
    if (!sampled && !recenter)
        return;
    if (recenter)
    {
        window_anchor_index_ = safe_index;
        has_window_anchor_ = true;
    }
    pending_frame_.state = state;
    pending_frame_.sequence = ++next_frame_sequence_;
    pending_frame_.reference_index = safe_index;
    pending_frame_.window_anchor_index = window_anchor_index_;
    pending_frame_.actual_points.clear();
    pending_frame_.actual_total = actual_points_.size();
    const PathPoint& center = path_[window_anchor_index_];
    for (auto it = actual_points_.rbegin(); it != actual_points_.rend() &&
         pending_frame_.actual_points.size() < 20; ++it)
    {
        if (std::abs(it->x - center.x) <= kWindowHalfSizeM
            && std::abs(it->y - center.y) <= kWindowHalfSizeM)
            pending_frame_.actual_points.push_back(*it);
    }
    pending_frame_.valid = true;
    latest_requested_sequence_ = pending_frame_.sequence;
    frame_pending_ = true;
    condition_.notify_one();
}

// 停止后台线程，保存完整实际轨迹和最终两种对比图。
void TrajectoryVisualizer::saveFinal()
{
    if (!enable_visualization_)
        return;
    stopWorker_();
    std::vector<Pose2D> actual_points;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        actual_points = actual_points_;
    }
    saveActualPath_(actual_points);
    renderGlobalComparison_(actual_points);
    if (has_window_anchor_)
    {
        Frame frame;
        frame.state = actual_points.empty()
            ? VehicleState2D{path_[window_anchor_index_].x,
                             path_[window_anchor_index_].y,
                             path_[window_anchor_index_].yaw, 0.0, 0.0}
            : VehicleState2D{actual_points.back().x, actual_points.back().y,
                             actual_points.back().yaw, 0.0, 0.0};
        frame.reference_index = window_anchor_index_;
        frame.window_anchor_index = window_anchor_index_;
        frame.actual_points = actual_points;
        frame.actual_total = actual_points.size();
#ifdef ECHOSIM_USE_OPENCV
        const cv::Mat image = renderFrame_(frame);
        if (!image.empty())
        {
            cv::imwrite((std::filesystem::path(output_directory_)
                         / "tracking_local_final.png").string(), image);
        }
#endif
    }
}

// 在控制主线程显示后台线程生成的最新局部窗口图像。
void TrajectoryVisualizer::pumpWindow()
{
    if (!enable_visualization_)
        return;
#ifdef ECHOSIM_USE_OPENCV
    cv::Mat image;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (image_pending_)
        {
            image = latest_image_;
            image_pending_ = false;
        }
    }
    if (!image.empty())
        cv::imshow("tracking_local", image);
    // 即使没有新图像也必须持续处理窗口消息，否则拖动和关闭操作会明显滞后。
    cv::waitKey(1);
#endif
}

// 请求后台线程退出并等待其结束。
void TrajectoryVisualizer::stopWorker_()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
        frame_pending_ = false;
    }
    condition_.notify_all();
    if (worker_.joinable())
        worker_.join();
}

// 后台线程只处理最新待渲染帧，避免阻塞控制循环。
void TrajectoryVisualizer::workerLoop_()
{
    while (true)
    {
        Frame frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_requested_ || frame_pending_; });
            if (stop_requested_ && !frame_pending_)
                return;
            frame = pending_frame_;
            frame_pending_ = false;
        }
        if (frame.valid)
        {
#ifdef ECHOSIM_USE_OPENCV
            cv::Mat image = renderFrame_(frame);
            if (!image.empty())
            {
                std::lock_guard<std::mutex> lock(mutex_);
                // 控制循环可能在本帧渲染时提交了更新帧，旧帧不能回退覆盖。
                if (frame.sequence == latest_requested_sequence_)
                {
                    latest_image_ = std::move(image);
                    image_pending_ = true;
                }
            }
#endif
        }
    }
}

// 绘制当前参考点周围 20m x 20m 的局部跟踪窗口。
cv::Mat TrajectoryVisualizer::renderFrame_(const Frame& frame) const
{
    const PathPoint& center = path_[frame.window_anchor_index];
    cv::Mat image(kLocalImageSize, kLocalImageSize, CV_8UC3,
                  cv::Scalar(255, 255, 255));
    std::vector<cv::Point> reference_pixels;
    const double min_x = center.x - kWindowHalfSizeM;
    const double max_x = center.x + kWindowHalfSizeM;
    const double min_y = center.y - kWindowHalfSizeM;
    const double max_y = center.y + kWindowHalfSizeM;
    std::vector<std::size_t> reference_indices;
    for (std::size_t index = 0; index < path_.size(); ++index)
    {
        if (path_[index].x < min_x || path_[index].x > max_x
            || path_[index].y < min_y || path_[index].y > max_y)
            continue;
        const cv::Point point = toLocalPixel(path_[index].x, path_[index].y,
                                             center.x, center.y);
        if (inImage(point))
        {
            reference_pixels.push_back(point);
            reference_indices.push_back(index);
        }
    }
    if (reference_pixels.size() > 1)
        cv::polylines(image, reference_pixels, false, cv::Scalar(0, 0, 255), 1);
    // 保留原有全局路径点数字标注，便于在实时窗口中核对控制索引。
    for (std::size_t index = 0; index < reference_pixels.size(); ++index)
        cv::putText(image, std::to_string(reference_indices[index]),
                    reference_pixels[index], cv::FONT_HERSHEY_SIMPLEX, 0.3,
                    cv::Scalar(0, 0, 255), 1);
    std::vector<cv::Point> actual_pixels;
    for (const Pose2D& actual : frame.actual_points)
    {
        const cv::Point point = toLocalPixel(actual.x, actual.y, center.x, center.y);
        if (inImage(point))
            actual_pixels.push_back(point);
    }
    if (actual_pixels.size() > 1)
        cv::polylines(image, actual_pixels, false, cv::Scalar(255, 0, 0), 1);
    for (const cv::Point& point : actual_pixels)
        cv::circle(image, point, 2, cv::Scalar(255, 0, 0), -1);
    const cv::Point vehicle = toLocalPixel(frame.state.x, frame.state.y,
                                           center.x, center.y);
    if (inImage(vehicle))
    {
        cv::circle(image, vehicle, 3, cv::Scalar(0, 128, 0), -1);
        // 用绿色线段表示车辆当前实际航向，线段起点为车辆位置。
        const cv::Point heading_end = toLocalPixel(
            frame.state.x + kVehicleHeadingLengthM * std::cos(frame.state.yaw),
            frame.state.y + kVehicleHeadingLengthM * std::sin(frame.state.yaw),
            center.x, center.y);
        if (inImage(heading_end))
            cv::line(image, vehicle, heading_end, cv::Scalar(0, 128, 0), 2);
    }
    auto draw_marker = [&](const Pose2D& pose, const char* label) {
        const cv::Point point = toLocalPixel(pose.x, pose.y, center.x, center.y);
        if (!inImage(point))
            return;
        cv::circle(image, point, 4, cv::Scalar(255, 0, 0), 1);
        cv::putText(image, label, point + cv::Point(4, -4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
    };
    draw_marker(start_, "start");
    for (std::size_t index = 0; index < waypoints_.size(); ++index)
    {
        const std::string label = "wp" + std::to_string(index + 1);
        const Pose2D& waypoint = waypoints_[index];
        const cv::Point point = toLocalPixel(waypoint.x, waypoint.y,
                                             center.x, center.y);
        if (inImage(point))
        {
            cv::circle(image, point, 4, cv::Scalar(0, 165, 255), -1);
            cv::putText(image, label, point + cv::Point(4, -4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3,
                        cv::Scalar(0, 165, 255), 1);
        }
    }
    draw_marker(goal_, "goal");

    std::vector<std::string> status_lines{
        "window_ref=" + std::to_string(frame.window_anchor_index),
        "control_ref=" + std::to_string(frame.reference_index)};
    const std::vector<std::string> path_index_lines = pathIndexStatusLines_(
        path_, waypoints_, goal_);
    status_lines.insert(status_lines.end(), path_index_lines.begin(),
                        path_index_lines.end());
    for (std::size_t index = 0; index < status_lines.size(); ++index)
    {
        cv::putText(image, status_lines[index], {5, 15 + static_cast<int>(index) * 15},
                    cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(0, 0, 0), 1);
    }
    return image;
}

// 保存实际路径点。
bool TrajectoryVisualizer::saveActualPath_(
    const std::vector<Pose2D>& actual_points) const
{
    std::ofstream output(std::filesystem::path(output_directory_) / "actual_path.txt");
    if (!output)
        return false;
    output << std::setprecision(17);
    for (const Pose2D& point : actual_points)
        output << point.x << ' ' << point.y << ' ' << point.yaw << '\n';
    return output.good();
}

// 绘制只包含全局路径和实际路径的白底最终对比图。
bool TrajectoryVisualizer::renderGlobalComparison_(
    const std::vector<Pose2D>& actual_points) const
{
#ifdef ECHOSIM_USE_OPENCV
    double min_x = std::min(start_.x, goal_.x);
    double max_x = std::max(start_.x, goal_.x);
    double min_y = std::min(start_.y, goal_.y);
    double max_y = std::max(start_.y, goal_.y);
    for (const PathPoint& point : path_)
    {
        min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
    }
    for (const Pose2D& point : actual_points)
    {
        min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
    }
    const double scale = 1.0;
    const int margin = 30;
    const int width = std::max(200, static_cast<int>((max_x - min_x) * scale) + 2 * margin);
    const int height = std::max(200, static_cast<int>((max_y - min_y) * scale) + 2 * margin);
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    auto to_pixel = [&](double x, double y) {
        return cv::Point(margin + static_cast<int>((x - min_x) * scale),
                         height - margin - static_cast<int>((y - min_y) * scale));
    };
    std::vector<cv::Point> reference_pixels;
    for (const PathPoint& point : path_)
        reference_pixels.push_back(to_pixel(point.x, point.y));
    if (reference_pixels.size() > 1)
        cv::polylines(image, reference_pixels, false, cv::Scalar(0, 0, 255), 2);
    std::vector<cv::Point> actual_pixels;
    for (const Pose2D& point : actual_points)
        actual_pixels.push_back(to_pixel(point.x, point.y));
    if (actual_pixels.size() > 1)
        cv::polylines(image, actual_pixels, false, cv::Scalar(255, 0, 0), 2);
    cv::circle(image, to_pixel(start_.x, start_.y), 5, cv::Scalar(0, 180, 0), -1);
    for (const Pose2D& waypoint : waypoints_)
        cv::circle(image, to_pixel(waypoint.x, waypoint.y), 5,
                   cv::Scalar(0, 165, 255), -1);
    cv::circle(image, to_pixel(goal_.x, goal_.y), 5, cv::Scalar(255, 0, 0), -1);
    return cv::imwrite((std::filesystem::path(output_directory_)
                        / "global_and_actual_final.png").string(), image);
#else
    (void)actual_points;
    return false;
#endif
}
