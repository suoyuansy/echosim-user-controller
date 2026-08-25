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
// 局部窗口几何：半边长 10 m（即 20m x 20m 窗口）、分辨率 0.1 m/像素、
// 图像边长 200 像素（= 20 m / 0.1 m，三者需保持数值自洽）。
constexpr double kWindowHalfSizeM = 10.0;
constexpr double kLocalResolutionM = 0.1;
constexpr int kLocalImageSize = 200;
constexpr double kVehicleHeadingLengthM = 1.0; // 实际航向线段长度，单位为米。

// 将窗口世界坐标转换为图像坐标。
// 换算：以窗口中心 (center_x, center_y) 为原点，世界 x -> 图像列 col
// （向右为正），世界 y -> 图像行 row 且上下翻转（图像 y 轴向下）：
//   col = (x - center_x + 10) / 0.1，row = 199 - (y - center_y + 10) / 0.1。
// 输入单位为米，输出为 OpenCV 像素坐标。
#ifdef ECHOSIM_USE_OPENCV
cv::Point toLocalPixel(double x, double y, double center_x, double center_y)
{
    // 相对中心的 x 偏移（米）换算为列：-10 m -> 第 0 列，+10 m -> 第 199 列。
    const int col = static_cast<int>(std::lround(
        (x - center_x + kWindowHalfSizeM) / kLocalResolutionM));
    // 相对中心的 y 偏移换算为行后翻转：+10 m（世界"北"）-> 第 0 行（图像顶部）。
    const int row = kLocalImageSize - 1 - static_cast<int>(std::lround(
        (y - center_y + kWindowHalfSizeM) / kLocalResolutionM));
    return {col, row};
}

// 判断像素是否落在 200x200 图像范围内；越界点由调用方丢弃不绘制。
bool inImage(const cv::Point& point)
{
    return point.x >= 0 && point.x < kLocalImageSize
        && point.y >= 0 && point.y < kLocalImageSize;
}
#endif
} // namespace

TrajectoryVisualizer::~TrajectoryVisualizer()
{
    // 即使外部忘了调用 saveFinal（异常退出路径），析构也要先停线程再销毁成员。
    stopWorker_();
}

// 初始化可视化对象，关闭可视化时不创建线程也不产生文件。
bool TrajectoryVisualizer::initialize(const Path& path,
                                      const Pose2D& start,
                                      const Pose2D& goal,
                                      const std::string& output_directory,
                                      bool enable_visualization,
                                      double sample_interval_sec)
{
    // 先停掉可能存在的旧渲染线程，再重建全部状态（支持重复 initialize）。
    stopWorker_();
    // 空路径无法可视化，直接返回失败。
    if (path.empty())
        return false;
    // 复制参考路径与任务信息；采样间隔下限 0.1 s 防止传入 0 导致逐帧采样。
    path_ = path;
    start_ = start;
    goal_ = goal;
    output_directory_ = output_directory;
    enable_visualization_ = enable_visualization;
    sample_interval_sec_ = std::max(0.1, sample_interval_sec);
    actual_points_.clear();
    has_sample_time_ = false;
    has_window_anchor_ = false;
    frame_pending_ = false;
#ifdef ECHOSIM_USE_OPENCV
    latest_image_.release();
    image_pending_ = false;
#endif
    stop_requested_ = false;
    // 可视化关闭：到此即初始化成功，但既不建目录也不创建线程/窗口。
    if (!enable_visualization_)
        return true;
    // 可视化开启：确保输出目录存在（最终结果文件写在这里）。
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
    // 启动后台渲染线程：图像生成在 worker 中进行，主线程只负责显示。
    worker_ = std::thread(&TrajectoryVisualizer::workerLoop_, this);
    // 初始化后立即提交一帧起点窗口，保持旧版本启动即显示实时窗口的行为。
    {
        std::lock_guard<std::mutex> lock(mutex_);
        window_anchor_index_ = 0;
        has_window_anchor_ = true;
        pending_frame_.state = {start_.x, start_.y, start_.yaw, 0.0, 0.0};
        pending_frame_.reference_index = 0;
        pending_frame_.window_anchor_index = 0;
        pending_frame_.actual_points.clear();
        pending_frame_.actual_total = 0;
        pending_frame_.valid = true;
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
    // 可视化关闭或无路径时什么都不做（控制循环每周期都会调用本函数）。
    if (!enable_visualization_ || path_.empty())
        return;
    // 参考序号夹紧到合法范围，防御跟踪层传入越界值。
    const std::size_t safe_index = std::min(reference_index, path_.size() - 1);
    // 所有共享状态的修改都在互斥锁内完成（后台线程会并发读取）。
    std::lock_guard<std::mutex> lock(mutex_);
    bool sampled = false;
    // 轨迹采样：首次调用或距上次采样超过采样间隔时，记录一个实际位姿点。
    if (!has_sample_time_ || timestamp_sec - last_sample_time_sec_ >= sample_interval_sec_)
    {
        actual_points_.push_back({state.x, state.y, state.yaw});
        last_sample_time_sec_ = timestamp_sec;
        has_sample_time_ = true;
        sampled = true;
    }
    // 判断车辆是否已走出当前局部窗口（相对窗口中心参考点，x 或 y 超过半边长）。
    const bool outside = !has_window_anchor_
        || std::abs(state.x - path_[window_anchor_index_].x) > kWindowHalfSizeM
        || std::abs(state.y - path_[window_anchor_index_].y) > kWindowHalfSizeM;
    // 窗口重定中心：走出窗口且最近参考点已变化才换中心，避免窗口来回抖动。
    const bool recenter = outside && (!has_window_anchor_
                                     || safe_index != window_anchor_index_);
    // 本周期既没采样也不用换窗：无需渲染新帧，直接返回。
    if (!sampled && !recenter)
        return;
    if (recenter)
    {
        // 新窗口中心取当前最近参考点。
        window_anchor_index_ = safe_index;
        has_window_anchor_ = true;
    }
    // 组装待渲染帧：车辆状态、控制参考序号、窗口中心序号及窗口内近期实际点。
    pending_frame_.state = state;
    pending_frame_.reference_index = safe_index;
    pending_frame_.window_anchor_index = window_anchor_index_;
    pending_frame_.actual_points.clear();
    pending_frame_.actual_total = actual_points_.size();
    // 从最新往回最多收集 20 个落在窗口内的实际点（逆序遍历），足够画出近期轨迹。
    const PathPoint& center = path_[window_anchor_index_];
    for (auto it = actual_points_.rbegin(); it != actual_points_.rend() &&
         pending_frame_.actual_points.size() < 20; ++it)
    {
        // 只保留距窗口中心 x/y 均不超过半边长的点。
        if (std::abs(it->x - center.x) <= kWindowHalfSizeM
            && std::abs(it->y - center.y) <= kWindowHalfSizeM)
            pending_frame_.actual_points.push_back(*it);
    }
    // 帧标记有效并唤醒后台线程渲染（新帧直接覆盖旧帧：只渲染最新状态）。
    pending_frame_.valid = true;
    frame_pending_ = true;
    condition_.notify_one();
}

// 停止后台线程，保存完整实际轨迹和最终两种对比图。
void TrajectoryVisualizer::saveFinal()
{
    if (!enable_visualization_)
        return;
    // 先停渲染线程：之后单线程访问共享数据，不再需要加锁。
    stopWorker_();
    // 快照完整实际轨迹（锁内拷贝，之后不再持锁）。
    std::vector<Pose2D> actual_points;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        actual_points = actual_points_;
    }
    // 依次输出 actual_path.txt（实际轨迹文本）与全局对比图。
    saveActualPath_(actual_points);
    renderGlobalComparison_(actual_points);
    // 再补一帧以最终车辆位置为中心的局部窗口图（tracking_local_final.png）。
    if (has_window_anchor_)
    {
        // 一个实际点都没有（如刚初始化就结束）时，用窗口中心参考点代替车辆位置。
        Frame frame;
        frame.state = actual_points.empty()
            ? VehicleState2D{path_[window_anchor_index_].x,
                             path_[window_anchor_index_].y,
                             0.0, path_[window_anchor_index_].yaw, 0.0, 0.0}
            : VehicleState2D{actual_points.back().x, actual_points.back().y,
                             0.0, actual_points.back().yaw, 0.0, 0.0};
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
    // 短暂持锁取走后台线程最新完成的图像；没有新图则本周期不刷新画面。
    cv::Mat image;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (image_pending_)
        {
            image = latest_image_;
            image_pending_ = false;
        }
    }
    // 只在有新图像时 imshow，减少无谓重绘开销。
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
    // 唤醒所有等待中的 worker（退出请求与新帧共用同一个条件变量）。
    condition_.notify_all();
    // 等待线程真正退出；若线程正在渲染，会先完成当前帧再检查退出标志。
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
            // 阻塞等待"有新帧或请求退出"；虚假唤醒由条件变量谓词兜底。
            condition_.wait(lock, [this] { return stop_requested_ || frame_pending_; });
            // 收到退出请求且没有遗留帧：直接结束线程。
            if (stop_requested_ && !frame_pending_)
                return;
            // 取走当前待渲染帧并清标记；等待期间的新帧会覆盖 pending_frame_，
            // 天然实现"只渲染最新帧、丢弃过期帧"。
            frame = pending_frame_;
            frame_pending_ = false;
        }
        // 锁外渲染，渲染结果再短暂持锁交回主线程显示。
        if (frame.valid)
        {
#ifdef ECHOSIM_USE_OPENCV
            cv::Mat image = renderFrame_(frame);
            if (!image.empty())
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_image_ = std::move(image);
                image_pending_ = true;
            }
#endif
        }
    }
}

// 绘制当前参考点周围 20m x 20m 的局部跟踪窗口。
// 输入：Frame（车辆状态、窗口中心参考点序号、窗口内近期实际点）；
// 输出：200x200 三通道图像。颜色约定：白底；红色 = 参考路径及参考点序号；
// 蓝色 = 实际轨迹及起终点标记；绿色 = 车辆当前位置与航向线段；
// 左上角黑色文本 = 状态信息（窗口/控制参考序号、点数统计）。
cv::Mat TrajectoryVisualizer::renderFrame_(const Frame& frame) const
{
    // 窗口中心 = 帧携带的参考点；所有世界坐标都相对它换算为像素。
    const PathPoint& center = path_[frame.window_anchor_index];
    // 白底画布，尺寸 kLocalImageSize x kLocalImageSize（0.1 m/像素）。
    cv::Mat image(kLocalImageSize, kLocalImageSize, CV_8UC3,
                  cv::Scalar(255, 255, 255));
    std::vector<cv::Point> reference_pixels;
    const double min_x = center.x - kWindowHalfSizeM;
    const double max_x = center.x + kWindowHalfSizeM;
    const double min_y = center.y - kWindowHalfSizeM;
    const double max_y = center.y + kWindowHalfSizeM;
    // 第一遍：收集落在窗口内的参考路径点及其在 path_ 中的序号。
    std::vector<std::size_t> reference_indices;
    for (std::size_t index = 0; index < path_.size(); ++index)
    {
        // 先用世界坐标粗筛（窗口外直接跳过），再换算为像素。
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
    // 参考路径：红色折线连接各点，并在每个点旁标注其在 path_ 中的序号。
    if (reference_pixels.size() > 1)
        cv::polylines(image, reference_pixels, false, cv::Scalar(0, 0, 255), 1);
    for (std::size_t index = 0; index < reference_pixels.size(); ++index)
        cv::putText(image, std::to_string(reference_indices[index]), reference_pixels[index],
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    // 第二遍：实际轨迹画为蓝色折线 + 小实心圆（半径 2 像素），
    // 与红色参考路径的偏离程度直观反映跟踪误差。
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
    // 第三遍：车辆当前位置画深绿色实心圆（半径 3 像素）。
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
    // 通用标记绘制：蓝色空心圆 + 蓝色标签（偏移 4 像素），用于起终点。
    auto draw_marker = [&](const Pose2D& pose, const char* label) {
        const cv::Point point = toLocalPixel(pose.x, pose.y, center.x, center.y);
        if (!inImage(point))
            return;
        cv::circle(image, point, 4, cv::Scalar(255, 0, 0), 1);
        cv::putText(image, label, point + cv::Point(4, -4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
    };
    draw_marker(start_, "start");
    draw_marker(goal_, "goal");

    // 左上角黑色状态文本（每行间隔 15 像素）：窗口中心参考序号、控制参考
    // 序号、累计实际点总数和本窗口可见实际点数。
    const std::vector<std::string> status_lines{
        "window_ref=" + std::to_string(frame.window_anchor_index),
        "control_ref=" + std::to_string(frame.reference_index),
        "actual_total=" + std::to_string(frame.actual_total),
        "visible_actual=" + std::to_string(actual_pixels.size())};
    for (std::size_t index = 0; index < status_lines.size(); ++index)
    {
        cv::putText(image, status_lines[index], {5, 15 + static_cast<int>(index) * 15},
                    cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(0, 0, 0), 1);
    }
    return image;
}

// 保存实际路径点。
// 输出 actual_path.txt，每行 "x y yaw"（米/弧度，17 位有效数字），
// 与 saveFinal 的对比图一起构成任务结束后的离线分析数据。
bool TrajectoryVisualizer::saveActualPath_(
    const std::vector<Pose2D>& actual_points) const
{
    std::ofstream output(std::filesystem::path(output_directory_) / "actual_path.txt");
    // 打开失败（目录不存在等）返回 false，由调用方忽略（可视化属辅助功能）。
    if (!output)
        return false;
    output << std::setprecision(17);
    for (const Pose2D& point : actual_points)
        output << point.x << ' ' << point.y << ' ' << point.yaw << '\n';
    return output.good();
}

// 绘制只包含全局路径和实际路径的白底最终对比图。
// 输出 global_and_actual_final.png。坐标换算：scale = 1.0 即 1 米/像素，
// 全图范围取全局路径与实际轨迹的联合包围盒，四周留 30 像素边距；
// x -> 列（向右），y -> 行并上下翻转（世界 y 向上）。颜色：红色折线 =
// 全局参考路径；蓝色折线 = 实际轨迹；绿色实心圆 = 起点；蓝色实心圆 = 终点。
bool TrajectoryVisualizer::renderGlobalComparison_(
    const std::vector<Pose2D>& actual_points) const
{
#ifdef ECHOSIM_USE_OPENCV
    // 第一遍：计算起点、终点、全局路径与实际轨迹的联合包围盒。
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
    // 换算参数：1 米/像素，最小边 200 像素，四周 30 像素白边。
    const double scale = 1.0;
    const int margin = 30;
    const int width = std::max(200, static_cast<int>((max_x - min_x) * scale) + 2 * margin);
    const int height = std::max(200, static_cast<int>((max_y - min_y) * scale) + 2 * margin);
    // 白底画布；to_pixel 完成世界米 -> 像素的平移、缩放与 y 轴上下翻转。
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    auto to_pixel = [&](double x, double y) {
        return cv::Point(margin + static_cast<int>((x - min_x) * scale),
                         height - margin - static_cast<int>((y - min_y) * scale));
    };
    // 红色粗线（宽 2）：全局参考路径。
    std::vector<cv::Point> reference_pixels;
    for (const PathPoint& point : path_)
        reference_pixels.push_back(to_pixel(point.x, point.y));
    if (reference_pixels.size() > 1)
        cv::polylines(image, reference_pixels, false, cv::Scalar(0, 0, 255), 2);
    // 蓝色粗线（宽 2）：实际轨迹；与红线的偏离程度即全程跟踪误差。
    std::vector<cv::Point> actual_pixels;
    for (const Pose2D& point : actual_points)
        actual_pixels.push_back(to_pixel(point.x, point.y));
    if (actual_pixels.size() > 1)
        cv::polylines(image, actual_pixels, false, cv::Scalar(255, 0, 0), 2);
    // 起点绿圆、终点蓝圆（半径 5 像素实心）。
    cv::circle(image, to_pixel(start_.x, start_.y), 5, cv::Scalar(0, 180, 0), -1);
    cv::circle(image, to_pixel(goal_.x, goal_.y), 5, cv::Scalar(255, 0, 0), -1);
    return cv::imwrite((std::filesystem::path(output_directory_)
                        / "global_and_actual_final.png").string(), image);
#else
    (void)actual_points;
    return false;
#endif
}
