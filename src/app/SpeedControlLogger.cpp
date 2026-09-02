#include "app/SpeedControlLogger.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace
{
constexpr double kHistoryDurationSec = 10.0;
constexpr double kSampleIntervalSec = 0.1;
constexpr int kImageWidth = 1000;
constexpr int kImageHeight = 600;
constexpr int kLeftMargin = 70;
constexpr int kRightMargin = 25;
constexpr int kTopMargin = 55;
constexpr int kBottomMargin = 60;
constexpr char kWindowName[] = "speed_control";
}

SpeedControlLogger::~SpeedControlLogger()
{
    close();
}

bool SpeedControlLogger::initialize(const std::string& output_directory,
                                    bool enabled)
{
    close();
    enabled_ = enabled;
    samples_.clear();
    next_sample_time_sec_ = 0.0;
    if (!enabled_)
        return true;

    std::error_code error;
    std::filesystem::create_directories(output_directory, error);
    if (error)
        return false;
    output_.open(std::filesystem::path(output_directory)
                 / "speed_control_log.txt", std::ios::trunc);
    if (!output_)
        return false;
    output_ << "# sim_time_sec target_speed_mps actual_speed_mps "
               "acceleration_mps2\n";
    output_ << std::fixed << std::setprecision(6);
#ifdef ECHOSIM_USE_OPENCV
    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
#endif
    return output_.good();
}

void SpeedControlLogger::update(double timestamp_sec,
                                double target_speed_mps,
                                double actual_speed_mps,
                                double acceleration_mps2)
{
    if (!enabled_ || !output_ || !std::isfinite(timestamp_sec))
        return;
    if (has_time_ && timestamp_sec < next_sample_time_sec_ - 1e-9)
        return;
    output_ << timestamp_sec << ' ' << target_speed_mps << ' '
            << actual_speed_mps << ' ' << acceleration_mps2 << '\n';
    if (output_)
    {
        last_time_sec_ = timestamp_sec;
        has_time_ = true;
        next_sample_time_sec_ = timestamp_sec + kSampleIntervalSec;
        samples_.push_back({timestamp_sec, target_speed_mps,
                            actual_speed_mps, acceleration_mps2});
        while (!samples_.empty()
               && timestamp_sec - samples_.front().time_sec
                    > kHistoryDurationSec)
            samples_.pop_front();
#ifdef ECHOSIM_USE_OPENCV
        renderWindow_();
        cv::waitKey(1);
#endif
    }
}

#ifdef ECHOSIM_USE_OPENCV
void SpeedControlLogger::renderWindow_() const
{
    if (samples_.empty())
        return;
    cv::Mat image(kImageHeight, kImageWidth, CV_8UC3,
                  cv::Scalar(248, 248, 248));
    const int left = kLeftMargin;
    const int right = kImageWidth - kRightMargin;
    const int top = kTopMargin;
    const int bottom = kImageHeight - kBottomMargin;
    const double last_time = samples_.back().time_sec;
    const double first_time = windowStartSec_(last_time);
    constexpr double time_span = 10.0;
    double max_abs_speed = 1.0;
    for (const Sample& sample : samples_)
        max_abs_speed = std::max(max_abs_speed,
            std::max(std::abs(sample.target_speed_mps),
                     std::abs(sample.actual_speed_mps)));
    max_abs_speed = std::ceil(max_abs_speed * 1.2);
    auto to_pixel = [&](double time_sec, double speed_mps) {
        const double x_ratio = (time_sec - first_time) / time_span;
        const double y_ratio = (speed_mps + max_abs_speed)
            / (2.0 * max_abs_speed);
        return cv::Point(
            left + static_cast<int>(std::lround(x_ratio * (right - left))),
            bottom - static_cast<int>(std::lround(y_ratio * (bottom - top))));
    };
    cv::rectangle(image, {left, top}, {right, bottom},
                  cv::Scalar(60, 60, 60), 1);
    // 速度窗口固定为 10 秒，每 0.1 秒一个细网格；每 1 秒绘制一条较深的主网格。
    for (int tick = 0; tick <= 100; ++tick)
    {
        const int x = left + static_cast<int>(std::lround(
            static_cast<double>(tick) / 100.0 * (right - left)));
        const cv::Scalar color = (tick % 10 == 0)
            ? cv::Scalar(205, 205, 205) : cv::Scalar(232, 232, 232);
        cv::line(image, {x, top}, {x, bottom}, color, 1);
        if (tick % 10 == 0)
        {
            const double label_time = first_time + tick * 0.1;
            cv::putText(image, std::to_string(static_cast<int>(std::lround(
                            label_time))),
                        {x - 12, bottom + 20}, cv::FONT_HERSHEY_SIMPLEX,
                        0.45, cv::Scalar(80, 80, 80), 1);
        }
    }
    for (int tick = 0; tick <= 4; ++tick)
    {
        const int y = top + static_cast<int>(std::lround(
            static_cast<double>(tick) / 4.0 * (bottom - top)));
        cv::line(image, {left, y}, {right, y}, cv::Scalar(225, 225, 225), 1);
    }
    cv::line(image, to_pixel(first_time, 0.0), to_pixel(last_time, 0.0),
             cv::Scalar(210, 210, 210), 1);
    for (std::size_t index = 1; index < samples_.size(); ++index)
    {
        const Sample& previous = samples_[index - 1];
        const Sample& current = samples_[index];
        cv::line(image, to_pixel(previous.time_sec, previous.target_speed_mps),
                 to_pixel(current.time_sec, current.target_speed_mps),
                 cv::Scalar(30, 100, 220), 2);
        cv::line(image, to_pixel(previous.time_sec, previous.actual_speed_mps),
                 to_pixel(current.time_sec, current.actual_speed_mps),
                 cv::Scalar(40, 150, 40), 2);
    }
    const Sample& latest = samples_.back();
    cv::putText(image, "target=" + std::to_string(latest.target_speed_mps)
                    + " m/s", {left, 25}, cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(30, 100, 220), 2);
    cv::putText(image, "actual=" + std::to_string(latest.actual_speed_mps)
                    + " m/s", {left + 260, 25},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(40, 150, 40), 2);
    cv::putText(image, "time=" + std::to_string(latest.time_sec) + " s",
                {left + 540, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(50, 50, 50), 2);
    cv::putText(image, "speed (m/s)", {8, top - 18},
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(50, 50, 50), 1);
    cv::putText(image, "recent 10 s", {right - 125, bottom + 38},
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(50, 50, 50), 1);
    cv::imshow(kWindowName, image);
}
#endif

void SpeedControlLogger::close()
{
    if (output_.is_open())
        output_.close();
#ifdef ECHOSIM_USE_OPENCV
    if (enabled_)
    {
        cv::destroyWindow(kWindowName);
        cv::waitKey(1);
    }
#endif
    enabled_ = false;
    has_time_ = false;
    next_sample_time_sec_ = 0.0;
    samples_.clear();
}
