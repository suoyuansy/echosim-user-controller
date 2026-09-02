#pragma once

#include <algorithm>
#include <fstream>
#include <deque>
#include <cstddef>
#include <string>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/core/mat.hpp>
#endif

// 按仿真时间记录纵向控制数据，供离线调节加速度 PID。
class SpeedControlLogger
{
public:
    ~SpeedControlLogger();

    bool initialize(const std::string& output_directory, bool enabled);
    void update(double timestamp_sec,
                double target_speed_mps,
                double actual_speed_mps,
                double acceleration_mps2);
    // 关闭实时窗口并释放资源，不保存速度图片。
    void close();

    // 固定速度窗口左边界：始终显示最新仿真时刻之前的 10 秒。
    static double windowStartSec_(double latest_time_sec)
    {
        return std::max(0.0, latest_time_sec - 10.0);
    }

private:
    struct Sample
    {
        double time_sec = 0.0;
        double target_speed_mps = 0.0;
        double actual_speed_mps = 0.0;
        double acceleration_mps2 = 0.0;
    };
#ifdef ECHOSIM_USE_OPENCV
    void renderWindow_() const;
#endif
    std::ofstream output_;
    bool enabled_ = false;
    bool has_time_ = false;
    double last_time_sec_ = 0.0;
    double next_sample_time_sec_ = 0.0;
    std::deque<Sample> samples_;
};
