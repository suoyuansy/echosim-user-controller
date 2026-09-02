#pragma once

#include <algorithm>
#include <cmath>

struct SimulationTimeStep
{
    double time_sec = 0.0;
    double dt_sec = 0.0;
    bool reported_time_valid = false;
};

// 用状态消息中的仿真时间驱动控制器；只有在整个运行过程都没有有效仿真时间时，
// 才使用固定步长兜底。重复/回退时间不会推进 PID 积分和停车驻留计时。
class SimulationClock
{
public:
    SimulationTimeStep update(double reported_time_sec,
                              double fallback_dt_sec = 0.1)
    {
        const bool finite = std::isfinite(reported_time_sec);
        if (finite && (!initialized_
                       || reported_time_sec > time_sec_ + 1e-9))
        {
            const double dt = initialized_
                ? std::clamp(reported_time_sec - time_sec_, 0.0, 1.0)
                : 0.0;
            time_sec_ = reported_time_sec;
            initialized_ = true;
            return {time_sec_, dt, true};
        }

        if (!initialized_ && std::isfinite(fallback_dt_sec)
            && fallback_dt_sec > 0.0)
        {
            time_sec_ += fallback_dt_sec;
            return {time_sec_, fallback_dt_sec, false};
        }
        return {time_sec_, 0.0, false};
    }

    double timeSec() const { return time_sec_; }
    bool initialized() const { return initialized_; }

private:
    double time_sec_ = 0.0;
    bool initialized_ = false;
};

