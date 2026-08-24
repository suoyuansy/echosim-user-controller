#include "TaskConfig.h"

#include <array>
#include <cstdlib>
#include <filesystem>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

std::filesystem::path findProjectRoot_()
{
    std::filesystem::path candidate = std::filesystem::absolute(
        std::filesystem::current_path());
    for (int level = 0; level < 8 && !candidate.empty(); ++level)
    {
        if ((std::filesystem::exists(candidate / "CMakeLists.txt")
             && std::filesystem::exists(candidate / "src" / "TaskConfig.cpp"))
            || std::filesystem::exists(candidate / "UserControllerTest.vcxproj"))
        {
            return candidate;
        }
        candidate = candidate.parent_path();
    }
    return std::filesystem::absolute(std::filesystem::current_path());
}

std::filesystem::path environmentPath_(const char* name)
{
    const char* value = std::getenv(name);
    if (value != nullptr && value[0] != '\0')
        return std::filesystem::path(value);
    return {};
}

bool looksLikeTerrainRoot_(const std::filesystem::path& path)
{
    return std::filesystem::exists(path / "level_1m")
        || std::filesystem::exists(path / "level_10m")
        || std::filesystem::exists(path / "level_100m")
        || std::filesystem::exists(path / "level_1000m");
}

std::filesystem::path findTerrainRoot_(const std::filesystem::path& project_root)
{
    for (const char* variable :
         {"USER_CONTROLLER_TERRAIN_ROOT", "ECHOSIM_TERRAIN_ROOT"})
    {
        const std::filesystem::path override_path = environmentPath_(variable);
        if (!override_path.empty())
            return std::filesystem::absolute(override_path);
    }

    const std::filesystem::path project_parent = project_root.parent_path();
    const std::array<std::filesystem::path, 4> candidates = {
        project_root / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
        project_root / "EchoSim" / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
        project_parent / "EchoSim" / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
        project_parent / "data" / "project" / "Onsite8_B" / "Heightmap" / "Moon2",
    };

    for (const std::filesystem::path& candidate : candidates)
    {
        if (looksLikeTerrainRoot_(candidate))
            return std::filesystem::absolute(candidate);
    }

    return std::filesystem::absolute(candidates[2]);
}
} // namespace

TaskConfig makeDefaultTaskConfig()
{
    const std::filesystem::path project_root = findProjectRoot_();

    TaskConfig config;
    config.start = {-901.787, -3016.257, 0.0};
    config.goal = { -465.065, -1269.836, 0.0};
    config.terrain_root = findTerrainRoot_(project_root);
    config.output_directory = project_root / "output";
    config.scan_bounds = {-5000.0, -5000.0, 5000.0, 5000.0};

    config.terrain.resolution_m = 2.0;
    config.terrain.slope_limit_deg = 20.0;
    config.terrain.roughness_limit = 0.20;
    config.terrain.query_batch_size = 64;

    config.planner.method = GlobalPlannerMethod::BidirectionalAStar;
    config.planner.cost_weight = 10.0;

    config.tracking.method = TrackerMethod::PurePursuit;
    config.tracking.geometry.wheelbase_m = 2.76;
    config.tracking.geometry.front_axle_offset_m = 1.41;
    config.tracking.geometry.rear_axle_offset_m = 1.35;
    config.tracking.geometry.max_front_wheel_angle_rad = 23.0 * kDegToRad;
    config.tracking.lqr_longitudinal_kp = 0.8;


    config.enable_debug_output = true;
    config.visualization_sample_interval_sec = 1.0;
    config.control_log_interval = 10;
    return config;
}
