#include "tracking/TerrainSurfaceQuery.h"

#include "terrainquery.h"

#include <algorithm>
#include <cmath>
#include <memory>

struct TerrainSurfaceQuery::Impl
{
    std::unique_ptr<terrain::TerrainQueryService> service;
    bool initialized = false;
};

TerrainSurfaceQuery::TerrainSurfaceQuery()
    : impl_(std::make_unique<Impl>())
{
}

TerrainSurfaceQuery::~TerrainSurfaceQuery()
{
    if (impl_ && impl_->service)
        impl_->service->Clear();
}

bool TerrainSurfaceQuery::initialize(const std::filesystem::path& terrain_root)
{
    if (!impl_ || terrain_root.empty()
        || !std::filesystem::is_directory(terrain_root))
        return false;

    impl_->service = std::make_unique<terrain::TerrainQueryService>();
    terrain::TerrainQueryConfig config;
    config.tiledMapRootDir = terrain_root.string();
    config.preferredQueryResolution = 1.0;
    config.preferredWheelResolution = 1.0;
    config.enableRoadFirst = false;
    config.enablePrefetch = false;
    config.enableRoughness = true;
    if (!impl_->service->Initialize(config, nullptr, {}))
    {
        impl_->service.reset();
        impl_->initialized = false;
        return false;
    }
    impl_->initialized = true;
    return true;
}

bool TerrainSurfaceQuery::query(double x, double y, double& slope_deg,
                                double& roughness) const
{
    if (!impl_ || !impl_->initialized || !impl_->service)
        return false;

    terrain::HeightQueryResult result;
    if (!impl_->service->QueryHeightNormal(x, y, result)
        || !result.valid
        || !std::isfinite(result.slopeLongitudinal)
        || !std::isfinite(result.slopeLateral)
        || !std::isfinite(result.roughness))
        return false;

    constexpr double kPi = 3.14159265358979323846;
    slope_deg = std::atan(std::hypot(result.slopeLongitudinal,
                                     result.slopeLateral))
        * 180.0 / kPi;
    roughness = std::max(0.0, result.roughness);
    return std::isfinite(slope_deg) && std::isfinite(roughness);
}
