#pragma once

#include <filesystem>
#include <memory>

// 轻量地形查询器：只负责在已有路径运行时查询当前位置的坡度和粗糙度，
// 不生成代价地图，也不参与 A* 规划。
class TerrainSurfaceQuery
{
public:
    TerrainSurfaceQuery();
    ~TerrainSurfaceQuery();

    TerrainSurfaceQuery(const TerrainSurfaceQuery&) = delete;
    TerrainSurfaceQuery& operator=(const TerrainSurfaceQuery&) = delete;

    bool initialize(const std::filesystem::path& terrain_root);
    bool query(double x, double y, double& slope_deg, double& roughness) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
