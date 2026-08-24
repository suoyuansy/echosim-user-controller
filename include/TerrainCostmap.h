#pragma once

#include "terrainquery.h"

#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

// 文件功能：定义地形查询边界、代价地图配置、栅格数据以及缓存接口。
struct TerrainBounds
{
    double min_x = 0.0; // 查询区域最小 X 坐标，单位为米。
    double min_y = 0.0; // 查询区域最小 Y 坐标，单位为米。
    double max_x = 0.0; // 查询区域最大 X 坐标，单位为米。
    double max_y = 0.0; // 查询区域最大 Y 坐标，单位为米。
};

struct TerrainCostmapConfig
{
    double resolution_m = 2.0; // 地图栅格分辨率，单位为米。
    double slope_limit_deg = 20.0; // 超过该坡度的栅格不可通行，单位为度。
    double roughness_limit = 0.20; // 超过该粗糙度的栅格不可通行。
    double slope_weight = 0.5; // 坡度代价融合权重。
    double roughness_weight = 0.5; // 粗糙度代价融合权重。
    int query_batch_size = 64; // 单次 TerrainService 查询的栅格边长。
};

struct TerrainGrid
{
    double origin_x = 0.0; // 地图原点 X 坐标，单位为米。
    double origin_y = 0.0; // 地图原点 Y 坐标，单位为米。
    double resolution_m = 2.0; // 当前地图分辨率，单位为米。
    double slope_limit_deg = 20.0; // 当前地图使用的坡度阈值。
    double roughness_limit = 0.20; // 当前地图使用的粗糙度阈值。
    double requested_width_m = 0.0; // 原始请求区域宽度，单位为米。
    double requested_height_m = 0.0; // 原始请求区域高度，单位为米。
    int rows = 0; // 栅格行数。
    int cols = 0; // 栅格列数。
    std::vector<float> height; // 栅格高程数据。
    std::vector<float> slope_deg; // 栅格坡度数据，单位为度。
    std::vector<float> roughness; // 栅格粗糙度数据。
    std::vector<float> cost; // 归一化融合代价，范围为 0 到 1。
    std::vector<std::uint8_t> valid; // 栅格是否有有效地形查询结果。

    // 判断地图是否为空。
    bool empty() const { return rows <= 0 || cols <= 0; }

    // 返回栅格行列对应的一维数组下标。
    std::size_t index(int row, int col) const
    {
        return static_cast<std::size_t>(row) * static_cast<std::size_t>(cols)
            + static_cast<std::size_t>(col);
    }

    // 判断栅格行列是否位于地图内部。
    bool inBounds(int row, int col) const
    {
        return row >= 0 && col >= 0 && row < rows && col < cols;
    }

    // 判断栅格有效且没有被地形阈值标记为障碍。
    bool isTraversable(int row, int col) const
    {
        if (!inBounds(row, col) || valid.empty() || cost.empty())
            return false;
        const std::size_t cell_index = index(row, col);
        return valid[cell_index] != 0 && cost[cell_index] < 1.0F;
    }

    // 将世界坐标转换为最近栅格。
    bool worldToGrid(double x, double y, int& col, int& row) const
    {
        if (empty() || resolution_m <= 0.0)
            return false;
        col = static_cast<int>(std::llround((x - origin_x) / resolution_m));
        row = static_cast<int>(std::llround((y - origin_y) / resolution_m));
        return inBounds(row, col);
    }

    // 将栅格中心转换为世界坐标。
    void gridToWorld(int row, int col, double& x, double& y) const
    {
        x = origin_x + static_cast<double>(col) * resolution_m;
        y = origin_y + static_cast<double>(row) * resolution_m;
    }
};

class TerrainCostmap
{
public:
    // 融合坡度和粗糙度代价，超过阈值的栅格固定为障碍代价 1。
    static float fuseCost(double slope_deg,
                          double roughness,
                          const TerrainCostmapConfig& config);

#ifdef ECHOSIM_USE_ECHOSIM_SDK
    // 分块调用 TerrainService，构建并裁剪有效地形代价地图。
    bool build(terrain::TerrainQueryService& service,
               const TerrainBounds& requested_bounds,
               const TerrainCostmapConfig& config);
#endif

    // 判断缓存文件是否完整存在。
    static bool hasCache(const std::string& directory);

    // 加载代价地图缓存；规划阶段可跳过高程、坡度和粗糙度矩阵。
    bool load(const std::string& directory, bool load_terrain_details = true);

    // 保存缓存文件及可选的代价地图预览图。
    bool save(const std::string& directory, bool save_preview_image = true) const;

    // 只重新生成代价地图灰度预览图。
    bool savePreviewImage(const std::string& directory) const;

    // 判断世界坐标是否落在当前地图范围内。
    bool covers(double x, double y) const;

    // 判断当前地图是否覆盖指定世界坐标边界。
    bool coversBounds(const TerrainBounds& bounds) const;

    const TerrainGrid& grid() const { return grid_; }
    TerrainGrid& grid() { return grid_; }

private:
    static bool write_matrix_(const std::string& path,
                            const std::vector<float>& values,
                            int rows,
                            int cols);
    static bool write_mask_(const std::string& path,
                          const std::vector<std::uint8_t>& values,
                          int rows,
                          int cols);
    static bool read_matrix_(const std::string& path,
                           std::vector<float>& values,
                           int rows,
                           int cols);
    static bool read_mask_(const std::string& path,
                         std::vector<std::uint8_t>& values,
                         int rows,
                         int cols);
    static bool read_json_number_(const std::string& text,
                               const std::string& key,
                               double& value);
    static bool read_json_integer_(const std::string& text,
                                const std::string& key,
                                int& value);
    static bool save_preview_png_(const std::string& path,
                               const TerrainGrid& grid);

    TerrainGrid grid_;
};
