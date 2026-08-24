// 文件功能：实现地形服务分块查询、坡度/粗糙度代价融合、缓存读写和代价地图预览输出。
#include "TerrainCostmap.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <utility>

#ifdef ECHOSIM_USE_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr float kObstacleCost = 1.0F;

// 功能：将浮点数限制到归一化代价范围 0 到 1。
double clamp01_(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

// 功能：拼接指定输出目录下的缓存文件路径。
std::filesystem::path cache_path_(const std::string& directory,
                                const char* fileName)
{
    return std::filesystem::path(directory) / fileName;
}

// 功能：整理查询区域边界，保证最小值不大于最大值。
TerrainBounds normalize_bounds_(TerrainBounds bounds)
{
    if (bounds.min_x > bounds.max_x)
        std::swap(bounds.min_x, bounds.max_x);
    if (bounds.min_y > bounds.max_y)
        std::swap(bounds.min_y, bounds.max_y);
    return bounds;
}

#ifdef ECHOSIM_USE_ECHOSIM_SDK
// 功能：将请求查询区域限制在配置允许的最大宽高内。
TerrainBounds limit_bounds_(TerrainBounds bounds)
{
    bounds = normalize_bounds_(bounds);

    return bounds;
}
#endif

// 功能：判断地形服务返回值是否为有限数。
bool is_finite_(double value)
{
    return std::isfinite(value);
}

// 功能：一次性读取文本缓存，减少逐个格式化输入造成的 I/O 开销。
bool read_text_buffer_(const std::string& path, std::string& text)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
        return false;
    const std::streampos endPosition = input.tellg();
    if (endPosition <= 0)
        return false;
    text.resize(static_cast<std::size_t>(endPosition));
    input.seekg(0, std::ios::beg);
    input.read(text.data(), static_cast<std::streamsize>(text.size()));
    return input.good() || input.eof();
}

// 功能：使用 from_chars 快速解析文本中的浮点或整数序列。
template <typename ValueType>
bool parse_text_values_(const std::string& text, std::vector<ValueType>& values)
{
    const char* cursor = text.data();
    const char* end = cursor + text.size();
    for (ValueType& value : values)
    {
        while (cursor < end
               && std::isspace(static_cast<unsigned char>(*cursor)) != 0)
        {
            ++cursor;
        }
        if (cursor >= end)
            return false;

        const std::from_chars_result result = std::from_chars(cursor, end, value);
        if (result.ec != std::errc() || result.ptr == cursor)
            return false;
        cursor = result.ptr;
    }
    return true;
}
} // namespace

// 功能：融合坡度和粗糙度代价，并将超阈值栅格标记为障碍。
float TerrainCostmap::fuseCost(double slope_deg,
                               double roughness,
                               const TerrainCostmapConfig& config)
{
    if (!is_finite_(slope_deg) || !is_finite_(roughness))
        return kObstacleCost;

    if (slope_deg > config.slope_limit_deg || roughness > config.roughness_limit)
        return kObstacleCost;

    const double slopeDenominator = std::max(config.slope_limit_deg, 1e-9);
    const double roughnessDenominator = std::max(config.roughness_limit, 1e-9);
    const double slopeCost = clamp01_(slope_deg / slopeDenominator);
    const double roughnessCost = clamp01_(roughness / roughnessDenominator);
    const double cost = config.slope_weight * slopeCost
        + config.roughness_weight * roughnessCost;
    return static_cast<float>(clamp01_(cost));
}

#ifdef ECHOSIM_USE_ECHOSIM_SDK
// 功能：按栅格分块调用 TerrainService，生成有效区域裁剪后的代价地图。
bool TerrainCostmap::build(terrain::TerrainQueryService& service,
                           const TerrainBounds& requestedBounds,
                           const TerrainCostmapConfig& config)
{
    if (config.resolution_m <= 0.0 || config.query_batch_size <= 0)
        return false;

    const TerrainBounds bounds = limit_bounds_(requestedBounds);
    const double width = std::max(0.0, bounds.max_x - bounds.min_x);
    const double height = std::max(0.0, bounds.max_y - bounds.min_y);
    const int cols = std::max(1, static_cast<int>(std::floor(width / config.resolution_m + 1e-9)) + 1);
    const int rows = std::max(1, static_cast<int>(std::floor(height / config.resolution_m + 1e-9)) + 1);

    TerrainGrid candidate;
    candidate.origin_x = bounds.min_x;
    candidate.origin_y = bounds.min_y;
    candidate.resolution_m = config.resolution_m;
    candidate.slope_limit_deg = config.slope_limit_deg;
    candidate.roughness_limit = config.roughness_limit;
    candidate.requested_width_m = width;
    candidate.requested_height_m = height;
    candidate.rows = rows;
    candidate.cols = cols;
    const std::size_t cellCount = static_cast<std::size_t>(rows)
        * static_cast<std::size_t>(cols);
    candidate.height.assign(cellCount, 0.0f);
    candidate.slope_deg.assign(cellCount, 0.0f);
    candidate.roughness.assign(cellCount, 0.0f);
    candidate.cost.assign(cellCount, kObstacleCost);
    candidate.valid.assign(cellCount, 0);

    int validCount = 0;
    for (int row0 = 0; row0 < rows; row0 += config.query_batch_size)
    {
        const int rowEnd = std::min(rows, row0 + config.query_batch_size);
        for (int col0 = 0; col0 < cols; col0 += config.query_batch_size)
        {
            const int colEnd = std::min(cols, col0 + config.query_batch_size);
            std::vector<terrain::HeightSamplePoint> points;
            std::vector<std::pair<int, int>> cells;
            points.reserve(static_cast<std::size_t>(rowEnd - row0)
                           * static_cast<std::size_t>(colEnd - col0));
            cells.reserve(points.capacity());

            for (int row = row0; row < rowEnd; ++row)
            {
                for (int col = col0; col < colEnd; ++col)
                {
                    points.push_back({
                        candidate.origin_x + static_cast<double>(col) * candidate.resolution_m,
                        candidate.origin_y + static_cast<double>(row) * candidate.resolution_m});
                    cells.emplace_back(row, col);
                }
            }

            std::vector<terrain::HeightQueryResult> results;
            service.QueryBatch(points, results);
            const std::size_t resultCount = std::min(points.size(), results.size());
            for (std::size_t i = 0; i < resultCount; ++i)
            {
                const auto [row, col] = cells[i];
                const terrain::HeightQueryResult& result = results[i];
                const std::size_t index = candidate.index(row, col);
                if (!result.valid || !is_finite_(result.z)
                    || !is_finite_(result.slopeLongitudinal)
                    || !is_finite_(result.slopeLateral)
                    || !is_finite_(result.roughness))
                {
                    continue;
                }

                const double slopeRatio = std::hypot(result.slopeLongitudinal,
                                                     result.slopeLateral);
                const double slope_deg = std::atan(slopeRatio) * 180.0 / kPi;
                const double roughness = std::max(0.0, result.roughness);
                candidate.height[index] = static_cast<float>(result.z);
                candidate.slope_deg[index] = static_cast<float>(slope_deg);
                candidate.roughness[index] = static_cast<float>(roughness);
                candidate.cost[index] = TerrainCostmap::fuseCost(slope_deg, roughness, config);
                candidate.valid[index] = 1;
                ++validCount;
            }
        }
    }

    if (validCount == 0)
        return false;

    int minRow = rows;
    int minCol = cols;
    int maxRow = -1;
    int maxCol = -1;
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            if (candidate.valid[candidate.index(row, col)] == 0)
                continue;
            minRow = std::min(minRow, row);
            minCol = std::min(minCol, col);
            maxRow = std::max(maxRow, row);
            maxCol = std::max(maxCol, col);
        }
    }

    if (maxRow < minRow || maxCol < minCol)
        return false;

    // 查询范围全部有效时直接转移候选地图，避免在 1m/5000m 地图上复制整张数组。
    if (minRow == 0 && minCol == 0 && maxRow == rows - 1 && maxCol == cols - 1)
    {
        grid_ = std::move(candidate);
        return true;
    }

    TerrainGrid cropped;
    cropped.origin_x = candidate.origin_x + static_cast<double>(minCol) * candidate.resolution_m;
    cropped.origin_y = candidate.origin_y + static_cast<double>(minRow) * candidate.resolution_m;
    cropped.resolution_m = candidate.resolution_m;
    cropped.slope_limit_deg = candidate.slope_limit_deg;
    cropped.roughness_limit = candidate.roughness_limit;
    cropped.requested_width_m = candidate.requested_width_m;
    cropped.requested_height_m = candidate.requested_height_m;
    cropped.rows = maxRow - minRow + 1;
    cropped.cols = maxCol - minCol + 1;
    const std::size_t croppedCount = static_cast<std::size_t>(cropped.rows)
        * static_cast<std::size_t>(cropped.cols);
    cropped.height.assign(croppedCount, 0.0f);
    cropped.slope_deg.assign(croppedCount, 0.0f);
    cropped.roughness.assign(croppedCount, 0.0f);
    cropped.cost.assign(croppedCount, kObstacleCost);
    cropped.valid.assign(croppedCount, 0);

    for (int row = 0; row < cropped.rows; ++row)
    {
        for (int col = 0; col < cropped.cols; ++col)
        {
            const std::size_t sourceIndex = candidate.index(minRow + row, minCol + col);
            const std::size_t targetIndex = cropped.index(row, col);
            cropped.height[targetIndex] = candidate.height[sourceIndex];
            cropped.slope_deg[targetIndex] = candidate.slope_deg[sourceIndex];
            cropped.roughness[targetIndex] = candidate.roughness[sourceIndex];
            cropped.cost[targetIndex] = candidate.cost[sourceIndex];
            cropped.valid[targetIndex] = candidate.valid[sourceIndex];
        }
    }

    grid_ = std::move(cropped);
    return true;
}
#endif

// 功能：将浮点矩阵按行写入文本缓存文件。
bool TerrainCostmap::write_matrix_(const std::string& path,
                                 const std::vector<float>& values,
                                 int rows,
                                 int cols)
{
    if (rows <= 0 || cols <= 0 || values.size() != static_cast<std::size_t>(rows) * cols)
        return false;
    std::ofstream output(path);
    if (!output)
        return false;
    output << std::setprecision(9) << std::scientific;
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            if (col != 0)
                output << ' ';
            output << values[static_cast<std::size_t>(row) * cols + col];
        }
        output << '\n';
    }
    return output.good();
}

// 功能：将有效掩码按行写入文本缓存文件。
bool TerrainCostmap::write_mask_(const std::string& path,
                                const std::vector<std::uint8_t>& values,
                                int rows,
                                int cols)
{
    if (rows <= 0 || cols <= 0 || values.size() != static_cast<std::size_t>(rows) * cols)
        return false;
    std::ofstream output(path);
    if (!output)
        return false;
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            if (col != 0)
                output << ' ';
            output << static_cast<int>(values[static_cast<std::size_t>(row) * cols + col]);
        }
        output << '\n';
    }
    return output.good();
}

// 功能：从文本缓存文件读取指定大小的浮点矩阵。
bool TerrainCostmap::read_matrix_(const std::string& path,
                                std::vector<float>& values,
                                int rows,
                                int cols)
{
    if (rows <= 0 || cols <= 0)
        return false;
    std::string text;
    if (!read_text_buffer_(path, text))
        return false;
    values.assign(static_cast<std::size_t>(rows) * cols, 0.0f);
    return parse_text_values_(text, values);
}

// 功能：从文本缓存文件读取指定大小的有效掩码。
bool TerrainCostmap::read_mask_(const std::string& path,
                              std::vector<std::uint8_t>& values,
                              int rows,
                              int cols)
{
    if (rows <= 0 || cols <= 0)
        return false;
    std::string text;
    if (!read_text_buffer_(path, text))
        return false;
    std::vector<int> parsedValues(static_cast<std::size_t>(rows) * cols, 0);
    if (!parse_text_values_(text, parsedValues))
        return false;
    values.assign(parsedValues.size(), 0);
    for (std::size_t index = 0; index < parsedValues.size(); ++index)
    {
        values[index] = static_cast<std::uint8_t>(parsedValues[index] == 0 ? 0 : 1);
    }
    return true;
}

// 功能：从缓存元数据文本中读取指定名称的浮点字段。
bool TerrainCostmap::read_json_number_(const std::string& text,
                                    const std::string& key,
                                    double& value)
{
    const std::regex pattern("\\\"" + key
        + "\\\"\\s*:\\s*([-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?)");
    std::smatch match;
    if (!std::regex_search(text, match, pattern))
        return false;
    try
    {
        value = std::stod(match[1].str());
    }
    catch (...)
    {
        return false;
    }
    return true;
}

// 功能：从缓存元数据文本中读取指定名称的整数型字段。
bool TerrainCostmap::read_json_integer_(const std::string& text,
                                     const std::string& key,
                                     int& value)
{
    double parsed = 0.0;
    if (!read_json_number_(text, key, parsed))
        return false;
    value = static_cast<int>(parsed);
    return true;
}

// 功能：按照障碍白色、代价灰度规则生成代价地图预览图。
bool TerrainCostmap::save_preview_png_(const std::string& path,
                                    const TerrainGrid& grid)
{
#ifdef ECHOSIM_USE_OPENCV
    if (grid.empty())
        return false;
    // 参考项目的底图规则：障碍固定为白色 255，非障碍代价映射到 0..254。
    cv::Mat image(grid.rows, grid.cols, CV_8UC1);
    for (int row = 0; row < grid.rows; ++row)
    {
        const int imageRow = grid.rows - 1 - row;
        for (int col = 0; col < grid.cols; ++col)
        {
            const std::size_t index = grid.index(row, col);
            const bool obstacle = grid.valid[index] == 0 || grid.cost[index] >= 1.0f;
            image.at<unsigned char>(imageRow, col) = obstacle
                ? static_cast<unsigned char>(255)
                : static_cast<unsigned char>(std::clamp(
                    static_cast<int>(std::lround(255.0 * grid.cost[index])), 0, 255));
        }
    }
    return cv::imwrite(path, image);
#else
    (void)path;
    (void)grid;
    return false;
#endif
}

// 功能：保存高程、坡度、粗糙度、有效掩码、代价和元数据缓存。
bool TerrainCostmap::save(const std::string& directory,
                          bool savePreviewImage) const
{
    if (grid_.empty())
        return false;
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
        return false;

    const bool heightSaved = write_matrix_(cache_path_(directory, "terrain_height.txt").string(),
                                         grid_.height, grid_.rows, grid_.cols);
    const bool slopeSaved = write_matrix_(cache_path_(directory, "terrain_slope.txt").string(),
                                        grid_.slope_deg, grid_.rows, grid_.cols);
    const bool roughnessSaved = write_matrix_(cache_path_(directory, "terrain_roughness.txt").string(),
                                            grid_.roughness, grid_.rows, grid_.cols);
    const bool validSaved = write_mask_(cache_path_(directory, "terrain_valid.txt").string(),
                                      grid_.valid, grid_.rows, grid_.cols);
    const bool costSaved = write_matrix_(cache_path_(directory, "costmap.txt").string(),
                                       grid_.cost, grid_.rows, grid_.cols);
    if (!heightSaved || !slopeSaved || !roughnessSaved || !validSaved || !costSaved)
        return false;

    std::ofstream metadata(cache_path_(directory, "terrain_metadata.json"));
    if (!metadata)
        return false;
    const double max_x = grid_.origin_x + (grid_.cols - 1) * grid_.resolution_m;
    const double max_y = grid_.origin_y + (grid_.rows - 1) * grid_.resolution_m;
    metadata << std::setprecision(17)
        << "{\n"
        << "  \"version\": 3,\n"
        << "  \"origin_x\": " << grid_.origin_x << ",\n"
        << "  \"origin_y\": " << grid_.origin_y << ",\n"
        << "  \"resolution_m\": " << grid_.resolution_m << ",\n"
        << "  \"slope_limit_deg\": " << grid_.slope_limit_deg << ",\n"
        << "  \"roughness_limit\": " << grid_.roughness_limit << ",\n"
        << "  \"requested_width_m\": " << grid_.requested_width_m << ",\n"
        << "  \"requested_height_m\": " << grid_.requested_height_m << ",\n"
        << "  \"rows\": " << grid_.rows << ",\n"
        << "  \"cols\": " << grid_.cols << ",\n"
        << "  \"max_x\": " << max_x << ",\n"
        << "  \"max_y\": " << max_y << ",\n"
        << "  \"coordinate_system\": \"Z-Up X-Y\",\n"
        << "  \"invalid_value\": 1.0,\n"
        << "  \"files\": {\n"
        << "    \"height\": \"terrain_height.txt\",\n"
        << "    \"slope\": \"terrain_slope.txt\",\n"
        << "    \"roughness\": \"terrain_roughness.txt\",\n"
        << "    \"valid\": \"terrain_valid.txt\",\n"
        << "    \"cost\": \"costmap.txt\"\n"
        << "  }\n"
        << "}\n";
    if (!metadata.good())
        return false;

    // PNG 仅作为可选预览，不影响数值缓存的可加载性。
    if (savePreviewImage)
        save_preview_png_(cache_path_(directory, "terrain_preview.png").string(), grid_);
    return true;
}

// 功能：检查完整缓存所需文件是否全部存在。
bool TerrainCostmap::hasCache(const std::string& directory)
{
    // 只有元数据和五个矩阵文件全部存在，才把目录视为可读取的完整缓存。
    constexpr const char* kCacheFiles[] = {
        "terrain_metadata.json",
        "terrain_height.txt",
        "terrain_slope.txt",
        "terrain_roughness.txt",
        "terrain_valid.txt",
        "costmap.txt"
    };
    for (const char* fileName : kCacheFiles)
    {
        std::error_code error;
        const std::filesystem::path path = cache_path_(directory, fileName);
        if (!std::filesystem::is_regular_file(path, error) || error)
            return false;
    }
    return true;
}

// 功能：在已有数值缓存的基础上重新输出灰度代价地图预览图。
bool TerrainCostmap::savePreviewImage(const std::string& directory) const
{
    if (grid_.empty())
        return false;
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
        return false;
    return save_preview_png_(cache_path_(directory, "terrain_preview.png").string(), grid_);
}

// 功能：读取并恢复地形代价地图缓存，不在加载阶段访问地形服务。
bool TerrainCostmap::load(const std::string& directory,
                          bool loadTerrainDetails)
{
    std::ifstream metadataFile(cache_path_(directory, "terrain_metadata.json"));
    if (!metadataFile)
        return false;
    const std::string metadata((std::istreambuf_iterator<char>(metadataFile)),
                               std::istreambuf_iterator<char>());

    double origin_x = 0.0;
    double origin_y = 0.0;
    double resolution = 0.0;
    double slope_limit_deg = 20.0;
    double roughness_limit = 0.20;
    double requested_width_m = 0.0;
    double requested_height_m = 0.0;
    int version = 0;
    int rows = 0;
    int cols = 0;
    if (!read_json_integer_(metadata, "version", version)
        || version < 3
        || !read_json_number_(metadata, "origin_x", origin_x)
        || !read_json_number_(metadata, "origin_y", origin_y)
        || !read_json_number_(metadata, "resolution_m", resolution)
        || !read_json_number_(metadata, "slope_limit_deg", slope_limit_deg)
        || !read_json_number_(metadata, "roughness_limit", roughness_limit)
        || !read_json_number_(metadata, "requested_width_m", requested_width_m)
        || !read_json_number_(metadata, "requested_height_m", requested_height_m)
        || !read_json_integer_(metadata, "rows", rows)
        || !read_json_integer_(metadata, "cols", cols)
        || resolution <= 0.0 || rows <= 0 || cols <= 0)
    {
        return false;
    }

    TerrainGrid loaded;
    loaded.origin_x = origin_x;
    loaded.origin_y = origin_y;
    loaded.resolution_m = resolution;
    loaded.slope_limit_deg = slope_limit_deg;
    loaded.roughness_limit = roughness_limit;
    loaded.requested_width_m = requested_width_m;
    loaded.requested_height_m = requested_height_m;
    loaded.rows = rows;
    loaded.cols = cols;
    if (loadTerrainDetails
        && (!read_matrix_(cache_path_(directory, "terrain_height.txt").string(),
                        loaded.height, rows, cols)
            || !read_matrix_(cache_path_(directory, "terrain_slope.txt").string(),
                           loaded.slope_deg, rows, cols)
            || !read_matrix_(cache_path_(directory, "terrain_roughness.txt").string(),
                           loaded.roughness, rows, cols)))
    {
        return false;
    }
    if (!read_mask_(cache_path_(directory, "terrain_valid.txt").string(),
                  loaded.valid, rows, cols))
    {
        return false;
    }

    // costmap.txt 是完整缓存的一部分，缺失时直接报告加载失败，不在运行时重新推导或查询。
    if (!read_matrix_(cache_path_(directory, "costmap.txt").string(),
                    loaded.cost, rows, cols))
    {
        return false;
    }

    grid_ = std::move(loaded);
    return true;
}

// 功能：判断世界坐标点是否落在当前有效地图矩形内。
bool TerrainCostmap::covers(double x, double y) const
{
    if (grid_.empty())
        return false;
    const double max_x = grid_.origin_x + (grid_.cols - 1) * grid_.resolution_m;
    const double max_y = grid_.origin_y + (grid_.rows - 1) * grid_.resolution_m;
    const double margin = 0.5 * grid_.resolution_m;
    return x >= grid_.origin_x - margin && x <= max_x + margin
        && y >= grid_.origin_y - margin && y <= max_y + margin;
}

// 功能：判断当前地图是否覆盖给定世界坐标边界。
bool TerrainCostmap::coversBounds(const TerrainBounds& bounds) const
{
    const TerrainBounds normalized = normalize_bounds_(bounds);
    return covers(normalized.min_x, normalized.min_y)
        && covers(normalized.max_x, normalized.max_y);
}
