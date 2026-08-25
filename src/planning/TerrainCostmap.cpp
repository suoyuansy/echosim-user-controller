// 文件功能：实现地形服务分块查询、坡度/粗糙度代价融合、缓存读写和代价地图预览输出。
#include "planning/TerrainCostmap.h"

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
// 不可通行栅格的固定融合代价：归一化代价的上限值 1。
// isTraversable() 以 cost < 1 作为可通行判据，因此该值同时充当"障碍"标记。
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
// 调用方可能传入任意顺序的对角点（如 start/goal 直接作为边界），此处统一
// 归一化为 min/max 形式，后续代码才能假设 min_x<=max_x、min_y<=max_y。
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
// 当前实现只做 min/max 归一化，没有额外截断宽高；保留该入口是为了
// 将来引入最大查询范围限制时不需要改动调用方。
TerrainBounds limit_bounds_(TerrainBounds bounds)
{
    bounds = normalize_bounds_(bounds);

    return bounds;
}
#endif

// 功能：判断地形服务返回值是否为有限数。
// 地形服务对查询不到的点位可能返回 NaN/Inf，这类值必须当作无效数据处理，
// 否则会污染代价计算并在后续 A* 搜索中传播。
bool is_finite_(double value)
{
    return std::isfinite(value);
}

// 功能：一次性读取文本缓存，减少逐个格式化输入造成的 I/O 开销。
// 步骤：以 ate 模式打开定位到文件尾拿到大小，一次 resize 后整块读入。
bool read_text_buffer_(const std::string& path, std::string& text)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
        return false;
    const std::streampos endPosition = input.tellg();
    if (endPosition <= 0)
        return false;
    text.resize(static_cast<std::size_t>(endPosition));
    input.seekg(0, std::ios::beg); // 回到文件头准备整块读取。
    input.read(text.data(), static_cast<std::streamsize>(text.size()));
    return input.good() || input.eof();
}

// 功能：使用 from_chars 快速解析文本中的浮点或整数序列。
// 输入 text 为整块读入的缓存文本；values 需预先按 rows*cols 分配好大小，
// 逐个跳过空白后用 from_chars 解析（无 locale 开销，快于 iostream/scanf）。
// 解析数量不足或遇到非法字符即返回 false，视为缓存损坏。
template <typename ValueType>
bool parse_text_values_(const std::string& text, std::vector<ValueType>& values)
{
    const char* cursor = text.data();
    const char* end = cursor + text.size();
    for (ValueType& value : values)
    {
        // 跳过当前值之前的空白分隔符（空格/换行/制表符）。
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
        cursor = result.ptr; // 移动到已消费位置的下一个字符。
    }
    return true;
}
} // namespace

// 功能：融合坡度和粗糙度代价，并将超阈值栅格标记为障碍。
// 输入：slope_deg 坡度（度）、roughness 粗糙度（无量纲）；输出归一化代价 0~1。
float TerrainCostmap::fuseCost(double slope_deg,
                               double roughness,
                               const TerrainCostmapConfig& config)
{
    // 地形服务返回 NaN/Inf 的点位视为障碍，避免无效数据进入搜索图。
    if (!is_finite_(slope_deg) || !is_finite_(roughness))
        return kObstacleCost;

    // 坡度或粗糙度任一超过阈值即硬性不可通行（默认坡度上限 18 度远低于
    // 比赛的俯仰 50 度/侧倾 45 度上限，为跟踪误差留出安全余量）。
    if (slope_deg > config.slope_limit_deg || roughness > config.roughness_limit)
        return kObstacleCost;

    // 用阈值本身作为归一化分母：代价 0 表示平地/平整，趋近 1 表示贴近阈值。
    // max(...,1e-9) 防止用户配置阈值为 0 时除零。
    const double slopeDenominator = std::max(config.slope_limit_deg, 1e-9);
    const double roughnessDenominator = std::max(config.roughness_limit, 1e-9);
    const double slopeCost = clamp01_(slope_deg / slopeDenominator);
    const double roughnessCost = clamp01_(roughness / roughnessDenominator);
    // 加权线性融合；两个权重默认各 0.5， clamp 保证权重和超过 1 时结果仍在 0~1。
    const double cost = config.slope_weight * slopeCost
        + config.roughness_weight * roughnessCost;
    return static_cast<float>(clamp01_(cost));
}

#ifdef ECHOSIM_USE_ECHOSIM_SDK
// 功能：按栅格分块调用 TerrainService，生成有效区域裁剪后的代价地图。
// 输入 requestedBounds 为世界坐标（米）查询边界，直接使用仿真 (X, Y) 坐标；
// 输出存入成员 grid_。整体流程：参数校验 -> 分配候选栅格 -> 按 64x64 块批量
// 查询并计算坡度/粗糙度/融合代价 -> 统计有效区域包围盒 -> 裁剪掉无效边缘。
bool TerrainCostmap::build(terrain::TerrainQueryService& service,
                           const TerrainBounds& requestedBounds,
                           const TerrainCostmapConfig& config)
{
    // 分辨率或批大小非正会导致死循环或除零，直接拒绝。
    if (config.resolution_m <= 0.0 || config.query_batch_size <= 0)
        return false;

    const TerrainBounds bounds = limit_bounds_(requestedBounds);
    const double width = std::max(0.0, bounds.max_x - bounds.min_x);
    const double height = std::max(0.0, bounds.max_y - bounds.min_y);
    // 栅格数按 floor(宽度/分辨率)+1 计算：+1 是因为两端边界点各占一个栅格中心，
    // +1e-9 容差避免宽度恰为分辨率整数倍时因浮点误差少算一列/一行。
    const int cols = std::max(1, static_cast<int>(std::floor(width / config.resolution_m + 1e-9)) + 1);
    const int rows = std::max(1, static_cast<int>(std::floor(height / config.resolution_m + 1e-9)) + 1);

    // 候选地图先全部初始化为"障碍且无效"：未被地形服务覆盖的栅格保持
    // cost=1、valid=0，后续 isTraversable 自然会将其排除。
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
    // 外两层按 query_batch_size（默认 64）把整张地图切成方块，每块调用一次
    // QueryBatch 批量查询——这是全局规划阶段的推荐用法，避免逐点查询的
    // 巨大开销。最后一块不足 64 时用 min 截断。
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

            // 组装该块内所有栅格中心的采样点：世界坐标 = origin + 行列号*分辨率。
            // 坐标系约定：直接传仿真世界 (X, Y)，Y 不取反。
            // cells 与 points 一一对应，用于查询结果回填时反查栅格行列。
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
            // 服务端返回条数可能少于请求条数，只处理有效前缀。
            const std::size_t resultCount = std::min(points.size(), results.size());
            for (std::size_t i = 0; i < resultCount; ++i)
            {
                const auto [row, col] = cells[i];
                const terrain::HeightQueryResult& result = results[i];
                const std::size_t index = candidate.index(row, col);
                // 任一字段无效或非有限（NaN/Inf）则该栅格保持"障碍且无效"默认值。
                if (!result.valid || !is_finite_(result.z)
                    || !is_finite_(result.slopeLongitudinal)
                    || !is_finite_(result.slopeLateral)
                    || !is_finite_(result.roughness))
                {
                    continue;
                }

                // 坡度合成：纵向/横向坡度分量是坡比（高差/水平距离），
                // hypot 得到总坡比，atan 转为坡度角（弧度）再乘 180/pi 转成度。
                const double slopeRatio = std::hypot(result.slopeLongitudinal,
                                                     result.slopeLateral);
                const double slope_deg = std::atan(slopeRatio) * 180.0 / kPi;
                // 粗糙度截断为非负，防御服务端偶发的负值。
                const double roughness = std::max(0.0, result.roughness);
                candidate.height[index] = static_cast<float>(result.z);
                candidate.slope_deg[index] = static_cast<float>(slope_deg);
                candidate.roughness[index] = static_cast<float>(roughness);
                // 融合归一化代价：超阈值栅格在 fuseCost 内被置为障碍值 1。
                candidate.cost[index] = TerrainCostmap::fuseCost(slope_deg, roughness, config);
                candidate.valid[index] = 1;
                ++validCount;
            }
        }
    }

    // 整张查询没有任何有效地形数据（例如查询区域完全在月面外）时构建失败。
    if (validCount == 0)
        return false;

    // 统计有效栅格的包围盒：地形服务覆盖范围可能只是请求矩形的一个子区域，
    // 后续需要把地图裁剪到该子区域，让 origin 与有效数据对齐。
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

    // 部分有效：按有效包围盒裁剪出新地图。新原点平移到包围盒左下角栅格中心
    // （世界坐标 = 原点 + 偏移行列数*分辨率，米）；requested 尺寸等元数据原样
    // 保留，用于缓存诊断与 coversBounds 判定。
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
    // 裁剪地图同样初始化为"障碍且无效"，随后逐格拷贝包围盒内的数据。
    cropped.height.assign(croppedCount, 0.0f);
    cropped.slope_deg.assign(croppedCount, 0.0f);
    cropped.roughness.assign(croppedCount, 0.0f);
    cropped.cost.assign(croppedCount, kObstacleCost);
    cropped.valid.assign(croppedCount, 0);

    // 逐格拷贝五个数据矩阵：源下标 = 候选地图中 (minRow+row, minCol+col)。
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
// 输出格式：每行 cols 个值、空格分隔、scientific 记数法；9 位有效数字
// 足够 float（约 7 位十进制有效精度）无损往返，读取时不会损失精度。
bool TerrainCostmap::write_matrix_(const std::string& path,
                                 const std::vector<float>& values,
                                 int rows,
                                 int cols)
{
    // 尺寸一致性校验：写入失败不如带着错位数据继续，直接返回 false。
    if (rows <= 0 || cols <= 0 || values.size() != static_cast<std::size_t>(rows) * cols)
        return false;
    std::ofstream output(path);
    if (!output)
        return false;
    // 逐行输出：行内以空格分隔、行间以换行分隔，与 parse_text_values_
    // 的"跳过空白按序解析"策略严格对应。
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
// 掩码以 0/1 整数文本存储；读取后统一二值化（非 0 即 1），兼容缓存中
// 偶尔出现的其他非零值。
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
    // 先按 int 解析再压回 uint8_t，避免 from_chars 对无符号类型的格式差异。
    std::vector<int> parsedValues(static_cast<std::size_t>(rows) * cols, 0);
    if (!parse_text_values_(text, parsedValues))
        return false;
    values.assign(parsedValues.size(), 0);
    for (std::size_t index = 0; index < parsedValues.size(); ++index)
    {
        // 二值化：任何非零值统一归一为 1（有效）。
        values[index] = static_cast<std::uint8_t>(parsedValues[index] == 0 ? 0 : 1);
    }
    return true;
}

// 功能：从缓存元数据文本中读取指定名称的浮点字段。
// 元数据是本类 save() 自己生成的扁平 JSON，因此用一条正则按
// "key": number 模式提取即可，无需引入完整 JSON 解析器。
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
        // 图像行号向下增长而栅格行号沿世界 +Y 向上增长，写入时上下翻转，
        // 保证预览图与俯视世界坐标方向一致（上=+Y）。
        const int imageRow = grid.rows - 1 - row;
        for (int col = 0; col < grid.cols; ++col)
        {
            const std::size_t index = grid.index(row, col);
            // 无效栅格（数据空洞）与超阈值栅格（cost>=1）都渲染为白色障碍。
            const bool obstacle = grid.valid[index] == 0 || grid.cost[index] >= 1.0f;
            // 可通行栅格：代价 0~1 线性映射为灰度 0~255（0=黑色平坦低代价）。
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
// 缓存目录 layout：五个行优先文本矩阵 + 一个 terrain_metadata.json 元数据
// （记录原点/分辨率/阈值/行列数等构建参数，供 load() 校验）+ 可选 PNG 预览。
bool TerrainCostmap::save(const std::string& directory,
                          bool savePreviewImage) const
{
    if (grid_.empty())
        return false;
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
        return false;

    // 依次落盘五个数值矩阵；任何一个失败都视为缓存不完整，整体返回失败，
    // 避免 hasCache()/load() 读到半套文件。
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
    // 右上角边界由原点和行列数推导（米），供 covers() 之外的诊断使用。
    const double max_x = grid_.origin_x + (grid_.cols - 1) * grid_.resolution_m;
    const double max_y = grid_.origin_y + (grid_.rows - 1) * grid_.resolution_m;
    // setprecision(17)：double 的完整十进制精度，保证原点/分辨率往返无损，
    // 否则缓存重载后栅格对齐会出现亚毫米级漂移。
    metadata << std::setprecision(17)
        << "{\n"
        // 缓存版本号：当前为 4。文件格式或字段语义不兼容变更时必须递增，
        // load() 会拒绝低于该版本的旧缓存（缓存失效机制）。
        // v3 -> v4：坡度阈值默认值由 20 度下调为 18 度（对齐测试标称坡度）。
        << "  \"version\": 4,\n"
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
        // 坐标系声明：Z 轴向上的 X-Y 平面，与仿真世界坐标一致（Y 不取反）。
        << "  \"coordinate_system\": \"Z-Up X-Y\",\n"
        // invalid_value：无效/障碍栅格在数值矩阵中的占位值约定。
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
// 步骤：读元数据 -> 校验版本与参数合法性 -> 按需读高程/坡度/粗糙度 ->
// 必读有效掩码与代价矩阵。任一步失败均返回 false，由上层决定重建。
bool TerrainCostmap::load(const std::string& directory,
                          bool loadTerrainDetails)
{
    std::ifstream metadataFile(cache_path_(directory, "terrain_metadata.json"));
    if (!metadataFile)
        return false;
    const std::string metadata((std::istreambuf_iterator<char>(metadataFile)),
                               std::istreambuf_iterator<char>());

    // 局部变量先赋默认值（对应当前默认配置），元数据缺字段时保持默认；
    // 但 version/origin/resolution/rows/cols 属于必检项，缺失或非法即失败。
    double origin_x = 0.0;
    double origin_y = 0.0;
    double resolution = 0.0;
    double slope_limit_deg = 18.0;
    double roughness_limit = 0.20;
    double requested_width_m = 0.0;
    double requested_height_m = 0.0;
    int version = 0;
    int rows = 0;
    int cols = 0;
    // 版本低于 4 的旧缓存直接拒绝：字段布局已不兼容，强制走重建流程
    // （这是缓存失效的唯一显式机制，参数口径变化靠递增版本号生效）。
    if (!read_json_integer_(metadata, "version", version)
        || version < 4
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
    // 高程/坡度/粗糙度三个明细矩阵只在需要时读取：纯规划（A* 只用 cost 和
    // valid）可跳过，显著减少大地图下的内存与解析时间。
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
    // 有效掩码是可通行性判定的一部分，无论是否加载明细都必须读取。
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

    // 全部字段校验通过才提交给成员 grid_，失败路径不会污染现有地图。
    grid_ = std::move(loaded);
    return true;
}

// 功能：判断世界坐标点是否落在当前有效地图矩形内。
// 输入为仿真世界坐标（米）。边界取栅格中心连成的矩形，再向外放宽半格
// （0.5*分辨率）余量：与 worldToGrid 的四舍五入规则一致，边缘栅格中心
// 外侧半格内的点仍会映射回地图。
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
// 先归一化输入边界（调用方可能传任意顺序对角点），再检查两个对角点；
// 由于地图与边界都是轴对齐矩形，对角覆盖即整块覆盖。
bool TerrainCostmap::coversBounds(const TerrainBounds& bounds) const
{
    const TerrainBounds normalized = normalize_bounds_(bounds);
    return covers(normalized.min_x, normalized.min_y)
        && covers(normalized.max_x, normalized.max_y);
}
