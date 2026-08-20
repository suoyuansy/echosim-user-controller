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

// 代价地图构建参数集合：分辨率、可通行性阈值（坡度/粗糙度上限）与两者的融合权重。
// 默认值即本模块实测调出的整组参数，模块所有者直接在此调参；app 层 TaskConfig
// 聚合本结构，仅在对特定任务需要覆盖个别字段时才在 makeDefaultTaskConfig() 显式赋值。
// 这些参数会随缓存元数据落盘，修改默认值意味着旧缓存与新判定口径不一致，
// 必须通过递增缓存版本号（terrain_metadata.json 的 "version" 字段）使旧缓存失效并触发重建。
struct TerrainCostmapConfig
{
    // 1 m 分辨率：兼顾 Moon2 建图时长与障碍判定精度。全图约 10 km，配合扫描
    // 走廊（app 层 scan_bounds）限定建图范围，避免启动过慢；再粗会放大障碍
    // 膨胀误差、恶化贴障路径的安全性。
    double resolution_m = 1.0; // 地图栅格分辨率，单位为米。
    // 坡度阈值 20 度：相对测试硬门槛（最大俯仰 50 度/最大侧倾 45 度）留出较大
    // 安全余量——代价地图只放行明显缓于比赛限值的坡面，把极限坡度留给跟踪
    // 阶段的动态姿态控制兜底。
    double slope_limit_deg = 20.0; // 超过该坡度的栅格不可通行，单位为度。
    // 粗糙度阈值 0.20：剔除高频起伏过大的区域（换算单位见 build() 内计算）。
    double roughness_limit = 0.20; // 超过该粗糙度的栅格不可通行。
    double slope_weight = 0.5; // 坡度代价融合权重。
    double roughness_weight = 0.5; // 粗糙度代价融合权重。
    // 每次批量查询 64x64=4096 个采样点（模块 1 约定：建图用批量查询，禁逐点
    // 查询），在 TerrainService 服务端吞吐与单次请求内存占用之间折中。
    int query_batch_size = 64; // 单次 TerrainService 查询的栅格边长。
};

// 地形栅格地图：以 origin（栅格 (0,0) 中心）为基准、按 resolution_m 步长沿世界
// +X/+Y 方向展开的二维栅格。行号 row 沿世界 +Y 增长，列号 col 沿世界 +X 增长，
// 与仿真世界坐标系 (X, Y) 同向，不做任何轴翻转。所有数据数组均为行优先一维布局，
// 下标统一通过 index(row, col) 计算。
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
    // 行优先布局：index = row * cols + col；height/slope_deg/roughness/cost/valid 共用。
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
    // 两个否决条件：valid==0 表示该栅格没有有效地形查询结果（数据空洞）；
    // cost>=1 表示坡度或粗糙度超过阈值（融合代价被固定为障碍值 1）。
    // 越界栅格一律视为不可通行，保证规划器不会越出地图边界。
    bool isTraversable(int row, int col) const
    {
        if (!inBounds(row, col) || valid.empty() || cost.empty())
            return false;
        const std::size_t cell_index = index(row, col);
        return valid[cell_index] != 0 && cost[cell_index] < 1.0F;
    }

    // 将世界坐标转换为最近栅格。
    // 输入为仿真世界坐标（米），按 (x-origin)/resolution 四舍五入（llround）
    // 取最近栅格中心；返回 false 表示该点落在地图范围之外。
    bool worldToGrid(double x, double y, int& col, int& row) const
    {
        if (empty() || resolution_m <= 0.0)
            return false;
        col = static_cast<int>(std::llround((x - origin_x) / resolution_m));
        row = static_cast<int>(std::llround((y - origin_y) / resolution_m));
        return inBounds(row, col);
    }

    // 将栅格中心转换为世界坐标。
    // 输出单位为米；与 worldToGrid 互逆（gridToWorld 无舍入，是精确映射）。
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
    // 输入：坡度单位为度、粗糙度无量纲；输出为归一化融合代价，范围 0 到 1。
    static float fuseCost(double slope_deg,
                          double roughness,
                          const TerrainCostmapConfig& config);

#ifdef ECHOSIM_USE_ECHOSIM_SDK
    // 分块调用 TerrainService，构建并裁剪有效地形代价地图。
    // 输入 requested_bounds 为世界坐标查询边界（米），config 为构建参数；
    // 成功时结果存入内部 grid_。返回 false 表示参数非法或查询区域内无
    // 任何有效地形数据。
    bool build(terrain::TerrainQueryService& service,
               const TerrainBounds& requested_bounds,
               const TerrainCostmapConfig& config);
#endif

    // 判断缓存文件是否完整存在。
    // 只检查元数据与五个数值矩阵文件是否齐全，不校验文件内容；
    // 版本与参数合法性校验推迟到 load() 中进行。
    static bool hasCache(const std::string& directory);

    // 加载代价地图缓存；规划阶段可跳过高程、坡度和粗糙度矩阵。
    // 输入 directory 为缓存目录；loadTerrainDetails=false 时只读取代价与
    // 有效掩码，供仅需要 costmap 的规划阶段节省内存与加载时间。
    bool load(const std::string& directory, bool load_terrain_details = true);

    // 保存缓存文件及可选的代价地图预览图。
    // 输入 directory 为缓存目录（不存在则创建）；save_preview_image=true 时
    // 额外输出灰度 PNG 预览，但预览失败不影响缓存保存结果。
    bool save(const std::string& directory, bool save_preview_image = true) const;

    // 只重新生成代价地图灰度预览图。
    bool savePreviewImage(const std::string& directory) const;

    // 判断世界坐标是否落在当前地图范围内。
    // 判定基于栅格中心并放宽半格（0.5*resolution_m）余量：因为 worldToGrid
    // 按最近栅格取整，位于边缘栅格中心外侧半格之内的点仍会映射回地图内。
    bool covers(double x, double y) const;

    // 判断当前地图是否覆盖指定世界坐标边界。
    // 地图与输入边界均为轴对齐矩形，因此只要两个对角点被 covers() 覆盖，
    // 整个矩形必然被覆盖。
    bool coversBounds(const TerrainBounds& bounds) const;

    const TerrainGrid& grid() const { return grid_; }
    TerrainGrid& grid() { return grid_; }

private:
    // 将浮点矩阵按行写入文本缓存文件（文件内辅助函数，name_ 后缀）。
    static bool write_matrix_(const std::string& path,
                            const std::vector<float>& values,
                            int rows,
                            int cols);
    // 将 0/1 有效掩码按行写入文本缓存文件。
    static bool write_mask_(const std::string& path,
                          const std::vector<std::uint8_t>& values,
                          int rows,
                          int cols);
    // 从文本缓存文件读取指定行数列数的浮点矩阵，尺寸或内容不符返回 false。
    static bool read_matrix_(const std::string& path,
                           std::vector<float>& values,
                           int rows,
                           int cols);
    // 从文本缓存文件读取有效掩码，解析后二值化为 0/1。
    static bool read_mask_(const std::string& path,
                         std::vector<std::uint8_t>& values,
                         int rows,
                         int cols);
    // 用轻量正则从缓存元数据文本中提取指定名称的浮点字段。
    static bool read_json_number_(const std::string& text,
                               const std::string& key,
                               double& value);
    // 从缓存元数据文本中提取指定名称的整数型字段（复用浮点解析后取整）。
    static bool read_json_integer_(const std::string& text,
                                const std::string& key,
                                int& value);
    // 按障碍白色、代价灰度规则生成代价地图 PNG 预览图（依赖 OpenCV）。
    static bool save_preview_png_(const std::string& path,
                               const TerrainGrid& grid);

    TerrainGrid grid_;
};
