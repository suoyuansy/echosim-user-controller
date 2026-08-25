# Onsite8B 月球车自主驾驶控制器

Onsite8B 月球车竞赛(EchoSim 仿真环境)的自主驾驶控制器,分层规划架构:

```text
TerrainCostmap(1 m 地形代价地图)
    -> GlobalPlanner(双向 A* 全局规划)
    -> PathOptimizer(视线捷径 / 迭代平滑 / 尖角圆弧化)
    -> PathTracker(Pure Pursuit / Stanley / LQR 轨迹跟踪)
    -> EchoSimRuntime(消息总线 + 0.1 s 控制循环)
```

三个模块(planning / optimization / tracking)由不同负责人分工开发,接口契约见
[模块接口说明.md](模块接口说明.md);`app/` 为集成层,负责配置与流程编排。

## 1. 环境要求

- Windows 10/11 x64
- Visual Studio 2022(含 "使用 C++ 的桌面开发" 负载,MSVC v143 工具集)
- CMake 3.16+ 与 Ninja(VS 安装器 "使用 C++ 的桌面开发" 负载默认包含)
- `third_party/` 依赖包(见下节,不入库,需单独获取)

## 2. 依赖准备(third_party/,仓库不含)

构建前需要把依赖包放到仓库根的 `third_party/` 目录,预期布局:

```text
third_party/
  echosim/           EchoSim SDK
    include/         libmsg(消息)、terrain(地形查询)头文件
    libs/            Release 库(libSimMsg.lib、TerrainServiceAPI.lib)
    libsdebug/       Debug 库
    bin/             运行时 DLL(esync.dll、libSimMsg.dll、TerrainServiceAPI.dll 等)
  opencv/            OpenCV 4.12
    include/         头文件
    lib/             opencv_world4120.lib
    bin/             opencv_world4120.dll
```

依赖包随竞赛工作区的 EchoSim 安装一起分发,从已配置的机器整体拷贝即可。
若依赖放在别处,配置时用 CMake 参数覆盖:

```powershell
-DECHOSIM_DEPENDENCY_ROOT=<echosim根目录> -DOPENCV_ROOT=<opencv根目录>
```

## 3. 构建

### 命令行(推荐)

在 "VS 2022 x64 Native Tools Command Prompt" 中,于仓库根执行:

```powershell
cmake -S . -B out\build\x64-Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build out\build\x64-Release
```

也可在普通 PowerShell 中执行仓库内的便捷脚本：

```powershell
.\build_release.bat
```

产物:`out\build\x64-Release\bin\UserController.exe`,构建时自动把
EchoSim/OpenCV 运行时 DLL 拷贝到 exe 旁边。

### Visual Studio "打开文件夹" 模式

直接用 VS 打开仓库根目录,`CMakeSettings.json` 已定义 `x64-Debug` /
`x64-Release` 两套配置,选择后 生成全部 即可。

### 单元测试(可选)

若存在 `tests/PathTrackerSelfTest.cpp`,`BUILD_TESTING=ON` 时会额外构建并
注册 `PathTrackerSelfTest` 目标(不依赖仿真器的跟踪器自测);文件不存在时
自动跳过。

## 4. 运行

1. **启动 EchoSim 仿真器**:打开 `EchoSim.exe`,加载竞赛工程
   `Onsite8_B.prj`,选择场景 **Moon2** 与对应测试;
2. **启动控制器**(先于测试开始,让它先连上消息总线):在仓库内任意子目录运行

   ```powershell
   out\build\x64-Release\bin\UserController.exe
   ```

   程序提示 `Select test (1-6):` 后输入与 EchoSim 场景一致的编号。六组测试的
   起点、终点和必经点已集中预置在 `src/app/TaskConfig.cpp`，无需运行前改代码。

3. **在 EchoSim 中开始测试**。控制器依次执行:
   加载或构建代价地图 -> 双向 A* 全局规划 -> 路径优化 -> 等到车辆状态后
   进入跟踪控制循环,从终点前约 20 m 平滑降速，并在终点容差内持续制动、
   停稳保持 3.2 s 后退出。

运行期(调试模式开启时)会弹出 OpenCV 实时轨迹窗口(红色=参考路径,
蓝色=实际轨迹,绿色=车辆位姿);控制台每 10 次成功发布打印一条 `[track]`
跟踪日志。比赛模式(`enable_debug_output=false`)下不弹窗、不写任何文件。

## 5. 参数配置

参数按 **模块隔离** 分层,调参入口:

| 参数类别 | 位置 | 说明 |
|---|---|---|
| 任务级(六组路线、扫描走廊、输出开关) | `src/app/TaskConfig.cpp` 的 `makeDefaultTaskConfig()` / `makeTaskConfigForTest()` | Test1–Test6 起终点和必经点集中配置，启动时交互选择 |
| 代价地图参数 | `include/planning/TerrainCostmap.h` | 分辨率、坡度/粗糙度阈值(修改后需递增缓存版本号,见下) |
| 全局规划参数 | `include/planning/GlobalPlanner.h` | A* 方法、地形代价权重 |
| 路径优化参数 | `include/optimization/PathOptimizer.h` | 捷径跨度、平滑轮数、曲率上限系数 |
| 跟踪参数 | `include/tracking/PathTracker.h` | 跟踪算法、前视距离、速度门控 |

注意:

- `optimizer.wheelbase_m` / `max_front_wheel_angle_rad` 与
  `tracking.geometry` 同名字段必须保持一致(规划与控制共用车辆参数),
  不一致时启动会打 `[warning]`。
- **比赛模式**:把 `include/app/TaskConfig.h` 中 `enable_debug_output`
  默认值改为 `false`,关闭实时轨迹窗口与全部调试文件输出(含代价地图缓存)。
- **地形根自动发现**:程序从工作目录向上最多 8 层定位项目根,再按常见布局
  探测 Moon2 地形根(需含 `level_1m`/`level_10m`/`level_100m`/`level_1000m`
  目录)。也可用环境变量显式指定(前者优先):

  ```powershell
  set USER_CONTROLLER_TERRAIN_ROOT=D:\path\to\Heightmap\Moon2
  set ECHOSIM_TERRAIN_ROOT=D:\path\to\Heightmap\Moon2
  ```

- **代价地图缓存**:首次建图较慢,成功后缓存到 `output/`;下次启动直接加载。
  分辨率/阈值等口径变化需递增 `TerrainCostmap.cpp` 写入的缓存版本号
  (`terrain_metadata.json` 的 `version` 字段)使旧缓存失效。

- **软风险与硬障碍分离**:坡度 18°、粗糙度 0.20 是软风险归一化基准，
  超过后仍可被 A* 以较高代价使用；只有地形无效、坡度超过 40° 或粗糙度
  超过 0.50 才写入 `hard_obstacle` 并禁止通行。当前缓存格式为 v5。
- **必经点规划**:规划流水线按起点、必经点、终点分段搜索，并根据必经点
  两侧的地形 A* 预览自动生成前后引导走廊。引导方向和距离由地形与车辆
  最小转弯半径计算，不为某个 Test 硬编码；最终路径仍按任务容差检查真实必经点。
- **安全校验行为**:优化器仍计算最大离散曲率、残余尖角和车体包络净空，
  但 `reject_unsafe_output` 默认是 `false`。指标超限时打印
  `[optimizer] unsafe path detected ...; continuing because safety rejection is disabled`
  并继续进入控制循环，避免仅因优化安全门控导致任务无法启动。若需要严格
  验证，将 `include/optimization/PathOptimizer.h` 中该参数设为 `true`。
  地形缓存损坏、全局规划无路径或真实必经点未满足等硬错误仍会终止运行。

## 6. 输出产物说明(output/,调试模式)

| 文件 | 内容 |
|---|---|
| `costmap.txt`、`terrain_valid.txt`、`terrain_hard_obstacle.txt` 等 / `terrain_metadata.json` | v5 代价地图缓存(高程/坡度/粗糙度/软代价/有效掩码/硬障碍掩码 + 元数据) |
| `terrain_preview.png` | 代价地图灰度预览(亮=高代价) |
| `global_path.txt` / `global_path_with_yaw.txt` | 模块一产出的原始栅格路径(每行 `x y [yaw]`) |
| `optimized_path.txt` / `optimized_path_with_yaw.txt` | 模块二优化后路径 |
| `global_path_on_costmap.png` | 优化路径叠加代价地图总览(红=路径,蓝=起点,绿=终点) |
| `actual_path.txt` | 实际行驶轨迹(按 2 s 采样) |
| `tracking_local_final.png` / `global_and_actual_final.png` | 跟踪结束后的局部/全局轨迹对比图 |

仿真评分结果由 EchoSim 生成在其工作区
`EchoSim/data/project/Onsite8_B/Results/<Test>/<Scenario>/` 下,不在本仓库内。

当前规划回归结果（2026-08-25，Moon2 v5 缓存）：Test3 和 Test4 均达到
`max discrete curvature=0.123 1/m`、`unrounded corners=0`，车体包络净空
分别为 0.127 m 和 0.126 m，并成功进入控制循环。该数据是规划阶段回归证据，
不等同于完整仿真评分。

## 7. 常见问题

| 现象 | 原因与处理 |
|---|---|
| 控制器退出并打印 `[error] ...` | 按出错阶段定位:消息总线初始化(先启动 EchoSim)、地形根发现(检查路径/环境变量)、建图、规划、跟踪 |
| 建图极慢或内存占用大 | 检查 `scan_bounds` 是否覆盖过大;已有缓存可跳过建图 |
| `[warning] optimizer 与 tracking 的车辆运动学参数不一致` | 两处车辆参数漂移,同步 `PathOptimizer.h` 与 `PathTracker.h` 的轴距/最大前轮角 |
| `[optimizer] unsafe path detected ... continuing ...` | 路径曲率、尖角或车体包络指标超限；默认仅告警并继续运行。需要阻断时设置 `reject_unsafe_output=true` |
| `[state] initial pose differs` | 任务配置起终点与场景实际不符,核对 `makeDefaultTaskConfig()` |
| `failed to connect runner: 127.0.0.1:9000`(EchoSim 侧) | 执行模式或 runner 配置不匹配,见 EchoSim 使用手册切换本地动画执行 |
| 仿真无评分/评分 0 | 查阈值、是否碰撞/翻车/卡死/越界/超时,以及是否在容差内到达终点 |

## 8. 目录结构

```text
include/    各模块头文件(模块参数默认值也在这里)
  common/       PathTypes.h 跨模块共享类型
  planning/     TerrainCostmap / GlobalPlanner
  optimization/ PathOptimizer
  tracking/     PathTracker / EchoSimRuntime
  app/          TaskConfig / PlanningPipeline / TrajectoryVisualizer
src/        与 include 对应的实现,src/app/UserController.cpp 为入口
tests/      可选的 PathTrackerSelfTest.cpp(不在库中时跳过)
out/        构建输出(不入库)
output/     调试产物与代价地图缓存(不入库)
third_party/ 依赖包(不入库,见第 2 节)
```
