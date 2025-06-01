# 基于 Realsense D455 和 SLAM 的现实场景游戏化项目

本项目旨在使用 Intel Realsense D455 深度相机，通过 SLAM (Simultaneous Localization and Mapping) 技术对现实场景进行三维建模，并将生成的模型导入游戏引擎 (如 Unity)，实现将现实场景融入游戏体验的目标。

## 阶段一：环境准备与 SLAM 系统配置

### 1. 系统与硬件要求

*   **操作系统**: Ubuntu 20.04
*   **ROS 版本**: ROS 1 Noetic
*   **相机**: Intel Realsense D455
*   **工作空间**: 已创建 ROS Noetic 工作空间 (例如: `~/workspace/ws_slam`)

### 2. 安装必要的 ROS 包

#### a. 安装 Realsense ROS 包

用于在 ROS 中驱动 Realsense 相机并获取数据。

```bash
sudo apt update
sudo apt install ros-noetic-realsense2-camera
```

#### b. 安装 RTAB-Map ROS 包

RTAB-Map (Real-Time Appearance-Based Mapping) 是一个强大的开源 SLAM 包，特别适合 RGB-D 相机进行三维建图和稠密重建。

```bash
sudo apt install ros-noetic-rtabmap-ros
```
*此命令会安装 `rtabmap`核心库以及所有相关的ROS节点、消息、启动文件等。*

### 3. 运行 SLAM 进行初步建图测试

#### a. 启动 Realsense 相机节点 (推荐配置)

打开一个新的终端，source ROS 环境，并启动相机节点。为了获得最佳的 SLAM 效果，建议启用深度对齐和IMU数据。

```bash
# 如果您的工作空间在 ~/workspace/ws_slam
source ~/workspace/ws_slam/devel/setup.bash 
# (或者根据您的实际路径 source /opt/ros/noetic/setup.bash)
roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:="linear_interpolation"
```

*   **参数说明**:
    *   `align_depth:=true`: 启用深度图像与彩色图像的对齐。这对于准确的三维重建至关重要。启用后，通常会发布话题如 `/camera/aligned_depth_to_color/image_raw`。
    *   `enable_gyro:=true`: 启用陀螺仪数据。
    *   `enable_accel:=true`: 启用加速度计数据。
    *   `unite_imu_method:="linear_interpolation"` (或 `"copy"`): 将加速度计和陀螺仪数据合并发布到标准的 `/camera/imu` 话题，方便 RTAB-Map 使用。
*   **检查**: 
    *   确保相机正常启动。通过 `rostopic list` 查看是否有以下关键话题：
        *   RGB图像: `/camera/color/image_raw`
        *   对齐后的深度图像: `/camera/aligned_depth_to_color/image_raw`
        *   RGB相机信息: `/camera/color/camera_info`
        *   IMU数据: `/camera/imu`
    *   观察终端输出，检查是否有 `uvc streamer watchdog triggered` 等USB相关的错误。如果出现，请尝试：
        *   确保相机连接到 USB 3.0+ 端口。
        *   更换高质量的 USB 线缆。
        *   将相机直接连接到电脑，避免使用 USB Hub。
    *   **IMU校准警告**: 您可能会看到关于 "IMU Calibration is not available" 的警告。对于初步测试可以暂时忽略，建议进行IMU校准。

#### b. 启动 RTAB-Map SLAM

打开另一个新的终端，source ROS 环境，并启动 RTAB-Map。请确保以下话题名称与您通过 `rostopic list` 看到的实际名称一致。

```bash
# 如果您的工作空间在 ~/workspace/ws_slam
source ~/workspace/ws_slam/devel/setup.bash
# (或者根据您的实际路径 source /opt/ros/noetic/setup.bash)
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    imu_topic:=/camera/imu \
    wait_for_transform_duration:=0.2
```
*   `rtabmap_args:="--delete_db_on_start"`: 这个参数使得每次启动时都创建一个新的地图数据库。如果想加载之前的地图或继续建图，可以移除此参数。
*   **话题重映射**: 
    *   `rgb_topic:=/camera/color/image_raw`
    *   `depth_topic:=/camera/aligned_depth_to_color/image_raw`
    *   `camera_info_topic:=/camera/color/camera_info`
    *   `imu_topic:=/camera/imu`
    *   `wait_for_transform_duration:=0.2`: 参数，给 TF 树一点时间准备。
*   **IMU 方向错误 (常见提示)**: 您可能会在 RTAB-Map 的终端中看到关于 `"IMU received doesn't have orientation set, it is ignored."` 的错误。这通常表示 `realsense2_camera` 发布的 `/camera/imu` 话题主要包含原始的加速度和角速度，而没有预先计算好的姿态(orientation)信息。RTAB-Map 注意到这一点并忽略了缺失的姿态部分。**如果RTAB-Map的里程计和建图功能仍在正常工作 (例如，您能看到里程计质量的日志输出，`rtabmapviz`中地图也在更新)，那么这个错误通常不影响基本使用，RTAB-Map会尝试仅使用加速度和角速度数据。** 后续可以通过添加IMU滤波器节点来提供带姿态的IMU数据以尝试提升性能。
*   **可视化**: 该命令会启动 RTAB-Map 的核心节点以及 `rtabmapviz` 可视化工具。

#### c. 开始建图

1.  在 `rtabmapviz` 窗口中，您应该能看到相机当前的彩色图像和深度图像（如果配置正确）。
2.  缓慢移动 D455 相机来扫描您希望建模的场景。
3.  观察 `rtabmapviz` 中的3D地图视图，它会实时显示构建的点云地图。
4.  注意观察是否有"回环检测 (Loop Closure)"发生，这对于构建一致性好的大范围地图非常重要。当相机回到之前经过的位置时，RTAB-Map会尝试识别并进行地图优化。

#### d. 保存地图数据

1.  RTAB-Map 在运行时会自动将地图数据保存在一个数据库文件（默认为 `~/.ros/rtabmap.db`）。
2.  建图完成后，可以在 `rtabmapviz` 的菜单中选择:
    *   `File` -> `Export 3D clouds...` (导出点云, 如 .ply, .pcd)
    *   `File` -> `Export 3D mesh...` (导出网格模型, 如 .ply, .obj)。**对于导入游戏引擎，推荐导出 `.obj` 格式的网格模型，因为它通常能更好地包含纹理信息。**

---

## 阶段二：数据采集与模型生成

在基本 SLAM 流程跑通后，为了获得更高质量和更可控的建图结果，推荐使用 `rosbag` 进行数据采集，然后离线处理。

### 1. 使用 `rosbag` 录制传感器数据

确保 `realsense2_camera` 节点正在以推荐配置运行 (启用深度对齐和IMU)。

在一个新的终端中，执行以下命令开始录制。请务必缓慢、平稳地移动相机，覆盖您想建模的所有区域，并尝试形成闭环（回到之前经过的地方）。

```bash
# 确保 rostopic list 中的话题名称与下面命令中的一致
# 特别是 /tf 和 /tf_static 对于离线处理很重要
rosbag record \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info \
  /camera/imu \
  /tf \
  /tf_static \
  -o ~/my_scene.bag # 您可以自定义输出路径和文件名
```

录制完成后，在 `rosbag record` 的终端按 `Ctrl+C` 停止。

### 2. 离线运行 RTAB-Map 处理 `rosbag`

**重要概念：两阶段离线处理**

为了在离线处理 `rosbag` 时获得最佳效果（既能保证建图的连续性，又能进行充分的全局优化），我们推荐采用以下两阶段处理方法：

1.  **阶段一：初始地图构建 (增量模式)**
    *   目的：使用增量模式 (`Mem/IncrementalMemory true`) 稳定地处理整个 `rosbag`，确保所有数据都被看到并构建出一个初步的、完整的地图。这个阶段的重点是跟踪的连续性。
    *   输出：一个包含初步地图的数据库文件。

2.  **阶段二：全局优化与精炼 (非增量模式)**
    *   目的：加载阶段一生成的数据库，然后使用非增量模式 (`Mem/IncrementalMemory false`) 和 `--Mem/InitWMWithAllNodes true` 来对整个地图进行全局优化、回环检测精炼和稠密重建。
    *   输出：一个经过优化的、更高质量的地图数据库和最终模型。

**至关重要的一步**: 在开始任何离线处理之前，必须告诉 ROS 系统我们将使用仿真时间。这通过设置 ROS 参数 `use_sim_time` 为 `true` 来完成。**此操作必须在启动 `rosbag play` 和 `rtabmap` 节点之前完成。**

在一个终端中，首先设置参数：
```bash
# 1. 设置ROS使用仿真时间 (必须在播放rosbag和启动rtabmap之前执行!)
rosparam set use_sim_time true
```
*   **注意**: 如果您在之前的尝试中已经启动了 `roscore` 或其他 ROS 节点而没有设置 `use_sim_time true`，建议先关闭所有 ROS 相关的终端和进程 (包括 `roscore`)，然后重新打开新的终端，首先执行 `rosparam set use_sim_time true`，再启动 `roscore` (如果需要手动启动的话)，然后进行后续操作。

#### 阶段一：初始地图构建 (使用 SIFT 特征)

在一个终端播放 `rosbag` 文件 (建议使用较慢的播放速率)：
```bash
# 2. 播放 rosbag 文件 (阶段一)
#    -r 0.1 表示以十分之一速度播放，给RTAB-Map更充分的处理时间
#    --clock 参数是必要的，因为 use_sim_time is true
#    确保将 ~/my_scene.bag 替换为您的 rosbag 文件路径
rosbag play --clock -r 0.1 ~/my_scene.bag
```

在另一个终端，启动 RTAB-Map 进行阶段一处理：
```bash
# 确保 rosparam set use_sim_time true 已在播放 rosbag 前执行
source ~/workspace/ws_slam/devel/setup.bash # 或您的ROS环境source命令

roslaunch rtabmap_ros rtabmap.launch \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    imu_topic:=/camera/imu \
    wait_for_transform_duration:=0.2 \
    database_path:="~/.ros/rtabmap_stage1_sift_incremental.db" \
    rtabmap_args:="--delete_db_on_start \
                   --Mem/IncrementalMemory true \
                   --Vis/FeatureType 1 \
                   --Vis/MaxFeatures 800 \
                   --Vis/MinInliers 20 \
                   --Odom/Strategy 0"
    # 对于阶段一，可以不加 --Rtabmap/DetectionRate 0，让其随rosbag结束而结束。
    # 或者在 rtabmapviz 中手动触发"Download map" 或 "Process buffered events"
```

*   **阶段一参数解释**:
    *   `database_path:="~/.ros/rtabmap_stage1_sift_incremental.db"`: 为阶段一指定新的数据库文件(SIFT)。
    *   `--delete_db_on_start`: 确保每次阶段一开始都是全新的构建。
    *   `--Mem/IncrementalMemory true`: **使用增量模式**，这对于初始跟踪和连续建图更鲁棒。
    *   `--Vis/FeatureType 1`: 使用 SIFT 特征检测器 (请根据您的RTAB-Map版本和编译情况确认SIFT对应的编号，通常是1或2，或者如果编译了非自由特征模块可能是其他值)。
    *   `--Vis/MaxFeatures 800`: SIFT 特征点数量。
    *   `--Vis/MinInliers 20`: 视觉里程计接受运动估计的最小内点数。
    *   `--Odom/Strategy 0`: 里程计策略为 Frame-to-Map。
*   运行完毕后，您将在 `~/.ros/` 目录下得到 `rtabmap_stage1_sift_incremental.db` 文件。

#### 阶段二：全局优化与精炼 (使用 SIFT 特征)

确保阶段一已成功完成并且 `rtabmap_stage1_sift_incremental.db` 文件已生成。

在一个终端**重新**播放 `rosbag` 文件 (通常与阶段一速率相同或稍快，这里保持0.1)：
```bash
# 3. 再次播放 rosbag 文件 (阶段二)
rosbag play --clock -r 0.1 ~/my_scene.bag
```

在另一个终端，启动 RTAB-Map 进行阶段二处理：
```bash
# 确保 rosparam set use_sim_time true 已在播放 rosbag 前执行
source ~/workspace/ws_slam/devel/setup.bash # 或您的ROS环境source命令

roslaunch rtabmap_ros rtabmap.launch \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    imu_topic:=/camera/imu \
    wait_for_transform_duration:=0.2 \
    database_path:="~/.ros/rtabmap_stage1_sift_incremental.db" \
    rtabmap_args:="--Mem/IncrementalMemory false \
                   --Mem/InitWMWithAllNodes true \
                   --Optimizer/Strategy 1 \
                   --Reg/Strategy 2 \
                   --Rtabmap/DetectionRate 0 \
                   --Vis/FeatureType 1 \
                   --Vis/MaxFeatures 800 \
                   --Vis/MinInliers 20 \
                   --Odom/Strategy 0"
    # 注意：这里没有 --delete_db_on_start，因为我们要加载阶段一的数据库
```

*   **阶段二参数解释**:
    *   `database_path:="~/.ros/rtabmap_stage1_sift_incremental.db"`: **加载阶段一生成的数据库**。
    *   **没有** `--delete_db_on_start`。
    *   `--Mem/IncrementalMemory false`: **使用非增量模式**进行全局处理。
    *   `--Mem/InitWMWithAllNodes true`: **极其重要！** 此参数确保 RTAB-Map 在启动时用数据库中所有已存在的节点来初始化其工作内存，这是进行有效全局优化的前提。
    *   `--Optimizer/Strategy 1`: 使用 g2o 进行图优化 (1=g2o, 2=GTSAM, 0=TORO)。
    *   `--Reg/Strategy 2`: 回环检测的精细配准策略使用视觉和ICP。
    *   `--Rtabmap/DetectionRate 0`: 确保 `rosbag` 播放完毕后，RTAB-Map 依然会处理所有缓冲事件并完成优化。
    *   视觉参数 (`--Vis/FeatureType 1`, `--Vis/MaxFeatures 800`, `--Vis/MinInliers 20`) 与阶段一保持一致，使用SIFT特征。
*   **如果 `delay` 依然巨大 (尤其在阶段一)**:
    *   再次确认 `rosparam get use_sim_time` 的输出是 `true`。

### 2.1. (重要) 离线处理时的参数调优指南

离线处理 `rosbag` 的核心优势在于能够通过迭代调整 RTAB-Map 的参数来优化最终地图的质量。这是一个反复实验和观察的过程。

**A. 理解可调参数范畴**
线
RTAB-Map 有大量参数，主要影响以下方面：

1.  **视觉里程计 (VO) / 视觉惯性里程计 (VIO)**：影响路径估计的准确性和鲁棒性。
    *   `Vis/MinInliers`: 用于估计运动的最小内点匹配数 (例如: 15, 20, 25)。增加此值提高鲁棒性，但特征少时易丢跟踪。
    *   `Vis/MaxFeatures`: 每帧提取的特征点数 (例如: 500, 1000)。更多特征可能提高精度，但增加计算量。
    *   `Vis/FeatureType`: 特征点类型 (例如: 6 代表 ORB, 2 代表 SURF)。ORB 较快，SURF 可能更鲁棒但慢。
    *   `Odom/Strategy`: 里程计策略 (0: Frame-to-Map, 1: Frame-to-Frame)。
    *   IMU相关 (若使用VIO): `Odom/SigmaAngularVelocity`, `Odom/SigmaLinearAcceleration` (IMU噪声模型)。

2.  **回环检测 (Loop Closure Detection)**：影响地图的全局一致性和修正累积误差。
    *   `Kp/MaxFeatures`: 用于回环检测词袋模型的特征数 (例如: 1000, 1500)。
    *   `LccBow/MinInliers`: 接受回环假设的最小内点数 (例如: 10, 15, 20)。
    *   `LccBow/InlierDistance`: 几何验证时特征点视为内点的最大距离 (像素或米)。
    *   `RGBD/LoopClosureReextractFeatures`: 是否在回环时重新提取特征 (true/false)。
    *   `Reg/Strategy`: 回环后的精细配准策略 (0: Visual, 1: ICP, 2: Visual+ICP)。
    *   `LccIcp/MaxCorrespondenceDistance`: ICP配准时点对的最大对应距离。

3.  **图优化 (Graph Optimization)**：在检测到回环后，调整整个轨迹和地图以最小化误差。
    *   `Optimizer/Strategy`: 选择优化器 (0: TORO, 1: g2o, 2: GTSAM)。g2o 或 GTSAM 通常更好。
    *   `Optimizer/Iterations`: 优化迭代次数 (例如: 100)。

4.  **建图 (Mapping)**：影响最终三维模型的细节、密度和噪点。
    *   `Grid/CellSize`: 栅格地图单元大小 (例如: 0.05, 0.02)。越小细节越多，但计算和存储开销越大。
    *   `Grid/RangeMax`, `Grid/RangeMin`: 点云整合到地图中的最大/最小深度。
    *   `Grid/DepthDecimation`: 深度图下采样因子，用于生成点云 (例如: 1, 2, 4)。越大点云越稀疏。
    *   `Grid/NoiseFilteringRadius`, `Grid/NoiseFilteringMinNeighbors`: 点云噪声滤波参数。

**B. 如何调整参数**

*   **主要方式：通过 `rtabmap_args` 传递** (在 `roslaunch rtabmap_ros rtabmap.launch ...` 后追加):
    ```bash
    rtabmap_args:="--GroupName/ParamName Value --AnotherGroup/ParamName2 Value2"
    ```
    例如: `rtabmap_args:="--Vis/MinInliers 20 --LccBow/MinInliers 15 --Grid/CellSize 0.03"`
*   **查找参数名称**: 参考 RTAB-Map GitHub Wiki，或使用 `rtabmap` (独立GUI) / `rtabmapviz` (Preferences) 界面查看。

**C. 在 `rtabmapviz` 中观察评估**

1.  **相机轨迹**: 是否平滑？是否严重漂移？里程计质量如何？
2.  **回环检测**: 是否能正确检测到回环？地图修正效果如何？有无错误回环？
3.  **三维地图质量**: 点云/网格的密度、噪点、平整度、物体轮廓清晰度、有无重影？
4.  **处理效率**: CPU占用是否过高？

**D. 迭代优化步骤**

1.  **基准运行**: 使用一套初始参数（如 `README` 中离线处理的命令）完整运行一遍 `rosbag`，保存数据库和模型作为基准。
2.  **针对性调整**: 根据基准运行中观察到的主要问题（如漂移、无回环、地图噪声大），查阅相关参数类别，尝试调整一到两个关键参数。
3.  **重新运行与比较**: 使用新参数重新播放 `rosbag` 并运行 RTAB-Map。在 `rtabmapviz` 中仔细比较新结果与之前结果的差异。
4.  **记录**: 简单记录每次使用的关键参数和主要效果变化。
5.  **重复**: 不断重复此过程，逐步逼近满意的地图效果。

**提示**:
*   耐心！调参需要时间和多次尝试。
*   一次主要关注一个方面的问题，并调整相关的少数几个参数。
*   当获得较好结果时，记得保存该参数配置和生成的数据库文件。

### 2.2. 从 RTAB-Map 导出 OBJ 模型

当您在 RTAB-Map (无论是实时运行还是离线处理 `rosbag` 后) 中获得了一个初步的或优化后的地图后，可以将其导出为 `.obj` 格式的网格模型，以便在 Mesh 编辑软件中处理或直接导入游戏引擎。

步骤如下 (在 `rtabmapviz` 图形界面中操作)：

1.  **确保地图已在 `rtabmapviz` 中加载并完整显示**。
    *   如果是离线处理 `rosbag`，确保所有数据已处理完毕。有时可能需要在 `rosbag` 播放结束后，在 `rtabmapviz` 的菜单中选择 `File` -> `Trigger a new map update` 或类似操作，或者在启动 `rtabmap.launch` 时使用 `rtabmap_args:="--Rtabmap/DetectionRate 0"` 确保所有缓冲事件得到处理。

2.  **打开导出菜单**：
    *   在 `rtabmapviz` 的顶部菜单栏中，点击 `File`。
    *   从下拉菜单中选择 `Export 3D mesh...`。
        *   (注意不要错选 `Export 3D clouds...`，那是导出点云的。我们需要的是带表面的网格模型。)

3.  **保存文件**：
    *   在弹出的文件保存对话框中，指定一个您希望保存模型的位置。
    *   **文件名**：输入您想要的文件名，例如 `my_scene.obj`。
    *   **文件类型**：确保选择的是 `OBJ files (*.obj)`。
    *   点击 `Save`。

4.  **配置导出选项 (如果弹出)**：
    *   点击 `Save` 后，可能会出现一个名为 "Exporting options" 或类似的对话框。
    *   **Texture (纹理)**: **务必确保与纹理相关的选项被勾选**，例如 `Export texture`, `Save texture` 或类似描述。这会同时生成 `.mtl` 文件和纹理图片。
    *   **Mesh Assembly (网格组装)**: 如果地图包含多个子图，通常选择将它们合并 (Assemble) 成单个网格。
    *   **Voxel size / Polygon count (体素大小/多边形数量)**: 某些情况下，这里可以进行初步的网格简化。对于首次导出，可以先使用默认值。如果模型太大，后续可以在 Mesh 编辑软件中进行更精细的简化。
    *   **Normals (法线)**: 通常也需要导出法线信息。
    *   根据提示点击 `OK` 或 `Export` 完成导出。

5.  **检查导出的文件**：
    *   导航到您选择的保存路径。您应该能找到以下文件：
        *   一个 `.obj` 文件 (例如 `my_scene.obj`)
        *   一个 `.mtl` 文件 (例如 `my_scene.mtl`)
        *   一个或多个纹理图片文件 (例如 `my_scene_texture_0.png`, `my_scene_texture_1.jpg` 等，文件名和格式可能有所不同)。
    *   **重要**: 为了使 Mesh 编辑软件或游戏引擎能正确加载模型及其纹理，请确保这三类文件（`.obj`, `.mtl`, 纹理图片）都存放在同一个文件夹内。

下一步，您就可以使用像 MeshLab 或 Blender 这样的软件来打开并处理这个 `.obj` 模型了。

### 3. 模型导出与优化

*   从 `rtabmapviz` 导出 `.obj` 网格模型。
*   使用 MeshLab, Blender 或 CloudCompare 等工具进行：
    *   模型清理 (去除噪点、孤立片元)
    *   网格简化 (减少面数，优化性能)
    *   孔洞填充
    *   纹理检查与修复
*   选取 MeshLab ，其是一个免费开源的软件化点云处理平台。
    *   启动：chmod +x MeshLab2023.12-linux.AppImage 
             ./MeshLab2023.12-linux.AppImage
    *   导入 `.obj` 网格模型：File -> Import Mesh -> .obj文件
    *   尝试了以下操作：
        *   陶宾平滑（平滑表面，减少噪声，并保持特征）：Filter -> Smoothing, Fairing and Deformation -> Taubin Smooth
        *   采样与简化（生成均匀分布点样本，创建更干净点云）：Filter -> Sampling -> Poisson-disk Sampling
        *   去除噪声（根据成面三点数）：Filter -> Cleaning and Repairing -> Remove Isolated Pieces (wrt Face Num.)
        *   重建（泊松方程表面重建）：Filter -> SRemeshing, Simplification and Reconstruction -> Surface Reconstruction: Screened Poisson
        *   填补空洞：Filters -> Remeshing, Simplification and Reconstruction -> Close Holes

## 阶段三：导入游戏引擎 (细节待补充)

*   导入 Unity (或您选择的其他引擎)。
*   设置材质、光照。
*   场景集成与交互。
