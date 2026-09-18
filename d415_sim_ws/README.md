# 桥梁 D415 独立相机仿真

小车、3 cm 相机辅助杆和钢桥联合仿真已新增独立入口：`./run_robot.sh`，默认仅录制 RGB 视频，深度仍实时显示。见 [联合仿真说明](src/bridge_robot_sim/README.md)。

ROS 2 Humble + Gazebo Fortress。复用 RealSense 官方 D415 模型与 Locus 相机宏，使用独立 RGB 和 Depth 视点；当前没有小车、磁吸、遥控或 SLAM。

## 1. 直接使用

先进入目录（下面各条命令分别执行）：

```bash
cd /home/lv/visual_camera_simulate/d415_sim_ws
```

启动桥梁、固定相机、RViz2 和自动数据录制：

```bash
./run.sh
```

RViz2 的 RGB 和 Depth preview 面板显示两路画面，三维区域显示深度点云和坐标系。Gazebo 窗口显示场景；其自由观察相机不是 D415。默认固定机位朝向一道正面板，可以看到通孔及底部 U 肋。

终端出现 `RECORDING:` 和 rosbag2 的订阅日志后开始采集。启动时等待 RGB、Depth 及两路 CameraInfo，就绪前不会生成空录制。若 90 秒内未就绪，会明确报错。

结束时在启动终端按一次 `Ctrl+C`，等待 `RECORDING CLOSED:` 和命令提示符返回。不要连续按多次或直接关终端，以免中断 bag 收尾。只关闭 Gazebo/RViz 窗口不等于停止全部后台节点；仍需在终端结束启动命令。

结束后导出最新一次录制的两个视频：

```bash
./export.sh
```

回放最近一次录制，自动重新生成深度预览和点云，不启动 Gazebo：

```bash
./replay.sh
```

**实时仿真和回放不要同时运行在同一 ROS_DOMAIN_ID 下**，避免同名话题和 `/clock` 冲突。回放结束后同样用 `Ctrl+C` 退出显示节点。

指定一次录制也可以：

```bash
./export.sh recordings/20260916_121854_346949
./replay.sh recordings/20260916_121854_346949
```

已有视频不会被静默覆盖；要重新导出可在指定目录后添加 `--force`。

## 2. 常用启动方式

只查看，不保存数据：

```bash
./run.sh record:=false
```

不打开 Gazebo 窗口，仅保留 RViz2 与录制：

```bash
./run.sh gui:=false
```

无可视化窗口（仍需要可用的图形渲染环境）：

```bash
./run.sh gui:=false rviz:=false
```

已知 2 米平面标定场景：

```bash
./run.sh world:=calibration record:=false
```

当前采用 X 显示下的 Ogre2；`gui:=false` 表示不创建 Gazebo GUI，并不承诺在无显卡/无显示服务的服务器上直接运行。

## 3. 配置与模型

主要配置文件：`src/bridge_d415_sim/config/camera.yaml`。

| 参数 | 当前值 | 含义 |
|---|---|---|
| width / height | 640 / 360 | 两路输出分辨率 |
| fps | 30 | 两路传感器目标频率 |
| color_hfov_deg | 69 | RGB 名义水平视场 |
| depth_hfov_deg | 65 | Depth 名义水平视场 |
| depth_near / depth_far | 0.45 / 3.0 | 仿真深度裁剪范围，米 |
| x / y / z | 0 / -1.5 / 1.2 | 测试安装坐标系的世界位置，米 |
| roll / pitch / yaw | 0 / 0.15 / π/2 | 安装姿态，弧度 |

使用理想无畸变、方形像素针孔模型：`fx=fy=width/(2*tan(hfov/2))`。垂直视场由图像比例推导，不同时强行指定与比例不一致的垂直视场。RGB 与 Depth 各有独立内参，不是实机标定值。原始深度为 `32FC1`，单位 **米**；超范围/无效值不应当作真实距离。

编辑配置后保存，停止并重新运行 `./run.sh`。用 `--symlink-install` 构建后，已有配置和脚本修改通常无需重编译；添加新文件或改变 CMake 安装规则后运行 `./build.sh`。

`calibration` 世界会将安装位姿覆盖为 `[0,0,1,0,0,0]`，用于验证前方 x=2 m 的平面，其他相机参数仍读取配置文件。

不要在 Gazebo GUI 中直接拖动固定相机来改变测试机位：测试支架的 `world → camera_mount` 是静态 TF，拖动仿真实体不会同步更新 ROS TF。请修改位姿配置并重启。

## 4. 桥梁复用

直接引用 `../gazebo_bridge/models/bridge`，不复制或改写桥梁 STL，不覆盖原 Classic 世界。保留原 `model.sdf` 的部件材质、碰撞和 0.001 缩放。Fortress 世界另存于 `src/bridge_d415_sim/worlds/bridge_camera.sdf`，统一绕 X 轴 +90°。

新增点光源 `inspection_light` 位于 `[0,-1.5,1.8]`，用于当前桥箱固定机位的可见光照明。它不是红外投射器，移动相机时也不会自动跟随；可在世界文件中调整。

本阶段验证了视觉加载，不代表桥梁复杂网格在 Fortress 物理引擎中的机器人接触已经验证。后续磁吸小车仍需独立测试非凸碰撞、通孔、接缝及接触法向。

## 5. 相机来源与适配

- 官方模型：系统包 `realsense2_description` 中的 `urdf/_d415.urdf.xacro` 和 `meshes/d415.stl`。
- 上游仿真：<https://github.com/locusrobotics/gz_sensor_descriptions>。
- 固定提交：`bf438856e58ec0af2ff2be04066b6942a77eadf3`，记录于 `upstream.repos`。
- 克隆目录 `src/gz_sensor_descriptions` 保持原样。适配后的通用 RGB/Depth 宏放在 `src/bridge_d415_sim/urdf/compat/`，保留 BSD-3-Clause 许可证；补丁见 `patches/locus_compat.patch`。

具体适配：

1. 从上游 `rgb+d` 思路复用两个独立传感器宏，允许指定安装坐标系、光学坐标系和裁剪范围。
2. RGB 位于 `d415_color_frame`；Depth 位于 `d415_depth_frame`。名义横向间距来自官方模型（15 mm），不将两者放在同一个 RGB 视点。
3. 使用 Fortress 的 `ignition-gazebo-*-system` 系统插件与 `ignition.msgs.*` 桥接类型。
4. 删除深度宏重复的 `triggered` 参数，以及 RGB 中无必要的全零畸变渲染配置；Ogre2 不支持该畸变 pass。
5. 本机 `ignition-sensors6 6.9.0` 已能原生发布深度 CameraInfo，不需要合成内参。两路原生 CameraInfo 保留在 `/d415/native/...` 供排查。
6. Fortress 将归并到同一刚体上的两台相机自动当作双目，给右侧传感器的投影矩阵加入基线项（源头：gz-sim `Sensors.cc` 的 `SetBaseline`）。当前话题分别在各自光学坐标系中，所以 `camera_support.py` 仅把 P[3]、P[7] 归零，保留 K、图像尺寸、原生时间戳和 frame_id；相机间基线通过官方 TF 表示，避免重复计入。2 米平面与点云中心测试已验证。

这里不提供真实红外双目匹配、主动红外投射、滚动快门、钢板反光误差、噪声或测距失败的物理复现。它是带官方外形、名义外参和分开视点的 RGB/Depth 几何仿真。

官方 Xacro 注明惯性数值不可靠，当前未校准，不用于相机自身动力学性能评估。保留官方相机碰撞体；后续安装到小车保护范围内，正常行驶时避免接触桥梁。不能把“不发生接触”理解为“禁用碰撞”。

## 6. ROS 2 接口

| 话题 | 数据 |
|---|---|
| `/d415/color/image_raw` | `rgb8`，640×360 |
| `/d415/color/camera_info` | RGB 自身光学坐标系内参 |
| `/d415/depth/image_raw` | `32FC1` 原始深度，米 |
| `/d415/depth/camera_info` | 深度自身光学坐标系内参 |
| `/d415/depth/preview` | 固定距离色标的 `bgr8` 深度预览，无效值黑色 |
| `/d415/depth/points` | `depth_image_proc` 生成的 XYZ 点云 |
| `/d415/native/color/camera_info` | 未归一化的原生 RGB 内参，用于诊断 |
| `/d415/native/depth/camera_info` | 未归一化的原生深度内参，用于诊断 |
| `/tf_static` | 固定支架和官方相机坐标变换 |
| `/clock` | 仿真时间 |

当前交付 XYZ 点云，没有把未配准的 RGB 直接贴到深度点云上。彩色点云可后续通过 `depth_image_proc/register_node` 配准后扩展，不是本版默认输出。红外坐标系存在于官方模型中，但不生成 IR 图像。

光学坐标约定：Z 朝前、X 向图像右、Y 向图像下。RGB 与 Depth 分辨率相同不代表像素一一对应。

## 7. 数据录制与导出

每次启动默认生成 `recordings/YYYYMMDD_HHMMSS_microseconds/`。记录 RGB、原始 Depth、两路规范化 CameraInfo、TF 和 `/clock`。静态测试没有动态 `/tf` 消息是正常现象；`/tf_static` 使用 transient-local QoS 保存已有变换。

图像录制使用可靠 QoS。XYZ 点云不重复录制，可以根据原始深度和内参恢复。原始图像不压缩，640×360、约30Hz的 RGB+float 深度大约占用 **3 GB/分钟**，长时间运行前请检查磁盘空间。

文件结构：

```text
run_config.yaml        # 参数、版本、机位、桥梁模型哈希
qos.yaml               # 录制和回放 QoS
record_status.json     # 录制器退出码
bag/                   # SQLite3 原始消息和 metadata.yaml
rgb.mp4                # export.sh 生成
depth_preview.mp4      # export.sh 生成，固定0.45～3m色标
frame_timestamps.csv   # 每个原始图像的消息时间戳和bag时间戳
export_report.json     # 原始帧数、输出视频帧数及时间范围
```

视频在停止录制后导出，**不是实时编码**。现成 rosbag2 保存完整原始数据，小型离线导出工具调用 CvBridge 与 FFmpeg，不开发编码器。视频按图像消息的仿真时间戳重采样到30fps，缺帧时保持上一帧；原始数据和原始时间戳不被改写。由于传感器实际周期约33ms（约30.303Hz），视频帧数略少于原始图像帧数是正常重采样，不表示原始 bag 丢帧。

两路视频各从自身第一帧起始；精确跨流同步使用 CSV 时间戳或 bag，不用播放器起点推断。仿真重置会造成时间回退，应结束当前录制并创建新会话；导出工具会拒绝非单调图像时间戳。

深度预览视频只是可视化，不保留可用于测量的深度精度。请始终使用 bag 中的原始深度做距离分析。

## 8. 验证

`validation/calibration/report.json`：2 m 平面，中心深度 2.000 m，点云中心 `[0,0,2]`，光学轴方向、独立内参、TF 通过。

`validation/bridge_live/report.json`：开启 Gazebo GUI、RViz2、录制时连续约60秒。RGB 与深度均为640×360，接收频率约30.25～30.26Hz；按仿真时间约30.303Hz，最大帧间隔33ms，无重复/回退时间戳。默认机位有效深度像素约96.9%，其余包含超距或无表面区域。

点云预览采用标准 `depth_image_proc` 和传感器 QoS，本次观测约21Hz，不保证每个原始深度帧都在可视化管线中显示。原始图像录制独立于该预览，保留约30Hz数据。

`validation/replay/report.json`：关闭 Gazebo 后，独立回放已保存的 bag，通过 RGB/Depth、内参、点云和 TF 检查。回放截图见 `validation/replay/rviz.png`，RViz Global Status 为 OK；回放点云观测约17Hz，两路原始图像约30Hz。

示例录制 `recordings/20260916_121854_346949`：约90.9秒，2755帧RGB、2754帧深度、静态TF齐全；两路MP4均为H.264、640×360、30fps。原始图像时间戳和导出统计在录制目录中。

重新运行验证（需已启动仿真，在第二终端）：

```bash
cd /home/lv/visual_camera_simulate/d415_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run bridge_d415_sim validate.py --seconds 60 --output validation/manual
```

标定世界可加 `--plane-distance 2`。脚本保存 JSON、RGB PNG 和原始深度 NPY；当前尺寸检查按本项目已约定的640×360验收。

## 9. 构建与后续接入小车

重建命令：`./build.sh`。只构建 Locus 的两个必要包及本项目，不构建无关的 Orbbec 包。

可复用相机宏在 `urdf/d415_module.xacro`，接口是 `bridge_d415(parent, cfg, origin)`。该宏不设置 static。当前 `test_camera.urdf.xacro` 单独将测试支架设为静态。

接入小车时使用车体安装 link 作为 parent，提供安装 origin；移除测试支架及它的静态 world TF，由小车运动与 robot_state_publisher 提供完整 TF。保留桥接、CameraInfo 适配、显示和录制。当前惯性近似需在整车质量方案中注明，避免重复计入相机质量。

## 10. 常见情况

- `gazebo` 打开的仍是 Classic；本工程通过 `ign gazebo` 启动 Fortress。
- 不要同时启动多份 `run.sh`：同名话题、相机和仿真时钟会冲突。
- `No image`：先看终端是否已输出相机生成成功、桥接成功及 `RECORDING`；RViz Topic QoS 使用 Best Effort。
- Ogre2/EGL、Qt 的部分警告不会阻止本机渲染，是否有效以收到图像及报告为准。
- 测试支架根 link 的惯性会触发 KDL 根惯性警告；robot_state_publisher 不计算动力学，该警告不影响相机 TF，Gazebo 仍读取模型惯性。
- Ctrl+C 时上游进程可能显示 `exit code -2` 或 Ruby 的退出信号日志；录制是否成功以 `RECORDING CLOSED`、`record_status.json` 和 bag 的 `metadata.yaml` 为准。
- 新建录制目录内只有 bag 而没有 MP4：先停止录制，再运行 `./export.sh`。
