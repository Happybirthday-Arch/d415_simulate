# 小车、3 cm 相机辅助杆与钢桥联合仿真

ROS 2 Humble + Gazebo Sim Fortress。一个世界中运行已有磁吸小车、钢桥和 D415，相机通过 **30 mm 辅助杆**固定到车身横板，默认仅保存 RGB 视频。深度图与点云持续实时输出。

## 启动

在工作区执行：

```bash
cd /home/lv/visual_camera_simulate/d415_sim_ws
./build_robot.sh
./run_robot.sh
```

构建完成后再次运行只需 `./run_robot.sh`。启动会打开 Gazebo、RViz2 和“小车 WASD 控制”窗口。点击控制窗口，使它获得键盘焦点，然后操作。

| 按键 | 动作 |
|---|---|
| W / S | 按住前进 / 倒车，直行目标轮速 15 rpm，约 4.95 cm/s |
| A / D | 前轮左打 / 右打，目标 ±14°；单独按下不驱动车轮 |
| W+A、W+D、S+A、S+D | 组合行驶和转向 |
| 松开 W/S | 减速停车；松开 A/D 后回中 |
| W+S / A+D | 对应方向输入抵消 |
| Space | 制动锁定 |
| Enter | 松开 WASD 后解除锁定，不自动起步 |
| 窗口失焦或关闭 | 停车；关闭控制窗口同时结束联合仿真 |

A/D 是前轮相对车身的方向，倒车时车头偏航方向与前进时相反。暂停或通信失联后先松开 WASD，再重新按下；控制器要求收到中立命令后才接受运动。控制命令以墙钟 50 Hz 发送，0.25 s 无有效命令进入停车状态。

在启动终端按一次 `Ctrl+C`，等待 `RGB RECORDING CLOSED` 和命令提示符返回，让视频完成收尾。Gazebo/关键节点退出也会结束本次联合启动。

## 录制选项

```bash
./run_robot.sh record_mode:=rgb       # 默认：RGB H.264 MP4
./run_robot.sh record_mode:=off       # 仅查看，不保存
./run_robot.sh record_mode:=full      # 明确启用完整 rosbag2
./run_robot.sh world:=robot_bench     # 独立平钢板，便于检查小车
```

三个模式均运行 RGB、Depth、Depth preview 和 XYZ 点云。`rgb` 模式不启动 rosbag2、不保存深度、点云、CameraInfo 或 TF；订阅 RGB 编码，并只读 `/clock` 识别世界时间重置（不保存时钟流），产物在 `recordings_robot/<时间>_rgb/`：

- `rgb.mp4`：640×360、30 fps、H.264，直接用视频播放器打开。
- `rgb_timestamps.csv`：很小的帧索引，保留仿真时间和补帧标记。
- `recording.json` / `rgb_encoder.log`：编码统计和错误信息。

视频以仿真时间为依据；墙钟运行缓慢不会人为拖慢回看速度。丢失的仿真帧以此前帧补齐；暂停不按墙钟重复写帧。世界时间回退、分辨率变化或长时间数据缺口会生成新视频片段。Fortress 在世界重置后可能暂时停止传感器输出，直到新仿真时间追上重置前的下一帧时刻；需要立即重新开始成像时，退出后重新启动即可。队列容量有限，过载丢帧会计入报告。2 Mbit/s 是目标码率，约 15 MB/min 的预算，静态画面实际通常更小。

`full` 模式保存原始 RGB、Depth、两路 CameraInfo、TF、时钟、实际关节状态、驾驶命令和真值位姿，并保存装配配置；不会同时生成 RGB 视频。可以使用原相机工作区的 `./replay.sh <录制目录>` 回放。只有 RGB 视频时不能恢复深度或 ROS 轨迹。

```bash
./run_robot.sh gui:=false rviz:=false keyboard:=false record_mode:=off
```

无窗口测试仍需可用的 Ogre2 渲染环境。`keyboard:=false` 时默认保持停车，供测试节点发送命令。不同实时仿真/回放不要共用 ROS_DOMAIN_ID；脚本默认 Gazebo 分区为 `bridge_robot_camera`。

## 装配与控制配置

`config/assembly.yaml`：辅助杆长 0.030 m，暂用直径 0.010 m 的实心圆杆和等效铝材质量；杆底位置 `[0.04213,0,0.057] m`，相机底部安装点 `[0.04213,0,0.087] m`，外壳前端 X=0.052 m，与横板前缘齐平。

相机和杆的显示、碰撞、传感器和惯量合入 `base_link`，等效固定连接；保留辅助杆及相机的完整静态 TF。总质量维持此前含附件的 2.70 kg 预算，相机 0.072 kg、杆约 0.00636 kg 从原等效车架质量中划出。原小车源模型保持独立，联合模型每次启动写到 `runtime/robot_*/`。

`config/drive.yaml`：15 rpm、轮速斜坡、转向角、力矩限幅和 PID 参数。轮轴采用有限力矩速度闭环，前轮转向采用有限力矩位置闭环；前后轮速根据 61 mm 后轴到转向轴距离、9 mm 前轮偏置补偿。带载转向初测表明 0.5 N·m 不足，因此转向限幅调为 5 N·m，仍保留 ±14.9° 物理停止和 ±15° 独立监测。

`config/recording.yaml`：视频帧率、目标码率、队列容量。安装及控制配置在下次启动时重新读取，无需手改生成的 SDF。相机成像配置仍来自 `bridge_d415_sim/config/camera.yaml`；其中原独立测试机位的世界 xyz/rpy 不用于车载安装。

## 接口和坐标

- `/robot/drive_command`：`geometry_msgs/Vector3`；x 为油门 −1～1，y 为转向 −1～1，z>0.5 为制动。此消息表达驾驶输入，不是空间方向或 Twist。
- `/robot/drive_status`：x 为当前前轮目标 rpm，y 为目标转角（度），z=1 表示禁用运动输入。
- `/joint_states`：真实关节位置与速度，单位 rad、rad/s。
- `/robot/ground_truth`：Gazebo 实际车体位姿，仿真真值，不是定位算法结果。
- `/tf`、`/tf_static`：world → base_link → 关节/辅助杆 → 相机及光学坐标。
- `/d415/color/image_raw`、`/d415/depth/image_raw`、两路 CameraInfo、`/d415/depth/preview`、`/d415/depth/points`：沿用既有相机接口。

深度范围仍为 0.45～3 m，近处钢板可能无有效深度。点云为 XYZ，不含 RGB 配准。相机及磁吸均是当前项目的等效仿真模型，跨棱边动作未纳入本次控制功能。

## 验证

复现入口（测试独立选择通信域并结束自己启动的进程）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/bridge_robot_sim/scripts/validate_attachment.py
python3 src/bridge_robot_sim/scripts/validate_integration.py --world robot_bench
python3 src/bridge_robot_sim/scripts/validate_integration.py --world bridge_robot --smoke
```

结果位于 `../gazebo_robot/validation/integration/`。测试期间为了检查成像，验收脚本会单独保存一张 RGB 和一个深度快照；这是测试证据，不是 `rgb` 录制器的产物。常规 `run_robot.sh record_mode:=rgb` 不保存深度数据。


已通过：装配几何、15 rpm 正反向、左右转角、松键/失联/制动停车、图像时刻 TF、桥内移动 RGB/Depth、仅 RGB 视频编码、可选 full 录制、键盘组合键与失焦行为，以及三类表面各 61 s 驻车。汇总见 [验证报告](../../../gazebo_robot/validation/integration/summary.json)。这些结果针对测试位置和参数，不代表任意跨棱边或全桥路径均已验证。
