# Gazebo Sim 磁吸小车

联合相机与 WASD 控制入口已新增：`../d415_sim_ws/run_robot.sh`。相机通过 3 cm 辅助杆安装，默认仅保存 RGB 视频；见 [联合仿真说明](../d415_sim_ws/src/bridge_robot_sim/README.md)。以下仍说明独立小车物理模型。

已在本机 **Gazebo Sim Fortress 6.18.0 + DART** 中生成并验证。前后各一个磁吸轮，前轮组件可转向，整车质量 **2.70 kg**，有效吸附时每轮磁力 **200 N**。本阶段交付模型与物理行为，尚未接入正式运动控制器或 D415。

**启动**

在钢桥中查看小车：

```bash
cd /home/lv/visual_camera_simulate/gazebo_robot
./run.sh
```

在独立钢板上近距离查看：

```bash
./run.sh robot_bench
```

启动入口调用 `ign gazebo`，自动设置模型、插件和磁性网格资源路径；使用的是 Gazebo Sim。默认小车位于桥内底板中间通道，轮心参考位置约 `[0, -1.5, 0.0416] m`，车头朝世界 +Y。GUI 已配置小车近景，相机可以正常旋转、缩放。

在终端按 `Ctrl+C` 停止本次仿真。默认不施加驱动力，小车不会主动巡航。

无 GUI 的服务器运行示例：

```bash
./run.sh robot_bench -s --iterations 3000
```

其他场景可用同样方式启动：

| 世界名称 | 用途 |
|---|---|
| `adhesion_wall` | 竖直钢板 |
| `adhesion_ceiling` | 倒挂钢板 |
| `adhesion_slope` | 45° 斜钢板 |
| `bridge_ceiling` / `bridge_roof` | 实际桥梁顶板内侧 / 外侧 |
| `bridge_underside` | 实际桥梁底板下侧 |
| `bridge_left_wall` / `bridge_right_wall` | 实际桥梁左右内侧壁 |
| `bridge_panel` / `bridge_seam` | 实际正面板 / 板件接缝 |

这些正式场景中的轮轴均可自由滚动。侧壁和斜面上，磁吸仍有效，但自由轮可能沿面滚动；驻车测试使用的轮轴制动夹具仅存在于 `validation/physics/*/world.sdf`，没有加入正式小车模型。测试专用的 `steering_fixture`、`single_wheel_fixture`、`hole`、`bridge_hole`、`out_of_range`、`nonmagnetic` 不作为默认展示入口。

**模型与关节**

源模型来自 `../basic_model/robot_simplified/` 的六组 STL 和 JSON 数据。生成脚本统一将毫米转换为米，并把网格移入对应 link 的局部坐标。源 CAD、原始 STL、既有桥梁模型和相机工程均保持原样。

```text
base_link                            车架 + 后电机
├── rear_wheel_joint                 无有限角度限制的滚动关节
│   └── rear_wheel_link
└── front_steering_joint              前轮组件转向关节
    └── front_steering_link           转向机构 + 前电机
        └── front_wheel_joint        无有限角度限制的滚动关节
            └── front_wheel_link
```

车体坐标：X 朝前轮，Y 向左，Z 朝车架；对应原 CAD 的 `-X、+Z、+Y`。轮径 63 mm、宽 32.5 mm、轴距约 70 mm。轴承外圆拟合验证了转向轴约经过 CAD `X=5 mm、Z=5.886110 mm`，沿 CAD Y 方向；保留了约 9 mm 的转向轴与前轮轮心偏置。

两只电机外壳随所属支架运动，不随轮子自转。轴承按输入简化模型作为转向组的等效刚体，未恢复其内部滚子、内外圈等独立运动细节。

Fortress 的 DART 后端未实现 SDF `continuous` 类型，本工程使用无有限位置上下限的 `revolute` 表达轮轴连续转动，已验证轮子转过完整一圈以上。

**转向硬限位**

用户要求的外部边界固定为 **±15°**。SDF 的物理停止位置设为 **±14.9°**，保留 0.1° 数值裕量；该限位不依赖后续控制器。`SteeringLimitMonitor` 在每个物理步检查实际角度，若越过 ±15° 或出现非有限值，会报错并停止仿真，不进行事后角度裁剪。

已测试双向 0.5 N·m、初始角速度 ±2 rad/s、1 ms / 0.5 ms 步长，以及双向 5 N·m、初始角速度 ±20 rad/s、1 ms 步长。实测最大绝对角度约 **14.900001°**。后续控制接入时应继续在已验证的力矩、角速度和步长范围内使用。

对 −15°～+15° 每 0.1° 扫描原始外形及派生碰撞包络，关键分离部件没有发生碰撞；前后轮最小间隙约 **3.568 mm**，转向组与后轮约 **2.813 mm**。圆柱碰撞的解析外径与 STL 离散外径有微米级差异，不能将报告的最后几位当作制造公差。

**质量与惯量**

| 刚体 | 质量 |
|---|---:|
| `front_wheel_link` | 0.700000 kg |
| `rear_wheel_link` | 0.700000 kg |
| `base_link` | 约 0.796219 kg |
| `front_steering_link` | 约 0.503781 kg |
| 合计 | **2.700000 kg** |

每只电机计入 0.31 kg。剩余 0.68 kg 暂按车架与转向组的 CAD 体积比例分配，分别约 0.486219 kg 和 0.193781 kg；这是质量分布的建模假设。

轮子和结构件采用封闭网格体积积分估计质心及惯量，再按用户指定质量归一化。两只电机原始 STL 各有 12 条非流形边，惯量采用电机外部凸包的等效分布。刚性合并使用平行轴定理；四个 link 的惯量均通过正定性及主惯量三角关系检查。这些质心和惯量尚未经过实物标定。

**碰撞与磁吸**

显示层保留原零件外形。轮子碰撞体由三个同轴圆柱组成：两侧半径 31.5 mm、中央槽底半径 30 mm；轴向长度分别为 13.5、5、14 mm，前后轮按各自实际轴向放置。碰撞层省略轮轴孔和键槽，显示与轮子惯量保留对应几何。

车架和转向组分别按两个、三个连通实体生成凸包碰撞体；电机使用外部凸包。凸包会省略部件内部孔洞，作为接触近似。启用模型自碰撞，直接连接的相邻 link 采用 DART 的关节邻接过滤；关键非相邻部件另有完整转向扫描。

`MagneticAdhesion` 在每个物理步读取钢桥真实三角网格，使用 BVH 查找轮缘附近的有限表面。磁性对象由插件中的模型名与网格显式指定；默认是原钢桥的全部 15 组碰撞网格。网格生成来源及 SHA-256 记录在 `surfaces/bridge_manifest.json`。

- 每只轮子在两侧轮缘共取四个作用点，按有效点数分配总计 200 N 的载荷，方向指向当地钢面。
- 施力通过 `Link::AddWorldWrench` 完成，力矩按作用点相对 link 原点计算；没有固定车体、关闭重力或持续设置车体位姿。
- 查询方向取自非自转的车架/转向架，轮子连续自转不会让磁力方向绕圈。
- 捕获间隙为 0.5 mm，释放间隙为 1.0 mm；这些是理想磁吸模型的仿真参数。范围内保持标称吸力，脱离有效表面后归零。
- 通孔和板外区域没有虚拟无限平面。单轮有效时只施加该轮的 200 N，不把另一轮的额度转移过来。
- 同一平面上单轮合力为 200 N；若轮缘跨越两个略有夹角的局部表面，分力标量和仍为 200 N，向量合力略小。本次桥梁拱顶中心位置约为 199.961 N/轮。

该模型覆盖钢桥可接触平面上的附着与滚动。任意棱边翻越、跨孔和完整跨面动作仍需结合后续控制验证。本阶段没有引入电磁场、真实气隙衰减曲线或磁滞。

**配置与重新生成**

主要配置为 `config/robot.yaml`，包含质量、转向范围、磁吸力、间隙、摩擦、阻尼和步长。默认轮钢摩擦系数为 0.8，物理步长 1 ms。

```bash
./build.sh
```

`build.sh` 重新生成模型、派生网格、世界文件，编译插件并运行几何查询单元检查。模型和世界均由 `scripts/generate_model.py` 生成，修改配置后需要重新运行该入口；手工修改生成文件会在下次生成时被覆盖。

本机已具备依赖：CMake、C++17 编译器、`libignition-gazebo6-dev`、`libignition-plugin1-dev`、`libfcl-dev`，以及 Python 的 NumPy、SciPy、PyYAML。核心模型不依赖 ROS；后续可通过固定的 link/joint 名称接入控制和相机。

**已完成的验证**

主要汇总为 `validation/summary.json`；各实验的世界文件、服务器日志、逐步 CSV 和 JSON 报告位于 `validation/physics/<case>/`。

| 验证 | 结果 |
|---|---|
| 六组网格零位装配、轴承轴线、质量及惯量 | 通过 |
| 原始外形与凸包包络转向扫描 | 通过 |
| 双向限位及压力测试 | 最大约 14.900001°，未超过 15° |
| 水平、竖直、倒挂、45° 斜面 | 各 60 s 仿真时间，使用测试轮轴制动，吸附持续有效 |
| 真实桥梁底板、顶板两侧、底板下侧、左右内壁、正面板、接缝 | 代表位置各 3 s 接触与吸附通过 |
| 非磁面、有限钢板孔洞、真实桥梁通孔、超捕获距离 | 不产生错误磁吸 |
| 单轮靠近钢面 | 前轮 200 N、后轮 0 N |
| 外力拉离 | 从吸附状态切换为 0 N，释放通过 |
| 临时轮轴力矩滚动 | 五种场景通过；实际桥内 2 s 移动约 0.352 m，轮角与位移一致 |
| 世界重置 | 服务返回成功，重置后磁吸和限位恢复 |
| Gazebo GUI | 独立钢板与钢桥内显示通过，截图见 `validation/gui/` |

钢桥验证针对表中代表位置，不是对全桥每个三角面逐一跑动力学。原始非凸桥梁碰撞保留孔洞和接缝；复杂正面板及交接处的求解明显慢于简单钢板，当前不保证全桥实时因子达到 1。

复现几何和物理验收：

```bash
python3 scripts/validate_geometry.py
python3 scripts/validate_physics.py --suite full
python3 scripts/validate_reset.py
python3 scripts/summarize_validation.py
```

单独运行较短测试可选择 `--suite smoke`、`limits`、`stress`、`negative`、`single`、`release`、`rolling` 或 `bridge`。测试会覆盖同名实验目录；完整物理验收需要数分钟，桥梁复杂部位更慢。GUI 检查使用 `python3 scripts/validate_gui.py bridge_robot`，需要可用显示服务。

如需观察每一步磁吸力、间隙、车体位姿和关节角度：

```bash
MAGNETIC_ROBOT_DIAGNOSTICS="$PWD/validation/manual.csv" ./run.sh robot_bench
```

`steps.csv` 中无有效候选表面时，间隙字段以 `1.0` 作为哨兵值，并非实测 1 m；力字段为该步磁吸合力，接触数来自轮子接触传感器。重置会使仿真时间回退，分析时应按时间段分开。

**实现依据**

系统插件按 [Fortress System 生命周期](https://gazebosim.org/api/gazebo/6/createsystemplugins.html)组织；施力使用 [Fortress Link API](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1Link.html)。关节范围采用 [SDFormat joint 限位定义](https://sdformat.org/spec/1.7/joint/)。实施方案和质量约定保存在 `../docs/magnetic_robot_gazebo_sim_plan.md`。
