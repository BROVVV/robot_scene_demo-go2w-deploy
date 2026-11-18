# GO2-W 自主搜索接入 Nav2、走廊居中与宽度自适应——一步到位修改计划

> 本文是交给代码修改 AI 的直接执行说明，不是讨论稿。
>
> 执行本文时，一次性完成代码、配置、测试和文档修改；不要启动机器狗，不要发送任何真实运动指令。最终默认保持“只规划、不执行”，待人工完成现场验收后再显式开启执行。

## 1. 最终目标

把 GO2-W 的自主搜索运动链路改成唯一的一条 Nav2 链路：

```text
语义搜索选择探索目标
  -> 在地图中生成安全的停靠位姿
  -> Nav2 ComputePathToPose 验证路径
  -> 走廊宽度检测与路径安全判定
  -> Nav2 NavigateToPose
  -> 速度平滑
  -> Collision Monitor
  -> 控制权仲裁
  -> GO2-W 速度桥
```

改造完成后必须满足：

1. 自主搜索不再使用 `RELATIVE_MOVE`、分段向前走、朝目标方位直接走等旧逻辑。
2. 自主搜索的真实机器人后端只允许 `nav2`，不存在失败后偷偷切回 `go2w_experimental` 的降级路径。
3. Nav2 使用完整占据栅格地图规划，不再只依赖相机眼前的一小块信息。
4. 在宽走廊里优先靠近中线走；在窄走廊里自动降低期望侧向间距和速度；窄到机身无法安全通过时拒绝执行。
5. 不在运行时频繁修改 Nav2 的 `inflation_radius`、机器人 footprint 或硬安全距离。
6. 相机只负责语义识别和目标观察，不再承担最后一道防撞职责。
7. 所有真实运动默认关闭。代码修改 AI 只完成实现、构建和离线测试，不执行现场运动测试。

## 2. 当前问题结论

当前自主搜索中 Nav2 实际没有接管机器人运动，依据如下：

- `scripts/go2w/start_autonomous_search_web.sh` 明确不启动 Nav2。
- `scripts/go2w/autonomous_search_worker.py` 默认使用 `go2w_experimental`。
- `app/navigation/go2w_experimental_backend.py` 本身不提供完整的全局地图规划和绕障能力。
- `configs/go2w/navigation_gate.yaml` 中 `nav2_allow_execute` 默认为 `false`。
- `robot_scene_nav2.launch.py` 的执行默认关闭。
- 现有搜索会生成 `RELATIVE_MOVE`，再拆成短距离前进；这不是 Nav2 的 `NavigateToPose`。
- 现有 `/go2w/slam/map_2d`、`/go2w/odom/fused` 与 Nav2 所需的 `map -> odom -> base_link` 坐标链没有被统一。
- `plain_slam_map_relay.py` 仅换了话题名，没有把地图消息的 `frame_id` 正确统一为 Nav2 的 `map`。

因此，本次不是“微调 Nav2 参数”，而是先把自主搜索真正接到 Nav2，再做居中和窄走廊适配。

## 3. 强制开发约束

### 3.1 禁止的做法

严格遵守用户要求，不做防御性编程和工程化扩张：

- 不新建抽象工厂、插件注册器、后端注册表或多层适配器。
- 不为未来可能出现的机器人、地图或导航器设计通用框架。
- 不保留真实搜索中的双后端、自动回退和兼容分支。
- 不用宽泛的 `try/except Exception` 吞掉错误。
- 不用错误后继续向前走的“保底动作”。
- 不添加无限重试、后台守护、自恢复状态机或复杂缓存。
- 不复制第二份 Nav2 配置。
- 不借机重构与本任务无关的模块。
- 不修改用户已有、但与本文无关的工作区改动。
- 不留下 `TODO`、空实现或伪接口。

### 3.2 必须保留的安全拒绝

以下行为是机器人运动的基本安全条件，不属于应删除的“防御性编程”：

- 没有地图、TF、激光雷达或有效定位时，禁止发出运动指令。
- 路径不存在、走廊过窄、目标落在障碍区时，拒绝该目标。
- Nav2 失败后停止并重新选择目标，禁止切换为相对前进。
- 控制源不是 `nav2` 时，Nav2 速度不得送到机器人。
- 急停、租约或运动授权无效时必须输出零速度。

这些检查必须“失败即停止”，不能静默绕过。

## 4. 本次修改边界

### 4.1 要修改

- 自主搜索后端选择和目标执行方式。
- Nav2 与现有搜索进程的直接 Action 接口。
- 地图、里程计和 TF 坐标链。
- Nav2 costmap、规划器、控制器、速度平滑及 Collision Monitor 配置。
- 走廊宽度识别、目标点安全间距和速度分级。
- 启动脚本、执行闸门、离线测试和使用说明。

### 4.2 不修改

- 相机模型、语义识别模型及其提示词。
- 手动遥控功能。
- GO2-W 底层 SDK 的通信协议。
- 与自主搜索无关的 Web 页面功能。
- 不为了本任务替换 SLAM 算法。

### 4.3 历史后端处理方式

`go2w_experimental` 文件可以暂时留在仓库，避免破坏独立诊断脚本；但它必须从自主搜索的生产入口、默认配置和可选参数中移除。真实自主搜索只能选择 `nav2`。测试环境允许使用 `mock`，但 `mock` 不能出现在真实启动脚本的默认路径中。

## 5. 固定的话题、坐标系和控制链

整个项目统一使用以下名称，禁止再增加别名：

| 用途 | 固定值 |
|---|---|
| 全局坐标系 | `map` |
| 局部里程计坐标系 | `odom_fused` |
| 机器人基座坐标系 | `base_link` |
| Nav2 地图话题 | `/map` |
| 运动权威里程计 | `/go2w/odom/fused` |
| Plain SLAM 位姿 | `/go2w/slam/odom_base` |
| Plain SLAM 地图 | `/go2w/slam/map_2d` |
| 激光扫描 | `/go2w/lidar/scan` |
| 定位可用标志 | `/go2w/nav/localization_valid` |
| Nav2 原始速度 | `/cmd_vel_nav_raw` |
| 平滑后速度 | `/cmd_vel_nav_smoothed` |
| Collision Monitor 输出 | `/go2w/nav2_cmd_vel` |
| 最终控制源 | `/go2w/control_source`，值必须为 `nav2` |

唯一允许的 TF 主链：

```text
map -> odom_fused -> base_link
```

不得同时存在第二个节点发布同一对 TF。Plain SLAM 原坐标只作为计算输入，不能直接成为 Nav2 的运动控制里程计。

## 6. 具体代码修改

以下文件名和职责已经确定。除非仓库的实际安装规则要求微调路径，不要自行设计另一套结构。

### 6.1 新增 Nav2 搜索后端

新增：

```text
app/navigation/nav2_search_backend.py
```

直接实现项目现有 `RobotBackend` 所需的最小接口，不增加新抽象层。后端构造函数接收搜索进程已经存在的 ROS 2 node 和 TF buffer，并创建：

- `ComputePathToPose` ActionClient；
- `NavigateToPose` ActionClient。

该后端只接受：

- `NAVIGATE_POSE`；
- `STOP`；
- `CANCEL`。

收到 `RELATIVE_MOVE` 必须明确返回“不支持”，不得转换为底层前进命令。

后端职责：

1. 将搜索动作中的 `x`、`y`、`yaw` 组装为 `map` 坐标系的 `PoseStamped`。
2. 先调用 `ComputePathToPose`，确认路径非空。
3. 调用本文第 7 节的走廊分析函数；不安全则不发送导航目标。
4. 安全时再调用 `NavigateToPose`。
5. 将 Action 状态映射为项目已有的 `PENDING/RUNNING/SUCCEEDED/FAILED/CANCELED`。
6. `cancel()` 必须取消当前 Nav2 goal；`stop()` 必须取消 goal 并通过已有控制链输出停止。
7. `get_pose()` 从 `map -> base_link` TF 读取当前二维位姿。

禁止把 `scripts/nav2_bridge_worker.py` 的文件 IPC 搬进自主搜索。该脚本继续只服务现有手动 Nav2 面板；自主搜索通过 ROS 2 Action 直接连接 Nav2。

### 6.2 新增 Nav2 坐标桥

新增：

```text
ros2_ws/src/robot_scene_nav_bringup/scripts/plain_slam_nav_frame_bridge.py
```

并在对应 `CMakeLists.txt` 中安装该脚本。

节点订阅：

- `/go2w/slam/map_2d`；
- `/go2w/slam/odom_base`；
- `/go2w/odom/fused`。

节点发布：

- `/map`，内容沿用 Plain SLAM 地图，但 `header.frame_id` 必须改为 `map`；
- TF `map -> odom_fused`；
- TF `odom_fused -> base_link`；
- `/go2w/nav/localization_valid`。

二维变换计算必须直接按下式实现：

```text
T_map_odom = T_map_base_from_plain_slam * inverse(T_odom_base_from_fused_odom)
```

只使用 `x`、`y` 和 `yaw`，不要引入三维定位框架。四元数只用于 ROS 消息转换。

`localization_valid=true` 的固定条件：

- 已收到地图；
- 两路 odometry 都已收到；
- 两路 odometry 数据均不超过 0.5 秒；
- 用于计算的两帧时间差不超过 0.1 秒；
- 位姿数值均为有限数；
- TF 已成功发布。

任一条件不满足就发布 `false`，不得沿用上一次的 `true`。

现有 `plain_slam_map_relay.py` 不再由 Nav2 launch 启动，避免 `/map` 多发布者。不要让 `sport_odom_bridge.py` 或 wheel odom 节点再发布竞争性的 `odom_fused -> base_link` TF。

### 6.3 修改 Nav2 launch

修改：

```text
ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py
```

要求：

1. 启动 `plain_slam_nav_frame_bridge.py`，不启动旧地图 relay。
2. Nav2 节点固定使用 `map/odom_fused/base_link`。
3. 启动 Planner Server、Controller Server、BT Navigator、Behavior Server、Lifecycle Manager、Velocity Smoother 和 Collision Monitor。
4. 明确完成速度话题 remap：

```text
controller_server /cmd_vel
  -> /cmd_vel_nav_raw
velocity_smoother input
  -> /cmd_vel_nav_raw
velocity_smoother output
  -> /cmd_vel_nav_smoothed
collision_monitor input
  -> /cmd_vel_nav_smoothed
collision_monitor output
  -> /go2w/nav2_cmd_vel
```

5. `/go2w/nav2_cmd_vel` 只能有一个发布者。
6. `execution_enabled` 仍默认 `false`。只规划模式可以启动地图、TF 和规划器，但不允许控制速度进入机器人。

### 6.4 修改 Nav2 参数

修改：

```text
ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml
ros2_ws/src/robot_scene_nav_bringup/config/velocity_smoother.yaml
ros2_ws/src/robot_scene_nav_bringup/config/collision_monitor.yaml
```

统一参数：

```yaml
global_frame: map
robot_base_frame: base_link
odom_frame: odom_fused
odom_topic: /go2w/odom/fused
scan_topic: /go2w/lidar/scan
```

机器人 footprint 采用当前项目中的实际矩形尺寸：

```yaml
footprint: "[[0.35, 0.215], [0.35, -0.215], [-0.35, -0.215], [-0.35, 0.215]]"
footprint_padding: 0.04
```

Global Costmap 和 Local Costmap 都启用障碍层与膨胀层。膨胀参数固定为：

```yaml
inflation_radius: 1.00
cost_scaling_factor: 2.0
```

说明：这不是要求机器人与墙永远保持 1 米。膨胀区是有梯度的软代价，会让宽走廊中线的代价最低；窄走廊仍可在满足 footprint 和硬安全距离时通过。

规划器继续使用 Navfn A*。必须明确：

```yaml
use_astar: true
allow_unknown: false
```

控制器继续使用 Regulated Pure Pursuit，并设置保守初值：

```yaml
desired_linear_vel: 0.10
lookahead_dist: 0.30
min_lookahead_dist: 0.20
max_lookahead_dist: 0.45
use_velocity_scaled_lookahead_dist: true
use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 1.5
use_cost_regulated_linear_velocity_scaling: true
cost_scaling_dist: 0.80
cost_scaling_gain: 0.50
regulated_linear_scaling_min_speed: 0.05
```

按当前 Nav2 Humble 插件的实际参数名填写，不要同时留下相同含义的旧参数。

配置一个简单的 progress checker，避免贴墙后无限原地耗着：

```yaml
required_movement_radius: 0.10
movement_time_allowance: 8.0
```

Velocity Smoother 的上限必须与项目现有速度桥一致：

```yaml
smoothing_frequency: 20.0
max_velocity: [0.10, 0.0, 0.50]
min_velocity: [-0.05, 0.0, -0.50]
max_accel: [0.20, 0.0, 0.80]
max_decel: [-0.25, 0.0, -1.00]
```

Collision Monitor 必须保留停止区和减速区，并基于 GO2-W 实际 footprint 设为：

- 停止区：前方 `0.60 m`、后方 `0.40 m`、左右各 `0.35 m`；
- 减速区：前方 `1.00 m`、后方 `0.60 m`、左右各 `0.60 m`；
- 停止区触发时输出零速度；
- 减速区触发时速度比例为 `0.35`。

如果配置格式要求多边形顶点，按上述边界直接生成矩形顶点，不自行增加第三个区域。

### 6.5 修改速度桥的定位闸门

修改现有 cmd_vel bridge 的节点和对应核心逻辑：

- 删除 Nav2 执行对 `/lio/valid` 的依赖；
- 改为订阅 `/go2w/nav/localization_valid`；
- 变量名统一为 `navigation_localization_fresh`；
- 只有以下条件同时成立时才接收 `/go2w/nav2_cmd_vel`：
  - 显式运动授权有效；
  - 急停未触发；
  - 控制租约有效；
  - 激光雷达新鲜；
  - `navigation_localization_fresh=true`；
  - `/go2w/control_source` 为 `nav2`。

不满足时立即输出零速度并给出一个明确的拒绝原因，不继续执行旧命令。

### 6.6 修改自主搜索后端选择

修改：

```text
scripts/go2w/autonomous_search_worker.py
scripts/go2w/run_semantic_exploration.py
app/navigation/config.py
与搜索请求模型相关的文件
```

要求：

1. 真实运行的默认后端改为 `nav2`。
2. 生产搜索请求只接受 `nav2`；测试代码可直接注入 `mock`，不通过生产请求参数暴露。
3. 删除自主搜索入口对 `go2w_experimental` 的默认值和回退。
4. 初始化 `Nav2SearchBackend` 时传入搜索进程已有 ROS node 和 TF buffer。
5. 如果 Nav2 action server、地图、TF 或定位无效，搜索任务进入“等待导航就绪”或明确失败，绝不退回相对前进。

### 6.7 替换搜索中的相对移动

重点修改 `scripts/go2w/run_semantic_exploration.py` 中 spatial V2 candidate generator 和动作执行分支。

真实 Nav2 模式下只允许产生：

- `NAVIGATE_POSE`；
- `STOP`；
- 无平移的观察动作，但观察转向也应表示为当前位置的 `NAVIGATE_POSE(x, y, target_yaw)`。

必须删除或绕过真实运行中的：

- `TopologyRouteExecutor` 产生的相对运动；
- `LocalGoalExecutor` 的 `RELATIVE_MOVE` 分段执行；
- 路径不可达时按 bearing 直接前进；
- 距离较远时拆成多个 `0.3 m` 前进段；
- 前方扇区被标成 no-return 后仍判断 `forward_safe=true` 的逻辑。

目标物搜索和接近目标都使用地图中的 `NAVIGATE_POSE`。Nav2 路径不可用时，本轮候选失败，搜索器改选其他候选或停止观察，不能自己向前探。

## 7. 走廊宽度自动识别与适配

### 7.1 原则

可以自动识别宽度并适配，但适配的是：

- 这个候选路径是否允许通过；
- 目标点在可行空间中的评分；
- 该段路径允许的最高线速度；

不动态修改：

- 机器人真实 footprint；
- `inflation_radius`；
- Collision Monitor 停止区；
- 硬安全间距。

这样可避免导航过程中 costmap 规则突然改变而导致路径抖动。

### 7.2 新增一个简单的走廊分析模块

新增：

```text
app/navigation/corridor_analyzer.py
```

只实现当前项目需要的三个函数，不设计类继承体系：

```python
measure_path_clearance(path, occupancy_grid) -> list[PathClearanceSample]
classify_path_clearance(samples) -> CorridorDecision
speed_limit_from_clearance(min_side_gap_m) -> float
```

数据类只保留实际使用字段：路径位置、左右到障碍距离、走廊宽度、机身左右净空、判定结果和速度上限。

### 7.3 宽度测量算法

对 `ComputePathToPose` 返回路径按每 `0.20 m` 采样一次：

1. 用相邻路径点计算当前切线方向。
2. 在切线法向的左右两侧，从路径中心按地图分辨率逐格射线搜索。
3. 遇到占据栅格值 `>= 65` 即认为遇到墙或障碍物。
4. 未知栅格按障碍物处理。
5. 左、右最大搜索距离均为 `2.0 m`。
6. 得到：

```text
corridor_width = left_distance + right_distance
left_body_gap = left_distance - 0.215 - 0.04
right_body_gap = right_distance - 0.215 - 0.04
min_side_gap = min(left_body_gap, right_body_gap)
```

其中 `0.215 m` 是半车宽，`0.04 m` 是 footprint padding。

路径前后端各忽略 `0.20 m`，避免起点和目标点附近的几何噪声干扰全路判定。除此之外不要增加滤波器。

### 7.4 固定安全规则

沿整条路径采用最小 `min_side_gap` 判定：

| 最小单侧机身净空 | 处理 |
|---|---|
| `< 0.40 m` | 路径拒绝，原因 `TOO_NARROW` |
| `0.40–0.80 m` | 允许，按净空线性降低速度 |
| `>= 0.80 m` | 允许，最高 `0.10 m/s` |

窄走廊速度按下面的唯一公式计算：

```text
speed = 0.05 + (min_side_gap - 0.40) / 0.40 * 0.05
speed = clamp(speed, 0.05, 0.10)
```

路径速度上限传给控制链的方式只选一种：在每次导航目标开始前设置 RPP 的 `desired_linear_vel`，目标结束后保持最后值，不做持续参数抖动。新目标开始时根据新路径重新设置一次。

### 7.5 自动居中的实现

走廊居中主要由 Nav2 costmap 完成：左右墙体膨胀产生代价梯度，中线代价最低，A* 会偏向中线。

除此之外，在探索目标候选评分中加入对称净空分：

```text
center_error = abs(left_body_gap - right_body_gap)
center_score = max(0, 1 - center_error / max(corridor_width, 0.01))
```

候选总分中 `center_score` 权重固定为 `0.25`。它只影响多个安全候选之间的选择，不允许覆盖“路径过窄”判定。

不要通过“看到左墙就向右打方向”的局部规则实现居中；转向完全交给 Nav2 Controller。

## 8. 安全停靠位姿生成

继续复用现有：

```text
app/navigation/viewpoint_planner.py
app/navigation/clearance_map.py
app/navigation/candidate_goal_generator.py
app/navigation/target_pose_generator.py
```

不要另建第二套 viewpoint planner。

修改接线方式：

1. 从最新占据栅格中生成障碍点/净空查询数据。
2. 对 frontier 或语义目标生成多个带朝向的 standoff 候选。
3. 候选点必须位于已知自由空间，并满足机身 footprint 与硬净空。
4. 对每个候选调用 `ComputePathToPose`。
5. 过滤无路径和 `TOO_NARROW` 候选。
6. 按探索收益、路径长度、目标观察角度、局部净空、`center_score` 选最佳候选。
7. 最终只发送最佳候选的 `NAVIGATE_POSE`。

目标不是 frontier 的质心，也不是障碍物本身，而是“机器人能安全停下并让相机看向目标区域”的位姿。

固定净空值：

```text
hard_clearance_m = 0.40
preferred_clearance_m = 0.80
```

如果局部环境容不下 `preferred_clearance_m`，允许降到 `hard_clearance_m`；低于硬值就拒绝。该自适应由候选筛选完成，不修改全局 Nav2 footprint。

## 9. 启动方式与默认不运动

新增：

```text
scripts/go2w/start_nav2_autonomous_search.sh
```

该脚本是新的唯一 Nav2 自主搜索入口，职责按顺序固定为：

1. 启动/检查相机、激光雷达、运动里程计和 Plain SLAM。
2. 启动 Nav2 bringup，默认 `execution_enabled:=false`。
3. 启动 frame bridge、速度平滑、Collision Monitor、控制仲裁和 cmd_vel bridge。
4. 等待 `/map`、TF、`/go2w/nav/localization_valid`、Nav2 lifecycle active。
5. 启动自主搜索 worker，后端固定为 `nav2`。
6. 默认只允许 `ComputePathToPose`，不允许 `NavigateToPose` 的速度进入机器人。

只保留一个显式开关：

```text
--enable-motion
```

没有这个参数时必须是只规划模式。传入该参数也仍需现有的人工运动确认/急停/租约闸门全部通过。禁止通过环境变量或另一个隐藏参数绕过。

现有 `start_autonomous_search_web.sh` 顶部打印明确提示：它是旧搜索入口，不能用于 Nav2 自主运动；如果项目的 Web UI 已统一依赖它，则将它改为直接调用新脚本，不再自己维护第二套启动流程。

## 10. Navigation Gate 修改

修改：

```text
configs/go2w/navigation_gate.yaml
相关 gate 评估脚本
```

默认值仍为：

```yaml
nav2_allow_execute: false
```

Gate 在只规划模式至少检查：

- `/map` 持续更新；
- `map -> odom_fused -> base_link` TF 连通；
- `/go2w/nav/localization_valid=true`；
- 激光扫描新鲜；
- Nav2 lifecycle 全部 active；
- `ComputePathToPose` 可返回有效路径；
- 速度链话题只有预期发布者；
- `/go2w/control_source` 能正确切到 `nav2`，但不实际放行速度。

只有人工现场完成第 13 节验收后，用户才可以手动把 `nav2_allow_execute` 改为 `true`。代码修改 AI 不得代替用户做这一步。

## 11. 精确文件清单

### 11.1 新增

```text
app/navigation/nav2_search_backend.py
app/navigation/corridor_analyzer.py
ros2_ws/src/robot_scene_nav_bringup/scripts/plain_slam_nav_frame_bridge.py
scripts/go2w/start_nav2_autonomous_search.sh
tests/test_nav2_search_backend.py
tests/test_corridor_analyzer.py
tests/test_plain_slam_nav_frame_bridge.py
tests/test_nav2_autonomous_search_contract.py
```

### 11.2 修改

```text
scripts/go2w/autonomous_search_worker.py
scripts/go2w/run_semantic_exploration.py
scripts/go2w/start_autonomous_search_web.sh
app/navigation/config.py
自主搜索请求模型所在文件
cmd_vel bridge 节点与核心逻辑所在文件
configs/go2w/navigation_gate.yaml
navigation gate 评估脚本
ros2_ws/src/robot_scene_nav_bringup/CMakeLists.txt
ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py
ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml
ros2_ws/src/robot_scene_nav_bringup/config/velocity_smoother.yaml
ros2_ws/src/robot_scene_nav_bringup/config/collision_monitor.yaml
README.md 或现有 GO2-W 自主搜索说明文档
```

实际修改前用 `rg` 找到“自主搜索请求模型”和“cmd_vel bridge”的准确文件，不要因为文件名与本文猜测不同而复制出新实现。

## 12. 必须新增或修改的测试

### 12.1 坐标桥测试

覆盖：

- 输入已知 `T_map_base` 和 `T_odom_base`，验证计算出的 `T_map_odom`。
- `/map` 的 frame 必须为 `map`，栅格数据、分辨率、原点不变。
- 数据过期、时间差过大、NaN 时 `localization_valid=false`。
- 正常同步数据时为 `true`。

### 12.2 Nav2 后端测试

用假的 Action server 覆盖：

- `NAVIGATE_POSE` 先规划后导航。
- 空路径不调用 NavigateToPose。
- `TOO_NARROW` 不调用 NavigateToPose。
- success、abort、cancel 状态映射。
- 收到 `RELATIVE_MOVE` 明确拒绝。
- 导航失败时不调用实验后端。

### 12.3 走廊分析测试

用人工构造占据栅格覆盖：

- 宽直走廊：测得左右距离近似相等，速度 `0.10 m/s`。
- 路径偏左：`center_score` 低于中线候选。
- 可通过的窄走廊：允许通过，速度在 `0.05–0.10 m/s`。
- 机身单侧净空小于 `0.40 m`：判为 `TOO_NARROW`。
- 弯道：用路径局部法向测量，不使用全局固定横轴。
- 未知栅格：按障碍处理。

### 12.4 搜索行为合同测试

必须断言：

- Nav2 模式的空间候选只包含 `NAVIGATE_POSE/STOP`。
- 搜索和目标接近都不生成 `RELATIVE_MOVE`。
- 无路径时不会按 bearing 向前走。
- 旋转观察被表达为当前位置、不同 yaw 的 `NAVIGATE_POSE`。
- 生产启动参数中不存在 `go2w_experimental` 选择。

### 12.5 配置合同测试

读取 YAML 和 launch 文件断言：

- frame 固定为 `map/odom_fused/base_link`。
- map、scan、odom 话题正确。
- inflation 参数与本文一致。
- 速度链顺序正确。
- `/go2w/nav2_cmd_vel` 只有 Collision Monitor 一个发布源。
- 默认 `execution_enabled=false` 且 `nav2_allow_execute=false`。

### 12.6 复用现有测试

确保以下已有测试继续通过，并根据新单链路行为更新断言：

```text
tests/test_go2w_clearance_viewpoint_e2e.py
tests/test_frontier_viewpoint_candidates.py
tests/test_viewpoint_planner.py
```

不要为了让测试通过而降低 `hard_clearance_m`。

## 13. 验证顺序

代码修改 AI 必须完成 13.1 和 13.2，然后停止；13.3 和 13.4 只能由现场人员完成。

### 13.1 离线验证

1. 对所有修改的 Python 文件运行语法检查。
2. 运行本文涉及的 pytest。
3. 运行受影响 ROS 2 package 的 `colcon build`。
4. 运行 package tests。
5. 用 `rg` 验证生产搜索路径没有 `RELATIVE_MOVE` 和 `go2w_experimental` 回退。
6. 检查 git diff，只包含本文范围和用户原有改动，不覆盖用户文件。

### 13.2 静止上电验证，只规划不运动

现场人员运行，但必须先在文档给出命令：

1. 不使用 `--enable-motion` 启动。
2. 检查 `/map` 的 `frame_id=map`。
3. 检查 TF 主链连续且没有重复发布者。
4. 静止 10 分钟，`map -> odom_fused` 不应发生无解释的大跳变。
5. 请求多个 `ComputePathToPose`，确认路径在障碍物膨胀区外并偏向走廊中间。
6. 确认 `/go2w/nav2_cmd_vel` 即使有规划也不会进入机器人。
7. 确认走廊分析日志包含宽度、最小侧向净空、判定和速度上限。

### 13.3 低速现场验收

由两人进行：一人监控，一人持急停。首次只用直走廊短目标。

> **执行前必读（2026-09-28 加，详见 README §9.1）：正前方软件不设防。**
> Nav2 的唯一观测源 `/go2w/lidar/scan` 在正前方一段连续扇区无回波：2026-09-28 实测
> 最长连续盲区 **137.0°（−68.0°..+69.0°）**、295/720 束（41%）从未返回。**该跨度会跳**
> （同日两次 133.0°/137.0°，更早 113.5°），按 **±70°** 记，别按窄值选站位。
> 因此 `/go2w/safety/front_clearance` 恒为 `inf`，而
> `evaluate_lidar_motion_readiness()` 只对 `None`/`NaN` 拒绝、对 `inf` 判为通过。
> **持急停的人是正前方唯一的保护**，站位要能直接看到机器人正前方。
> 下面那条"遇临时障碍时 Collision Monitor 能先减速、再停止"**不得用正前方的障碍去测**——
> 正前方测不出来，会得到一个假通过；要测就用侧前方的障碍。
> "机身到墙面不小于 0.40 m"须**用尺量**，不要读 `front_clearance`。
> 该缺口经现场拍板决定"照常验收、人工兜底"；补前向观测（如改接独立的 Hesai）属于另一条线，
> 不要混进本次验收。

通过条件：

- 机器人实际控制源始终显示 `nav2`。
- 不出现 `RELATIVE_MOVE` 日志。
- 宽走廊中路径中心相对中线横向误差不超过 `0.15 m`。
- 机器人机身到任一墙面的实测距离不小于 `0.40 m`。
- 遇临时障碍时 Collision Monitor 能先减速、再停止。
- 定位失效、激光过期或 TF 中断时立即停止。
- 相机不会因为贴墙而长期只看到墙面。

### 13.4 自主搜索验收

在至少一条宽走廊、一条可通行窄走廊和一个不可通行入口中分别测试：

- 宽走廊优先中线。
- 窄走廊自动降速，不修改 footprint 和 inflation 参数。
- 不可通行入口返回 `TOO_NARROW` 并改选其他目标。
- Nav2 失败后不会直接向前补走。
- 搜索过程中无碰墙、无卡死、无多控制源争抢。

只有全部通过后，才允许用户自行把执行 gate 改成 `true` 并在启动时使用 `--enable-motion`。

## 14. 日志要求

每次候选决策只打印一条结构化摘要，至少包含：

```text
candidate_id
goal_x
goal_y
goal_yaw
path_length_m
min_left_body_gap_m
min_right_body_gap_m
min_side_gap_m
corridor_decision
speed_limit_mps
center_score
accepted
reject_reason
```

拒绝原因只使用固定枚举：

```text
NO_MAP
NO_LOCALIZATION
NO_PATH
GOAL_OCCUPIED
TOO_NARROW
NAV2_NOT_READY
```

不要同时输出多份相同信息，也不要在高频控制循环里刷屏。

## 15. 给执行修改 AI 的工作步骤

必须按下面顺序连续完成，中途不要让用户选择方案：

1. 阅读本文列出的现有实现和测试。
2. 先检查 `git status`，记录并保护用户已有修改。
3. 用 `rg` 确认准确文件、类名、消息类型和 Nav2 Humble 参数名。
4. 实现 frame bridge 和对应单元测试。
5. 修改 Nav2 frame、costmap、速度链和 Collision Monitor 配置。
6. 实现 `corridor_analyzer.py` 和测试。
7. 实现 `nav2_search_backend.py` 和假 Action server 测试。
8. 把 semantic exploration 的真实动作全部切成 `NAVIGATE_POSE`。
9. 修改启动脚本、后端默认值、运动 gate 和 cmd_vel bridge。
10. 完成配置合同测试和行为合同测试。
11. 运行格式检查、pytest、colcon build/test。
12. 查看最终 diff，修复所有与本文验收条件不一致之处。
13. 更新运行说明，明确默认只规划以及现场验收步骤。
14. 向用户报告修改文件、测试结果和仍需现场人工完成的项目。

如果在实现时发现当前消息定义缺少 `NAVIGATE_POSE` 所需字段，只在现有动作模型中增加 `x/y/yaw/frame_id` 四个字段；不要新建第二套消息系统。

## 16. 完成定义

只有同时满足以下条件，才可以对用户说“修改完成”：

- 自主搜索真实后端唯一为 Nav2。
- 生产路径不存在相对前进和实验后端回退。
- `map -> odom_fused -> base_link` 唯一且连续。
- 地图 frame 被正确统一为 `map`。
- 走廊宽度可从地图和路径自动测量。
- 宽走廊居中、窄走廊降速、过窄拒绝的测试全部通过。
- Nav2 完整速度链和 Collision Monitor 接线正确。
- 默认只规划、不运动。
- 所有新增和受影响测试通过，ROS 2 package 构建通过。
- 未覆盖或删除用户已有的无关修改。

代码完成不等于真实机器人运动验收完成。未经第 13.3 和 13.4 节人工现场验证，不得声称已经证明“不会撞墙”。

