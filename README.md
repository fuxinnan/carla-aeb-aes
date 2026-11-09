# 基于 CARLA 的 AEB/AES 主动安全系统与闭环评测平台

本仓库是一套自动紧急制动（AEB）和自动紧急转向（AES）的闭环评测平台。它在固定时步上完成感知、威胁评估、制动/转向决策和运动积分，再把每一帧放进 CARLA 0.9.16 的 `Town10HD_Opt`，用同步模式推进世界并跟随自车镜头。日志、指标、调参、消融、难例挖掘和回归都从同一次运行的帧记录里读出，策略代码和评测代码分开。

时步是 0.05 s。坐标系与 CARLA 世界系一致：左手系，米。X 向前，Y 向右，Z 向上。航向单位是弧度，0 沿 +X，俯视顺时针为正。`steer` 无量纲，范围 -1 到 1，向右为正。

## 整条链路

一帧里面的顺序是固定的。本帧读到的是上一帧命令作用之后的运动，本帧新命令要到下一帧才改变车速和航向。

```text
场景脚本给出自车和目标的当前位姿
        │
        ▼
真值跟踪器生成航迹，可选遮挡模型丢掉被挡住的目标
        │
        ▼
威胁评估：匀速、航向不变，计算 TTC、最近间距、能否单靠制动避开
        │
        ▼
策略决策
  无干预 / 固定 TTC / 状态机 AEB / 状态机 + AES
        │
        ▼
协调器：只有刹不住而且存在可行避让轨迹时才进入 AES
        │
        ▼
附着圆限制纵横向加速度，执行器限制制动斜率和踏板
        │
        ▼
写入 frames.jsonl / events.jsonl
        │
        ▼
CARLA 同步世界：贴位姿、移动镜头、画状态框、world.tick()
        │
        ▼
汇总指标 → 调参与消融 → 难例挖掘 → 邻域加密 → 回归
```

CARLA 在这条链路里负责地图、同步时钟和画面。演员关闭物理，每帧用 `set_transform` 放到直道上的对应位置，因此窗口里的轨迹和日志里的帧是同一套数。碰撞判定使用平面包围盒重叠，写在事件里的检测器是 `oriented_box`。0.9.16 连续 `load_world` 会让客户端断言退出，所以同一场实验只加载一次地图，换场景时只销毁车辆、行人和传感器。

## 功能

### 感知和威胁

`perception.GroundTruthTracker` 把当前帧的目标真值收成航迹。`in_path` 表示目标压在自车车宽走廊上。目标整体已经落到后保险杠后面时，不算在路径上。

`perception.VisibilityModel` 默认关闭，遮挡比例为 0。打开后，更近的包围盒挡住视线时，远目标不会交给策略，并在 `Track.first_seen_s` 记下第一次看见的时间。遮挡场景用它对比“行人一直可见”和“先被旁车挡住”。

`threat.ThreatAssessor` 的预测假设是 `constant_velocity_no_steer`。跟车且同向、横向相对速度很小时，纵向追尾 TTC 是保险杠间距除以靠近速度。横穿和切入用双方包围盒在匀速、航向不变下的第一次重叠时刻。没有有效 TTC 时 `ttc_s` 留空，并写明原因，不填 0。

无效原因包括：`no_track`、`not_closing`、`target_behind`、`no_collision_course`、`passed_conflict`、`beyond_horizon`。`closing_speed_mps` 靠近为正。`brake_avoidable` 用当前速度和最大减速度 6 m/s² 判断：刹停距离 `v²/(2a)` 不超过到冲突点的前向距离时，认为单靠制动来得及。

### 四种策略

| 策略名 | 行为 |
| --- | --- |
| `no_intervention` | 油门、制动、转向都是 0，期望速度保持场景巡航速度 |
| `fixed_ttc` | TTC 低于 `ttc_brake_s` 时请求全力制动，否则为 0。相等不算低于。状态只有 `IDLE` 和 `FULL` |
| `state_machine` | `IDLE` → `WARN` → `PARTIAL` → `FULL` → `EXIT`。警告不制动。进入制动要连续确认，退出用更高的阈值，并受最短保持帧数约束 |
| `aeb_aes` | 状态机负责制动。刹不住且某一侧轨迹可行时，沿该轨迹做纯追踪 |

状态机的主要门限在 `configs/aeb/round5.yaml`，第 9 轮选出的配置在 `configs/aeb/round9/chosen_fixed.yaml` 和 `chosen_state.yaml`。占位值是：固定 TTC 1.4 s、请求减速度 6 m/s²；状态机确认 3 帧，警告 2.4 s，部分制动 1.6 s / 3.5 m/s²，全力制动 0.9 s / 6.0 m/s²。离开阈值高于进入阈值。第 9 轮在调参集上重新搜索后，固定 TTC 仍是 1.4 s，状态机确认改为 1 帧、部分制动进入改为 1.8 s。

每次状态变化写一条 `aeb_state` 事件，带上帧号、前后状态、原因和当时的 TTC。初始 `IDLE` 不写事件。车速不高于 `stop_speed_mps` 时以 `stopped` 退出。

执行器把请求减速度换成制动踏板：`brake = min(max_brake, decel * brake_per_decel)`。减速度还受 `max_decel_mps2` 和斜率 `slew_decel_mps3` 限制，所以爬升过程中请求踏板和实际踏板可以不同。实测纵向加速度是相邻帧车速差除以时步，向前为正。`brake_delay_frames` 是请求减速度和实测减速度先后跨过 0.5 m/s² 的帧数差。

### AES 和协调

左右各一条五次多项式。横向以向右为正，向左终点的横向坐标为负。两端的横向斜率和相对车道的曲率都是 0。

一条轨迹要同时满足三件事才可行：

- 车身四角投影到中心线上的横向坐标落在可行驶边界内。
- 与目标的匀速预测包围盒在时间上没有重叠。
- 自行车模型下的横向加速度、前轮转角和转角变化率不超过 `vehicle.lincoln.mkz_2020` 的限值。轴距 2.85 m。横向加速度上限 4.0 m/s²，前轮转角上限 0.52 rad，转角变化率上限 0.70 rad/s。

拒绝原因包括 `target_too_close`（表面间距短于 8 m 且重叠）、`target_overlap`、`road_boundary`、`lat_accel`、`steer_angle`、`steer_rate`。两侧都可行且代价接近时保留向左。

协调器默认只用 AEB。`brake_avoidable` 为假且存在可行轨迹时才进入 AES，之后沿已选定轨迹跟踪，走完记 `aes_completed`。每帧原因是下面之一：`aeb_only_brake_sufficient`、`aeb_only_no_conflict`、`aes_selected`、`aes_tracking`、`aes_infeasible`、`aes_completed`。附着圆起作用时原因后面再接 `|friction_limited`。

附着圆取 `mu = 0.72`、`g = 9.81`，半径约 7.06 m/s²。纵向减速度和横向加速度的合成超出这个圆时，两边按同一比例缩小。路径跟踪用纯追踪，前视距离是车速乘增益，再夹到 3 到 8 m。

### 场景

早期两条模板直接采样 CARLA 地图上的直道，由 `scenarios.runner` 生成演员：

| 模板 | 内容 |
| --- | --- |
| `pedestrian_crossing` | 行人触发前停在路侧，触发后以固定速度横穿自车道 |
| `lead_vehicle_brake` | 前车触发前匀速，触发后按给定减速度减速。不交给交通管理器 |

触发类型固定为 `distance_to_target`：道路弧长上，自车参考点到目标参考点的距离降到 `trigger.distance_m` 时，目标从下一帧开始动作。结束条件是碰撞、双方停车或超时。`hard_negative` 为真的实例是硬负例，用来统计误制动。行人横穿里的 `clear_pass` 就是这种例子：人从远处走完车道，自车到达时人已经让开。

第 9 轮在直线上扩充了六个模板。`load_spec` 仍然只接受上面两个早期模板；新模板由 `round9` 读取网格 YAML。

| 模板 | 内容 |
| --- | --- |
| `vehicle_cut_in` | 邻道车辆接近后横向切入自车道 |
| `occlusion_reveal` | 近处大车挡住后方行人，行人再横穿出来 |
| `left_avoidable` | 静止车辆偏在车道右侧，向左的轨迹可以绕开 |
| `right_avoidable` | 静止车辆偏在车道左侧，向右的轨迹可以绕开 |
| `both_unavoidable` | 自车道和左右邻道同时被挡住 |
| `parallel_vehicle` | 旁车并行。这条是硬负例，正常情况不该制动 |

网格按速度和初始距离划分，调参集和测试集的数字不重叠。

| 划分 | 自车速度 | 初始距离 | 数量 |
| --- | --- | --- | --- |
| 调参 `configs/scenarios/round9/tuning` | 8、11 m/s | 16、26 m | 6 个模板 × 4 个点 = 24 |
| 测试 `configs/scenarios/round9/test` | 10、14 m/s | 18、32 m | 同样 24 |

调参阶段不允许打开测试目录。选出配置之后，测试集只跑一次。

### 指标

`metrics/` 只读 `frames.jsonl`、`events.jsonl`、`summary.json`，不改策略。

- 安全：是否碰撞、碰撞相对速度、最小表面间距。没有碰撞时相对速度留空。
- 误制动：只在硬负例上有定义。出现制动请求，或状态进入 `PARTIAL` / `FULL`，记为误制动。普通场景避开碰撞不算误制动，`false_brake` 为 null。
- 干预：进入 `PARTIAL` 或 `FULL` 的次数、进入 `EXIT` 的次数、带原因的状态切换次数。
- 响应：第一帧有效 TTC 到请求减速度达到 0.5 m/s² 的帧数差；请求到实测减速度达到同一门槛的帧数差。
- 控制：峰值减速度，以及实测纵向加速度的最大绝对变化率 `peak_jerk_mps3`。

第 9 轮的目标函数是碰撞率加 0.5 倍硬负例误制动率，越低越好。同分再比误制动率、平均状态切换，最后靠近第 5 轮的占位阈值。

## 环境和使用前检查

| 项目 | 要求 |
| --- | --- |
| 操作系统 | 本文命令按 Windows PowerShell 书写，逻辑同样适用于其他系统 |
| CARLA | 0.9.16，需要地图 `Town10HD_Opt` |
| Python | 3.12，与 CARLA wheel 的标签一致 |
| 必装包 | `numpy`、`PyYAML`、与仿真器同版本的 `carla` |
| 出图 | `matplotlib`；写 mp4 时本仓库会优先用 `opencv-python` |

Windows 上的 wheel 一般在 CARLA 发行目录的 `PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-win_amd64.whl`。

```powershell
pip install numpy pyyaml matplotlib opencv-python
pip install D:\CARLA\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl
```

先开仿真器窗口，确认左下或日志里的端口是 2000，再回到项目根目录执行 Python。不要同时开两个 CARLA。

```powershell
D:\CARLA\CarlaUE4.exe -quality-level=Low -windowed -ResX=1280 -ResY=720
Set-Location "D:\CARLA\基于CARLA的AEBAES主动安全系统与闭环评测平台"
```

可以用环境探针看 Python、CARLA 包和端口是否通：

```powershell
python env_probe.py
```

`carla_python.importable` 应为 true，`carla_server.connected` 在仿真器已启动时应为 true。单元测试不连接仿真器：

```powershell
python -m unittest discover -s tests -q
```

## 使用手册

下面的命令都在项目根目录执行。观看时镜头在自车后上方。自车框按 AEB 状态着色：`IDLE` 灰、`WARN` 黄、`PARTIAL` 橙、`FULL` 红、`EXIT` 蓝。当前威胁头顶是 `TTC 1.25s` 或 `TTC --`。

`--pace` 是每一帧至少占用的墙钟秒数。仿真时步仍是 0.05 s，这个参数只改变画面快慢，不改变决策。数值越大越慢。`--pace 0` 按机器最快速度播放，适合批量。

### 看行人横穿

自车 10 m/s，行人从左侧 4 m 处以 1.5 m/s 横穿，触发距离 27 m。默认每帧停留 0.25 s。

```powershell
python -m scenarios.runner --config configs/scenarios/pedestrian_crossing_collision.yaml --strategy state_machine --repeats 1 --aeb-config configs/aeb/round5.yaml
```

换策略时把 `--strategy` 改成 `no_intervention`、`fixed_ttc` 或 `aeb_aes`。`--repeats` 是同一 YAML、同一种子的重复次数，观看时用 1。结果写到 `runs/round5/<场景>_<策略>_<时间>/`。终端会打印触发帧、是否碰撞、碰撞相对速度和制动延迟帧数，并给出俯视预览、稳定性图和诊断图的路径。

常用参数：

| 参数 | 作用 |
| --- | --- |
| `--config` | 场景 YAML |
| `--strategy` | 四种策略之一 |
| `--aeb-config` | AEB 与执行器 YAML，默认 `configs/aeb/round5.yaml` |
| `--repeats` | 重复次数，默认 20。观看用 1 |
| `--pace` | 每帧墙钟秒数，默认 0.25 |
| `--run-root` | 结果目录 |
| `--debug-draw` / `--no-debug-draw` | 是否在 CARLA 里画状态框，默认画 |

还想再慢一点：

```powershell
python -m scenarios.runner --config configs/scenarios/pedestrian_crossing_collision.yaml --strategy state_machine --repeats 1 --aeb-config configs/aeb/round5.yaml --pace 0.40
```

### 看前车制动

自车 11 m/s，前车 6 m/s，间隙 22 m，触发距离 14 m，触发后前车以 4 m/s² 减速。

```powershell
python -m scenarios.runner --config configs/scenarios/lead_vehicle_brake_collision.yaml --strategy state_machine --repeats 1 --aeb-config configs/aeb/round5.yaml
```

### 看第 9 轮场景

`--scene` 选择模板和种子。命令行上的 `--ego-speed` 和 `--distance` 只作用于这一次观看，默认 5 m/s 和 36 m，方便看清接近；YAML 文件本身保持测试集里的原始数字。`--timeout` 默认 16 s，`--pace` 默认 0.15 s，`--hold` 是结束后在最后一帧停留的秒数。

左侧可避让，状态机：

```powershell
python -m sim.carla_stage --scene configs/scenarios/round9/test/left_avoidable_v10_d18.yaml --strategy state_machine --aeb-config configs/aeb/round9/chosen_state.yaml
```

同一条路加上 AES：

```powershell
python -m sim.carla_stage --scene configs/scenarios/round9/test/left_avoidable_v10_d18.yaml --strategy aeb_aes --aeb-config configs/aeb/round9/chosen_state.yaml
```

遮挡后出现的行人。要打开遮挡模型时加 `--occlusion`：

```powershell
python -m sim.carla_stage --scene configs/scenarios/round9/test/occlusion_reveal_v10_d18.yaml --strategy state_machine --aeb-config configs/aeb/round9/chosen_state.yaml --occlusion
```

其他模板把文件名换成 `vehicle_cut_in_*.yaml`、`right_avoidable_*.yaml`、`both_unavoidable_*.yaml` 或 `parallel_vehicle_*.yaml`。终端第一行会打印这次实际使用的车速、间距，以及文件里原来的数字。

若要按文件里的 10 m/s、18 m 原样播放，显式写上：

```powershell
python -m sim.carla_stage --scene configs/scenarios/round9/test/left_avoidable_v10_d18.yaml --strategy state_machine --aeb-config configs/aeb/round9/chosen_state.yaml --ego-speed 10 --distance 18 --pace 0.25
```

### 空控制回路

这条只检查同步时步、自车状态和传感器，控制量恒为 0。默认渲染并跟随出生点上的自车。

```powershell
python -m sim.empty_loop --config configs/sync_empty.yaml --frames 200
```

### 批量跑早期场景

目录名必须是 `tuning` 或 `test`。每个 YAML 依次跑无干预、固定 TTC和状态机。种子只用 YAML 里的。两次运行之间只清演员。这一路不等待 `--pace`，画面会很快。

```powershell
python run_batch.py --scenes configs/scenarios/tuning --repeat 1 --aeb-config configs/aeb/round5.yaml
python run_batch.py --scenes configs/scenarios/test --repeat 1 --aeb-config configs/aeb/round5.yaml
```

输出在 `runs/round6/<划分>_<时间>/`，其中包括 `episodes.csv`、`comparison.md`、`report.html`，以及总览、延迟和结果网格图。清单里的 `evaluation.split` 写成目录名，`used_for_tuning` 只有调参集为 true。

### 第 9 轮：搜索、消融、稳定性和测试

这条命令会连上已经打开的 CARLA，在同一张地图上跑完整条调参流程。场景很多，每帧都 `tick`，需要留出较长时间。它会清空并重写 `runs/round9/logs` 和 `figures/round9`。

```powershell
python -m round9.run
```

内部顺序：

1. 把调参网格和测试网格写到 `configs/scenarios/round9/tuning` 与 `test`。搜索期间只读调参目录。
2. 在固定 TTC 的 1.0、1.4、1.8、2.2 s，以及状态机的确认帧数 1 或 3、部分制动进入阈值 1.4 或 1.8 s 上搜索。目标函数是碰撞率 + 0.5 × 硬负例误制动率。
3. 把选出的两份配置写到 `configs/aeb/round9/chosen_fixed.yaml` 和 `chosen_state.yaml`，并记下 `tuning_log.json`。此时测试集尚未打开。
4. 消融只改选定状态机的一项：收紧迟滞、去掉风险确认、去掉分级制动。迟滞的离开阈值至少比进入阈值高 0.05 s。
5. 四种策略在调参集上重复 3 次，检查碰撞结果是否翻转。靠近边界的情况是未碰撞但最小间距小于 0.5 m，或碰撞但相对速度小于 1.5 m/s。
6. 打开测试集，四种策略各跑一次，写入 `test_run.json`。
7. 对调参集里的遮挡场景各跑两遍：遮挡关闭和遮挡打开。比较行人第一次被看见和第一次出现有效 TTC 的帧。

报告在 `runs/round9/report.html` 和 `runs/round9/comparison.md`。

### 第 10 轮：难例挖掘、邻域和回归

挖掘不改策略，也不改测试集。它只读已经写好的第 9 轮日志。阈值在 `configs/round10/mining.yaml`：

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `near_miss_m` | 0.5 | 未碰撞但最小间距小于该值，算近失 |
| `state_switch_count` | 12 | 状态切换不少于该次数，算抖动 |
| `path_error_m` | 0.35 | 横向跟踪误差绝对值超过该值 |
| `slow_response_frames` | 3 | 请求到实测减速度的延迟达到该帧数，线索记为响应慢 |
| `log_roots` | 第 9 轮测试日志和搜索日志 | 挖掘时扫描的目录 |

一条运行只要命中下面任意一条，就进入难例：

- 发生碰撞。
- 未碰撞，且最小间距小于 `near_miss_m`。
- 状态切换次数达到 `state_switch_count`。
- 出现 `aes_infeasible`。
- 横向跟踪误差大于 `path_error_m`。
- 帧号不连续。

`suspected_cause` 只是一条待核对的线索，按下面的优先级取一个：

| 线索 | 时间关系 |
| --- | --- |
| `false_trigger` 误触发 | 硬负例上出现了制动 |
| `late_brake` 晚制动 | 有效风险不晚于最后一帧仍可制动避开的时刻，制动请求却更晚 |
| `aes_infeasible` | 仲裁原因里出现 AES 不可行 |
| `late_discovery` 晚发现 | 第一次有效风险已经落在可避开窗口之后，或者日志里没有可避开帧却发生了碰撞 |
| `slow_response` 响应慢 | 请求减速度到实测减速度的帧数差达到阈值 |
| `tracking_error` 跟踪误差大 | 横向误差超过阈值 |
| `data_gap` 缺帧 | 帧号不连续 |
| `unspecified` 无线索 | 进了难例，但对不上上面的时间关系 |

可避开的最后一帧是这样定的：出现过有效 TTC 之后，`brake_avoidable` 还为真的最后一帧；下一帧变成假时，这一帧就是临界帧。制动请求帧是请求减速度第一次达到 0.5 m/s² 的帧。

整轮挖掘、抽样、邻域和回归：

```powershell
python -m round10
```

它会：

1. 扫描 `mining.yaml` 里的日志根目录。
2. 从晚制动里按模板去重，抽出 3 条；再各抽 1 条晚发现和 1 条 AES 不可行。
3. 把案例写成 `regression/cases/*.json`。清单里有模板、速度、距离、策略和配置路径，供以后重跑。
4. 在源场景的速度 ±1 m/s、距离 ±2 m 上加密，写到 `configs/scenarios/round9/neighborhood/tuning`。非正的速度或距离会丢掉。这个目录名是 `tuning`，第 9 轮的加载器可以直接读。出现时机取源日志里第一次有效 TTC 的时间，记在同级的 `sampling.json`，不写进场景 YAML。往 `configs/scenarios/test` 或 `configs/scenarios/round9/test` 写文件会直接报错。
5. 用清单把回归案例在 CARLA 里重跑，和上一次的碰撞结果比较，写出 `regression/last_result.json` 和 `regression/diff.md`。新失败、已修复、未变分别标记。
6. 给抽出的晚制动案例各做俯视动画，以及风险出现、制动开始、最近点三张静帧。
7. 汇总到 `runs/round10/report.html`，图在 `figures/round10/`。

只重跑已有回归案例、不重新挖掘：

```powershell
python -m round10.regress
```

回归读取案例 JSON 里的清单，调用和当时相同的模板、速度、距离、策略和 AEB 配置，不回读旧的 `frames.jsonl`。因此它检查的是当前代码是否还复现原来的碰撞结论。

### 看板和演示视频

这两步只读已经生成的运行目录和图，不改策略。

```powershell
python dashboard/build_dashboard.py
python -m viz.make_demo
```

`dashboard/dashboard.html` 把模板数量、策略、一份 manifest 摘要、第 6 轮和第 9 轮的对照表，以及难例的三张静帧嵌进同一个文件，页面不引用外部地址。

`python -m viz.make_demo` 从日志回放五段俯视视频：无干预碰撞、固定阈值硬负例、状态机避开、状态机加 AES 避开、晚制动失败。字幕包含策略名、AEB 状态、当前 TTC 和速度。封面和片段清单在 `videos/manifest.json`。

## 一次运行留下的文件

每个 `rep_XX` 目录是一次独立运行。

`manifest.json` 记录这次实验能不能复现。主要字段有 `schema_version`、`run_id`、`scene_id`、`strategy_name`、`strategy_version`、UTC 时间、CARLA 版本、Python 版本、地图、固定时步、是否同步、物理子步、Python / NumPy / 交通管理器种子、天气、策略配置原文路径和 SHA-256。项目根目录自己是 git 仓库时，`code_version` 写 HEAD。第 9 轮经 CARLA 播放的运行还会带一份 `carla` 对象，标明地图、端口和当时的服务端版本。

`frames.jsonl` 每行一帧。自车位置和速度在 `ego` 里，威胁在 `threat` 里，AEB/AES 状态、触发原因、退出原因、仲裁原因、请求控制和实际控制分字段存放。横向量包括 `aes_direction`、`path_error_m`、`road_departure_m`、`side_gap_m`、`lateral_accel_mps2`。旧日志缺这些字段时按空值读回。数值缺失保持空，不写成 0。

`targets.jsonl` 每行是一帧里的一条航迹。`events.jsonl` 记录场景规格、目标触发、AEB 状态变化、第一次看见、碰撞和场景结束。`summary.json` 给出结果 `collision`、`avoided` 或 `incomplete`，以及最小间距、最小 TTC、首次触发时间、峰值减速度、起末速度、时长和帧数。

第 6 轮批量目录另有 `episodes.csv`（每次运行一行）、`comparison.md`（三种策略对照，重复是否一致写在表外）和单文件 `report.html`。第 9 轮、第 10 轮的总报告分别在 `runs/round9/report.html` 和 `runs/round10/report.html`。

## 目录

| 路径 | 内容 |
| --- | --- |
| `schemas.py` `run_logger.py` `experiment.py` | 帧记录、五份日志和运行清单 |
| `perception/` | 真值跟踪、遮挡 |
| `threat/` | TTC 和制动可行性 |
| `aeb/` | 固定 TTC、状态机、阈值加载 |
| `aes/` | 五次多项式、动力学限值和离线用例 |
| `arbiter/` | 制动与转向的切换原因 |
| `control/` | 制动执行器、纯追踪、附着圆 |
| `strategies/` | 无干预，以及 AEB+AES 策略入口 |
| `scenarios/` | 早期两个模板、直道搜索、CARLA 生成和闭环积分 |
| `round9/` | 六模板网格、搜索、消融、稳定性、遮挡对照 |
| `round10/` | 难例规则、邻域、回归和总入口 |
| `sim/` | CARLA 同步会话、镜头、调试框、单条场景播放 |
| `metrics/` | 只读日志的指标、对照表和 HTML 报告 |
| `viz/` | 俯视回放、威胁时间轴、AEB 诊断、演示视频 |
| `dashboard/` | 把已有图表收成一个离线 HTML |
| `configs/scenarios/` | 早期场景、第 9 轮网格、邻域 |
| `configs/aeb/` | 第 5 轮占位阈值和第 9 轮搜索、选定、消融配置 |
| `configs/aes/` `configs/control/` | AES 轨迹限值和纯追踪参数 |
| `configs/round10/mining.yaml` | 难例阈值和日志根目录 |
| `regression/` | 回归案例、上次结果和 `diff.md` |
| `tests/` | 不启动 CARLA 的单元测试 |
| `runs/` `figures/` `videos/` | 运行日志、图和演示视频 |

## 和 CARLA 对接时的约束

- 仿真器先启动，Python 后连接 `localhost:2000`。客户端超时后会说明服务端没有就绪。
- 地图使用 `Town10HD_Opt`。直道按出生点顺序查找，需要大约 80 到 120 m 的同一车道、没有路口、相邻采样转角不大。
- 自车蓝图是 `vehicle.lincoln.mkz_2020`，前车和障碍车常用 `vehicle.tesla.model3`，行人是 `walker.pedestrian.0001`。行人不再挂 AI 控制器，避免和每帧 `set_transform` 抢位置。
- 观看命令会打开渲染。批量的 `round9.run` 和 `round10` 同样推进同步世界，但帧与帧之间不等待。
- 退出时销毁本次生成的演员，关闭同步模式，并把世界设置恢复到进入会话之前。若上次异常停在同步模式，下次连接会先补一次 tick 再关掉同步，避免必须重启仿真器。
