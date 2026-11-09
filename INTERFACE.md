# 接口摘要（第 2 轮起每轮贴上）

环境快照见 `env.json`。本轮不导入 carla。

## 坐标系

- 世界系：左手系，米。X 向前，Y 向右，Z 向上。航向单位 rad，0 沿 +X，俯视顺时针为正。与 CARLA 世界系同向。
- 车体系：原点在车辆包围盒中心。X 沿车头，Y 向右，Z 向上。
- 相对量写在自车车体系：纵向向前为正，横向向右为正。
- `steer` 无量纲，范围 -1~1，向右为正。其余角度用弧度。

## 数据类型（schemas.py）

`SCHEMA_VERSION = 1`

- `EgoState`：世界位置 `x_m/y_m/z_m`，姿态 `roll_rad/pitch_rad/yaw_rad`，车体速度 `v_long_mps/v_lat_mps/v_up_mps`，水平速率 `speed_mps`，`yaw_rate_radps`，车体加速度 `a_long_mps2/a_lat_mps2`，前轮转角 `steer_road_rad`（未知 None）。
- `Track`：`track_id/source/class_name`，世界位姿和车体速度，包围盒 `length_m/width_m/height_m`，相对自车 `rel_x_m/rel_y_m/rel_v_long_mps/rel_v_lat_mps`，`in_path`。
- `ThreatResult`：`ttc_s`、`pred_min_distance_m` 允许 None。二者有任一为 None 时 `invalid_reason` 必填；都有效时 `invalid_reason` 必须是 None。另有 `threat_id/in_path/closing_speed_mps/overlap_ratio`。`closing_speed_mps` 靠近为正，等于 `-rel_v_long_mps`。
- `ControlRequest`：策略请求，`throttle/brake/steer/desired_speed_mps`。
- `ActualControl`：实际执行量，字段同上，与请求分开。`desired_speed_mps` 是下发的速度命令，不是测量车速。
- `FrameRecord`：`run_id/scene_id/strategy_name/frame/sim_time_s/source_tag/ego/threat/aeb_state/aes_state/trigger_reason/exit_reason/arbitration_reason/control_request/actual_control/quality_flags`。
- `EpisodeSummary`：`outcome/collided/min_distance_m/min_ttc_s/first_trigger_time_s/aeb_first_trigger_time_s/aes_first_trigger_time_s/peak_decel_mps2/speed_drop_mps/initial_speed_mps/final_speed_mps/duration_s/frame_count/invalid_frame_count`。未知测量保持 None，不写 0。

AEB 状态：`IDLE` `WARN` `PARTIAL` `FULL` `EXIT`。AES 状态：`IDLE` `ACTIVE` `EXIT`。

序列化：`dump_line`、`parse_line`、`dump_pretty`、`load_dataclass`。JSON `null` 读回是 None。

## 运行日志（run_logger.py）

`RunLogger(run_dir, manifest)` 创建目录。五个文件：`manifest.json`、`frames.jsonl`、`targets.jsonl`、`events.jsonl`、`summary.json`。

- `log_frame(frame)`：追加一帧，flush + fsync。
- `log_target(frame, sim_time_s, track)`：一行 `{"frame", "sim_time_s", "track"}`。
- `log_event(event_type, sim_time_s, detail=None)`：一行 `{"event_type", "sim_time_s", "detail"}`。
- `write_summary(summary)`：只覆盖 `summary.json`。
- `read_frames()`：按文件顺序读回 `list[FrameRecord]`。
- `close()`。

## 清单（experiment.py）

`build_manifest(...)` 不导入 carla。`carla_version` 由调用方传入，缺省 null。`python_version` 用当前解释器。

键：`schema_version`、`run_id`、`scene_id`、`strategy_name`、`strategy_version`、`created_at_utc`、`carla_version`、`python_version`、`map`、`fixed_delta_seconds`、`synchronous_mode`、`physics_substep`（`substepping`、`max_substep_delta_time_s` 单位秒、`max_substeps`）、`seeds`（`python`、`numpy`、`traffic_manager`）、`weather`、`strategy_config`、`strategy_config_path`、`strategy_config_sha256`、`code_version`。

`code_version` 只在项目根自己是 git 仓库时写 HEAD；否则 null。配置文件支持 `.yaml/.yml/.json`，校验和是原始字节的 SHA-256。

## 可视化（viz/common.py）

只读运行目录。

- `load_run(run_dir) -> RunData(frames, targets, events, manifest, summary)`。帧表列名由字段展平，例如 `ego_speed_mps`、`ego_a_long_mps2`、`threat_ttc_s`、`control_request_brake`、`actual_control_brake`、`aeb_state`。数值列里的缺失是 NaN，不是 0。
- `AEB_STATE_COLORS`：IDLE `#9E9E9E`，WARN `#F9A825`，PARTIAL `#FB8C00`，FULL `#E53935`，EXIT `#1E88E5`。
- `STRATEGY_COLORS`：`aeb #E53935`，`aes #43A047`，`aeb_aes #8E24AA`，`baseline #546E7A`，`driver #6D4C41`。其他名字用 `strategy_color(name)` 稳定取色。
- `save_fig(fig, run_dir, name)` 保存到 `figures/{scene_id}_{name}.png`。
- `plot_run_overview(run_dir)`：四幅共享时间轴，速度、减速度（制动为正）、TTC、AEB 色带。

## 场景（第 3 轮）

`scenarios/` 只做两个模板：`pedestrian_crossing`、`lead_vehicle_brake`。模板在 `scenarios/templates/`，参数实例在 `configs/scenarios/`。实例给出种子、自车初速、目标速度、触发距离，以及行人或前车自己的数值；不能改 `behavior`，也不能把目标交给交通管理器。

- `load_spec(path) -> ScenarioSpec`。触发类型固定为 `distance_to_target`：道路弧长坐标里，自车参考点到目标参考点的距离降到 `trigger.distance_m` 时，目标从下一帧开始动作。
- `pedestrian_crossing(spec)`、`lead_vehicle_brake(spec)`：生成自车和唯一目标。调用前把 `(session, road_samples)` 放进 `spec.runtime`。行人触发前静止，之后以固定速度横穿；前车触发前匀速，之后按 `decel_mps2` 减速。
- 结束条件：碰撞、双方停车、超时。`hard_negative` 写进清单。碰撞判定是平面包围盒重叠，事件 `detector` 为 `oriented_box`。
- 每次重复清掉车辆、行人和传感器，再按同一个场景种子生成。地图已经是目标地图时不再 `load_world`：0.9.16 连续重载会让客户端在 `Simulator.h` 断言 `_episode != nullptr`。本轮策略名是 `no_intervention`。
- 清单 `scenario` 写入规格关键字段和道路边界。事件有 `scenario_spec`、`target_trigger`、`collision`、`scenario_end`。`scenario_result.trigger_frame` 和 `collision_relative_speed_mps` 在跑完后补进清单。
- `viz/scenario_preview.py` 画 `scenario_preview.png`。`viz/stability.py` 按运行序号画触发帧和碰撞相对速度。俯视回放在有 `targets.jsonl` 和 `target_trigger` 时叠目标轨迹和触发标记。

## 感知和威胁（第 4 轮）

这一轮不实现 AEB 或 AES。策略名仍是 `no_intervention`，版本 `round4-0`。

- `perception.GroundTruthTracker`：`BaseTracker` 的真值实现。`update(timestamp_s, ego, observations, ego_length_m, ego_width_m)` 只接受与当前帧时间对齐的 `TargetTruth`，输出 `Track`。`Track.timestamp_s` 是这帧仿真时间。`in_path` 表示目标当前压在自车车宽走廊上，整车落在后保险杠后面则不是。
- `perception.VisibilityModel`：本轮不计算遮挡。`apply` 原样返回，`occlusion_ratio` 恒为 0。接口留给第 9 轮。
- `threat.ThreatAssessor`：预测假设是 `constant_velocity_no_steer`，写在 `ThreatResult.prediction`。跟车且同向、横向相对速度很小时，纵向追尾 TTC 是保险杠间距除以靠近速度。横穿和切入用双方包围盒在匀速、航向不变下的重叠时刻。没有有效 TTC 时 `ttc_s` 是 None，并写 `invalid_reason`，不填 0。
- 无效原因：`no_track`、`not_closing`、`target_behind`、`no_collision_course`、`passed_conflict`、`beyond_horizon`。目标已经离开自车走廊、且未来不会再碰上时用 `passed_conflict`。
- `ThreatResult.conflict_x_m/conflict_y_m` 是首次接触点的世界坐标。`brake_avoidable` 用当前速度和 `max_decel_mps2`：刹停距离 `v^2/(2a)` 不超过到冲突点的前向距离，或在没有冲突点时不超过当前表面间距。默认最大减速度 6 m/s^2。
- `strategies.NoIntervention.decide(cruise_speed_mps)`：油门、制动、转向都是 0，期望速度是场景巡航速度。
- 帧日志里的 `threat` 用上面的结果。`viz/threat.py` 画 `threat_timeline` 和 `threat_snapshot_<frame>`。无效时段用灰底并注明原因。五个手算示意图在 `figures/unit_cases/`。
- `--debug-draw`：主威胁框在无效时为灰，制动来不及时为红，来得及时为琥珀，头顶文字是 `TTC 1.25s` 或 `TTC --`。
- 旧日志缺 `conflict_*`、`brake_avoidable`、`prediction`、`timestamp_s` 时按 None 读回。未知字段仍然报错。

## AEB 与执行层（第 5 轮）

这一轮不做 AES，也不调阈值。阈值在 `configs/aeb/round5.yaml`，版本 `round5-0`。三种策略都能跑横穿和前车制动：`no_intervention`、`fixed_ttc`、`state_machine`。

- `Policy.decide(ego, tracks, threat) -> ControlRequest`。`ControlRequest.target_decel_mps2` 是策略请求的减速度，单位 m/s²，制动为正。`ActualControl.applied_decel_mps2` 是限幅、限斜率之后的值。制动踏板由 `brake = min(max_brake, decel * brake_per_decel)` 得到。请求踏板按限幅后的请求减速度计算，实际踏板按斜率限制后的减速度计算，爬升过程中两者不同。
- `FixedTtcAeb`：TTC 低于 `ttc_brake_s` 请求 `target_decel_mps2`，否则为 0。相等不算低于。不看上一帧请求。状态只有 IDLE 和 FULL。原因：`ttc_below_threshold`、`ttc_above_threshold`、`ttc_unavailable`。
- `StateMachineAeb`：IDLE、WARN、PARTIAL、FULL、EXIT。连续 `confirm_frames` 帧 TTC 不高于部分制动阈值，才允许从 IDLE 或 WARN 进入制动。WARN 不制动。进入和退出用不同阈值。降级和退出还要满足 `min_hold_frames`，加力不受这个限制。目标丢失时维持当前制动 `lost_hold_frames`，之后再计 `clear_frames` 才以 `target_lost` 退出。威胁高于释放阈值并持续 `clear_frames` 时以 `threat_cleared` 退出。车速不高于 `stop_speed_mps` 时立刻以 `stopped` 退出。退出后 TTC 再次不高于警告阈值，以 `rearmed` 回到 WARN。
- 每次状态变化写 `events.jsonl`，`event_type` 为 `aeb_state`，`detail` 含 `frame/from/to/reason/ttc_s`。原因不能空。初始 IDLE 不写事件。
- 执行是运动学闭环。本帧命令从下一帧才改变车速。实测纵向加速度是相邻帧车速差除以时步，向前为正；帧 0 没有上一帧，记 0 并打 `decel_unobserved`。`EpisodeSummary.brake_delay_frames` 是请求减速度和实测减速度先后跨过 0.5 m/s² 的帧数差。没有制动时为 None。
- 零制动时，自车弧长、触发帧和碰撞帧与 `rollout` 相同。`--strategy` 选择策略，`--aeb-config` 选择 YAML。AEB 避开碰撞也算这次跑完，进程退出码不因此变成失败。
- `viz/aeb_diag.py`：`aeb_diagnostic` 画速度、TTC、状态色带，以及请求和实测两条减速度，并标出延迟帧数。`aeb_states.mmd` 按实际转移生成，`aeb_states` 图只画本段经过的状态。对比图把三种策略的速度、表面间距和制动起点画在一起。颜色：无干预 `#546E7A`，固定 TTC `#C62828`，状态机 `#1565C0`。
- `--debug-draw` 时自车框按 AEB 状态着色。状态切换的那一帧写 ASCII，例如 `AEB FULL ttc_below_full`。俯视回放里自车包围盒用同一套状态色。
- 旧日志缺 `target_decel_mps2`、`applied_decel_mps2`、`brake_delay_frames` 时按 None 读回。

## 指标和第一张对照表（第 6 轮）

这一轮不改 AEB 阈值，也不改指标定义去迁就结果。`metrics/` 只读 `frames.jsonl`、`events.jsonl`、`summary.json`。

- 硬负例是行人横穿的 `clear_pass` 实例，`end.hard_negative` 为 true。行人从自车前方远处走完车道，自车到达时人已经让开。调参集在 `configs/scenarios/tuning/`，测试集在 `configs/scenarios/test/`。两套的种子、触发距离不一样。测试集只出对照。
- 误制动只在硬负例上统计：出现制动请求，或状态进入 `PARTIAL`/`FULL`。没有碰撞不等于误制动，所以普通场景的 `false_brake` 是 null。
- 安全：`collided`、碰撞事件里的 `relative_speed_mps`、`summary.min_distance_m`。没有碰撞时相对速度保持 null。
- 干预：`activation_count` 是进入 `PARTIAL` 或 `FULL` 的次数，`exit_count` 是进入 `EXIT` 的次数，`state_switch_count` 是带原因的 `aeb_state` 事件数。
- 响应：`risk_to_request_frames` 是第一帧有效 TTC 到请求减速度达到 0.5 m/s² 的帧数差。`request_to_decel_frames` 是请求到实测减速度达到同一门槛的帧数差，实测仍是车速差分。
- 控制：`peak_decel_mps2` 用汇总里的峰值减速度。`peak_jerk_mps3` 是实测纵向加速度的最大绝对变化率，单位 m/s³。
- `run_batch.py --scenes <tuning 或 test 目录> --repeat N` 对目录里每个 YAML 跑无干预、固定 TTC、状态机。种子只用 YAML 里的。每次运行前清掉上一轮 actor，不在中途 `load_world`。清单 `evaluation.split` 写 `tuning` 或 `test`，`used_for_tuning` 只有调参集为 true。
- 输出在结果目录：`episodes.csv` 每次运行一行，`comparison.md` 是三列策略、无合并单元格的指标表，并附重复是否一致。`overview.png`、`delays.png`、`outcome_grid.png` 都由这些指标画出。`report.html` 把表和图以 base64 嵌在同一个文件里。
- 状态机若比固定 TTC 碰撞更多，报告里的「日志核对」列出场景、重复序号和两边的风险到请求延迟。本轮到此为止，不改阈值。

## AES 候选轨迹（第 7 轮）

这一轮不接入车辆控制，也不改 AEB。`aes/` 只回答一条路能不能躲开。

- 配置在 `configs/aes/round7.yaml`，版本 `round7-0`。向左、向右各一条五次多项式，横向偏移、纵向长度、时间间隔和纵向加速度都从 YAML 读。速度剖面是恒定加速度，0 表示保持当前车速。两端的横向斜率和对车道的曲率都是 0。
- 横向以向右为正。向左的终点横向坐标是负的。轨迹沿 `RoadSample` 中心线展开。
- 可行性同时看三件事。车身四角投到中心线上的横向坐标必须落在可行驶边界内，不只看轨迹中心。目标未来位置用第 4 轮的匀速、航向不变预测，和自车包围盒做时间对齐的重叠检查。动力学用自行车模型：横向加速度 `v²κ`，前轮转角 `atan(轴距·κ)`，转角变化率是相邻点的差分。
- 限值按 `vehicle.lincoln.mkz_2020` 取，不照搬一套空默认值。轴距 2.85 m。横向加速度上限 4.0 m/s²，约 0.41 g，低于干燥路面大约 8 m/s² 的附着，留作可控的紧急变道。前轮转角上限 0.52 rad，约 30°。转角变化率上限 0.70 rad/s，约 40°/s。30 m、3.5 m 偏移、10 m/s 的参考轨迹峰值是 2.21 m/s²、0.063 rad、0.211 rad/s。把纵向缩到 10 m 会超过横向加速度和转角变化率，缩到 8 m 还会超过前轮转角。
- 当前表面间距短于 `close_gap_m`（8 m）又与目标重叠，原因是 `target_too_close`。更远的重叠是 `target_overlap`。车身出界是 `road_boundary`。超限分别是 `lat_accel`、`steer_angle`、`steer_rate`。
- `AesPlan`：`avoidable`、`direction`、轨迹点（含时间 `t_s` 和累计代价）、`rejections`。没有可行轨迹时 `direction` 是 None，不硬选。两侧都可行且代价差不超过 `cost_tie_epsilon` 时保留向左，向右的原因是 `equal_cost_keep_left`。另一侧代价更高时原因是 `higher_cost`。
- 四个离线用例：左侧可避让、右侧可避让、两侧都不可避让、目标已太近来不及。可行驶边界来自直线道路采样的车道半宽 1.75 m，向左右各扩一条车道，写成 `boundary_source`。总览图是 `figures/aes_cases.png`：可行轨迹实线，不可行虚线，目标未来包围盒由浅到深，拒绝原因写在图上。`figures/aes_dynamics.png` 画横向加速度、前轮转角和转角变化率，并标出限值；最后一行是纵向 10 m 的对照，用来确认限值线会挡住过急的轨迹。

## 制动转向协调（第 8 轮）

这一轮把 AES 接进运动学闭环，不改 AEB 阈值。策略名 `aeb_aes`，版本 `round8-0`。`decide(ego, tracks, threat)` 的接口不变。

- 路径跟踪用纯追踪，写在 `control/pure_pursuit.py`。运动学自行车没有侧偏角，曲率就是横摆角速度除以车速，所以不用 Stanley。前视距离是 `lookahead_gain_s` 乘车速，再夹到 3~8 m。曲率先受横向加速度和横摆角速度限制，前轮转角再受 0.52 rad 和 0.70 rad/s 的变化率限制。配置在 `configs/control/round8.yaml`。
- 独立脚本 `control/track_demo.py` 让自车以 8 m/s 跟踪半径 40 m 的右转圆弧，起步偏在路径左侧 0.10 m。稳态误差大约 `ld²/(2R) = 0.16 m`，门槛在跑之前写成 0.35 m。实测最大绝对值 0.113 m。图是 `figures/path_tracking.png`，上图参考和实际轨迹，下图误差和门槛。
- `arbiter/coordinator.py` 默认只用 AEB。`ThreatResult.brake_avoidable` 为 false，而且 `AesPlan.avoidable` 为 true，才进入 AES。之后沿已选定轨迹跟踪，走完记 `aes_completed`。每帧原因是 `aeb_only_brake_sufficient`、`aeb_only_no_conflict`、`aes_selected`、`aes_tracking`、`aes_infeasible`、`aes_completed` 之一。附着圆起作用时再接 `|friction_limited`。
- 附着圆在 `control/friction.py`。纵向减速度和横向加速度的合成不超过 `mu * g`。`mu` 取 0.72，`g` 取 9.81 m/s²，半径 7.06 m/s²。超出时两边按同一比例缩小，不能一边全力制动、一边极限转向。6 m/s² 加 6 m/s² 的请求会被缩小；单独 6 m/s² 制动不动。
- 刹不住的场景是 `configs/scenarios/round8/pedestrian_unavoidable.yaml`：自车 12 m/s，行人从左侧 1.5 m 处以 1.2 m/s 向右走，触发距离 16 m。同一条路上跑无干预、固定 TTC、状态机、状态机加 AES。对照表在 `runs/round8/comparison.md`。
- 帧日志新增字段都有默认值，旧日志缺这些字段时读成 None：`aes_direction`、`path_error_m`、`road_departure_m`、`side_gap_m`、`lateral_accel_mps2`。`lateral_accel_mps2` 是横向位移的二次导数，向右为正。`road_departure_m` 是车身超出可行驶边界的距离，界内为 0。`side_gap_m` 是与侧向目标的表面间距；没有侧向目标时是车身到可行驶边界的余量。
- 转向从下一帧才改变位置。零转向时自车仍走原来的纵向积分，和第 5 轮一致。执行层把请求里的 `steer` 原样写进 `ActualControl`，制动斜率限制不改转向。
- 图：`figures/friction_circle.png` 是每帧实测的纵横向加速度和 7.06 m/s² 的圆。`figures/arbitration_timeline.png` 用色带标出每次切换和原因。`figures/aes_compare_bev.png` 把四条策略画在同一条路上，叉是碰撞，方块是侧向间距最小的帧。

## 场景扩充和调参（第 9 轮）

这一轮不改 AEB、AES 的判断代码，也不改 `configs/aeb/round5.yaml`。新场景由 `round9/` 在直线上做运动学闭环，不经过 CARLA 生成器。六个模板在 `scenarios/templates/`：车辆切入、遮挡后出现、左侧可避让、右侧可避让、两侧不可避让、旁车并行。`load_spec` 仍然只接受原来的两个模板。

- 遮挡在 `perception/visibility.py`。默认关闭，不传自车时原样返回航迹，遮挡比例为 0。打开后，更近包围盒挡住视线的远目标不会交给策略，并在 `Track.first_seen_s` 记下第一次看见的时间。旧日志没有这个字段时读成 None。
- 网格按写死的划分，数字不重叠。调参集是 8、11 m/s 和 16、26 m，测试集是 10、14 m/s 和 18、32 m。六个模板各 4 个点，一边 24 条。文件在 `configs/scenarios/round9/tuning` 和 `test`。`configs/aeb/round9/tuning_log.json` 的 `test_dir_opened` 是 false，打开过的路径都在 tuning 目录。测试集是选定配置之后才读的，只跑一次，记在 `test_run.json`。
- 目标函数是碰撞率加 0.5 倍硬负例误制动率，越低越好。同分再比误制动、状态切换，最后靠近第 5 轮占位值。旁车并行没有产生制动，误制动率都是 0，所以这一轮的分数就是碰撞率。固定 TTC 选出 1.4 s。1.8 s 和 2.2 s 的碰撞率相同，切换更多。状态机选出确认 1 帧、部分制动 1.8 s。确认改成 3 帧、部分制动仍是 1.8 s 时，调参集碰撞从 3/24 升到 6/24。
- 消融只改选定状态机的配置。去掉迟滞时离开阈值只比进入阈值高 0.05 s，这是加载器允许的最小间隙。碰撞率不变，平均状态切换从 3.42 增到 3.79。去掉风险确认没有差别，因为选出的配置确认帧数已经是 1。去掉分级制动后部分制动和全力制动都是 6.0 m/s²，碰撞率不变，切换降到 2.92。
- 调参集用选出的四种策略重复 3 次。翻转比例 0，可接受，没有再跑。靠近边界的是最小间距小于 0.5 m 的未碰撞，或相对速度小于 1.5 m/s 的碰撞，列在报告里。
- 遮挡对照用调参集里的四条遮挡场景，同一套状态机各跑两次。行人全可见时第 0 帧就有有效 TTC；挡住之后第一次看见和第一次有效 TTC 都推迟到第 22 到 29 帧。其中三条从全可见不碰撞变成遮挡后碰撞。8 m/s、初始距离 26 m 这一条看见得晚，但仍然躲开了。
- 测试集碰撞率：无干预 0.708，固定 TTC 0.208，状态机 0.208，状态机加 AES 0.083。固定 TTC 和状态机撞上的是同一批，碰撞相对速度都是 7.786 m/s；状态机的平均切换是 2.88 次，固定 TTC 是 15.46 次。AES 躲开了 14 m/s、18 m 的左右两侧障碍，也躲开了 10 m/s、18 m 的遮挡行人。14 m/s、18 m 的三车横墙，以及 14 m/s、32 m 的遮挡行人，四种策略里 AES 也没躲开。
- 图和表在 `runs/round9/report.html`，按参数网格、消融、稳定性、遮挡、测试集分章。热力图调参集和测试集分开。对照表另有 `runs/round9/comparison.md`。

## 难例挖掘和回归（第 10 轮）

这一轮只读已有日志和清单，不改策略，也不改测试集里的场景。往 `configs/scenarios/test` 或 `configs/scenarios/round9/test` 写文件会直接报错。

- 筛选阈值在 `configs/round10/mining.yaml`。碰撞、最小间距小于 0.5 m、状态切换不少于 12 次、出现 `aes_infeasible`、横向跟踪误差大于 0.35 m、帧号不连续，满足任意一条就进难例。日志来自第 9 轮的测试集和搜索目录，一共 288 次运行，筛出 162 条。
- `suspected_cause` 只是线索。晚制动：有效风险不晚于可避免的最后一帧，制动请求却更晚。晚发现：第一次有效风险已经落在可避免窗口之后，或者日志里没有可避免帧。另外还有 AES 不可行、响应慢、误触发、跟踪误差和缺帧。对不上这些时间关系的记成无线索。本轮计数是无线索 109 条、晚发现 39 条、AES 不可行 8 条、晚制动 6 条。误触发和响应慢没有出现，因为旁车并行没有制动，执行层延迟一般是 1 帧。
- 回放只读清单里的模板、速度、距离、策略和配置路径，不回读原来的帧日志。`python -m round10.regress` 重跑 `regression/cases`，和上一次的碰撞结果比较，写出 `regression/diff.md`。新增失败、已修复、未变分别是红、绿、灰。这一次 5 条都是未变。
- 抽到的 3 条晚制动都是确认 3 帧的状态机，场景是 11 m/s、16 m 的左右障碍和三车横墙。风险在第 0 帧，可避免到第 1 帧，请求在第 2 帧。回放的三个帧号和原日志一致，所以线索还在。图在 `figures/round10/replay/`，每个案例有俯视动画和风险出现、制动开始、最近点三张静帧。
- 邻域只改速度 ±1 m/s、距离 ±2 m，写到 `configs/scenarios/round9/neighborhood/tuning`。场景字段和第 9 轮网格相同，`load_split` 不用改就能读。出现时机取源日志里第一次有效 TTC 的时间，记在 `neighborhood/sampling.json`，没有加进场景文件。
- 总图在 `runs/round10/report.html`。

## 汇总看板和演示视频（附录）

不改策略，也不改场景。看板和视频都从已经写好的运行目录和指标里生成。

- `python dashboard/build_dashboard.py` 写出单个 `dashboard/dashboard.html`。图用 base64 嵌在文件里，页面不引用外部地址。里面有模板、实例、策略和重复次数，一份 Town10HD_Opt 的 manifest 摘要，第 6 轮和第 9 轮的对照表和图可以折叠，难例行可以展开到风险出现、制动开始、最近点三张静帧。
- `python -m viz.make_demo` 从日志回放五段俯视视频，字幕写策略名、AEB 状态、当前 TTC 和速度。五段是：无干预碰撞、固定阈值硬负例、状态机避开、状态机加 AES 避开、晚制动失败。每段同时写封面和 `videos/manifest.json`（场景编号、策略、对应运行目录）。
- `env.json` 有显卡，没有摄像头字段，CARLA 服务端也没连接，所以没有第三人称镜头，只有俯视。
- 固定阈值在硬负例 `pedestrian_crossing_clear_pass` 上的日志全程是 IDLE，对照表记的是未误刹，字幕按这个写。同一条硬负例上误刹的是状态机，数字在第 6 轮测试表里。
