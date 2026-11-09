| 指标 | 无干预 | 固定 TTC | 状态机 |
| --- | --- | --- | --- |
| 碰撞率 | 0.667（重复极差 0.000） | 0.667（重复极差 0.000） | 0.667（重复极差 0.000） |
| 碰撞相对速度均值 (m/s) | 10.597（重复极差 0.000） | 5.415（重复极差 0.000） | 6.620（重复极差 0.000） |
| 最小间距均值 (m) | 0.587（重复极差 0.000） | 0.587（重复极差 0.000） | 0.739（重复极差 0.000） |
| 硬负例误制动率 | 0.000（重复极差 0.000） | 0.000（重复极差 0.000） | 1.000（重复极差 0.000） |
| 激活次数均值 | 0.000（重复极差 0.000） | 0.667（重复极差 0.000） | 1.333（重复极差 0.000） |
| 退出次数均值 | 0.000（重复极差 0.000） | 0.000（重复极差 0.000） | 0.333（重复极差 0.000） |
| 状态切换次数均值 | 0.000（重复极差 0.000） | 0.667（重复极差 0.000） | 2.667（重复极差 0.000） |
| 风险到请求延迟均值 (帧) | — | 0.000（重复极差 0.000） | 9.000（重复极差 0.000） |
| 请求到实际减速延迟均值 (帧) | — | 1.000（重复极差 0.000） | 1.000（重复极差 0.000） |
| 峰值减速度均值 (m/s²) | 0.000（重复极差 0.000） | 4.000（重复极差 0.000） | 5.167（重复极差 0.000） |
| 加加速度峰值均值 (m/s³) | 0.000（重复极差 0.000） | 20.000（重复极差 0.000） | 30.000（重复极差 0.000） |

| 场景 | 策略 | 重复 | 碰撞 | 碰撞相对速度 (m/s) | 最小间距 (m) | 误制动 | 风险到请求 (帧) | 请求到减速 (帧) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| lead_vehicle_brake_collision | 固定 TTC | 0 | 1 | 1.303 | 0.000 | — | 0 | 1 |
| lead_vehicle_brake_collision | 固定 TTC | 1 | 1 | 1.303 | 0.000 | — | 0 | 1 |
| lead_vehicle_brake_collision | 无干预 | 0 | 1 | 9.124 | 0.000 | — | — | — |
| lead_vehicle_brake_collision | 无干预 | 1 | 1 | 9.124 | 0.000 | — | — | — |
| lead_vehicle_brake_collision | 状态机 | 0 | 1 | 3.115 | 0.000 | — | 2 | 1 |
| lead_vehicle_brake_collision | 状态机 | 1 | 1 | 3.115 | 0.000 | — | 2 | 1 |
| pedestrian_crossing_clear_pass | 固定 TTC | 0 | 0 | — | 1.762 | 否 | — | — |
| pedestrian_crossing_clear_pass | 固定 TTC | 1 | 0 | — | 1.762 | 否 | — | — |
| pedestrian_crossing_clear_pass | 无干预 | 0 | 0 | — | 1.762 | 否 | — | — |
| pedestrian_crossing_clear_pass | 无干预 | 1 | 0 | — | 1.762 | 否 | — | — |
| pedestrian_crossing_clear_pass | 状态机 | 0 | 0 | — | 2.216 | 是 | 23 | 1 |
| pedestrian_crossing_clear_pass | 状态机 | 1 | 0 | — | 2.216 | 是 | 23 | 1 |
| pedestrian_crossing_collision | 固定 TTC | 0 | 1 | 9.527 | 0.000 | — | 0 | 1 |
| pedestrian_crossing_collision | 固定 TTC | 1 | 1 | 9.527 | 0.000 | — | 0 | 1 |
| pedestrian_crossing_collision | 无干预 | 0 | 1 | 12.070 | 0.000 | — | — | — |
| pedestrian_crossing_collision | 无干预 | 1 | 1 | 12.070 | 0.000 | — | — | — |
| pedestrian_crossing_collision | 状态机 | 0 | 1 | 10.126 | 0.000 | — | 2 | 1 |
| pedestrian_crossing_collision | 状态机 | 1 | 1 | 10.126 | 0.000 | — | 2 | 1 |

| 场景 | 策略 | 重复次数 | 碰撞是否一致 | 相对速度极差 (m/s) | 误制动是否一致 |
| --- | --- | --- | --- | --- | --- |
| lead_vehicle_brake_collision | 固定 TTC | 2 | 是 | 0.000 | 是 |
| lead_vehicle_brake_collision | 无干预 | 2 | 是 | 0.000 | 是 |
| lead_vehicle_brake_collision | 状态机 | 2 | 是 | 0.000 | 是 |
| pedestrian_crossing_clear_pass | 固定 TTC | 2 | 是 | — | 是 |
| pedestrian_crossing_clear_pass | 无干预 | 2 | 是 | — | 是 |
| pedestrian_crossing_clear_pass | 状态机 | 2 | 是 | — | 是 |
| pedestrian_crossing_collision | 固定 TTC | 2 | 是 | 0.000 | 是 |
| pedestrian_crossing_collision | 无干预 | 2 | 是 | 0.000 | 是 |
| pedestrian_crossing_collision | 状态机 | 2 | 是 | 0.000 | 是 |
