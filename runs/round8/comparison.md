| 策略 | 碰撞 | 碰撞相对速度 (m/s) | 结束 |
| --- | --- | --- | --- |
| 无干预 | 1 | 12.060 | collision |
| 固定 TTC | 1 | 2.818 | collision |
| 状态机 AEB | 1 | 4.512 | collision |
| 状态机 AEB+AES | 0 | — | timeout |

同一条刹不住的行人横穿：自车 12 m/s，行人从左侧 1.5 m 处以 1.2 m/s 向右走，触发距离 16 m。
无干预、固定 TTC、状态机 AEB 的碰撞相对速度分别是 12.060、2.818、4.512 m/s。
状态机 AEB+AES 碰撞=0，结束原因是 timeout。
它在制动来不及的第一帧选择向左，跟踪中路径误差 -0.018 到 0.017 m，侧向间距最小 0.363 m。
AEB+AES 每帧原因：aeb_only_brake_sufficient、aes_completed、aes_selected、aes_tracking
这条轨迹的制动和转向合成加速度没有顶到附着圆，圆上的点都在 7.06 m/s² 里面。附着圆限制由单独的 6+6 m/s² 请求验证。
