# Omega.7 → FR3 连续遥操

这是新的独立控制链，只使用 `omega7_fresh`、`omega7_servo_controller` 和下面的入口。旧包
`omega7_teleop` 不参与运行。

```bash
# 假机
./ops/run_omega7_servo.sh fake

# 真机
./ops/run_omega7_servo.sh real 172.16.0.2
```

脚本必须直接运行在交互式终端中。启动前应停止仓库的其他 FR3 容器；入口发现已有同镜像
容器时会拒绝启动。

## 控制链

```text
Omega.7 (200 Hz 采样)
  → DHD raw world 到 FR3 base 的相对末端目标
  → MoveIt 碰撞感知 IK（最多同时一个请求）
  → 关节目标限速
  → omega7_servo_controller
  → 1 kHz Ruckig 连续参考
  → 真机 effort 关节阻抗 / 假机 position 接口
```

空格键是机械臂运动锁存开关。第一次按下时，节点检查 Omega 数据、关节状态和速度、IK
服务、末端 TF 以及 servo 控制器状态，然后锁存当前主从位姿；再次按下会停用机械臂。
Omega 夹持器始终只控制 Franka Hand，不参与机械臂使能。

## 方向、比例和行程

三次现场单轴日志确认操作者前后、左右、上下动作分别主要落在 DHD raw X、Y、Z。此前的
交换取反矩阵正是造成“左右变前后、前后变左右、上下相反”的原因。现在平移使用单位矩阵
直接映射到 `fr3_link0`：

| 物理单轴动作 | DHD raw 主轴（用户日志） | 正 raw 轴映射到 FR3 base |
| --- | --- | --- |
| 前后 | X | `+X → +X` |
| 左右 | Y | `+Y → +Y` |
| 上下 | Z | `+Z → +Z` |

`A = I₃ = [[1,0,0],[0,1,0],[0,0,1]]`，行列式为 `+1`。平移先对 raw world delta
应用 `A`，再乘 `position_scale`；两端腕部朝向不会额外旋转平移方向或引入混轴。
`position_scale: [2,2.5,2.5]` 表示 Omega 前后每移动 1 cm，末端目标移动 2 cm；
左右（raw Y）或上下（raw Z）每移动 1 cm，末端目标移动 2.5 cm。

手腕旋转使用同一个单位矩阵 `A`。DHD 姿态是 world-from-device，节点使用主动旋转四元数
`(x,y,z,w)` 计算空间增量 `R_delta = R_current R_zeroᵀ`，再计算
`R_FR3_delta = A R_delta Aᵀ`，最后左乘 FR3 初始姿态：
`R_target = R_FR3_delta R_FR3_zero`。这让旋转轴映射到同一组 FR3 base 轴。

每约两秒，诊断同时输出 `DHD raw delta xyz`、`mapped FR3 base desired delta xyz`、IK Hz
和 joint tracking error；base delta 显示工作空间裁剪后的目标，便于现场只动单轴时直接查轴向。

目标受 `workspace_min/max` 限制；到达边界时终端会打印期望位置和允许范围。默认最低 TCP
高度从 `0.05 m` 调整为 `0.00 m`，允许夹爪下探到 `fr3_link0` 的基准平面以抓取低矮物体。
如果实际桌面不在 base `z=0`，应把 `workspace_min` 的第三项改成实测桌面安全高度。主手停止后，
如果从臂尚未到达最终目标，节点仍会继续请求 IK 和追赶，不会把最后目标丢掉。

真机方向校准约 10 秒（此步骤描述现场操作；本次改动没有运行真机）：

1. 清空机械臂周围空间，按空格使能并等到 `Arm enabled`。零点就是该时刻 Omega 和 FR3 TCP
   的姿态。
2. 只沿前后缓慢移动约 5 mm，等一条诊断：raw X 应是主分量，base X 应同号且是主分量。
3. 回零后分别只沿左右、上下移动约 5 mm：raw 主轴应依次为 Y、Z，映射目标应依次主要
   落在 base Y、Z，并与 raw 同号。每次只动一个方向；轴向或符号不符时按空格停机。

本版不使用 Omega-local 或 TCP-local 平移语义。假机无法验证真实操作者方向。

IK 分支连续性比较相邻的完整 IK 解；它不拿 IK 解和限速后的 servo 目标比较。若 IK 解
跳变，节点会用实测关节角重试一次；仍不连续时保持同一个待求末端位姿，周期性重发最近的
有效关节目标，直到 IK 恢复或连续三次发生真正的 IK 服务错误。被拒绝的末端增量不会写入
`limited_pose`，所以恢复后会从原来的安全增量继续。

## 实时性

`command_rate_hz: 50` 是 IK 请求上限。MoveIt `/compute_ik` 一次只处理一个在途请求，
所以真实输出频率取决于 IK 延迟，不能保证达到 50 Hz。关节步长按两次成功 IK 之间的真实
时间计算，因此 IK 低于 50 Hz 时不会再次把速度按比例缩小。

默认限值：

- 笛卡尔平移 `0.15 m/s`，旋转 `0.60 rad/s`。
- 节点关节目标 J1–J6 `0.12 rad/s`，J7 `0.10 rad/s`。
- 底层 Ruckig `0.18 rad/s`、`0.60 rad/s²`、`3.0 rad/s³`。
- 目标超时 `0.25 s`，单次关节目标跳变 `0.04 rad`，最大跟踪误差 `0.08 rad`。

节点目标上游限速低于底层 Ruckig 限值，J1–J6 留 `0.06 rad/s`、J7 留 `0.08 rad/s`
余量用于控制器追赶。此前两次真机停止前的 tracking error 到达 `0.080 rad`，所以保留
`0.08 rad` 停机阈值，没有放宽；通过降低上游目标速度给底层连续轨迹留出追赶余量。
需要调节时先看诊断中的实际 IK Hz 和跟踪误差。行程不足优先检查 `workspace_min/max`，
再调 `position_scale`；调整关节目标速度时仍要低于控制器 Ruckig 限值。

## 夹爪响应

Omega gap 仍线性映射到 Franka Hand 宽度（0–25 mm 映射到 0–80 mm），但真机现在按抓取
意图选择动作：

- 空闲时使用 `/franka_gripper/move` 做位置跟随，默认速度 `0.10 m/s`。
- `gripper_max_move_step=0.080 m` 覆盖夹爪完整行程，因此一次 Move 直接朝当前最新开度连续运动，
  不再每 10 mm 停止并重新启动 action。优先使用新鲜 `/joint_states` 手指宽度；启动时尚无测量则
  从最后发送宽度开始，首次默认为全开 `0.080 m`。
- 普通闭合不会仅因到达固定 squeeze 阈值就切换 Grasp；只有检测到物体接触，或 Omega gap 到达
  `omega_gripper_engage_gap=0.001 m`（与 full-close 阈值相同）时，才进入 `/franka_gripper/grasp`。
  正常 Grasp 目标宽度仍由 gap 映射；默认速度 `0.05 m/s`、目标抓力
  `20 N`。Grasp 的 `epsilon_inner` 不超过 `0.005 m`，`epsilon_outer` 会扩展到目标宽度以上的
  剩余物理行程（最多 80 mm），因为 Franka 按 `target ± epsilon` 判断停止宽度；这样不同宽度的
  物体接触后不会仅因超出固定 15 mm 容差而被判失败。
- 接触判断只在一个朝内 Move 明确以 `ABORTED` 结束后进行，不会在 Move 仍 active 时根据停滞时间
  抢先判断。若此时 `/joint_states` 手指宽度新鲜、Omega 仍保持闭合意图，且实测宽度比本次已接受的
  精确 Move 目标大至少 `gripper_contact_error_threshold=6 mm`，才判定物体挡住夹爪并切换到 Grasp；
  Grasp 目标采用实测接触宽度。Move 成功会清除此判断，再从新鲜测量宽度应用最新 Omega 目标。
  更窄的后续 Omega 目标不会修改当前已接受 Move 的误差基准。反向张开会立即清除接触候选；主动取消、
  action 状态/结果不明确或关节测量过期也不会触发 force grasp，而是记录诊断并只在目标变化后安全重试。
- Omega gap 到 `omega_gripper_full_close_gap=1 mm` 或更小，即使缺少新鲜关节反馈/接触检测未触发，
  也会走 Grasp 而不是继续 Move，并将目标设为闭合端点。接触检测命中则优先采用实测接触宽度。
  Grasp 成功后锁存持握，不因继续捏合而重复发送更窄目标；失败时仍可用更新后的更窄目标重试。
- 显式 full-close Grasp 只有 gap 回升到 `omega_gripper_release_gap=0.015 m` 才退出；自动接触 Grasp
  还要求 gap 比接触时再打开 `omega_gripper_contact_release_margin=2 mm`，且至少到达 15 mm。
  这避免在接触点本来已大于 15 mm 时立即释放，同时不把接触抓取错误地锁到全开。退出时 Move 目标至少打开到最近一次成功 Grasp 的实测宽度
  加上有效 `epsilon_outer` 和 `gripper_release_margin=0.010 m`；若 Omega 映射要求开得更多，则采用更宽目标，
  并限制在 80 mm 最大行程内，再恢复普通位置跟随，
  以免刚释放就把手指重新合回物体上。

Move/Grasp 各自最多有一个在途动作。同类型的新目标只保留最新值，当前动作结束后马上发送，
不会对 Omega 每个 200 Hz 样本都取消并重发。Move↔Grasp 意图切换时只请求一次取消，等服务端
停止当前夹爪命令后发送最新意图。默认 `gripper_deadband=1 mm`、最小命令间隔 `50 ms`；这减少
动作队列积压和取消等待。Move 仍是完整 action 而非流式 servo：真机实测每条 action 有明显的
固定启动/结束开销，原来的 10 mm 分段会让夹爪反复减速、停止、再启动。现在直接发送完整目标，
开合过程更连续；同模式的新目标仍需等当前 action 完成后应用，方向反转的响应速度取决于当前
action 的剩余时间。任何拒绝、失败或取消都会记录日志，不会对同一个失败目标无限重试。

假机保留原有行为：Fake Hardware 不调用 Move/Grasp action，而是将 Omega gap 映射出的手指位置
发布到 `/joint_states`。它只验证位置映射，不模拟真实 Grasp 的夹持力或物体接触。

之前物体滑落的主要软件原因是只使用 Move：它控制位置，不会在碰到物体后持续施加目标抓力；
Omega gap 映射的目标宽度通常小于物体宽度，Move 可能因接触/控制误差而结束，却没有可靠的力保持。
现在闭合 Move 只有在 action 明确失败、手指测量仍新鲜且相对本段目标存在足够误差时才转为
Grasp 力闭合/保持；成功的慢 Move 不会被误判为接触。Grasp 成功会锁存，
直到松开阈值触发释放。仍可能因抓力设置偏低、物体表面摩擦小/形状不利、负载或机械臂加速度较大
而滑落。增大抓力会提高夹紧能力，也会增大物体受力；应从默认值开始，在物体与夹爪安全的前提下
逐步调节。节点没有独立的滑移传感器，Grasp 成功结果也不能替代现场确认。

## 安全与故障恢复

设备或关节状态超时、控制器失活、关节速度异常、连续 IK 错误、关节限位或跟踪误差越界
都会关闭机械臂使能。IK 分支跳变会保持最后有效目标并继续尝试，不会改变 IK 分支后突然跳动。
控制器会在非实时状态线程对首次锁存故障输出一条带原因的 ERROR 日志。原因包括目标消息结构、
关节名/顺序、非有限目标、目标接近关节限位、目标跳变、Ruckig 错误、跟踪误差、命令接口写入
失败，以及无效控制周期或实测状态；适用时日志包含关节编号、观测值和阈值。原有
`/omega7_servo_controller/fault_latched` Bool 话题保持兼容。短暂目标超时会先停在当前轨迹附近；
新鲜目标恢复后，控制器先用实测关节位置和速度重新初始化轨迹，再按 `max_target_jump` 检查
新目标并接管。
控制器 spawner 激活失败时整套 launch 会退出，避免出现夹爪可用而机械臂控制器未启动的假象。

真机若在 Desk 显示 `User stopped`、reflex 或锁定，先停止脚本，在 Desk 中确认现场安全、
解除停止并解锁，再重新运行入口。脚本不会自动清除机器人错误。RViz 中
`/recognize_objects not available` 与机械臂伺服无关。

首次真机验证应清空机械臂周围区域，先按上面的步骤分别测试前后、左右和上下三个方向；
确认 2 倍比例和实际 IK/跟踪诊断正常后，再进行组合运动。离手前按空格确认机械臂已停用；
实体急停仍是异常情况下的停机手段。

## 参数与验证

主参数位于 `config/omega7_fresh.yaml`，控制器参数位于
`../omega7_servo_controller/config/controllers.{real,fake}.yaml`。

```bash
cd ros_ws
python3 -m pytest src/omega7_fresh/test
colcon build --symlink-install --packages-select omega7_servo_controller omega7_fresh
colcon test --packages-select omega7_servo_controller omega7_fresh
bash src/omega7_servo_controller/test/run_fake_smoke.sh
bash src/omega7_servo_controller/test/run_full_fake_smoke.sh
```

可在已 source 工作区的终端显式关闭机械臂：

```bash
ros2 service call /omega7_fresh/set_motion std_srvs/srv/SetBool '{data: false}'
```

锁存状态发布在 `/omega7_fresh/motion_enabled`，日志保存在
`reports/omega7_fresh_ros/`。
