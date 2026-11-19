# 天工 3.0 接口调查与硬件边界

核查日期：2026-11-14。这里区分本机归档证据、公开示例与尚未验证的控制权限。新仓库没有创建运动发布者，也没有替换原厂平衡控制器。

## 已核实的来源

- 本机实际模型：`models/tiangong3.urdf`，SHA-256 为 `ddeb5f101da23e73d4d9f1c9d7e29b890b4a81a2443b747336e31f673b47225c`。模型根链、关节原点及左右关节名由 XML 读取；不能用其他天工型号的同名文件代替。
- 工作区原始证据：`robot_control_info/20261031_125330_cc6d87a7/{manifest.json,original_control.zip,integration_report.json,effective_bridge_parameters.yaml}`，以及 `robot_control_info/tg3_control_checks/20261031_115010_82c50b6f/ros_graph.json`。归档包含实际运行的原厂 release `20260814_16`、启动 shim、遥控配置和源码。
- 已有适配器：`tg3_grasp_remote_v2_20261031/joystick_bridge/joint_control_service.py`、`right_arm_hand_remote.py`、`trial_switch.py`。这些文件提供本机曾使用的混合控制、反馈校验、接管与恢复逻辑，不能据此宣称新双臂流程已通过真机。
- [官方 SDK](https://github.com/Open-X-Humanoid/xhumanoid_sdk) 面向机器人本体 Ubuntu 24.04、ROS 2 Jazzy；[实际单关节源码](https://github.com/Open-X-Humanoid/xhumanoid_sdk/blob/main/single_joint_control/python/single_joint_control/cmd_publisher.py) 控制 ID 21，使用 0.02 秒周期。它验证了消息的使用方法，没有提供经本机验证的完整双臂仲裁流程。

## 14 关节顺序、单位与限位

所有新仓库向量固定为左臂 7 个值，再接右臂 7 个值，位置单位 rad、速度单位 rad/s。不能按反馈数组的排列位置认关节，必须按 `name` 中的电机 ID 排序。

| 序号 | 左臂 ID / URDF 关节名 | 右臂 ID / URDF 关节名 | 原厂源码角度范围（左；右） |
|---|---|---|---|
| 0 / 7 | 11 / shoulder_pitch_l_joint | 21 / shoulder_pitch_r_joint | [-2.84489, 2.84489]；相同 |
| 1 / 8 | 12 / shoulder_roll_l_joint | 22 / shoulder_roll_r_joint | [-0.191986, 3.33358]；[-3.33358, 0.191986] |
| 2 / 9 | 13 / shoulder_yaw_l_joint | 23 / shoulder_yaw_r_joint | [-2.84489, 2.84489]；相同 |
| 3 / 10 | 14 / elbow_pitch_l_joint | 24 / elbow_pitch_r_joint | [-2.46, 0.191986]；相同 |
| 4 / 11 | 15 / elbow_yaw_l_joint | 25 / elbow_yaw_r_joint | [-2.84489, 2.84489]；相同 |
| 5 / 12 | 16 / wrist_pitch_l_joint | 26 / wrist_pitch_r_joint | [-1.32645, 1.32645]；相同 |
| 6 / 13 | 17 / wrist_roll_l_joint | 27 / wrist_roll_r_joint | [-1.32645, 1.32645]；相同 |

新模型取 URDF 与原厂范围的交集，再向内收缩 0.05 rad。肘部 -2.46 是本机原厂源码记录的较紧限位；不能恢复成模型中更宽的值。它不是重新测得的机械认证。模型计算坐标根为 `waist_pitch_link`，不是自动获得的地面世界坐标；末端是 `left_tcp_link` / `right_tcp_link`，其固定连接离腕部 0.044 m，尚未等同于实测瓶子抓握中心。

## 实际消息与话题

| 用途 | 已观测接口 | 已核实结构及限制 |
|---|---|---|
| 臂命令 | `/arm/cmd`，`ros2_bridge_msgs/msg/ArmCtrl` | 本机源码设置 `header`、`mode`、`label`、`reserved`、`ctrl`；`ctrl` 是电机命令集合 |
| 单电机命令 | `ros2_bridge_msgs/msg/MotorCtrl` | `name,kp,kd,pos,spd,tor,cur`；绝对关节目标，不是笛卡尔位姿或增量位移 |
| 实测关节 | `/robot_state`，`ros2_bridge_msgs/msg/RobotState` | 实际读取路径为 `message.arm.status`，逐项检查 `name,pos,error`。必须 14 项齐全、ID 不重复、位置有限、错误码为零 |
| 模式诊断 | `/hric/robot/rl_state`，`diagnostic_msgs/msg/DiagnosticStatus` | `values` 中包含 `current_state,child_state,status`，另有 `level`；重复键、缺键与异常状态不能沿用上一帧正常结果 |
| 实体遥控输入 | `/sbus_data`，`sensor_msgs/msg/Joy` | 本机已有发布者是 `joystick`。现有 G/C 与按键手势属于原厂仲裁，不是新程序可省略的授权手续 |
| 右手反馈/命令 | `/right_hand/motor_status` / `/right_hand/set_motor_multi` | `brainco_hand_msgs/msg/MotorStatus` / `SetMotorMulti`；属于已存在的右手驱动，本仓库未增加硬件手部写入 |

运行 `python -m tg3_pico_dualarm.preflight --seconds 10` 会直接对已安装的消息类调用 `get_fields_and_field_types()`，输出真实字段和类型，再建立反馈订阅。没有在本仓库伪造 `.msg` 定义。完整消息定义以机器人上的安装版本为准。

原厂源码记录 `mode=0` 为位置模式、`mode=1` 为力位混合模式；已有右臂适配器使用混合模式、`label=0,reserved=165`。其中每臂 Kp 数组为 `[300,300,200,300,100,100,100]`，Kd 为 `[15,15,15,15,5,5,5]`，源码默认电流值为 10。这里只记录已见配置，不是给新双臂写入器的调参建议。官方单关节示例另用 Kp=50、Kd=2，进一步说明不能把示例参数直接替代本机原厂配置。

## 模式、仲裁与真实急停的缺口

旧右臂流程请求并确认 `HBWALK`，等待 200 ms 接管窗口，检查新鲜反馈，再接管右臂；退出后经过原有停止、保持或恢复路径。`SWR` / `HBWALK` 出现在预检接受范围，并不等于已经授予双臂控制权限。原厂源码确有 `_publish_both_arms` 和 11..17、21..27 的组装代码；尚未确认它与本机整机控制器共享双臂时的许可状态、独占资源、切换瞬态、恢复条件和故障反馈。

2026-11-14 的右臂试验已观察到连续跟随及松开保持；姿态正则化补丁的原生检查通过，但补丁后的硬件试验被发布者身份预检挡住。异常涉及六个控制话题。已查证恢复代码保留 `__node:=joystick_bridge`，尚待实际端点列表、进程参数和恢复日志定位原因。不能把允许名单改宽来让预检通过。

六个需审核的既有控制话题是 `/arm/cmd`、`/right_hand/set_motor_multi`、`/hric/robot/cmd_vel`、`/hric/robot/fsm_state_cmd`、`/hric/robot/stand_cmd`、`/hric/robot/fsm_resume_cmd`。只读探针区分“缺少预期发布者”“出现额外发布者”“节点元数据未知”，避免把缺失统称为冲突。它会列出额外导航端点；不会凭一次静默观察给它颁发控制权限。

没有核实能报告物理急停状态的字段或话题。`RobotState.estop` 因而使用 `None`，`STOP` 模式、实体遥控 C 键、软件 disarm 都不被转换为“物理急停已释放”。可见的机器人状态、实际急停回路与新软件的使能状态是不同概念。

## 当前可运行的边界

`robot.py` 提供 `RobotProtocol.read_state()` 与 `submit(JointCommand)`。`DryRunRobot` 接受 14 个目标并记录有界历史，只更新模拟状态；它拒绝过期、未来、倒序命令。它不模拟负载、摩擦、接触、力矩或抓瓶成功率。

`ReadOnlyRobot` 可包装真实反馈读取函数，但所有 `submit` 都拒绝。没有 `--enable-hardware`、环境变量开关、JSON 许可文件或内置运动发布者。未来硬件写入必须通过单独实现并现场核实的适配器注入；其接管、停机和恢复流程必须与原厂控制权限共同验证。

`preflight.py` 在一个持续存在的 ROS context 中观察 5～120 秒，使用本地单调时钟的接收时间检查关节反馈 0.25 秒、模式反馈 0.5 秒的新鲜度。它没有假造消息原始时间戳。即使 `observation_ready=true`，`hardware_ready` 仍为 false，且会列出缺失的双臂仲裁适配器、未验证的物理急停接口和只读边界。

新流程复用模型、成熟 IK、输入与安全过滤的离线部分，不改变原厂腿部、底盘或平衡控制。双臂硬件使能所缺的不是另一个话题名称，而是经厂家接口及现场测试确认的双臂控制权交接与故障恢复契约。

## ROS 2 包与观察桥接节点

仓库提供 `ament_python` 包、`bridge_node` 可执行入口和 launch 文件。默认节点仅发布标准观察数据：

| 观察话题 | 标准消息 | 内容 |
|---|---|---|
| `/tg3_teleop/dry_run/joint_states` | `sensor_msgs/msg/JointState` | 14 个模拟关节位置，frame_id 明确包含 `dry_run_simulated`；不伪造连续测量速度或力矩 |
| `/tg3_teleop/diagnostics` | `std_msgs/msg/String` | JSON：模拟状态、最后模拟目标、提交结果、软件 disarm、真实反馈观察结果及硬件阻塞原因 |

以下为机器人具备 ROS 2 Jazzy 与已安装自定义消息之后的标准构建方式；本仓库开发验证所在的 Windows 环境没有执行 `colcon build`，不能宣称原生 ROS launch 已跑通。

```bash
# 将仓库放进 ~/tg3_ws/src/tg3_pico_dualarm 后，在工作空间根运行
cd ~/tg3_ws
source /opt/ros/jazzy/setup.bash
source ~/xos/setup.bash
colcon build --packages-select tg3_pico_dualarm --symlink-install
source install/setup.bash
ros2 launch tg3_pico_dualarm teleop.launch.py mode:=dry_run
```

只读比较现有机器人反馈使用：

```bash
ros2 launch tg3_pico_dualarm observe_robot.launch.py
```

这会订阅现有 `/robot_state` 和模式诊断，不发任何机器人命令；实际反馈通过 `ReadOnlyRobot` 单独暴露，不会覆盖模拟位置。物理急停仍报告未知。

`teleop.launch.py mode:=hardware` 或 `ros2 run tg3_pico_dualarm bridge_node --mode hardware` 会在创建节点与发布者之前明确拒绝。没有 force 开关。launch 名称中的 teleop 不代表已授权真机双臂接管。

独立 `bridge_node` 提供模拟状态的 ROS 可观察性和程序内 `submit_simulated(JointCommand)` 接口，启动时保持模型参考位置；它没有自动启动 PICO 输入或 IK 循环。完整输入、映射与 IK 离线控制由 `tg3-vr serve` / `demo` 执行。

需要同时使用 PICO 输入循环与 ROS 观察时，使用 `tg3-vr serve --ros`，可按平台能力添加 `--preview` 或配置原生 Placo。该选项通过 `RosDryRunSession` 的同一模拟适配器执行控制和发布观察数据，不另建一个互不相通的模拟状态。`RosDryRunSession.robot` 满足现有 RobotProtocol；程序内调用 `submit` 后，ROS JointState 与 JSON 诊断会反映同一组被接受的 14 关节目标。它使用私有 ROS context 与独立 executor 线程，关闭时停止线程并销毁节点；线程异常会使模拟器软件 disarm，并拒绝后续目标。这个集成选项仍没有机器人运动输出。


## 0.2.0 官方格式器与具体缺失证据

`tiangong_protocol.ArmCommandFormatter`依据厂家公开 `single_joint_control` 的消息字段建立格式器；对实际安装消息类做字段/类型核对，使用显式14项Kp/Kd、独立ROS时钟及真实电机编号。这里只在内存中形成消息，没有publish接口，官方示例未设置的保留位接管数值不会被加入。

本地厂家代码存在一次构造双臂14电机ArmCtrl的函数，因此缺失项不是“没有命令数组”或“关节编号不清楚”。目前没有获得：实际低层ArmCtrl消费者的双臂所有权逻辑、每状态arms_control_status/stop与HBWALK臂配置、接管与恢复的厂家契约，以及可读取的物理急停状态。现有归档没有这些定义；已有双臂动作函数不先请求FSM，不能据此证明HBWALK会释放左臂。

现场需要核对已安装的真实IDL/字段、实际控制订阅节点、该订阅节点对应的配置或SDK支持说明。确认之前不能把“能格式化14关节”当作“已支持安全双臂并发执行”。
