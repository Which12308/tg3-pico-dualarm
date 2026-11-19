# 上游项目核查与复用决定

核查日期：2026-11-14。以下链接均访问了仓库、README、源码目录和许可证；“仓库存在”不表示已在这台天工上通过硬件测试。本文核查的是当天公开状态，运行依赖仍应固定版本。

| 项目 | 核实内容 | 本项目的复用边界 |
|---|---|---|
| [XR-Robotics](https://github.com/XR-Robotics) / [Teleop Sample Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python) | Python 遥操作框架，含仿真及 UR、ARX、Galaxea 等硬件适配；提供左右控制器位姿、相对映射、Placo 求解和数据记录接口。README 的安装脚本测试环境为 Ubuntu 22.04；没有现成天工双臂适配器。[许可证 MIT](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/main/LICENSE) | 继续复用其坐标变换与相对位姿映射思路/固定源码，成熟 Placo 任务求解；机器人通信、模式权限和电机顺序必须单独适配。旧项目固定样例提交 `79e5cb8a56e3455515ce1b476e993c764ec58739`；新仓库的具体固定来源见随包来源说明 |
| [PC Service Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) | Python 包 `xrobotoolkit-sdk` 提供左右手柄、头显位姿、按钮/轴和时间戳；需要另外运行 PC Service 并连接头显。当前提供 Linux x86_64、aarch64、Windows x64 的预编译接口，Python ≥3.10。[包装器 MIT](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/LICENSE)，[二进制来源说明](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/vendor/README.md) 列出 SDK 架构和 glibc 要求 | 作为可选原生追踪后端；不能把它当机器人写入接口。没有安装或连接该服务时，应明确报后端不可用，不能生成假追踪值。SDK 二进制与依赖需分别保留其许可声明 |
| [Unity Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client) | 面向 PICO 的客户端，配套 PC Service 传输控制器 6DoF、头部与选配追踪数据，要求设备与接收端连通；README 示例为 PICO 4 Ultra。头显摄像头原始数据还涉及企业授权。[主体 MIT](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/blob/main/LICENSE.txt)，PICO Integration 使用独立许可 | 可选择部署其现成客户端；本机已跑通的 WebXR 方案也可继续用于左右控制器输入。两条输入路径应收敛到同一数据契约，不能在缺少实际服务时声称原生链路已验证 |
| [天工官方 SDK](https://github.com/Open-X-Humanoid/xhumanoid_sdk) | ROS 2 Jazzy / Ubuntu 24.04 本体示例。实际[单关节控制源码](https://github.com/Open-X-Humanoid/xhumanoid_sdk/blob/main/single_joint_control/python/single_joint_control/cmd_publisher.py) 用 `/arm/cmd`、`ArmCtrl/MotorCtrl` 与 `/robot_state`，演示 ID 21 的位置和混合控制；README 的后续计划包含多关节/全身例程。[Apache-2.0](https://github.com/Open-X-Humanoid/xhumanoid_sdk/blob/main/LICENSE) | 核对本机消息与单位；不能从单关节例程推断完整双臂仲裁、平衡共享、急停状态和恢复权限。新仓库只读探针与模拟后端已可工作，硬件写入所缺的契约详见 `robot_interfaces.md` |
| [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) | 提供 PICO/Quest/AVP 输入、双臂操作与采集经验；目标为 G1/H1/H1_2/H2/R1 等宇树机器人，依赖 Unitree SDK2。README 说明了手柄/裸手、录制及头朝向参考模式。[主许可证 Apache-2.0](https://github.com/unitreerobotics/xr_teleoperate/blob/main/LICENSE)，并列出第三方组件许可 | 参考操作流程、朝向标定和录制设计；不运行其 Unitree 电机通信，不复用其关节 ID、增益或低层命令到天工。它不是天工通用即插即用驱动 |

## 采用成熟 IK，不另写数值求解器

本仓库的双臂任务使用 [Placo 0.9.26](https://pypi.org/project/placo/0.9.26/)；机器人几何来自本机已核实模型。Placo 是以 Pinocchio 为基础的任务求解库；这里使用其关节限位、速度限位、末端位置/位姿任务和弱关节姿态任务。[位置/关节任务文档](https://placo.readthedocs.io/en/v0.9.26/kinematics/joints_task.html)、[正则化文档](https://placo.readthedocs.io/en/v0.9.26/kinematics/regularization.html) 支持这种姿态偏好的实现。

双臂模型只保留已知运动链，尚未构建完整带网格的碰撞场景。简单工作区/距离检查不能被标成完整自碰撞证明。姿态偏好用于减轻冗余关节随意绕转；单个手柄只有末端位姿，不能测得操作者真实肘位置。

此前原生环境暴露过 CMeel 共享库分散在系统与虚拟环境的问题；应把相容依赖安装进同一隔离环境。Placo 0.9.26 的 PyPI 元数据固定 Pin 3.8.0，该版本依赖 Boost 1.89。当前 Python 3.12 的 Boost wheel 要求 NumPy 2.3 系列；ROS 自定义消息类型支持也应在同一环境实际导入测试，不能仅验证 `import rclpy`。

## 当前复用结论

保留已验证的 PICO 输入与 XRoboToolkit/Placo 变换求解基础，补齐左右输入、显式标定、独立接管和记录契约。天工输出侧使用清晰的 RobotProtocol；随仓库交付的是真实可运行的模拟/只读实现。后续接入经核实的硬件适配器时，不必改写输入和 IK，更不能把接口缺口藏在“生产可用”的标识里。
