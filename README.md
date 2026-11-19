# 天工 3.0 × PICO 双臂遥操作 · 源码审阅

这是 2026-11-18 最后上机版本（候选 08）的审阅快照。已去除操作台预填的现场地址和测试中引用的该地址，省略了本机记录、机器人模型、安装二进制和自动 CI。复现运行需要项目原始资产及相应原生环境。

本次真机通过了输入接收、厂家接管和停用恢复。右握持键触发了跟随，原生求解已运行，但右臂只短暂响应，随后因 `xarm_feedback_lead_exceeded` 退出。持续跟随和抓取尚未通过。

- [controller.py](src/tg3_pico_dualarm/controller.py) 将手柄输入映射为关节目标并执行滤波。
- [safety.py](src/tg3_pico_dualarm/safety.py) 限制速度、加速度以及目标与反馈的偏差。
- [xarm_adapter.py](src/tg3_pico_dualarm/xarm_adapter.py) 在提交时重新读取实测反馈并逐关节检查偏差。
- [bounded_xarm_transport.py](src/tg3_pico_dualarm/bounded_xarm_transport.py) 约束实际发送的手臂指令。
- [现场问题与旧遥控的区别](REVIEW_STATUS.md)
