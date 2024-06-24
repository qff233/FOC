# 基于Embassy操作系统开发的高性能高精度FOC控制固件库

### 文件分类

- ./crates/foc FOC的具体实现
- ./crates/stm32-g474 Axdr开发板的接口具体实现

  ps: Axdr作者来自电磁诡力★FOC交流群：957377627
  具体网页：https://www.disnox.top/docs/AxDrive-L_user_manual

### 当前实现的算法

- 电流环前馈PID
- 速度环前馈PID
- 位置环前馈PID
- 基于Pll的速度追踪算法
