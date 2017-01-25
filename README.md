#“恩智浦” 杯智能汽车大赛 - 新电五队托管项目

 `天津科技大学` `新电社` `新电五队` `摄像头组`

 **在遵循 GPL v3 协议的前提下，合理参考工程，欢迎开源共享！** 


### 硬件详情：

- 车模：B
- 伺服器：SD-5
- 主控：MK60FN1M0VLQ15 + MK60DN512VLL10 (双芯)
- 电机驱动：BTN7971B
- 摄像头：野火 OV7725
- 陀螺仪：Invensense MPU6050 （未使用）
- 编码器：龙邱512线 霍尔增量编码器

### 编译环境：

- IAR for ARM 7.5 （请尽可能保证软件版本相同，防止出现意外情况）

### 目录结构：

- `[lib]` -- 拉普兰德底层
- `[pcb]` -- PCB文件
- `[project]` -- 工程文件，包含 **Logic - 逻辑处理器** 和 **Ctrl - 控制处理器** 