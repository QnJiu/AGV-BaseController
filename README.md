
<img width="240" height="426" alt="电机控制与通信" src="https://github.com/user-attachments/assets/778c851a-9a83-44ff-8f3b-205f0c821e35" />


AGV-BaseController: 基于 STM32 与 ros2 的两轮差速底盘控制系统
嵌入式底层控制 (STM32 端)
* **实时任务调度**：基于 FreeRTOS 构建，将系统拆分为通信 (`CommTask`)、测速解算 (`SensorTask`) 与电机执行 (`MotorTask`) 三个独立线程，有效避免阻塞，确保控制周期精准。
* **高精度闭环控制**：采用增量式 PID 算法，结合定时器四倍频模式采集霍尔编码器脉冲，实现对空载/负载状态下电机转速的丝滑抗扰动控制。
* **完善的保护机制**：涵盖了 PWM 限幅保护、串口溢出预防以及系统栈溢出（Stack Overflow）优化。
  硬件清单

* **主控芯片**：STM32F407VET6 (基于 Cortex-M4 内核)
* **电机驱动**：TB6612FNG 双路直流电机驱动模块
* **执行机构**：带霍尔编码器的直流减速电机（2 个）
* **通信模块**：USB 转 TTL 串口模块（如 CH340）
* **供电系统**：12V 动力电池组与 LM2596 降压模块
