# STM32 智能救援小车固件（CubeMX 友好形态）

本目录只包含**业务层代码**，不含任何 CubeMX 生成文件，
与 CubeMX 工程零冲突。配置步骤见
[CUBEMX_配置与合并指南.md](./CUBEMX_配置与合并指南.md)。

## 文件结构

```
firmware_framework/
├── CUBEMX_配置与合并指南.md   # CubeMX 逐步点选 + main.c 3 行插入清单
├── README.md
└── Core/
    ├── Inc/
    │   ├── app.h              # 业务宏定义 + App_Init/App_Tick 接口
    │   ├── pi_protocol.h      # 树莓派 A5 5A 帧协议 (CRC16-CCITT)
    │   ├── stp23l.h           # STP23L 激光测距 (DMA/中值滤波)
    │   ├── motor_pid.h        # 抗饱和位置式速度 PID
    │   ├── kinematics.h       # 三轮全向轮运动学 + 物理尺度常量
    │   ├── odometry.h         # 编码器里程计 (x/y/θ)
    │   └── wdg.h              # IWDG 4s 看门狗
    └── Src/
        ├── app.c              # 业务总装: 闭环/状态机/按键/安全/HAL 回调
        ├── pi_protocol.c
        ├── stp23l.c
        ├── motor_pid.c
        ├── kinematics.c
        ├── odometry.c
        └── wdg.c
```

## 使用方式（3 步）

1. **CubeMX 按指南配置并生成工程**（芯片 STM32F407VET6，7 个定时器 + 3 路 USART + DMA + 5 个 GPIO）
2. **复制 14 个文件**到工程的 `Core/Inc`、`Core/Src`（Keil 还需把 7 个 .c 加入工程组）
3. **main.c 插 3 行**：

```c
/* USER CODE BEGIN Includes */   →  #include "app.h"
/* USER CODE BEGIN 2 */          →  App_Init();
/* USER CODE BEGIN 3 */ (循环内) →  App_Tick();
```

重新 Generate 时代码不会丢失（全部在独立模块或 USER CODE 保留区内）。

## 引脚分配（与 CubeMX User Label 对应）

| 引脚 | 功能 | Label |
|------|------|-------|
| PA8/PE11 | 电机1 PWM 双向 | TIM1_CH1/CH2 |
| PE13/PE14 | 电机2 PWM 双向 | TIM1_CH3/CH4 |
| PC6/PC7 | 电机3 PWM 双向 | TIM8_CH1/CH2 |
| PA0/PA1 | 编码器1 | TIM2 |
| PA6/PA7 | 编码器2 | TIM3 |
| PB6/PB7 | 编码器3 | TIM4 |
| PE5/PE6 | 左/右铲板舵机 50Hz | TIM9_CH1/CH2 |
| PB8 | 摄像头舵机 50Hz | TIM10_CH1 |
| PA9/PA10 | 树莓派 UART 115200 | USART1 |
| PA2/PA3 | 左 STP23L 230400 DMA | USART2 |
| PB10/PB11 | 右 STP23L 230400 DMA | USART3 |
| PE3 | 一键启动按键（短按发车/长按解急停） | START_BUTTON |
| PE4/PE7 | 碰撞开关 | COLLISION_LEFT/RIGHT |
| PC8/PC9 | 状态 LED / 蜂鸣器 | LED_STATUS/BUZZER |

## 运行时行为

```
10ms  编码器 → 里程计 → 3 路速度环 PID → PWM
每轮  按键消抖 → 转运状态机 → LED
50ms  碰撞急停 / 视觉心跳超时监督
100ms 编码器 + 测距遥测回传树莓派
每轮  树莓派命令 TTL 监督 / 测距读取 / IWDG 喂狗
```

控制链统一单位为「编码器计数/10ms」（500 线 × 4 倍频 × 减速比 30 = 60000 计数/轮周）。
安全层：碰撞 EXTI 急停 → 命令 TTL 250ms 停车 → 心跳 1s 超时回 IDLE → IWDG 4s 复位。

## 上电后必须标定（代码内为占位初值）

1. 三个电机转向与编码器极性（抬轮验证）
2. PID 参数 `PID_KP/KI/KD`（[app.h](Core/Inc/app.h)）
3. `PI_CMD_1000_COUNTS` 满速映射、轮径/减速比/轮距（[kinematics.h](Core/Inc/kinematics.h)）
4. 铲板三个舵机角度、HSE 晶振频率（非 8MHz 时改 CubeMX 时钟树）

调试观察变量（ST-LINK Watch）：
`debug_state`、`debug_ctrl_mode`、`debug_enc_delta[]`、`debug_motor_target/output[]`、
`debug_odom_x/y/theta`、`debug_distance_left/right`。
