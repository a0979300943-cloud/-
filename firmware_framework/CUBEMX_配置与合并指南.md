# CubeMX 配置与代码合并指南

适用：STM32CubeMX 6.x + STM32F407VET6，目标工具链 Keil MDK-ARM（AC6 推荐）或 STM32CubeIDE。
全程只需鼠标点选 + 最后复制 14 个文件 + main.c 插入 **3 行**。

---

## 一、新建工程

1. `File → New Project → MCU Selector`，输入 **STM32F407VET6**，双击选中，Start Project。

## 二、System Core 基础配置

### 1. SYS
| 选项 | 值 |
|------|-----|
| Debug | **Serial Wire**（必选，否则烧录一次后 SWD 失联） |
| Timebase Source | SysTick（默认） |

### 2. RCC
| 选项 | 值 |
|------|-----|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator**（按板载 8MHz 晶振） |

### 3. NVIC（优先级分组保持默认 4 bits 抢占）
先在各外设页面使能中断，最后统一在 `System Core → NVIC` 调整优先级：

| 中断 | Preemption Priority |
|------|-------------------|
| USART2 global interrupt | 0 |
| USART3 global interrupt | 0 |
| DMA1 stream1 global interrupt (USART3_RX) | 1 |
| DMA1 stream5 global interrupt (USART2_RX) | 1 |
| EXTI line4 interrupt (左碰撞) | 2 |
| EXTI line[9:5] interrupts (右碰撞 PE7) | 2 |
| EXTI line3 interrupt (启动按键) | 3 |
| USART1 global interrupt (树莓派) | 5 |

## 三、Clock Configuration（直接填数值回车，CubeMX 自动配 PLL）

```
Input frequency:    HSE 8 MHz
PLL Source Mux:     HSE
PLL:                PLLM = 8
                    PLLN = 336
                    PLLP = 2
                    PLLQ = 7
System Clock Mux:   PLLCLK
HCLK  (AHB):        168 MHz
APB1 Prescaler:     /4  →  42 MHz (定时器倍频后 84MHz)
APB2 Prescaler:     /2  →  84 MHz (定时器倍频后 168MHz)
```

## 四、GPIO 配置（引脚总表）

在芯片引脚图上逐个点选，再到 `System Core → GPIO` 逐行核对参数，
**User Label 必须与下表完全一致**（会生成 app.c 依赖的宏名）：

| 引脚 | 模式 | 上下拉 | 触边沿 | User Label |
|------|------|--------|--------|-----------|
| PE3 | External Interrupt Mode | Pull-up | Rising/Falling | `START_BUTTON` |
| PE4 | External Interrupt Mode | Pull-up | Falling | `COLLISION_LEFT` |
| PE7 | External Interrupt Mode | Pull-up | Falling | `COLLISION_RIGHT` |
| PC8 | GPIO_Output | No pull | — | `LED_STATUS` |
| PC9 | GPIO_Output | No pull | — | `BUZZER` |

输出引脚细节：Output Push Pull、Low speed、GPIO output level = Low。
中断使能：在 GPIO 表格 NVIC 列勾选 EXTI line3 / line4 / line[9:5]。

> 硬件：PE3/PE4/PE7 各接一个常开按键/微动开关到 GND（内部上拉，按下为低电平）。

## 五、定时器配置（7 个）

### 1. TIM1 — 电机 1、2 PWM（APB2 定时器时钟 168MHz）

- Combined Channels 下方，将 Channel1~Channel4 依次选为 **PWM Generation CHx**
- Parameter Settings：

| 参数 | 值 |
|------|-----|
| Prescaler (PSC) | 0 |
| Counter Mode | Up |
| Counter Period (ARR) | 16799 |
| auto-reload preload | Enable |
| CH1~CH4 PWM Mode | PWM mode 1 |
| CH1~CH4 Pulse (CCR) | 0 |
| CH polarity | High |

→ PWM 频率 = 168MHz / 16800 = **10kHz**

**引脚必须手动指定**（PA9/PA10 已被 USART1 占用，CubeMX 不会给默认脚），
在芯片引脚图上逐个点选：

| 通道 | 引脚 | 接法 |
|------|------|------|
| TIM1_CH1 | **PA8**（默认） | 电机1 AIN1 |
| TIM1_CH2 | **PE11**（引脚图手选 TIM1_CH2） | 电机1 AIN2 |
| TIM1_CH3 | **PE13**（手选 TIM1_CH3） | 电机2 BIN1 |
| TIM1_CH4 | **PE14**（手选 TIM1_CH4） | 电机2 BIN2 |

> 引脚依据（F407 AF1）：PA8/PE9=CH1、PA9/PE11=CH2、PA10/PE13=CH3、PA11/PE14=CH4。
> PA9/PA10 归 USART1，故 CH2/CH3 只能走 PE 侧；CH4 统一也用 PE14，避开 USB 引脚 PA11。
> app.c 中通道映射（CH1+CH2=电机1，CH3+CH4=电机2）与此表一致，无需改代码。

### 2. TIM8 — 电机 3 PWM

- Channel1 = PWM Generation CH1，Channel2 = PWM Generation CH2
- PSC=0，ARR=16799，auto-reload preload=Enable，CCR=0
- 默认引脚 PC6/PC7 即目标引脚，无需改

### 3. TIM2 — 编码器 1（32 位）

- Clock Source 设为 **Encoder Mode**
- Encoder Mode: **TI12**
- Polarity: Rising Edge ×2；IC Filter: 0
- Counter Period: **4294967295**
- 引脚 PA0/PA1 自动分配

### 4. TIM3 — 编码器 2（16 位）

- Encoder Mode TI12，Period **65535**，Filter 0
- 引脚 PA6/PA7 自动分配

### 5. TIM4 — 编码器 3（16 位）

- Encoder Mode TI12，Period **65535**，Filter 0
- 引脚 PB6/PB7 自动分配

### 6. TIM9 — 铲板舵机 PWM（50Hz）

- Channel1 = PWM Generation CH1，Channel2 = PWM Generation CH2
- PSC = **167**，ARR = **19999**，Pulse(CCR) = **1500**，auto-reload preload=Enable
- 引脚 PE5/PE6 自动分配
- → 计数时钟 168MHz/168 = 1MHz，周期 20000 计数 = 20ms = **50Hz**

### 7. TIM10 — 摄像头舵机 PWM（50Hz）

- Channel1 = PWM Generation CH1
- PSC=167，ARR=19999，Pulse=1500
- 引脚 PB8 自动分配

## 六、串口配置

### USART1 — 树莓派通信
| 参数 | 值 |
|------|-----|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity / Stop | None / 1 |
| NVIC | ☑ USART1 global interrupt |
| DMA | 无 |

引脚 PA9(TX)/PA10(RX)。

### USART2 — 左 STP23L
| 参数 | 值 |
|------|-----|
| Mode | Asynchronous |
| Baud Rate | **230400**，8N1 |
| NVIC | ☑ USART2 global interrupt |
| DMA Settings → Add | **USART2_RX**：DMA1 Stream5 / Channel4，Mode=**Circular**，Priority=High，Memory Increment=☑，Periph/Mem Data Width=Byte |

引脚 PA2(TX)/PA3(RX)。

### USART3 — 右 STP23L
同 USART2：230400、8N1、使能 NVIC；
DMA Add **USART3_RX**：DMA1 Stream1 / Channel4，Circular，High。
引脚 PB10(TX)/PB11(RX)。

> 添加 DMA 后 CubeMX 会自动勾选对应 DMA Stream 中断，回 NVIC 页核对优先级。

## 七、Project Manager

1. **Project 页**：填工程名/路径；Toolchain/IDE 选 `MDK-ARM V5.27`（或 CubeIDE）。
2. **Code Generator 页**（重要）：
   - ☑ **Generate peripheral initialization as a pair of '.c/.h' files per peripheral**
   - ☑ Keep User Code when re-generating（默认勾上，勿取消）
   - ☑ Copy only the necessary library files
3. 点 **GENERATE TOOL CODE**，生成后先编译一次，确认 0 Error（空工程基线）。

## 八、合并业务代码（14 个文件 + 3 行代码）

### 1. 复制文件

把本框架 `Core/Inc` 的 7 个头文件和 `Core/Src` 的 7 个源文件，
复制到 CubeMX 工程对应的 `Core/Inc`、`Core/Src`：

```
app.h / app.c                 ← 业务总装（状态机/闭环/按键/安全）
pi_protocol.h / .c            ← 树莓派 A5 5A 协议
stp23l.h / .c                 ← 激光测距 DMA 解析
motor_pid.h / .c              ← 速度环 PID
kinematics.h / .c             ← 三轮全向轮运动学
odometry.h / .c               ← 里程计
wdg.h / .c                    ← IWDG 看门狗
```

- **CubeIDE**：刷新工程即可自动纳入编译。
- **Keil**：在 Project 树 `Application/User/Core` 组右键 →
  `Add Existing Files`，把 7 个 `.c` 全部加入。

### 2. main.c 只插 3 处

```c
/* === 第 1 处: USER CODE BEGIN Includes === */
#include "app.h"
/* USER CODE END Includes */
```

```c
  /* === 第 2 处: USER CODE BEGIN 2 ===
     位于全部 MX_xxx_Init(); 之后 */
  App_Init();
  /* USER CODE END 2 */
```

```c
  /* Infinite loop 内 === 第 3 处: USER CODE BEGIN 3 === */
    App_Tick();
  /* USER CODE END 3 */
```

不要把任何回调函数写进 main.c：
`HAL_GPIO_EXTI_Callback`、`HAL_UART_RxCpltCallback` 等已在 app.c 内实现，
链接器会自动覆盖 HAL 库的 weak 空函数。

### 3. 编译选项

- `stm32f4xx_hal_conf.h` 中确认 `#define HAL_IWDG_MODULE_ENABLED`（CubeMX 默认开启）。
- **Keil AC5 用户**：Options → C/C++ → Language C 选 **c99**；推荐直接切 AC6。
- 浮点：app.c 使用 `cosf/sinf/sqrtf/fabsf` 单精度函数，CubeMX 默认的
  sp-only FPU 设置直接兼容，无需改动。
- IWDG **不需要**在 CubeMX 勾选，wdg.c 调用 `HAL_IWDG_Init()` 时会自行开启 LSI。

## 九、编译后自检顺序

1. 0 Error 烧录 → LED 慢闪 = 进入 IDLE（此时电机不转，闭环目标为 0）
2. ST-LINK Watch 窗口看 `debug_state`（IDLE=0）、`debug_odom_x/y`
3. 手转轮子看 `debug_enc_delta[]` 符号/数值
4. 测距对墙看 `debug_distance_left/right`
5. 短按 PE3：`debug_state` 进入 SEARCH（=1，本地原地扫描兜底）
6. 碰撞开关接地：立即进入 EMERGENCY_STOP（=8）+ 蜂鸣器；长按 1s 解除
7. 接树莓派：心跳建立后发 MOTOR_COMMAND，观察 `debug_ctrl_mode=1`；
   拔 TX 线，250ms 后自动回 0 停车

## 十、后续重新生成代码时

CubeMX 改配置重新 Generate 时：
- 14 个业务文件和 main.c 中 3 段 USER CODE 都会保留，不会丢失；
- 若新增/删除了外设，只需重新确认 app.c 中引用的句柄名不变
  （htim1/2/3/4/8/9/10、huart1/2/3）。
