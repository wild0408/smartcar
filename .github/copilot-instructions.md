# TC264 Smart Car Codebase Instructions

## Project Overview
智能小车项目基于逐飞科技(Seekfree)的TC264开源库，运行在Infineon AURIX TC264D微控制器上。这是一个完整的视觉循迹智能小车系统，采用模块化架构，支持实时图像处理、PID控制、元素识别和路径规划。

## Architecture & Core Components

### Module Organization
- **`code/`** - 应用层功能模块（所有自定义智能小车功能）
- **`user/`** - 用户层：CPU多核心入口点（cpu0_main.c, cpu1_main.c）和中断服务例程
- **`libraries/`** - 硬件抽象层和设备驱动
  - `zf_driver/` - MCU外设底层驱动（PWM, GPIO, 编码器等）
  - `zf_device/` - 传感器/外设设备驱动（MT9V03X摄像头, TFT180显示屏等）
  - `zf_common/` - 通用工具函数和类型定义
  - `infineon_libraries/` - Infineon官方iLLD库

### Control Flow Architecture
1. **主循环** (`cpu0_main.c`): 轮询 `vision.image_ready` 标志，处理图像和显示
2. **10ms中断** (`isr.c`): CCU60_CH0定时器中断调用 `smart_car_control()` 执行闭环控制
3. **DMA中断**: 摄像头采集完成后设置 `vision.image_ready = 1`

### Key Subsystems

**Vision Tracking** (`vision_track.c/.h`):
- OTSU自适应阈值或固定阈值(220)二值化
- 从图像底部向上扫描，提取左右边缘和中心线
- 加权计算偏差值 `vision_get_deviation()` 返回 -80~80 范围
- 全局变量: `vision_track_t vision` 和 `uint8 image_data[120][188]`

**Motor Control** (`motor_control.c/.h`):
- DRV8701驱动模式: PWM控制速度，DIR引脚控制方向
- 编码器反馈: TIM5(左轮) TIM6(右轮)，10ms调用 `motor_update_speed()` 更新速度
- 舵机控制: 50Hz PWM，duty范围500-1000对应±45度
- 全局结构体: `car_control_t car` 包含左右电机和转向舵机

**PID Control** (`pid_control.c/.h`):
- 标准位置式PID: P+I+D，带积分限幅和输出限幅
- 每个PID实例包含: kp/ki/kd参数, integral/last_error状态, output_max/min限幅

**Smart Car** (`smart_car.c/.h`):
- 主控制逻辑: 整合速度PID(左右轮)和方向PID
- 场景自适应PID: 正常/直线/曲线场景预设不同参数
- 状态机: CAR_STOP, CAR_RUNNING, CAR_PAUSE, CAR_DEBUG
- 避障状态机: AVOID_IDLE → AVOID_TURNING_LEFT/RIGHT → AVOID_BYPASSING → AVOID_RETURNING

**Element Recognition** (`element_recognition.c/.h`):
- 识别赛道元素: 十字路口/圆岛/坡道/障碍物/停车点/斑马线
- 状态跟踪: FOUND → ENTERING → IN_ELEMENT → LEAVING → PASSED
- 全局变量: `element_recognition_t element_recog`

**Path Planning** (`path_planning.c/.h`):
- 路径节点队列: 最多20个节点，每个包含type/distance/target_speed/target_angle
- 决策模式: MANUAL(手动), AUTO(自动), OPTIMAL(最优路径)
- 成本计算: distance_cost + time_cost + risk_cost

## Hardware Configuration

### Pin Assignments (参考 `推荐IO分配.txt`)
```c
// 电机 (code/motor_control.h)
#define MOTOR_LEFT_PWM      ATOM0_CH7_P02_7
#define MOTOR_LEFT_DIR      P02_6
#define MOTOR_RIGHT_PWM     ATOM0_CH5_P02_5
#define MOTOR_RIGHT_DIR     P02_4

// 编码器
#define ENCODER_LEFT        TIM5_ENCODER  // P10_3/P10_1
#define ENCODER_RIGHT       TIM6_ENCODER  // P20_3/P20_0

// 舵机
#define SERVO_PWM_PIN       ATOM1_CH1_P33_9

// MT9V03X摄像头
// 数据口: P00_0~P00_7, VSYNC: P02_0, PCLK: P02_1
// 配置串口: RX P02_2, TX P02_3

// TFT180显示屏: SPI2 (P15_3/CLK, P15_5/MOSI, P15_2/CS0)
```

### Critical Hardware Notes
- **Boot引脚避免使用**: P14_2~P14_6, P10_5, P10_6（可能导致启动失败）
- **DRV8701电机驱动**: PWM频率17kHz，占空比0-8000（max 10000）
- **编码器方向**: 右轮编码器计数取反 (`-encoder_get_count()`)

## Development Patterns

### Type System
使用逐飞库自定义类型 (`libraries/zf_common/zf_common_typedef.h`):
```c
uint8, uint16, uint32  // 无符号整数
int8, int16, int32     // 有符号整数
vuint8, vint32         // volatile修饰
```

### Initialization Sequence
```c
// user/cpu0_main.c: core0_main()
clock_init();              // 必须保留，获取时钟频率
debug_init();              // 调试串口
tft180_init();             // 显示屏
smart_car_init();          // 智能小车（内部调用motor_init, vision_init等）
pit_ms_init(CCU60_CH0, 10);// 10ms定时器中断
cpu_wait_event_ready();    // 等待多核初始化
smart_car_start();         // 启动控制
```

### Interrupt Handling
所有中断函数使用 `IFX_INTERRUPT()` 宏，必须在开头调用 `interrupt_global_enable(0)` 开启中断嵌套:
```c
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);  // 开启嵌套
    smart_car_control();         // 控制逻辑
    pit_clear_flag(CCU60_CH0);   // 清除中断标志
}
```

### Memory Sections
使用 `#pragma section` 指定变量存储位置:
```c
#pragma section all "cpu0_dsram"
// 全局变量放在CPU0的RAM中
#pragma section all restore
```

### PID Tuning Workflow
1. 修改 `smart_car.h` 中的PID宏定义 (如 `DIRECTION_PID_KP_NORMAL`)
2. 或运行时调用 `smart_car_set_speed_pid(kp, ki, kd)`
3. 场景切换: `smart_car_set_pid_scene(PID_SCENE_CURVE)` 加载预设参数

### Image Processing Flow
```c
// vision_track.c
if (vision.image_ready) {
    uint8 threshold = otsu_threshold(mt9v03x_image, size);  // 或使用固定阈值220
    image_binarization(threshold);         // 写入image_data[][]
    vision_find_track_edge();              // 边缘检测，填充track.left_edge/right_edge
    vision_get_deviation();                // 计算加权偏差，存入vision.error
    vision.image_ready = 0;                // 清除标志
}
```

## Build & Debug

### Build System
- IDE: AURIX Development Studio (ADS) v1.10.2
- 工具链: TASKING TriCore C/C++ Compiler
- Makefile位置: `Debug/makefile`
- 链接脚本: `Lcf_Tasking_Tricore_Tc.lsl`
- 输出文件: `Debug/Seekfree_TC264_Opensource_Library.elf` 和 `.hex`

### Build Command
```bash
cd Debug
make clean
make all   # 调用cctc编译器和链接器
```

### 修改工程名称
执行 `AURIX修改工程名称.bat` 批量重命名文件和配置

### 清理临时文件
执行 `删除临时文件.bat` 清理编译中间文件

## Common Tasks

### 添加新的控制模块
1. 在 `code/` 创建 `.c/.h` 文件
2. 在 `code/car_headfile.h` 添加头文件引用
3. 在 `smart_car_init()` 调用初始化函数
4. 在 `smart_car_control()` 或主循环集成控制逻辑

### 调试图像处理
1. 修改 `vision_track.h` 中的阈值/扫描参数
2. 调用 `vision_show_image_with_lines_tft180()` 在TFT屏显示边缘线
3. 使用 `printf` 输出到调试串口 (UART0)

### 调整电机参数
1. 修改 `motor_control.h` 中的 `MOTOR_MAX_DUTY` (当前8000)
2. 调整 `BASE_SPEED` (`smart_car.h`, 当前200/10ms)
3. 确保 `MOTOR_PWM_FREQ` 在10-25kHz范围（DRV8701推荐）

### 元素识别集成
1. 在 `element_recognition.c` 实现 `element_detect_xxx()` 函数
2. 在 `element_recognition_process()` 调用检测函数
3. 在 `smart_car_control()` 根据 `element_recog.current_element.type` 执行策略
4. 示例: 障碍物避让使用状态机 `smart_car.avoid_state` 控制转向序列

## Important Conventions

- **函数命名**: `模块名_动作` (如 `vision_init`, `pid_calculate`)
- **结构体命名**: `模块名_t` (如 `pid_t`, `motor_t`, `vision_track_t`)
- **枚举命名**: `类型_enum` (如 `car_state_enum`, `element_type_enum`)
- **全局变量**: 使用 `extern` 声明在头文件，定义在对应 `.c` 文件
- **中文注释**: 代码注释以中文为主，保持与原有代码风格一致
- **GPL3.0许可**: 所有新增代码必须保留逐飞科技的版权声明

## Debugging Tips

- **查看实时速度**: `car.left_motor.current_speed` 和 `car.right_motor.current_speed` (编码器脉冲数/10ms)
- **查看偏差值**: `vision.error` (-80~80) 和 `vision.deviation` (-1.0~1.0)
- **检查轨道状态**: `vision.track_found` (0/1) 和 `vision.track.valid_rows` (有效行数>=50则认为找到)
- **PID输出**: `smart_car.direction_pid.output` (转向角度) 和 `speed_pid_left/right.output` (PWM占空比)
- **中断频率**: 确保 `pit_ms_init(CCU60_CH0, 10)` 为10ms周期，不要改变否则影响PID调参

## Performance Considerations

- **图像处理**: 188x120灰度图，避免在主循环做复杂计算，OTSU算法已优化
- **编码器更新**: 必须每10ms调用 `motor_update_speed()` 后清零计数器
- **中断优先级**: `isr_config.h` 定义优先级，摄像头DMA > 定时器 > 串口
- **避免阻塞**: 主循环和中断内不要使用 `system_delay_ms()`，除非在调试模式
