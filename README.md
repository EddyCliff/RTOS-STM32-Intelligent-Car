- [RTOS-STM32-Intelligent-Car](#rtos-stm32-intelligent-car)
  - [项目整体框架](#项目整体框架)
  - [环境数据采集进程](#环境数据采集进程)
    - [MPU6050简介](#mpu6050简介)
      - [如何获得这些数据？](#如何获得这些数据)
      - [配置管脚](#配置管脚)
    - [MPU6050的eMPL库](#mpu6050的empl库)
    - [MPU6050宏定义](#mpu6050宏定义)
  - [小车PID控制进程](#小车pid控制进程)
    - [电机的分类](#电机的分类)
    - [MC3386电机驱动芯片](#mc3386电机驱动芯片)
      - [管脚配置](#管脚配置)
    - [正交码盘与STM32正交编码器](#正交码盘与stm32正交编码器)
      - [管脚配置](#管脚配置-1)
    - [超声波测距模块](#超声波测距模块)
      - [HC-SR04超声波测距模块介绍](#hc-sr04超声波测距模块介绍)
      - [HC-SR04工作原理：](#hc-sr04工作原理)
    - [小车直立行走任务分解](#小车直立行走任务分解)
    - [小车的平衡控制流程](#小车的平衡控制流程)
    - [PID参数设计(以直立环为例)](#pid参数设计以直立环为例)
      - [1. 确定平衡小车的机械中值：](#1-确定平衡小车的机械中值)
      - [2\_1. 确定kp值的极性（令kd=0）](#2_1-确定kp值的极性令kd0)
      - [2\_2 确定kp值的大小（令kd=0）](#2_2-确定kp值的大小令kd0)
  - [交互进程实现](#交互进程实现)
    - [ESP32](#esp32)
    - [通信逻辑](#通信逻辑)

# RTOS-STM32-Intelligent-Car
本项目是基于FreeRTOS的STM32超声波智能避障平衡小车，实现了小车的自平衡、超声波避障以及通过智能手机远程控制的功能。


## 项目整体框架
1. 开始`main`
2. `FS_Explorer_Init`外设模块初始化
3. `Freertos_Init`操作系统初始化
4. 启动系统
5. `StartTask_200HZ`环境数据采集进程
6. `StartTask_100HZ`小车PID控制进程
7. `StartTask_Interaction`交互进程
8. 中断处理程序

**环境数据采集进程**
1. 获取MPU6050数据
2. `osDelay(5)`

**小车PID控制进程**
1. 启动超声波测距
2. 读取左右轮子的转速
3. 确定PID控制参数
4. 直立环-速度环-转向环
5. 控制电机
6. `osDelay(10)`

**交互进程**
1. 创建系统消息队列
2. 初始化ESP32通信接口
3. 初始化ESP32
4. 发送数据给上位机
5. 接收处理上位机数据
6. `osDelay(10)`

**中断处理程序**
1. ESP32接收中断处理
2. 超声波回应中断处理

## 环境数据采集进程
### MPU6050简介

MPU6050、IIC（I2C），和STM32之间存在联系，通常在嵌入式系统中一起使用。

1. MPU6050: MPU6050是一种常用的九轴运动传感器，包括三轴陀螺仪和三轴加速度计，以及温度传感器。它可以测量物体的运动和倾斜等信息。MPU6050通常通过IIC（I2C）接口与微控制器或处理器通信。

2. IIC（I2C）：IIC（Inter-Integrated Circuit）是一种串行通信协议，用于连接微控制器、传感器、存储器等外部设备。MPU6050通常使用IIC接口与主控制器（如STM32）进行通信，以传输传感器数据和控制命令。

3. STM32: STM32是一系列基于ARM Cortex-M内核的微控制器。它们常用于嵌入式系统开发，并具有丰富的外设接口和功能。STM32可以连接到MPU6050传感器，通过IIC总线与其通信，从而读取传感器数据并执行相关控制任务，如姿态测量、运动跟踪等。

所以，MPU6050通过IIC接口与STM32微控制器连接，允许STM32读取传感器数据并执行各种应用，例如机器人姿态控制、无人机飞行控制、虚拟现实头显姿态追踪等。这种组合常见于嵌入式系统和物联网设备中。

#### 如何获得这些数据？

1）初始化MPU6050

2）移植MPU6050的eMPL库

3）获取原始数据以及姿态角

#### 配置管脚
MPU6050 IIC的两个管脚连接到STM32的两个管脚(PC11 和 PD0)
所以我们是不是要直接把PC11 和 PD0配置成IIC呢？当然这是一种思路。

我们这次的驱动是用的MPU6050常用的一种demo。

我只需要把PC11 和 PD0配置成普通的GPIO。然后我们对这两个GPIO采用IIC时序模拟，让它产生出IIC时序，然后来驱动MPU6050。

我们采用这种方法的好处就是：

我们可以脱离STM32 IIC控制器的限制，任意的两根管脚都可以当做IIC使用。

如果我想移植到其他平台，我只需要改一下管脚或是延时时间就可以用了。

而我们采用的STM32 IIC控制器的话，就只能针对STM32F4这一款芯片去使用，其他单片机就没法用了。

### MPU6050的eMPL库
为了获取欧拉角，我们要使用MPU6050_EMPL库对Mpu6050获取的原始数据（加速度，角速度）进行处理，从而获得欧拉角。所以我们要移植MPU6050_EMPL库到我们的工程。

### MPU6050宏定义

- `MPU6050` 宏用于选择与 MPU6050 传感器相关的代码。由于 eMPL 库支持多种不同型号的 MPU，您需要设置 `MPU6050` 宏以确保只编译与 MPU6050 相关的代码。

- 在 `inv_mpu.c` 文件中，根据 `MPU6050` 宏的设置，编译器将只选择与 MPU6050 传感器兼容的部分代码，以确保库与您的硬件适配
- 我们在工程的target里面进行设置，定义MPU6050为全局define。 

## 小车PID控制进程
### 电机的分类
1. 直流电机
   - 无刷直流电机
   - 有刷直流电机
2. 交流电机
   - 单相电机
   - 三相电机

### MC3386电机驱动芯片
IN1和IN2的电平顺序决定了电机的正反转

高电平引脚的电压值决定了电机转动的速度

比如：IN1为高电平，IN2为低电平，那么电机正转，IN1为低电平，IN2为高电平，那么电机反转。

PWM波调节高电平的IN1管脚，从而调节电机的平均电压，从而控制电机速度。

#### 管脚配置
电机1

（左边管脚是电机驱动板的管脚，右边管脚是STM32开发板上与之相连的对应管脚）

`IN1` → `PC3` 在STM32cubemx上把STM32的这个管脚配置GPIO_Output功能

`IN2` → `PA3` 在STM32cubemx上把STM32的这个管脚配置GPIO_Output功能

功能：IN1和IN2一起控制电机的正反转

`PWMA` → `PA2` (TIM5-CH3) 在STM32cubemx上把STM32的这个管脚配置为定时器的PWM功能

功能：PWMA控制电机的转速

### 正交码盘与STM32正交编码器
编码器有两个功能：测量电机的速度，检测电机是正转还是反转。

#### 管脚配置 
电机1

`INC_LEFT_A` → `PE9` （TIM1-CH1） 在STM32cubemx上把STM32的这个管脚配置为定时器的编码器功能

`INC_LEFT_B` → `PE11`（TIM1-CH2） 在STM32cubemx上把STM32的这个管脚配置为定时器的编码器功能

### 超声波测距模块

#### HC-SR04超声波测距模块介绍

HC-SR04特点：

- 可提供2cm-400cm的非接触式距离感测功能

- 测距精度可达高到3mm

- 模块包括超声波发射器、接收器与控制电路。

- 5v电源输入

#### HC-SR04工作原理：

1. 给脉冲触发引脚（trig）输入一个长为20us的高电平方波   

2. 输入方波后，模块会自动发射8个40KHz的声波，与此同时回波引脚（echo）端的电平会由0变为1；（此时应该启动定时器计时）     

3.当超声波返回被模块接收到时，回波引 脚端的电平会由1变为0；（此时应该停止定时器计数），定时器记下的这个时间即为超声波由发射到返回的总时长。     

4.根据声音在空气中的速度为344米/秒，即可计算出所测的距离。

我们在Car_Task_100HZ里面不断地去启动我们的超声波模块去检测我们的距离，如果我们检测到了距离，中断信号就会触发，会进入两次中断，一次是上升沿中断，一次是下降沿中断，上升沿中断启动定时器进行计数，下降沿中断停止定时器停止计数，统计传播的时间，根据声速得出距离。

### 小车直立行走任务分解
我们要求车模在直立的状态下以两个轮子在地面上随意行走，相比四轮

着地状态，车模控制任务更为复杂。为了能够方便找到解决问题的办法，首先将复杂的问题分解成简单的问题进行讨论。

从控制角度来看，车模作为一个控制对象，它的控制输入量是两个电机的转动速度。车模运动控制任务可以分解成以下三个基本控制任务

（1） 控制车模平衡：通过控制两个电机正反向运动保持车模直立平衡状态；

（2） 控制车模速度：通过调节车模的倾角来实现车模速度控制，实际上最后还是演变成通过控制电机的转速来实现车轮速度的控制。

（3） 控制车模方向：通过控制两个电机之间的转动差速实现车模转向控制

### 小车的平衡控制流程
1. **传感器数据获取**：首先，MPU6050传感器负责获取小车的倾斜、加速度和角速度等数据。这些数据用于了解小车的当前状态，包括它是否正在倾斜以及倾斜的速度和方向。

2. **数据处理**：CPU（例如STM32）接收从MPU6050传感器获取的数据。它会对这些数据进行处理，计算出小车的倾斜程度和倾斜速度等信息。这些信息将用于决定如何调整电机来保持平衡。

3. **控制算法**：基于从传感器获得的信息，CPU将使用控制算法，通常是PID（比例-积分-微分）控制算法，来生成一个控制信号。这个信号表示电机应该以何种方式调整输出以保持小车平衡。

4. **PWM输出**：生成的控制信号通常是PWM信号。PWM信号会传递给小车上的电机，以改变它们的速度或功率。电机的调整方式取决于控制算法的输出，以确保小车不会倾倒。

5. **反馈循环**：这是一个持续的过程。电机的运行会影响小车的状态，因此传感器会不断提供反馈数据。这个反馈信息将用于动态调整控制算法生成的PWM信号，以应对小车的状态变化。

整个过程是一个反馈控制循环，其中CPU通过处理传感器数据，计算控制信号，然后将控制信号传递给电机，从而不断调整小车的状态以保持平衡。这需要精确的传感器、有效的控制算法和适当的电机控制来实现。

### PID参数设计(以直立环为例)
平衡小车直立环的调试过程包括确定平衡小车的机械中值、确定kp值的极性（也就是正负号）和大小、kd值的极性和大小等步骤。

#### 1. 确定平衡小车的机械中值：

把平衡小车放在地面上，绕电机轴旋转平衡小车，记录能让小车接近平衡的角度，一般都在0°附近的。我们调试的小车正好是0度，所以就是Bias=Angle-0

#### 2_1. 确定kp值的极性（令kd=0）

首先我们估计kp的取值范围。我们的PWM设置的是8000代表占空比100%，再考虑避免电机的死区，我们直立环返回的PWM在6000左右的时候电机就会满载。

假如我们设定kp值为800，那么平衡小车在±10°的时候就会满转。根据我们的感性认识，这显然太大了，那我们就可以估计kp值在0~800之间。

首先大概我们给一个值kp=-200,我们可以观察到，小车往哪边倒，电机会往那边加速让小车到下，就是一个我们不愿看到的正反馈的效果。说明kp值的极性反了，接下来我们设定kp=200,这个时候可以看到平衡小车有直立的趋势，虽然响应太慢，但是，我们可以确定kp值是正的。具体的数据接下来再仔细调试。

#### 2_2 确定kp值的大小（令kd=0）

- 设定kp=100,这个时候我们可以看到，小车虽然有平衡的趋势，但是显然响应有点慢了。

- 设定kp=250,这个时候我们可以看到，小车虽然有平衡的趋势，而且响应有所加快，总体感觉良好。

- 设定kp=400,这个时候我们可以看到，小车的响应明显加快，而且来回推动小车的时候，会有一定幅度的低频抖动。说明这个时候kp值已经足够大了，需要增加微分控制削弱p控制，抑制低频抖动。

经过总体比较： 我们选择参数为kp = 200；

## 交互进程实现
### ESP32
Wi-Fi&蓝牙解决方案                             
ESP32可作为独立系统运行应用程序或是主机MCU的从设备，通过SPI/SDIO或I2C/UART接口提供Wi-Fi和蓝牙功能。   

超低功耗     
ESP32专为移动设备、可穿戴电子产品和物联网应用而设计，具有业内高水平的低功耗性能，包括精细分时钟门控、省电模式和动态电压调整等。

高度集成                                     
ESP32将天线开关、RFbalun、功率放大器、接收低噪声放大器、集成的自滤波器、电源管理模块等功能集于一体。   
ESP32只需极少的外围器即可实现强大的处理性能、可靠的安全性能，和Wi-Fi&蓝牙功能。

性能稳定   
ESP32性能稳定，工作温度范围达到-40°C到+125°。        
自适应校准电路实现了动态电压调整，可以消除外部电路的缺陷并适应外部条件的变化。

### 通信逻辑
ESP32与小车的交互进程是通过串口通信（UART）和一组定义的AT命令来实现的。这个过程可以分为以下几个步骤：

1. **连接模式选择**：首先，全局变量 `NET_MODE` 用于选择连接模式。这个变量可以是蓝牙模式（BLE_MODE）或Wi-Fi模式，取决于项目的需要。

2. **ESP32初始化**：ESP32模块的初始化由一系列AT命令实现，例如设置Wi-Fi模式、创建AP名称、配置蓝牙广播数据等。这些命令通过串口发送给ESP32。

3. **发送命令**：在 ESP32 模块初始化后，可以使用 `ESPSend_Cmd` 函数向 ESP32 发送不同的AT命令。这些命令可以是控制蓝牙广播、Wi-Fi连接、服务器初始化等。这些命令可以触发 ESP32 执行不同的操作。

4. **接收响应**：每次发送命令后，代码会等待来自 ESP32 的响应。当响应被接收时，将检查响应中是否包含预期的响应字符串。如果匹配，命令状态将设置为成功。

5. **数据交互**：根据连接模式，ESP32 与小车可以进行数据交互。在蓝牙模式下，ESP32 用于处理客户端连接和数据读写。在Wi-Fi模式下，ESP32 用于处理客户端的连接和数据传输。这可以通过解析响应和发送数据来实现。

6. **连接状态跟踪**：在 `WIFI_CONNECT_FLAG` 和 `BLE_CONNECT_FLAG` 这两个全局变量中，跟踪着Wi-Fi和蓝牙的连接状态。这些标志用于检测连接是否建立或中断。

7. **错误处理**：如果命令未收到响应或响应超时，代码会将命令状态设置为失败，以及在连接方面的处理，如果连接中断或重新建立，会触发相应操作。

总的来说，ESP32与小车之间的交互过程涉及使用AT命令通过UART串口与ESP32模块进行通信，ESP32模块执行相应的操作，并向小车发送数据。此外，ESP32模块也可以处理小车发送的数据。这种通信方式可以适应不同的连接模式，例如蓝牙或Wi-Fi。

