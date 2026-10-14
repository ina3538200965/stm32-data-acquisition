# STM32 嵌入式入职训练工程

<!-- TODO: 正文待补 —— 今天这份文件只有一个目的：先把目录划分钉死。
     目录结构能讲清楚，README 的内容后面自然会跟着长出来。 -->

## 目录结构

```
Project/
├── Project.uvprojx          # Keil 工程结构本身（要提交）
├── User/                    # 入口
│   ├── main.c               #   只做一件事：把控制权交给应用层
│   └── stm32f10x_conf.h     #   标准外设库配置
├── driver/                  # 硬件相关层 —— 换芯片就重写这一层
│   ├── driver_led.c
│   └── driver_led.h
├── app/                     # 业务逻辑层 —— 不碰寄存器
│   ├── app.c
│   ├── app.h
│   ├── stm32f10x_it.c       #   中断服务
│   └── stm32f10x_it.h
├── protocol/                # 帧协议层 —— 纯软件，可脱离硬件单测
│   ├── protocol.c
│   └── protocol.h
├── test/                    # 验证层
│   ├── test_protocol.c
│   ├── test_protocol.h
│   └── host_test.c          #   PC 端入口，不进 Keil 工程
├── Library/                 # ST 标准外设库（第三方）
└── Start/                   # CMSIS + 启动文件（第三方）
```

## 分层依赖规则

```
main.c → app → driver → HAL/寄存器
              → protocol   (零硬件依赖)
              → test
```

- `app` 不知道自己跑在 STM32 上
- `protocol` 没有任何 `#include "stm32f10x.h"`
- `test` 只依赖 `protocol`，因此可以在 PC 上直接编译运行

（正文待写）
