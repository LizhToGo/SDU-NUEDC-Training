# Board 模块说明

本文档对应 `Board/board.h` 和 `Board/board.c`。这个模块不是具体外设驱动，而是工程里的通用工具层，主要提供串口打印、日志输出和延时函数。

## 1. 文件作用

| 文件 | 作用 |
|---|---|
| `Board/board.h` | 对外声明常用类型、打印函数、日志宏和延时函数 |
| `Board/board.c` | 实现 UART0 发送、`printf` 重定向、`lc_printf()`、`LOG_D()` 和延时函数 |

## 2. 初始化关系

`Board/board.c` 本身不负责初始化 UART0 的引脚和外设。UART0 的初始化来自 SysConfig 自动生成代码。

当前 UART0 配置来自 `main.syscfg`：

| 项目 | 当前值 |
|---|---|
| `UART0_TX` | `PA28` |
| `UART0_RX` | `PA31` |
| 波特率 | `115200` |
| 数据格式 | 8 数据位，无校验，1 停止位 |

正常调用顺序是：

```c
SYSCFG_DL_init();
lc_printf("hello\r\n");
```

`SYSCFG_DL_init()` 会初始化 `UART_0_INST`，之后 `lc_printf()`、`printf()` 和 `LOG_D()` 才能正常通过串口输出。

如果没有先调用 `SYSCFG_DL_init()`，串口外设可能还没有被配置，打印函数就不能可靠工作。

## 3. board.h 里有什么

### 3.1 常用类型别名

`board.h` 里定义了几个简写类型：

```c
u8   -> uint8_t
u16  -> uint16_t
u32  -> uint32_t
u64  -> uint64_t
```

它们只是简写，和标准整数类型含义一致。

### 3.2 lc_printf()

`lc_printf()` 是工程里常用的串口打印函数：

```c
lc_printf("speed=%d\r\n", speed);
```

它的用法接近 `printf()`，最终通过 UART0 发出去。

注意：`board.c` 中内部缓冲区大小是 512 字节，一次不要打印太长的字符串。

### 3.3 LOG_D()

`LOG_D()` 是带文件名、函数名和行号的调试日志宏：

```c
LOG_D("sensor=%d", sensor_value);
```

输出内容会带上类似这样的前缀：

```text
[main.c Func:main Line:20] sensor=123
```

调试某个函数在哪里被调用、某个变量在哪一行变化时，用它比 `lc_printf()` 更方便。

### 3.4 delay_us() / delay_ms()

延时函数：

```c
delay_us(100);
delay_ms(500);
```

`delay_1us()` 和 `delay_1ms()` 目前与 `delay_us()`、`delay_ms()` 作用相同，只是保留了另一套命名。

## 4. board.c 里怎么实现

### 4.1 uart0_sendChar()

这是最底层的单字节发送函数。

它会先等待 UART0 空闲：

```c
while (DL_UART_isBusy(UART_0_INST) == true);
```

然后发送一个字节：

```c
DL_UART_Main_transmitData(UART_0_INST, dat);
```

所以当前串口发送是阻塞式的：如果串口还没发完，程序会等一会儿。

### 4.2 uart0_sendString()

循环调用 `uart0_sendChar()`，把字符串一个字符一个字符发出去。

这个函数是 `static`，只在 `board.c` 内部使用，外部文件不能直接调用。

### 4.3 fputc()

`fputc()` 把 C 标准库的 `printf()` 重定向到 UART0。

因此你也可以写：

```c
printf("hello\r\n");
```

不过当前工程更推荐统一使用 `lc_printf()`，这样风格更一致。

### 4.4 LOG_Debug_Out()

这是 `LOG_D()` 宏实际调用的函数。它会把文件名、函数名、行号拼到日志前面，再通过 UART0 输出。

一般不需要手动直接调用 `LOG_Debug_Out()`，直接用 `LOG_D()` 就行。

### 4.5 delay_us() / delay_ms()

延时函数底层调用：

```c
delay_cycles(...);
```

计算依据是 `CPUCLK_FREQ`。这个宏来自 SysConfig 生成的 `ti_msp_dl_config.h`，所以时钟配置变化时，延时计算也会跟着工程配置走。

## 5. 常见用法

```c
#include "board.h"

int main(void)
{
    SYSCFG_DL_init();

    lc_printf("system start\r\n");
    LOG_D("debug value=%d", 123);

    while (1) {
        lc_printf("loop\r\n");
        delay_ms(1000);
    }
}
```

## 6. 注意事项

1. 串口打印前必须先调用 `SYSCFG_DL_init()`。
2. `lc_printf()` 和 `LOG_D()` 都是阻塞式发送，频繁打印会影响主循环速度。
3. 延时函数是忙等待，延时时 CPU 基本不能做别的事。
4. `lc_printf()` 内部缓冲区为 512 字节，一次打印不要超过这个长度。
5. 后续如果需要更高性能，可以把串口输出改成中断或 DMA，但当前调试阶段阻塞式打印更简单稳定。
6. 蓝牙串口模块到货后，如果继续使用 UART0，仍然可以复用 `lc_printf()` 输出调试信息。
