

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "board.h"
#include "ti/driverlib/m0p/dl_core.h"

/* Shared printf buffer; keep UART log lines below this size. */
static char g_uart_printf_buffer[512];

/*
 * 当前 Board 模块主要负责三件事：
 * 1. 将 printf/lc_printf 的输出发送到 UART0；
 * 2. 提供基于 CPUCLK_FREQ 的忙等待延时函数。
 *
 * UART0 的引脚和波特率不在这里配置，而是在 main.syscfg 中配置，
 * 并由 main.c 开头调用的 SYSCFG_DL_init() 完成初始化。
 */

/**
 * @brief Blocking transmit of one byte through UART0.
 */
static void uart0_sendChar(uint8_t dat)
{
    /* UART0 忙时等待，空闲后再发送当前字符。 */
    while( DL_UART_isBusy(UART_0_INST) == true );

    /* 发送单个字符。 */
    DL_UART_Main_transmitData(UART_0_INST, dat);
}


/**
 * @brief Blocking transmit of a NUL-terminated string through UART0.
 */
static void uart0_sendString(char* str)
{
    /* 逐字节发送字符串，直到遇到字符串结束符。 */
    while( str!=0 && *str!=0 )
    {
        /* 发送当前字符，然后移动到下一个字符。 */
        uart0_sendChar(*str++);
    }
}

/* 将 C 标准库 printf 使用的 fputc 重定向到 UART0。 */
/**
 * @brief Retarget C library putchar/printf output to UART0.
 */
int fputc(int ch, FILE *f)
{
    /* 发送单个字符。 */
    uart0_sendChar( (uint8_t)ch );

    return ch;
}

/**
 * @brief Format one log line into a fixed buffer and send it through UART0.
 */
int lc_printf(char* format,...)
{
    va_list args;
    va_start(args, format);

    /* 创建缓冲区保存格式化后的字符串，一次打印不要超过 512 字节。 */
    memset(g_uart_printf_buffer, 0, sizeof(g_uart_printf_buffer));
    int len = vsnprintf(g_uart_printf_buffer, sizeof(g_uart_printf_buffer), format, args);

    va_end(args);

    /* 通过 UART0 阻塞发送格式化后的字符串。 */
    uart0_sendString(g_uart_printf_buffer);

    return len;
}


/* ================ 延时函数封装 =================== */

/** Busy-wait for microseconds using CPUCLK_FREQ. */
void delay_us(int __us) { delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); }

/** Busy-wait for milliseconds using CPUCLK_FREQ. */
void delay_ms(int __ms) { delay_cycles( (CPUCLK_FREQ / 1000)*__ms); }
