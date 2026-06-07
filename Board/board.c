

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "board.h"
#include "ti/driverlib/m0p/dl_core.h"

static char g_uart_printf_buffer[512];

/*
 * 当前 Board 模块主要负责三件事：
 * 1. 将 printf/lc_printf 的输出发送到 UART0；
 * 2. 提供带文件名、函数名、行号的 LOG_D 调试输出；
 * 3. 提供基于 CPUCLK_FREQ 的忙等待延时函数。
 *
 * UART0 的引脚和波特率不在这里配置，而是在 main.syscfg 中配置，
 * 并由 main.c 开头调用的 SYSCFG_DL_init() 完成初始化。
 */

static void uart0_sendChar(uint8_t dat)
{
    /* UART0 忙时等待，空闲后再发送当前字符。 */
    while( DL_UART_isBusy(UART_0_INST) == true );

    /* 发送单个字符。 */
    DL_UART_Main_transmitData(UART_0_INST, dat);
}


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
int fputc(int ch, FILE *f)
{
    /* 发送单个字符。 */
    uart0_sendChar( (uint8_t)ch );

    return ch;
}

int LOG_Debug_Out(const char* __file, const char* __func, int __line, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    /* 拼接日志前缀，方便定位日志来自哪个文件、函数和行号。 */
    char log_buff[64] = {0};
    sprintf(log_buff, "[%s Func:%s Line:%d] ",__file,__func,__line);

    /* 创建缓冲区保存完整日志内容。 */
    memset(g_uart_printf_buffer, 0, sizeof(g_uart_printf_buffer));
    strcpy(g_uart_printf_buffer, log_buff);
    int len = vsnprintf(g_uart_printf_buffer + strlen(g_uart_printf_buffer),
        sizeof(g_uart_printf_buffer) - strlen(g_uart_printf_buffer),
        format,
        args);

    va_end(args);

    /* LOG_D 自动补一组换行，普通 lc_printf 不会自动补。 */
    char temp_buff[] = "\r\n";
    strncat(g_uart_printf_buffer,
        temp_buff,
        sizeof(g_uart_printf_buffer) - strlen(g_uart_printf_buffer) - 1U);
    uart0_sendString(g_uart_printf_buffer);

    return len;
}



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

void delay_us(int __us) { delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); }
void delay_ms(int __ms) { delay_cycles( (CPUCLK_FREQ / 1000)*__ms); }

void delay_1us(int __us) { delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); }
void delay_1ms(int __ms) { delay_cycles( (CPUCLK_FREQ / 1000)*__ms); }
