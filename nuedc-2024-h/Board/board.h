
#ifndef __BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"

/*
 * Board 模块是当前工程的通用工具层。
 *
 * 它不直接配置 UART、GPIO 或时钟；这些外设由 main.syscfg 生成的
 * SYSCFG_DL_init() 初始化。应用代码在调用 lc_printf() 或延时函数前，
 * 应先完成 SYSCFG_DL_init()。
 */

/* 使用可变参数实现的类 printf 串口打印函数，底层通过 UART0 阻塞发送。 */
/**
 * @brief 类 printf 的 UART0 阻塞输出辅助函数。
 */
int lc_printf(char* format,...);

/* 忙等待延时函数，时钟频率来自 SysConfig 生成的 CPUCLK_FREQ。 */
/** 忙等待指定微秒数。 */
void delay_us(int __us);

/** 忙等待指定毫秒数。 */
void delay_ms(int __ms);

#endif
