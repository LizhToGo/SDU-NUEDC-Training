#ifndef APP_SERVICES_H
#define APP_SERVICES_H

#include <stdint.h>

/** 设置 ST011 声光输出为触发或空闲。 */
void st011_set_active(uint8_t active);

/** 按 elapsed_ms 推进非阻塞 ST011 脉冲计时。 */
void st011_service(uint32_t elapsed_ms);

/** 忙等待延时，同时持续维护待完成的 ST011 脉冲。 */
void delay_ms_with_st011(uint32_t total_ms);

/** 启动一次非阻塞 ST011 脉冲。 */
void st011_start_pulse(uint32_t pulse_ms);

/** 启动一次阻塞 ST011 脉冲并等待完成。 */
void st011_pulse(uint32_t pulse_ms);

/** 等待所有待完成的非阻塞 ST011 脉冲结束。 */
void st011_finish_pending_pulse(void);

/** 轮询 UART 任务输入；若有停车命令则返回 1。 */
uint8_t task_uart_stop_requested(void);

#endif /* APP_SERVICES_H */
