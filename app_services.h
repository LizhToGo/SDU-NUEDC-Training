#ifndef APP_SERVICES_H
#define APP_SERVICES_H

#include <stdint.h>

void st011_set_active(uint8_t active);
void st011_service(uint32_t elapsed_ms);
void delay_ms_with_st011(uint32_t total_ms);
void st011_start_pulse(uint32_t pulse_ms);
void st011_pulse(uint32_t pulse_ms);
void st011_finish_pending_pulse(void);
uint8_t task_uart_stop_requested(void);

#endif /* APP_SERVICES_H */
