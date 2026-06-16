#ifndef APP_SERVICES_H
#define APP_SERVICES_H

#include <stdint.h>

/** Set the ST011 sound/light output active or inactive. */
void st011_set_active(uint8_t active);

/** Advance the non-blocking ST011 pulse timer by elapsed_ms. */
void st011_service(uint32_t elapsed_ms);

/** Busy-delay while continuing to service pending ST011 pulses. */
void delay_ms_with_st011(uint32_t total_ms);

/** Start a non-blocking ST011 pulse. */
void st011_start_pulse(uint32_t pulse_ms);

/** Start a blocking ST011 pulse and wait until it completes. */
void st011_pulse(uint32_t pulse_ms);

/** Wait until any pending non-blocking ST011 pulse has finished. */
void st011_finish_pending_pulse(void);

/** Poll UART task input and return 1 when a stop command is pending. */
uint8_t task_uart_stop_requested(void);

#endif /* APP_SERVICES_H */
