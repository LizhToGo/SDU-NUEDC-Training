#ifndef TASK_DISPATCHER_H
#define TASK_DISPATCHER_H

/**
 * @file task_dispatcher.h
 * @brief Blocking task selection and top-level task dispatch loop.
 *
 * Keeps UART/button task selection out of main.c. The concrete task routines
 * are included by main.c before this header so the generated firmware layout
 * remains header-only.
 */

#include "app_config.h"
#include "app_services.h"
#include "app_task_ids.h"
#include "board.h"
#include "bsp_tb6612.h"

/**
 * @brief Apply the small amount of per-task setup shared by button and UART starts.
 *
 * Task2 intentionally refreshes the yaw zero before starting. Task3/4 also
 * start one ST011 pulse here so button/UART starts are visibly acknowledged
 * before the shared race loop begins.
 */
static void prepare_task_start(task_id_t task_id)
{
    if (task_id == TASK_ID_2) {
        TB6612_Brake();
        (void)jy62_zero_to_current("task_start_zero", 0U);
    } else if ((task_id == TASK_ID_1) ||
        (task_id == TASK_ID_3) || (task_id == TASK_ID_4)) {
        TB6612_Brake();
    }

    if ((task_id == TASK_ID_3) || (task_id == TASK_ID_4)) {
        st011_start_pulse(RACE_START_ALARM_MS);
    }
}

/**
 * @brief Dispatch a decoded task id to the corresponding high-level routine.
 *
 * This function is deliberately a thin switch-like table: it should not carry
 * control parameters for the tasks themselves.
 */
static void run_selected_task(task_id_t task_id)
{
    if (task_id == TASK_ID_1) {
        run_task1_ab();
    } else if (task_id == TASK_ID_2) {
        run_task2_abcd();
    } else if (task_id == TASK_ID_3) {
        run_race_laps(1U);
    } else if (task_id == TASK_ID_4) {
        run_race_laps(4U);
    } else if (task_id == TASK_ID_5) {
        TB6612_Brake();
        run_motor_pid_stream();
    } else if (task_id == TASK_ID_6) {
        TB6612_Brake();
        run_task6_ac_c_turn_test();
    } else if (task_id == TASK_ID_7) {
        TB6612_Brake();
        run_motor_pd_stream();
    } else if (task_id == TASK_ID_10) {
        TB6612_Brake();
        run_task10_ab_zero_test();
    } else {
        TB6612_Brake();
        st011_pulse(TASK1_START_ALARM_MS);
        lc_printf("TASK id=%u not implemented yet\r\n", task_id);
    }
}

/**
 * @brief Block forever waiting for task commands and run the selected task.
 *
 * wait_task_uart_command() also polls the four physical task buttons, so this
 * loop is the single contest-mode entry point after boot.
 */
static void run_task_dispatcher(void)
{
    task_id_t task_id;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: buttons A26/A24/B24/A22 or UART0 HEX bytes 01..07,10; 00=stop while running; ASCII t01..t10 still ok\r\n");

    while (1) {
        task_id = wait_task_uart_command();

        if (task_id == TASK_ID_STOP) {
            TB6612_Brake();
            continue;
        }

        prepare_task_start(task_id);
        run_selected_task(task_id);
    }
}

#endif /* TASK_DISPATCHER_H */
