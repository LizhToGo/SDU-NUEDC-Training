#ifndef TASK_DISPATCHER_H
#define TASK_DISPATCHER_H

/**
 * @file task_dispatcher.h
 * @brief 阻塞式任务选择和顶层任务调度循环。
 *
 * 将 UART/按键任务选择从 main.c 中拆出。具体任务函数在 main.c
 * 引入本头文件前已经可见，因此当前工程仍保持头文件式组织。
 */

#include "app_config.h"
#include "app_services.h"
#include "app_task_ids.h"
#include "board.h"
#include "bsp_tb6612.h"

/**
 * @brief 执行按键启动和 UART 启动共用的少量任务前准备。
 *
 * 任务二启动前先刷新一次航向零点，随后 AB 直线段还会在起步声光和
 * 稳定等待后再次确认零点；这样按键和串口启动路径都能得到同样的
 * 航向参考。任务三/四会在这里启动一次 ST011 提示，再进入共享竞速循环。
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
 * @brief 将解析后的任务编号分发到对应顶层任务函数。
 *
 * 该函数刻意保持为很薄的分发表，不承载各任务自身的控制参数。
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
    } else {
        TB6612_Brake();
        st011_pulse(TASK1_START_ALARM_MS);
        lc_printf("TASK id=%u not implemented yet\r\n", task_id);
    }
}

/**
 * @brief 永久等待任务命令，并运行选中的任务。
 *
 * wait_task_uart_command() 同时轮询四个实体按键，因此该循环就是上电后
 * 唯一的验收模式入口。
 */
static void run_task_dispatcher(void)
{
    task_id_t task_id;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: buttons A26/A24/B24/A22 or UART0 HEX bytes 01..04; 00=stop while running; ASCII t01..t04 still ok\r\n");

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
