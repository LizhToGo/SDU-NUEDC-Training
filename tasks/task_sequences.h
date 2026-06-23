#ifndef TASK_SEQUENCES_H
#define TASK_SEQUENCES_H

/**
 * @file task_sequences.h
 * @brief 任务一和任务二的顶层验收流程。
 *
 * 本文件只编排任务顺序，把直线、弧线、声光提示和异常退出处理
 * 留在各自模块中，避免 main.c 继续堆叠业务逻辑。
 */

#include <stdint.h>

#include "app_config.h"
#include "app_services.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "straight/straight_line.h"
#include "race/race_laps.h"

/**
 * @brief 执行任务一：A 到 B 直线，并带起点/终点声光提示。
 */
static void run_task1_ab(void)
{
    const straight_line_segment_config_t config = {
        .tag = "TASK1_AB",
        .zero_heading = 1U,
        .start_alarm_ms = TASK1_START_ALARM_MS,
        .stop_alarm_ms = TASK1_FINISH_ALARM_MS,
        .line_arm_count = TASK1_B_LINE_ARM_COUNT,
        .force_stop_count = TASK1_FORCE_STOP_COUNT,
        .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
        .yaw_corr_enable = 1U,
        .entry_brake_enable = 1U,
        .fixed_yaw_target_enable = 0U,
        .fixed_yaw_target_cdeg = 0
    };

    (void)run_straight_to_line_segment(&config);
}

/**
 * @brief 任务二异常退出的统一收尾。
 */
static void task2_abort_sequence(void)
{
    st011_finish_pending_pulse();
}

/**
 * @brief 执行任务二：AB 直线、BC 弧线、CD 直线、DA 弧线。
 *
 * 任务二不是完整复用任务三的整圈状态机，而是只把 BC/DA 两段接到竞速
 * 弧线控制上；AB 与 CD 仍然按任务二路线单独执行直线到线。
 */
static void run_task2_abcd(void)
{
    uint8_t reason;

    {
        const straight_line_segment_config_t config = {
            .tag = "TASK2_AB",
            /* 任务二从 A 点起跑时重新建立相对航向零点，后续 CD 的 180° 目标才有明确参考。 */
            .zero_heading = 1U,
            .start_alarm_ms = TASK1_START_ALARM_MS,
            .stop_alarm_ms = 0U,
            .line_arm_count = TASK1_B_LINE_ARM_COUNT,
            .force_stop_count = TASK1_FORCE_STOP_COUNT,
            .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
            .yaw_corr_enable = 1U,
            .entry_brake_enable = 1U,
            .fixed_yaw_target_enable = 0U,
            .fixed_yaw_target_cdeg = 0
        };

        reason = run_straight_to_line_segment(&config);
    }
    if (reason != 1U) {
        task2_abort_sequence();
        return;
    }
    /* AB 成功到 B 后只启动非阻塞声光，不刹停，让车辆自然衔接进入 BC 弧线。 */
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    /* BC 只复用竞速弧线“跑到出弧点”的部分，出 C 后立即交给 CD 直线。 */
    reason = run_task2_bc_race_arc("TASK2_BC");
    if (reason != 1U) {
        task2_abort_sequence();
        return;
    }

    /* CD 使用固定 180° 航向直线，到 D 点后再进入 DA 弧线。 */
    reason = run_task2_cd_exit_angle_straight("TASK2_CD");
    if (reason != 1U) {
        task2_abort_sequence();
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    /* DA 回到 A 点后任务二结束，最后统一刹车和收尾声光。 */
    reason = run_task2_da_race_arc("TASK2_DA");
    if (reason != 1U) {
        task2_abort_sequence();
        return;
    }

    TB6612_Brake();
    st011_finish_pending_pulse();
}

#endif
