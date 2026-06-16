#ifndef TASK_SEQUENCES_H
#define TASK_SEQUENCES_H

/**
 * @file task_sequences.h
 * @brief Top-level Task1 and Task2 contest sequences.
 *
 * Keeps the high-level task choreography out of main.c while preserving the
 * existing straight, arc, alarm, abort, and Task2 RAM dump behavior.
 */

#include <stdint.h>

#include "app_config.h"
#include "app_services.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "race/race_laps.h"
#include "race/task2_ram_log.h"
#include "straight/straight_line.h"
#include "turn/arc_segment.h"

/**
 * @brief Execute Task1: A->B straight segment with start/finish alarms.
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
 * @brief Emit a compact Task2 sequence-level event into the RAM log.
 */
static void task2_log_sequence_event(const char *tag,
    uint8_t event,
    uint8_t reason)
{
    task2_ram_log_event(tag,
        event,
        reason,
        0U,
        0,
        0,
        0,
        0U,
        0,
        0U,
        0U,
        0,
        0);
}

/**
 * @brief Common Task2 abort path: record reason, finish alarm, dump RAM log.
 */
static void task2_abort_sequence(const char *tag, uint8_t reason)
{
    task2_log_sequence_event(tag, TASK2_EVT_ABORT, reason);
    st011_finish_pending_pulse();
    task2_ram_log_dump();
}

/**
 * @brief Execute Task2: AB straight, BC arc, CD straight, DA arc.
 */
static void run_task2_abcd(void)
{
    uint8_t reason;

    task2_ram_log_reset();
    task2_log_sequence_event("TASK2_AB", TASK2_EVT_START, 0U);

    {
        const straight_line_segment_config_t config = {
            .tag = "TASK2_AB",
            .zero_heading = 0U,
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
        task2_abort_sequence("TASK2_AB", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    reason = run_task2_bc_race_arc_debug("TASK2_BC");
    if (reason != 1U) {
        task2_abort_sequence("TASK2_BC", reason);
        return;
    }

    reason = run_task2_cd_exit_angle_straight("TASK2_CD");
    if (reason != 1U) {
        task2_abort_sequence("TASK2_CD", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    task2_log_sequence_event("TASK2_CD", TASK2_EVT_COMPLETE, reason);

    reason = run_task2_da_race_arc("TASK2_DA");
    if (reason != 1U) {
        task2_abort_sequence("TASK2_DA", reason);
        return;
    }

    TB6612_Brake();
    task2_log_sequence_event("TASK2_DA", TASK2_EVT_COMPLETE, reason);
    st011_finish_pending_pulse();
    task2_ram_log_dump();
}

#endif
