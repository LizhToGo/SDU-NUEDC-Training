#ifndef RACE_LAPS_H
#define RACE_LAPS_H

/**
 * @file race_laps.h
 * @brief 任务三/任务四跑圈顶层编排。
 *
 * 本头文件在 main.c 声明竞速配置类型和辅助钩子后引入。底层转向、
 * 前进、找线等动作在 race_primitives.h，中层阶段状态机在
 * race_phase.h。
 */

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_motion_utils.h"
#include "app_services.h"
#include "app_straight.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "bsp_tb6612.h"

/**
 * @brief 竞速阶段在点位或保护距离结束时保存的结果快照。
 */
typedef struct {
    uint8_t reason;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t yaw_progress_cdeg;
    uint8_t ir_ok;
    uint8_t nav_ok;
    ir_tracking_sample_t sample;
} line_result_t;

/**
 * @brief 当前 AC/CB/BD/DA 阶段的静态参数。
 */
typedef struct {
    const char *phase_name;
    const char *point_name;
    const char *force_name;
    int32_t point_arm_count;
    int32_t force_count;
    int32_t phase_turn_dir;
    int32_t straight_target_cdeg;
    uint8_t arc_mode;
} race_phase_config_t;

/**
 * @brief 阶段辅助函数共享的完整竞速运行期状态。
 */
typedef struct {
    line_result_t result;
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    straight_pid_t diff_pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    uint32_t elapsed_ms;
    uint32_t phase_start_ms;
    uint32_t report_elapsed_ms;
    uint32_t nav_frame_delta;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t total_distance_count;
    int32_t phase_distance_count;
    int32_t phase_start_count;
    int32_t filtered_error;
    int32_t last_filtered_error;
    int32_t last_turn;
    int32_t yaw_start;
    int32_t yaw_cdeg;
    int32_t yaw_raw_cdeg;
    int32_t phase_yaw_cdeg;
    int32_t gyro_z_mdps;
    int32_t gyro_z_filtered_mdps;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    int32_t yaw_progress_cdeg;
    int32_t raw_error;
    int32_t derivative;
    int32_t base_pwm;
    int32_t left_pwm;
    int32_t right_pwm;
    int32_t target_speed_diff;
    int32_t line_turn;
    int32_t nav_turn;
    int32_t control_turn;
    int32_t heading_error_cdeg;
    int32_t expected_yaw_cdeg;
    int32_t arc_actual_yaw_cdeg;
    uint8_t lap_count;
    uint8_t phase;
    uint8_t target_laps;
    uint8_t nav_ok;
    uint8_t nav_update_flags;
    uint8_t straight_point_count;
    uint16_t straight_line_seen_count;
    uint8_t ir_ok;
    uint8_t line_valid;
    uint8_t line_lost_seen;
    uint8_t straight_point_candidate;
    uint8_t edge_point_seen;
    uint8_t point_ready;
    uint8_t stop_reason;
} race_context_t;

/**
 * @brief 将内部停止/点位原因码转换为日志文本。
 */
static const char *race_reason_name(uint8_t reason)
{
    if (reason == 0U) {
        return "none";
    }
    if (reason == 1U) {
        return "point";
    }
    if (reason == 2U) {
        return "force";
    }
    if (reason == 3U) {
        return "uart_stop";
    }
    if (reason == 5U) {
        return "nav_invalid";
    }
    if (reason == 6U) {
        return "yaw";
    }
    return "timeout";
}

/**
 * @brief 将阶段序号 0..3 转为 AC/CB/BD/DA 文本。
 */
static const char *race_phase_name(uint8_t phase)
{
    if (phase == 0U) {
        return "AC";
    }
    if (phase == 1U) {
        return "CB";
    }
    if (phase == 2U) {
        return "BD";
    }
    return "DA";
}

#include "race_primitives.h"
#include "race_phase.h"

/**
 * @brief 任务二 BC 弧线退出后专用的 CD 固定航向直线段。
 */
static uint8_t run_task2_cd_exit_angle_straight(const char *tag)
{
    const straight_line_segment_config_t config = {
        .tag = tag,
        .zero_heading = 0U,
        .start_alarm_ms = 0U,
        .stop_alarm_ms = 0U,
        .line_arm_count = TASK2_STRAIGHT_SEARCH_START_COUNT,
        .force_stop_count = RACE_STRAIGHT_FORCE_COUNT,
        .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
        .yaw_corr_enable = 1U,
        .entry_brake_enable = 0U,
        .fixed_yaw_target_enable = 1U,
        .fixed_yaw_target_cdeg = TASK2_CD_STRAIGHT_TARGET_CDEG
    };

    return run_straight_to_line_segment(&config);
}

/**
 * @brief 为任务二单独启动一个竞速弧线阶段。
 *
 * 任务二只复用任务三/四的弧线巡线、航向辅助和出弧点判断，不执行竞速
 * 阶段点位后的快速转向动作。这样可以保持任务二路线为 AB 直行、BC 弧线、
 * CD 直行、DA 弧线。
 */
static void task2_prepare_race_arc_context(race_context_t *ctx,
    uint8_t race_phase)
{
    ctx->target_laps = 1U;
    ctx->lap_count = 0U;
    ctx->phase = race_phase;
    ctx->phase_start_ms = 0U;
    ctx->phase_start_count = 0;

    /*
     * 单段弧线从当前姿态和当前编码器距离重新开始计量，避免继承 AB/CD
     * 直线段的距离和滤波状态。
     */
    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    race_reset_segment_control(ctx);
    race_read_navigation_state(ctx, 1U);
}

/**
 * @brief 运行一个复用竞速状态机的任务二弧线段，直到正常出弧或保护退出。
 */
static uint8_t run_task2_race_arc_phase(const char *tag, uint8_t race_phase)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;
    uint8_t phase = (race_phase == 1U) ? 1U : 3U;

    /*
     * 只允许复用竞速 CB(左弧) 或 DA(右弧) 两种弧线阶段。
     * 当前任务二车场路线的 BC/DA 都按右弧线接入，因此默认映射到 DA。
     */
    task2_prepare_race_arc_context(&ctx, phase);
    race_configure_phase(&ctx, &phase_config);
    /* 覆盖日志名称，让串口输出仍然能直接看出这是任务二的 BC/DA 段。 */
    phase_config.phase_name = tag;
    phase_config.point_name = (phase == 1U) ? "TASK2_C_EXIT" : "TASK2_A_EXIT";
    phase_config.force_name = (phase == 1U) ? "TASK2_C_FORCE" : "TASK2_A_FORCE";

    race_log_printf("%s start: reuse_race_arc src=%s turn=%ld yaw0=%ld nav=%u\r\n",
        tag,
        race_phase_name(phase),
        phase_config.phase_turn_dir,
        ctx.yaw_start,
        ctx.nav_ok);

    while (ctx.elapsed_ms < TASK3_ARC_MAX_RUN_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        ctx.elapsed_ms += CONTROL_PERIOD_MS;
        ctx.report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            ctx.stop_reason = 3U;
            break;
        }

        race_update_loop_state(&ctx, &phase_config);
        race_compute_loop_control(&ctx, &phase_config);

        if (race_check_phase_point(&ctx, &phase_config) != 0U) {
            /*
             * 任务三/四在点位后会执行快速转向或前推；任务二这里只需要
             * “出弧成功”这个事件，后续动作由 run_task2_abcd() 按路线编排。
             */
            race_capture_result(&ctx, 1U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                1U);
            ctx.stop_reason = 1U;
            break;
        }

        if (ctx.phase_distance_count >= phase_config.force_count) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                0U);
            ctx.stop_reason = 2U;
            break;
        }

        TB6612_SetDifferential((int16_t)ctx.left_pwm,
            (int16_t)ctx.right_pwm);
        race_log_periodic_data(&ctx, &phase_config);
    }

    if (ctx.stop_reason == 0U) {
        ctx.stop_reason = 4U;
    }
    if (ctx.stop_reason != 1U) {
        TB6612_Brake();
    }

    race_log_printf("%s stop: reason=%s src=%s t=%lu dist=%ld yaw=%ld yprog=%ld nav=%u ir=%u raw=0x%02X mask=0x%02X cnt=%u\r\n",
        tag,
        race_reason_name(ctx.stop_reason),
        race_phase_name(phase),
        ctx.elapsed_ms,
        ctx.phase_distance_count,
        ctx.yaw_cdeg,
        ctx.yaw_progress_cdeg,
        ctx.nav_ok,
        ctx.ir_ok,
        (ctx.ir_ok != 0U) ? ctx.sample.raw : 0xFFU,
        (ctx.ir_ok != 0U) ? ctx.sample.line_mask : 0U,
        (ctx.ir_ok != 0U) ? ctx.sample.active_count : 0U);

    return ctx.stop_reason;
}

/**
 * @brief 任务二 BC 弧线，按当前车场路线复用竞速 DA 右弧线控制。
 */
static uint8_t run_task2_bc_race_arc(const char *tag)
{
    return run_task2_race_arc_phase(tag, 3U);
}

/**
 * @brief 任务二 DA 弧线，直接复用竞速 DA 右弧线控制。
 */
static uint8_t run_task2_da_race_arc(const char *tag)
{
    return run_task2_race_arc_phase(tag, 3U);
}

/**
 * @brief 使用共享 AC/CB/BD/DA 阶段循环执行任务三/任务四。
 */
static void run_race_laps(uint8_t target_laps)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;
    uint32_t control_period_ms;

    race_init_lap_context(&ctx, target_laps);
    if (ctx.stop_reason != 0U) {
        race_finish_lap_context(&ctx);
        return;
    }
    control_period_ms = (ctx.target_laps == TASK4_LAP_COUNT) ?
        RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS;

    while ((ctx.elapsed_ms < RACE_TOTAL_MAX_RUN_MS) &&
        (ctx.lap_count < ctx.target_laps)) {
        race_configure_phase(&ctx, &phase_config);

        delay_ms_with_st011(control_period_ms);
        ctx.elapsed_ms += control_period_ms;
        ctx.report_elapsed_ms += control_period_ms;

        if (task_uart_stop_requested() != 0U) {
            ctx.stop_reason = 3U;
            break;
        }

        race_update_loop_state(&ctx, &phase_config);
        race_compute_loop_control(&ctx, &phase_config);

        if (race_check_phase_point(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 1U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                1U);

            if (race_execute_point_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }

            race_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        if (race_check_straight_force_turn(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                0U);

            if (race_execute_straight_force_turn_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }

            race_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        if (ctx.phase_distance_count >= phase_config.force_count) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                0U);

            race_advance_segment(&ctx, 0U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        TB6612_SetDifferential((int16_t)ctx.left_pwm, (int16_t)ctx.right_pwm);
        race_log_periodic_data(&ctx, &phase_config);
    }

    race_finish_lap_context(&ctx);
}

#endif
