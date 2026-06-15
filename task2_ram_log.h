#ifndef TASK2_RAM_LOG_H
#define TASK2_RAM_LOG_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_services.h"
#include "app_straight.h"
#include "board.h"
#include "bsp_ir_tracking.h"

#if TASK2_RAM_LOG_ENABLE
enum {
    TASK2_SEG_AB = 0,
    TASK2_SEG_BC = 1,
    TASK2_SEG_CD = 2,
    TASK2_SEG_DA = 3
};

enum {
    TASK2_EVT_START = 1,
    TASK2_EVT_POINT = 2,
    TASK2_EVT_STOP = 3,
    TASK2_EVT_ABORT = 4,
    TASK2_EVT_COMPLETE = 5
};

typedef struct {
    uint32_t t_ms;
    uint16_t dist_count;
    int16_t yaw_cdeg;
    int16_t phase_yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t expected_yaw_cdeg;
    int16_t heading_error_cdeg;
    int16_t gzlp_x100_mdps;
    int16_t motor_b_total;
    int16_t motor_a_total;
    int16_t distance_error;
    int16_t distance_correction;
    int16_t motor_b_speed;
    int16_t motor_a_speed;
    int16_t pid_error;
    int16_t feedforward_correction;
    int16_t feedback_correction;
    int16_t correction;
    int16_t line_turn;
    int16_t nav_turn;
    int16_t control_turn;
    int16_t target_speed_diff;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    uint8_t seg;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    uint8_t flags;
} task2_ram_sample_t;

typedef struct {
    uint32_t t_ms;
    uint16_t dist_count;
    int16_t yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t motor_b_total;
    int16_t motor_a_total;
    uint8_t seg;
    uint8_t event;
    uint8_t reason;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    uint8_t flags;
} task2_ram_event_t;

static task2_ram_sample_t g_task2_ram_samples[TASK2_RAM_SAMPLE_CAPACITY];
static task2_ram_event_t g_task2_ram_events[TASK2_RAM_EVENT_CAPACITY];
static uint16_t g_task2_ram_sample_count;
static uint16_t g_task2_ram_sample_overflow;
static uint8_t g_task2_ram_event_count;
static uint8_t g_task2_ram_event_overflow;

static int16_t task2_sat_i16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t task2_sat_u16(int32_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value > 65535) {
        return 65535U;
    }
    return (uint16_t)value;
}

static uint8_t task2_ram_segment_from_tag(const char *tag)
{
    if ((tag != 0) && (tag[6] == 'B') && (tag[7] == 'C')) {
        return TASK2_SEG_BC;
    }
    if ((tag != 0) && (tag[6] == 'C') && (tag[7] == 'D')) {
        return TASK2_SEG_CD;
    }
    if ((tag != 0) && (tag[6] == 'D') && (tag[7] == 'A')) {
        return TASK2_SEG_DA;
    }
    return TASK2_SEG_AB;
}

static uint8_t task2_ram_enabled_for_tag(const char *tag)
{
    return ((tag != 0) &&
        (tag[0] == 'T') &&
        (tag[1] == 'A') &&
        (tag[2] == 'S') &&
        (tag[3] == 'K') &&
        (tag[4] == '2')) ? 1U : 0U;
}

static const char *task2_ram_seg_name(uint8_t seg)
{
    if (seg == TASK2_SEG_BC) {
        return "BC";
    }
    if (seg == TASK2_SEG_CD) {
        return "CD";
    }
    if (seg == TASK2_SEG_DA) {
        return "DA";
    }
    return "AB";
}

static const char *task2_ram_event_name(uint8_t event)
{
    if (event == TASK2_EVT_START) {
        return "start";
    }
    if (event == TASK2_EVT_POINT) {
        return "point";
    }
    if (event == TASK2_EVT_STOP) {
        return "stop";
    }
    if (event == TASK2_EVT_ABORT) {
        return "abort";
    }
    if (event == TASK2_EVT_COMPLETE) {
        return "complete";
    }
    return "unknown";
}

static const char *task2_ram_reason_name(uint8_t reason)
{
    if (reason == 1U) {
        return "line_or_yaw";
    }
    if (reason == 2U) {
        return "force";
    }
    if (reason == 3U) {
        return "uart_stop";
    }
    return "timeout";
}

static uint8_t task2_ram_flags(uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t nav_ok,
    uint8_t marker)
{
    uint8_t flags = 0U;

    flags |= (ir_ok != 0U) ? 0x01U : 0U;
    flags |= ((ir_ok == 0U) || ((sample != 0) && (sample->line_lost != 0U))) ?
        0x02U : 0U;
    flags |= (nav_ok != 0U) ? 0x04U : 0U;
    flags |= (marker != 0U) ? 0x08U : 0U;

    return flags;
}

static void task2_ram_log_reset(void)
{
    g_task2_ram_sample_count = 0U;
    g_task2_ram_sample_overflow = 0U;
    g_task2_ram_event_count = 0U;
    g_task2_ram_event_overflow = 0U;
}

static void task2_ram_log_sample(const char *tag,
    uint32_t elapsed_ms,
    int32_t distance_count,
    int32_t yaw_cdeg,
    int32_t phase_yaw_cdeg,
    int32_t yaw_progress_cdeg,
    int32_t expected_yaw_cdeg,
    int32_t heading_error_cdeg,
    int32_t gyro_z_filtered_mdps,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t nav_ok,
    uint8_t marker,
    int32_t motor_b_total,
    int32_t motor_a_total,
    const straight_drive_output_t *drive,
    int32_t correction,
    int32_t line_turn,
    int32_t nav_turn,
    int32_t control_turn,
    int32_t target_speed_diff,
    int32_t motor_b_pwm,
    int32_t motor_a_pwm)
{
    task2_ram_sample_t *log;

    if (g_task2_ram_sample_count >= TASK2_RAM_SAMPLE_CAPACITY) {
        g_task2_ram_sample_overflow++;
        return;
    }

    log = &g_task2_ram_samples[g_task2_ram_sample_count++];
    log->t_ms = elapsed_ms;
    log->dist_count = task2_sat_u16(distance_count);
    log->yaw_cdeg = task2_sat_i16(yaw_cdeg);
    log->phase_yaw_cdeg = task2_sat_i16(phase_yaw_cdeg);
    log->yaw_progress_cdeg = task2_sat_i16(yaw_progress_cdeg);
    log->expected_yaw_cdeg = task2_sat_i16(expected_yaw_cdeg);
    log->heading_error_cdeg = task2_sat_i16(heading_error_cdeg);
    log->gzlp_x100_mdps = task2_sat_i16(gyro_z_filtered_mdps / 100);
    log->motor_b_total = task2_sat_i16(motor_b_total);
    log->motor_a_total = task2_sat_i16(motor_a_total);
    log->distance_error = (drive != 0) ? task2_sat_i16(drive->distance_error) : 0;
    log->distance_correction = (drive != 0) ? task2_sat_i16(drive->distance_correction) : 0;
    log->motor_b_speed = (drive != 0) ? task2_sat_i16(drive->motor_b_speed) : 0;
    log->motor_a_speed = (drive != 0) ? task2_sat_i16(drive->motor_a_speed) : 0;
    log->pid_error = (drive != 0) ? task2_sat_i16(drive->pid_error) : 0;
    log->feedforward_correction = (drive != 0) ?
        task2_sat_i16(drive->feedforward_correction) : 0;
    log->feedback_correction = (drive != 0) ?
        task2_sat_i16(drive->feedback_correction) : 0;
    log->correction = task2_sat_i16(correction);
    log->line_turn = task2_sat_i16(line_turn);
    log->nav_turn = task2_sat_i16(nav_turn);
    log->control_turn = task2_sat_i16(control_turn);
    log->target_speed_diff = task2_sat_i16(target_speed_diff);
    log->motor_b_pwm = task2_sat_i16(motor_b_pwm);
    log->motor_a_pwm = task2_sat_i16(motor_a_pwm);
    log->seg = task2_ram_segment_from_tag(tag);
    log->raw = (ir_ok != 0U) ? sample->raw : 0xFFU;
    log->line_mask = (ir_ok != 0U) ? sample->line_mask : 0U;
    log->active_count = (ir_ok != 0U) ? sample->active_count : 0U;
    log->flags = task2_ram_flags(ir_ok, sample, nav_ok, marker);
}

static void task2_ram_log_event(const char *tag,
    uint8_t event,
    uint8_t reason,
    uint32_t elapsed_ms,
    int32_t distance_count,
    int32_t yaw_cdeg,
    int32_t yaw_progress_cdeg,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t nav_ok,
    uint8_t marker,
    int32_t motor_b_total,
    int32_t motor_a_total)
{
    task2_ram_event_t *log;

    if (g_task2_ram_event_count >= TASK2_RAM_EVENT_CAPACITY) {
        g_task2_ram_event_overflow++;
        return;
    }

    log = &g_task2_ram_events[g_task2_ram_event_count++];
    log->t_ms = elapsed_ms;
    log->dist_count = task2_sat_u16(distance_count);
    log->yaw_cdeg = task2_sat_i16(yaw_cdeg);
    log->yaw_progress_cdeg = task2_sat_i16(yaw_progress_cdeg);
    log->motor_b_total = task2_sat_i16(motor_b_total);
    log->motor_a_total = task2_sat_i16(motor_a_total);
    log->seg = task2_ram_segment_from_tag(tag);
    log->event = event;
    log->reason = reason;
    log->raw = (ir_ok != 0U) ? sample->raw : 0xFFU;
    log->line_mask = (ir_ok != 0U) ? sample->line_mask : 0U;
    log->active_count = (ir_ok != 0U) ? sample->active_count : 0U;
    log->flags = task2_ram_flags(ir_ok, sample, nav_ok, marker);
}

static void task2_ram_dump_line_pause(void)
{
#if TASK2_DUMP_LINE_DELAY_MS > 0
    delay_ms(TASK2_DUMP_LINE_DELAY_MS);
#endif
}

static void task2_ram_log_dump(void)
{
    uint16_t i;

    lc_printf("TASK2_RAM_BEGIN samples=%u/%u sample_ov=%u events=%u/%u event_ov=%u\r\n",
        g_task2_ram_sample_count,
        TASK2_RAM_SAMPLE_CAPACITY,
        g_task2_ram_sample_overflow,
        g_task2_ram_event_count,
        TASK2_RAM_EVENT_CAPACITY,
        g_task2_ram_event_overflow);
    task2_ram_dump_line_pause();

    lc_printf("TASK2_CFG line_period=%d arc_period=%d ab_arm=%d ab_force=%d arc_base=%d arc_diff=%d/%d yaw_arm=%d yaw_stop=%d force=%ld\r\n",
        TASK1_REPORT_PERIOD_MS,
        TASK2_ARC_REPORT_PERIOD_MS,
        TASK1_B_LINE_ARM_COUNT,
        TASK1_FORCE_STOP_COUNT,
        TASK11_ARC_BASE_PWM,
        TASK11_DA_ARC_ENTRY_TARGET_DIFF,
        TASK11_DA_ARC_CRUISE_TARGET_DIFF,
        TASK11_ARC_POINT_YAW_ARM_CDEG,
        TASK2_ARC_ALIGN_TARGET_CDEG,
        (int32_t)TASK2_ARC_FORCE_STOP_COUNT);
    task2_ram_dump_line_pause();

    lc_printf("TASK2_DUMP_SECTION name=EVT count=%u\r\n", g_task2_ram_event_count);
    task2_ram_dump_line_pause();
    for (i = 0U; i < g_task2_ram_event_count; i++) {
        const task2_ram_event_t *log = &g_task2_ram_events[i];
        lc_printf("TASK2_EVT idx=%u seg=%s event=%s reason=%s t=%lu dist=%u yaw=%d yprog=%d raw=0x%02X mask=0x%02X cnt=%u flags=0x%02X B=%d A=%d\r\n",
            i,
            task2_ram_seg_name(log->seg),
            task2_ram_event_name(log->event),
            task2_ram_reason_name(log->reason),
            log->t_ms,
            log->dist_count,
            log->yaw_cdeg,
            log->yaw_progress_cdeg,
            log->raw,
            log->line_mask,
            log->active_count,
            log->flags,
            log->motor_b_total,
            log->motor_a_total);
        task2_ram_dump_line_pause();
    }
    lc_printf("TASK2_DUMP_SECTION_END name=EVT\r\n");
    task2_ram_dump_line_pause();

    lc_printf("TASK2_DUMP_SECTION name=SAMPLE count=%u\r\n", g_task2_ram_sample_count);
    task2_ram_dump_line_pause();
    for (i = 0U; i < g_task2_ram_sample_count; i++) {
        const task2_ram_sample_t *log = &g_task2_ram_samples[i];
        lc_printf("TASK2_DATA idx=%u seg=%s t=%lu dist=%u yaw=%d pyaw=%d yprog=%d exp=%d herr=%d gzlp=%d raw=0x%02X mask=0x%02X cnt=%u flags=0x%02X B_total=%d A_total=%d d_err=%d d_corr=%d B_spd=%d A_spd=%d v_err=%d ff=%d fb=%d corr=%d line_turn=%d nav_turn=%d turn=%d tdiff=%d pwm=%d/%d\r\n",
            i,
            task2_ram_seg_name(log->seg),
            log->t_ms,
            log->dist_count,
            log->yaw_cdeg,
            log->phase_yaw_cdeg,
            log->yaw_progress_cdeg,
            log->expected_yaw_cdeg,
            log->heading_error_cdeg,
            log->gzlp_x100_mdps,
            log->raw,
            log->line_mask,
            log->active_count,
            log->flags,
            log->motor_b_total,
            log->motor_a_total,
            log->distance_error,
            log->distance_correction,
            log->motor_b_speed,
            log->motor_a_speed,
            log->pid_error,
            log->feedforward_correction,
            log->feedback_correction,
            log->correction,
            log->line_turn,
            log->nav_turn,
            log->control_turn,
            log->target_speed_diff,
            log->motor_b_pwm,
            log->motor_a_pwm);
        task2_ram_dump_line_pause();
    }
    lc_printf("TASK2_DUMP_SECTION_END name=SAMPLE\r\n");
    task2_ram_dump_line_pause();
    lc_printf("TASK2_RAM_END\r\n");
}
#else
#define task2_ram_log_reset() ((void)0)
#define task2_ram_log_sample(tag, elapsed_ms, distance_count, yaw_cdeg, phase_yaw_cdeg, yaw_progress_cdeg, expected_yaw_cdeg, heading_error_cdeg, gyro_z_filtered_mdps, ir_ok, sample, nav_ok, marker, motor_b_total, motor_a_total, drive, correction, line_turn, nav_turn, control_turn, target_speed_diff, motor_b_pwm, motor_a_pwm) ((void)0)
#define task2_ram_log_event(tag, event, reason, elapsed_ms, distance_count, yaw_cdeg, yaw_progress_cdeg, ir_ok, sample, nav_ok, marker, motor_b_total, motor_a_total) ((void)0)
#define task2_ram_log_dump() ((void)0)
#define task2_ram_enabled_for_tag(tag) (0U)
#endif

#endif
