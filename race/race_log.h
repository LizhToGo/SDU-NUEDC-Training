#ifndef RACE_LOG_H
#define RACE_LOG_H

/**
 * @file race_log.h
 * @brief Race RAM log storage and dump helpers.
 *
 * Stores Task3/Task4 race windows, events, and segment summaries in RAM and
 * emits RACE_* logs when enabled.
 */

enum {
    RACE_RAM_EVENT_START = 1,
    RACE_RAM_EVENT_POINT = 2,
    RACE_RAM_EVENT_FORCE = 3,
    RACE_RAM_EVENT_ADVANCE_START = 4,
    RACE_RAM_EVENT_ADVANCE_STOP = 5,
    RACE_RAM_EVENT_TURN_START = 6,
    RACE_RAM_EVENT_TURN_STOP = 7,
    RACE_RAM_EVENT_COMPLETE = 8,
    RACE_RAM_EVENT_SEGMENT_START = 9
};

#define RACE_LOG_FLAG_IR_OK            (0x01U)
#define RACE_LOG_FLAG_LINE_LOST        (0x02U)
#define RACE_LOG_FLAG_EDGE_SEEN        (0x04U)
#define RACE_LOG_FLAG_NAV_FRAME        (0x08U)
#define RACE_LOG_FLAG_GUIDE_SEEN       (0x10U)
#define RACE_LOG_FLAG_START_WINDOW     (0x20U)
#define RACE_LOG_FLAG_ARC_MODE         (0x40U)
#define RACE_LOG_FLAG_PREDICT_STOP     (0x80U)

#ifndef RACE_DUMP_SUM_ENABLE
#define RACE_DUMP_SUM_ENABLE           (1)
#endif
#ifndef RACE_DUMP_EVT_ENABLE
#define RACE_DUMP_EVT_ENABLE           (1)
#endif
#ifndef RACE_DUMP_WIN_ENABLE
#define RACE_DUMP_WIN_ENABLE           (1)
#endif
#ifndef RACE_DUMP_WIN_STRIDE
#define RACE_DUMP_WIN_STRIDE           (1)
#endif
#ifndef RACE_RAM_WINDOW_LOG_STRIDE
#define RACE_RAM_WINDOW_LOG_STRIDE     (1)
#endif

/* Elapsed time spent in post-point actions; used to keep event timestamps monotonic. */
static uint32_t g_race_post_point_elapsed_ms;

#if RACE_RAM_LOG_ENABLE
/**
 * @brief High-rate sample captured near point windows.
 */
typedef struct {
    uint32_t t_ms;
    uint16_t dist_count;
    int16_t yaw_cdeg;
    int16_t yaw_raw_cdeg;
    int16_t phase_yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t expected_yaw_cdeg;
    int16_t heading_error_cdeg;
    int16_t line_error;
    int16_t nav_turn;
    int16_t control_turn;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    int16_t gyro_z_x100_mdps;
    int16_t gzlp_x100_mdps;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    uint8_t lap;
    uint8_t phase;
    uint8_t line_mask;
    uint8_t active_count;
    uint8_t flags;
    uint8_t nav_update_flags;
    uint8_t nav_frame_delta;
} race_window_log_t;

/**
 * @brief Discrete race event record: point, force stop, turn start/stop, etc.
 */
typedef struct {
    uint32_t t_ms;
    uint16_t dist_count;
    uint16_t phase_dist_count;
    int16_t yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t yaw_delta_cdeg;
    int16_t expected_yaw_cdeg;
    int16_t heading_error_cdeg;
    int16_t nav_turn;
    int16_t gzlp_x100_mdps;
    int16_t line_error;
    int16_t motor_b_total;
    int16_t motor_a_total;
    uint8_t lap;
    uint8_t phase;
    uint8_t event;
    uint8_t reason;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    uint8_t flags;
} race_event_log_t;

/**
 * @brief Per-segment summary accumulated from many window samples.
 */
typedef struct {
    uint32_t start_ms;
    uint32_t end_ms;
    uint16_t dist_count;
    uint16_t sample_count;
    uint16_t nav_sample_count;
    uint16_t nav_lost_count;
    uint16_t nav_frame_count;
    uint16_t nav_stale_count;
    uint16_t line_seen_count;
    uint16_t first_line_dist_count;
    uint16_t last_line_dist_count;
    uint16_t line_span_count;
    uint16_t end_gap_count;
    uint16_t lost_count;
    uint16_t max_lost_streak_count;
    uint16_t end_lost_streak_count;
    uint16_t max_abs_error;
    uint16_t max_abs_heading_error;
    uint16_t max_abs_gyro_z_x100_mdps;
    uint16_t max_abs_gzlp_x100_mdps;
    int16_t yaw_start_cdeg;
    int16_t yaw_end_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t end_heading_error_cdeg;
    int16_t avg_abs_error;
    int16_t avg_abs_heading_error;
    int16_t avg_gyro_z_x100_mdps;
    int16_t avg_gzlp_x100_mdps;
    int16_t avg_line_turn;
    int16_t avg_nav_turn;
    int16_t avg_turn;
    uint8_t lap;
    uint8_t phase;
    uint8_t reason;
    uint8_t point_mask;
    uint8_t point_flags;
    uint8_t nav_update_flags;
} race_summary_log_t;

/**
 * @brief Working accumulator used while a race segment is running.
 */
typedef struct {
    uint32_t start_ms;
    uint32_t sample_count;
    uint32_t nav_sample_count;
    uint32_t nav_lost_count;
    uint32_t nav_frame_count;
    uint32_t nav_stale_count;
    uint32_t line_seen_count;
    int32_t first_line_dist_count;
    int32_t last_line_dist_count;
    uint32_t lost_count;
    uint32_t current_lost_streak_count;
    uint32_t max_lost_streak_count;
    int32_t yaw_start_cdeg;
    int32_t sum_abs_error;
    int32_t max_abs_error;
    int32_t sum_abs_heading_error;
    int32_t max_abs_heading_error;
    int32_t sum_gyro_z_mdps;
    int32_t max_abs_gyro_z_mdps;
    int32_t sum_gyro_z_filtered_mdps;
    int32_t max_abs_gyro_z_filtered_mdps;
    int32_t sum_line_turn;
    int32_t sum_nav_turn;
    int32_t sum_turn;
    uint8_t lap;
    uint8_t phase;
    uint8_t nav_update_flags;
} race_segment_accum_t;

/**
 * @brief RAM backing store for race window, event, and summary logs.
 */
typedef struct {
    race_window_log_t window_log[RACE_RAM_WINDOW_CAPACITY];
    race_event_log_t event_log[RACE_RAM_EVENT_CAPACITY];
    race_summary_log_t summary_log[RACE_RAM_SUMMARY_CAPACITY];
} race_ram_storage_t;

#if TASK5_RAM_LOG_ENABLE
typedef union {
    race_ram_storage_t race;
    task5_ram_log_t task5[TASK5_RAM_LOG_CAPACITY];
} task_ram_log_storage_t;

static task_ram_log_storage_t g_task_ram_log_storage;
#define g_race_window_log (g_task_ram_log_storage.race.window_log)
#define g_race_event_log (g_task_ram_log_storage.race.event_log)
#define g_race_summary_log (g_task_ram_log_storage.race.summary_log)
task5_ram_log_t * const g_task5_ram_log = g_task_ram_log_storage.task5;
uint16_t g_task5_ram_log_count;
uint16_t g_task5_ram_log_overflow;
#else
static race_ram_storage_t g_race_ram_storage;
#define g_race_window_log (g_race_ram_storage.window_log)
#define g_race_event_log (g_race_ram_storage.event_log)
#define g_race_summary_log (g_race_ram_storage.summary_log)
#endif

static race_segment_accum_t g_race_segment_accum;
static uint16_t g_race_window_log_count;
static uint16_t g_race_event_log_count;
static uint8_t g_race_summary_log_count;
static uint16_t g_race_window_log_overflow;
static uint16_t g_race_event_log_overflow;
static uint8_t g_race_summary_log_overflow;
static uint16_t g_race_window_log_decimator;
static uint8_t g_race_log_lap;
static uint8_t g_race_log_phase;
static uint32_t g_race_post_point_base_ms;
static int32_t g_race_post_point_phase_dist_count;

/**
 * @brief Start timestamp context for actions that happen after a point.
 */
static void race_post_point_context_begin(uint32_t point_ms,
    int32_t phase_distance_count)
{
    g_race_post_point_base_ms = point_ms;
    g_race_post_point_elapsed_ms = 0U;
    g_race_post_point_phase_dist_count = phase_distance_count;
}

/**
 * @brief Convert a post-point local timestamp into race-global elapsed time.
 */
static uint32_t race_post_point_event_ms(uint32_t local_ms)
{
    return g_race_post_point_base_ms +
        g_race_post_point_elapsed_ms +
        local_ms;
}

/**
 * @brief Saturate a signed value before storing compact race log fields.
 */
static int16_t race_sat_i16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

/**
 * @brief Saturate a non-negative value before storing compact race log fields.
 */
static uint16_t race_sat_u16(int32_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value > 65535) {
        return 65535U;
    }
    return (uint16_t)value;
}

/**
 * @brief Convert race RAM event id to dump text.
 */
static const char *race_ram_event_name(uint8_t event)
{
    if (event == RACE_RAM_EVENT_START) {
        return "start";
    }
    if (event == RACE_RAM_EVENT_POINT) {
        return "point";
    }
    if (event == RACE_RAM_EVENT_FORCE) {
        return "force";
    }
    if (event == RACE_RAM_EVENT_ADVANCE_START) {
        return "advance_start";
    }
    if (event == RACE_RAM_EVENT_ADVANCE_STOP) {
        return "advance_stop";
    }
    if (event == RACE_RAM_EVENT_TURN_START) {
        return "turn_start";
    }
    if (event == RACE_RAM_EVENT_TURN_STOP) {
        return "turn_stop";
    }
    if (event == RACE_RAM_EVENT_COMPLETE) {
        return "complete";
    }
    if (event == RACE_RAM_EVENT_SEGMENT_START) {
        return "segment_start";
    }
    return "unknown";
}

/**
 * @brief Clear all race RAM logs and post-point timing state.
 */
static void race_ram_log_reset(void)
{
    g_race_window_log_count = 0U;
    g_race_event_log_count = 0U;
    g_race_summary_log_count = 0U;
    g_race_window_log_overflow = 0U;
    g_race_event_log_overflow = 0U;
    g_race_summary_log_overflow = 0U;
    g_race_window_log_decimator = 0U;
    g_race_log_lap = 0U;
    g_race_log_phase = 0U;
    g_race_segment_accum.sample_count = 0U;
}

/**
 * @brief Set the active lap/phase context for later primitive action logs.
 */
static void race_ram_log_set_context(uint8_t lap, uint8_t phase)
{
    g_race_log_lap = lap;
    g_race_log_phase = phase;
}

/**
 * @brief Start accumulating summary statistics for a new segment.
 */
static void race_ram_log_segment_reset(uint8_t lap,
    uint8_t phase,
    uint32_t start_ms,
    int32_t yaw_start_cdeg)
{
    g_race_segment_accum.start_ms = start_ms;
    g_race_segment_accum.sample_count = 0U;
    g_race_segment_accum.nav_sample_count = 0U;
    g_race_segment_accum.nav_lost_count = 0U;
    g_race_segment_accum.nav_frame_count = 0U;
    g_race_segment_accum.nav_stale_count = 0U;
    g_race_segment_accum.line_seen_count = 0U;
    g_race_segment_accum.first_line_dist_count = -1;
    g_race_segment_accum.last_line_dist_count = -1;
    g_race_segment_accum.lost_count = 0U;
    g_race_segment_accum.current_lost_streak_count = 0U;
    g_race_segment_accum.max_lost_streak_count = 0U;
    g_race_segment_accum.yaw_start_cdeg = yaw_start_cdeg;
    g_race_segment_accum.sum_abs_error = 0;
    g_race_segment_accum.max_abs_error = 0;
    g_race_segment_accum.sum_abs_heading_error = 0;
    g_race_segment_accum.max_abs_heading_error = 0;
    g_race_segment_accum.sum_gyro_z_mdps = 0;
    g_race_segment_accum.max_abs_gyro_z_mdps = 0;
    g_race_segment_accum.sum_gyro_z_filtered_mdps = 0;
    g_race_segment_accum.max_abs_gyro_z_filtered_mdps = 0;
    g_race_segment_accum.sum_line_turn = 0;
    g_race_segment_accum.sum_nav_turn = 0;
    g_race_segment_accum.sum_turn = 0;
    g_race_segment_accum.lap = lap;
    g_race_segment_accum.phase = phase;
    g_race_segment_accum.nav_update_flags = 0U;
    race_ram_log_set_context(lap, phase);
}

/**
 * @brief Add one loop sample into the active segment accumulator.
 */
static void race_ram_log_segment_sample(uint8_t ir_ok,
    uint8_t nav_ok,
    const ir_tracking_sample_t *sample,
    int32_t phase_distance_count,
    int32_t line_turn,
    int32_t nav_turn,
    int32_t control_turn,
    int32_t heading_error_cdeg,
    int32_t gyro_z_mdps,
    int32_t gyro_z_filtered_mdps,
    uint32_t nav_frame_delta,
    uint8_t nav_update_flags)
{
    int32_t abs_error = 0;
    int32_t abs_heading_error = 0;
    int32_t abs_gyro_z = 0;
    int32_t abs_gzlp = 0;

    g_race_segment_accum.sample_count++;
    g_race_segment_accum.nav_frame_count += nav_frame_delta;
    g_race_segment_accum.nav_update_flags |= nav_update_flags;
    if (nav_frame_delta == 0U) {
        g_race_segment_accum.nav_stale_count++;
    }
    if (nav_ok != 0U) {
        g_race_segment_accum.nav_sample_count++;
        abs_heading_error = abs_i32(heading_error_cdeg);
        g_race_segment_accum.sum_abs_heading_error += abs_heading_error;
        if (abs_heading_error > g_race_segment_accum.max_abs_heading_error) {
                g_race_segment_accum.max_abs_heading_error = abs_heading_error;
        }
        g_race_segment_accum.sum_gyro_z_mdps += gyro_z_mdps;
        g_race_segment_accum.sum_gyro_z_filtered_mdps += gyro_z_filtered_mdps;
        abs_gyro_z = abs_i32(gyro_z_mdps);
        if (abs_gyro_z > g_race_segment_accum.max_abs_gyro_z_mdps) {
            g_race_segment_accum.max_abs_gyro_z_mdps = abs_gyro_z;
        }
        abs_gzlp = abs_i32(gyro_z_filtered_mdps);
        if (abs_gzlp > g_race_segment_accum.max_abs_gyro_z_filtered_mdps) {
            g_race_segment_accum.max_abs_gyro_z_filtered_mdps = abs_gzlp;
        }
    } else {
        g_race_segment_accum.nav_lost_count++;
    }
    if ((ir_ok == 0U) || (sample->line_lost != 0U)) {
        g_race_segment_accum.lost_count++;
        g_race_segment_accum.current_lost_streak_count++;
        if (g_race_segment_accum.current_lost_streak_count >
            g_race_segment_accum.max_lost_streak_count) {
            g_race_segment_accum.max_lost_streak_count =
                g_race_segment_accum.current_lost_streak_count;
        }
    } else {
        abs_error = abs_i32(sample->error);
        if (g_race_segment_accum.first_line_dist_count < 0) {
            g_race_segment_accum.first_line_dist_count = phase_distance_count;
        }
        g_race_segment_accum.last_line_dist_count = phase_distance_count;
        g_race_segment_accum.line_seen_count++;
        g_race_segment_accum.current_lost_streak_count = 0U;
        g_race_segment_accum.sum_abs_error += abs_error;
        if (abs_error > g_race_segment_accum.max_abs_error) {
            g_race_segment_accum.max_abs_error = abs_error;
        }
    }
    g_race_segment_accum.sum_line_turn += line_turn;
    g_race_segment_accum.sum_nav_turn += nav_turn;
    g_race_segment_accum.sum_turn += control_turn;
}

/**
 * @brief Close the active segment accumulator into one summary record.
 */
static void race_ram_log_segment_finish(uint8_t reason,
    uint32_t end_ms,
    int32_t dist_count,
    int32_t yaw_end_cdeg,
    int32_t yaw_progress_cdeg,
    int32_t heading_error_cdeg,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t point_flags)
{
    race_summary_log_t *log;
    int32_t sample_count = (int32_t)g_race_segment_accum.sample_count;
    int32_t nav_sample_count = (int32_t)g_race_segment_accum.nav_sample_count;
    int32_t first_line_dist_count = g_race_segment_accum.first_line_dist_count;
    int32_t last_line_dist_count = g_race_segment_accum.last_line_dist_count;

    if (sample_count <= 0) {
        return;
    }
    if (g_race_segment_accum.lap >= RACE_RAM_LOG_MAX_LAPS) {
        return;
    }
    if (g_race_summary_log_count >= RACE_RAM_SUMMARY_CAPACITY) {
        g_race_summary_log_overflow++;
        return;
    }

    log = &g_race_summary_log[g_race_summary_log_count++];
    log->start_ms = g_race_segment_accum.start_ms;
    log->end_ms = end_ms;
    log->dist_count = race_sat_u16(dist_count);
    log->sample_count = race_sat_u16(sample_count);
    log->nav_sample_count = race_sat_u16(nav_sample_count);
    log->nav_lost_count = race_sat_u16((int32_t)g_race_segment_accum.nav_lost_count);
    log->nav_frame_count = race_sat_u16((int32_t)g_race_segment_accum.nav_frame_count);
    log->nav_stale_count = race_sat_u16((int32_t)g_race_segment_accum.nav_stale_count);
    log->line_seen_count = race_sat_u16((int32_t)g_race_segment_accum.line_seen_count);
    log->first_line_dist_count = race_sat_u16(first_line_dist_count);
    log->last_line_dist_count = race_sat_u16(last_line_dist_count);
    log->line_span_count = (last_line_dist_count >= first_line_dist_count) ?
        race_sat_u16(last_line_dist_count - first_line_dist_count) : 0U;
    log->end_gap_count = (last_line_dist_count >= 0) ?
        race_sat_u16(dist_count - last_line_dist_count) : 0U;
    log->lost_count = race_sat_u16((int32_t)g_race_segment_accum.lost_count);
    log->max_lost_streak_count =
        race_sat_u16((int32_t)g_race_segment_accum.max_lost_streak_count);
    log->end_lost_streak_count =
        race_sat_u16((int32_t)g_race_segment_accum.current_lost_streak_count);
    log->max_abs_error = race_sat_u16(g_race_segment_accum.max_abs_error);
    log->max_abs_heading_error = race_sat_u16(g_race_segment_accum.max_abs_heading_error);
    log->max_abs_gyro_z_x100_mdps =
        race_sat_u16(g_race_segment_accum.max_abs_gyro_z_mdps / 100);
    log->max_abs_gzlp_x100_mdps =
        race_sat_u16(g_race_segment_accum.max_abs_gyro_z_filtered_mdps / 100);
    log->yaw_start_cdeg = race_sat_i16(g_race_segment_accum.yaw_start_cdeg);
    log->yaw_end_cdeg = race_sat_i16(yaw_end_cdeg);
    log->yaw_progress_cdeg = race_sat_i16(yaw_progress_cdeg);
    log->end_heading_error_cdeg = race_sat_i16(heading_error_cdeg);
    log->avg_abs_error = race_sat_i16(g_race_segment_accum.sum_abs_error / sample_count);
    log->avg_abs_heading_error = (nav_sample_count > 0) ?
        race_sat_i16(g_race_segment_accum.sum_abs_heading_error / nav_sample_count) : 0;
    log->avg_gyro_z_x100_mdps = (nav_sample_count > 0) ?
        race_sat_i16((g_race_segment_accum.sum_gyro_z_mdps / nav_sample_count) / 100) : 0;
    log->avg_gzlp_x100_mdps = (nav_sample_count > 0) ?
        race_sat_i16((g_race_segment_accum.sum_gyro_z_filtered_mdps / nav_sample_count) / 100) : 0;
    log->avg_line_turn = race_sat_i16(g_race_segment_accum.sum_line_turn / sample_count);
    log->avg_nav_turn = race_sat_i16(g_race_segment_accum.sum_nav_turn / sample_count);
    log->avg_turn = race_sat_i16(g_race_segment_accum.sum_turn / sample_count);
    log->lap = g_race_segment_accum.lap;
    log->phase = g_race_segment_accum.phase;
    log->reason = reason;
    log->point_mask = ((ir_ok != 0U) && (sample != 0)) ? sample->line_mask : 0U;
    log->point_flags = point_flags;
    log->nav_update_flags = g_race_segment_accum.nav_update_flags;
}

/**
 * @brief Decide whether a high-rate window sample should be retained.
 */
static uint8_t race_ram_window_should_log(uint8_t lap,
    int32_t phase_distance_count,
    int32_t point_arm_count)
{
    int32_t window_start = point_arm_count - RACE_RAM_WINDOW_BEFORE_COUNT;

    if (lap >= RACE_RAM_LOG_MAX_LAPS) {
        return 0U;
    }
    if (window_start < 0) {
        window_start = 0;
    }
    if (phase_distance_count <= RACE_RAM_WINDOW_AFTER_START_COUNT) {
        return 1U;
    }
    return (phase_distance_count >= window_start) ? 1U : 0U;
}

/**
 * @brief Append one high-rate race window sample.
 */
static void race_ram_log_window_sample(uint8_t lap,
    uint8_t phase,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t edge_seen,
    uint8_t extra_flags,
    uint32_t elapsed_ms,
    int32_t phase_distance_count,
    int32_t yaw_cdeg,
    int32_t yaw_raw_cdeg,
    int32_t phase_yaw_cdeg,
    int32_t yaw_progress_cdeg,
    int32_t expected_yaw_cdeg,
    int32_t heading_error_cdeg,
    int32_t nav_turn,
    int32_t control_turn,
    int32_t motor_b_pwm,
    int32_t motor_a_pwm,
    int32_t gyro_z_mdps,
    int32_t gyro_z_filtered_mdps,
    int32_t roll_cdeg,
    int32_t pitch_cdeg,
    uint32_t nav_frame_delta,
    uint8_t nav_update_flags)
{
    race_window_log_t *log;
    uint8_t line_lost = ((ir_ok == 0U) || (sample->line_lost != 0U)) ? 1U : 0U;

#if RACE_RAM_WINDOW_LOG_STRIDE > 1
    if ((g_race_window_log_decimator++ % RACE_RAM_WINDOW_LOG_STRIDE) != 0U) {
        return;
    }
#endif

    if (g_race_window_log_count >= RACE_RAM_WINDOW_CAPACITY) {
        g_race_window_log_overflow++;
        return;
    }

    log = &g_race_window_log[g_race_window_log_count++];
    log->t_ms = elapsed_ms;
    log->dist_count = race_sat_u16(phase_distance_count);
    log->yaw_cdeg = race_sat_i16(yaw_cdeg);
    log->yaw_raw_cdeg = race_sat_i16(yaw_raw_cdeg);
    log->phase_yaw_cdeg = race_sat_i16(phase_yaw_cdeg);
    log->yaw_progress_cdeg = race_sat_i16(yaw_progress_cdeg);
    log->expected_yaw_cdeg = race_sat_i16(expected_yaw_cdeg);
    log->heading_error_cdeg = race_sat_i16(heading_error_cdeg);
    log->line_error = race_sat_i16((ir_ok != 0U) ? sample->error : 0);
    log->nav_turn = race_sat_i16(nav_turn);
    log->control_turn = race_sat_i16(control_turn);
    log->motor_b_pwm = race_sat_i16(motor_b_pwm);
    log->motor_a_pwm = race_sat_i16(motor_a_pwm);
    log->gyro_z_x100_mdps = race_sat_i16(gyro_z_mdps / 100);
    log->gzlp_x100_mdps = race_sat_i16(gyro_z_filtered_mdps / 100);
    log->roll_cdeg = race_sat_i16(roll_cdeg);
    log->pitch_cdeg = race_sat_i16(pitch_cdeg);
    log->lap = lap;
    log->phase = phase;
    log->line_mask = (ir_ok != 0U) ? sample->line_mask : 0U;
    log->active_count = (ir_ok != 0U) ? sample->active_count : 0U;
    log->flags = (uint8_t)((ir_ok != 0U) ? RACE_LOG_FLAG_IR_OK : 0U);
    log->flags |= (uint8_t)((line_lost != 0U) ? RACE_LOG_FLAG_LINE_LOST : 0U);
    log->flags |= (uint8_t)((edge_seen != 0U) ? RACE_LOG_FLAG_EDGE_SEEN : 0U);
    log->flags |= (uint8_t)((nav_frame_delta != 0U) ? RACE_LOG_FLAG_NAV_FRAME : 0U);
    log->flags |= extra_flags;
    log->nav_update_flags = nav_update_flags;
    log->nav_frame_delta = (nav_frame_delta > 255U) ? 255U : (uint8_t)nav_frame_delta;
}

/**
 * @brief Append one race event row to RAM log storage.
 */
static void race_ram_log_event(uint8_t event,
    uint8_t reason,
    uint8_t lap,
    uint8_t phase,
    uint8_t extra_flags,
    uint32_t elapsed_ms,
    int32_t distance_count,
    int32_t phase_distance_count,
    int32_t yaw_cdeg,
    int32_t yaw_progress_cdeg,
    int32_t yaw_delta_cdeg,
    int32_t expected_yaw_cdeg,
    int32_t heading_error_cdeg,
    int32_t nav_turn,
    int32_t gyro_z_filtered_mdps,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    int32_t motor_b_total,
    int32_t motor_a_total)
{
    race_event_log_t *log;
    uint8_t line_lost = ((ir_ok == 0U) || ((sample != 0) && (sample->line_lost != 0U))) ? 1U : 0U;

    if ((lap >= RACE_RAM_LOG_MAX_LAPS) && (event != RACE_RAM_EVENT_COMPLETE)) {
        return;
    }
    if (g_race_event_log_count >= RACE_RAM_EVENT_CAPACITY) {
        g_race_event_log_overflow++;
        return;
    }

    log = &g_race_event_log[g_race_event_log_count++];
    log->t_ms = elapsed_ms;
    log->dist_count = race_sat_u16(distance_count);
    log->phase_dist_count = race_sat_u16(phase_distance_count);
    log->yaw_cdeg = race_sat_i16(yaw_cdeg);
    log->yaw_progress_cdeg = race_sat_i16(yaw_progress_cdeg);
    log->yaw_delta_cdeg = race_sat_i16(yaw_delta_cdeg);
    log->expected_yaw_cdeg = race_sat_i16(expected_yaw_cdeg);
    log->heading_error_cdeg = race_sat_i16(heading_error_cdeg);
    log->nav_turn = race_sat_i16(nav_turn);
    log->gzlp_x100_mdps = race_sat_i16(gyro_z_filtered_mdps / 100);
    log->line_error = race_sat_i16(((ir_ok != 0U) && (sample != 0)) ? sample->error : 0);
    log->motor_b_total = race_sat_i16(motor_b_total);
    log->motor_a_total = race_sat_i16(motor_a_total);
    log->lap = lap;
    log->phase = phase;
    log->event = event;
    log->reason = reason;
    log->raw = ((ir_ok != 0U) && (sample != 0)) ? sample->raw : 0xFFU;
    log->line_mask = ((ir_ok != 0U) && (sample != 0)) ? sample->line_mask : 0U;
    log->active_count = ((ir_ok != 0U) && (sample != 0)) ? sample->active_count : 0U;
    log->flags = (uint8_t)((ir_ok != 0U) ? RACE_LOG_FLAG_IR_OK : 0U);
    log->flags |= (uint8_t)((line_lost != 0U) ? RACE_LOG_FLAG_LINE_LOST : 0U);
    log->flags |= extra_flags;
}

/**
 * @brief Return expected yaw target at the start of a race phase.
 */
static int32_t race_phase_start_expected_yaw_cdeg(uint8_t phase,
    uint8_t target_laps)
{
    uint8_t task4_mode = (target_laps == TASK4_LAP_COUNT) ? 1U : 0U;

    if (phase == 0U) {
        return task4_mode ? RACE_TASK4_AC_HEADING_TARGET_CDEG :
            RACE_TASK3_AC_HEADING_TARGET_CDEG;
    }
    if (phase == 2U) {
        return task4_mode ? RACE_TASK4_BD_HEADING_TARGET_CDEG :
            RACE_TASK3_BD_HEADING_TARGET_CDEG;
    }
    return 0;
}

/**
 * @brief Record the opening snapshot for one race segment.
 */
static void race_log_segment_start_snapshot(uint8_t target_laps,
    uint8_t lap,
    uint8_t phase,
    uint32_t elapsed_ms,
    uint8_t nav_ok,
    int32_t yaw_cdeg,
    int32_t gyro_z_filtered_mdps)
{
    ir_tracking_sample_t sample = {0};
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t expected_yaw_cdeg =
        race_phase_start_expected_yaw_cdeg(phase, target_laps);
    int32_t heading_error_cdeg = 0;
    uint8_t ir_ok;
    uint8_t extra_flags = 0U;

    if ((phase == 1U) || (phase == 3U)) {
        extra_flags |= RACE_LOG_FLAG_ARC_MODE;
    }
    if (nav_ok != 0U) {
        heading_error_cdeg = ((phase == 0U) || (phase == 2U)) ?
            normalize_cdeg(yaw_cdeg - expected_yaw_cdeg) : 0;
    }

    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    ir_ok = IRTracking_ReadSample(&sample);
    race_ram_log_event(RACE_RAM_EVENT_SEGMENT_START,
        0U,
        lap,
        phase,
        extra_flags,
        elapsed_ms,
        0,
        0,
        (nav_ok != 0U) ? yaw_cdeg : 0,
        0,
        0,
        expected_yaw_cdeg,
        heading_error_cdeg,
        0,
        (nav_ok != 0U) ? gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        motor_b_total,
        motor_a_total);
}

/**
 * @brief Optional pacing delay between race dump lines.
 */
static void race_ram_dump_line_pause(void)
{
#if RACE_DUMP_LINE_DELAY_MS > 0
    delay_ms(RACE_DUMP_LINE_DELAY_MS);
#endif
}

/**
 * @brief Optional pacing delay between race dump sections.
 */
static void race_ram_dump_section_pause(void)
{
#if RACE_DUMP_SECTION_DELAY_MS > 0
    delay_ms(RACE_DUMP_SECTION_DELAY_MS);
#endif
}

/**
 * @brief Dump all race RAM logs over UART in sections.
 */
static void race_ram_log_dump(uint8_t target_laps)
{
    uint16_t i;
    uint32_t seq = 0U;
    uint16_t window_count = g_race_window_log_count;
    uint16_t event_count = g_race_event_log_count;
    uint16_t summary_count = g_race_summary_log_count;
    uint16_t window_overflow = g_race_window_log_overflow;
    uint16_t event_overflow = g_race_event_log_overflow;
    uint16_t summary_overflow = g_race_summary_log_overflow;
    uint8_t task4_mode = (target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
    int32_t ac_target_cdeg = task4_mode ?
        RACE_TASK4_AC_HEADING_TARGET_CDEG :
        RACE_TASK3_AC_HEADING_TARGET_CDEG;
    int32_t bd_target_cdeg = task4_mode ?
        RACE_TASK4_BD_HEADING_TARGET_CDEG :
        RACE_TASK3_BD_HEADING_TARGET_CDEG;
    int32_t line_base_pwm = task4_mode ?
        RACE_TASK4_STRAIGHT_BASE_PWM : RACE_STRAIGHT_BASE_PWM;
    int32_t arc_base_pwm = task4_mode ?
        RACE_TASK4_ARC_BASE_PWM : RACE_ARC_BASE_PWM;

    if (window_count > RACE_RAM_WINDOW_CAPACITY) {
        window_overflow++;
        window_count = RACE_RAM_WINDOW_CAPACITY;
    }
    if (event_count > RACE_RAM_EVENT_CAPACITY) {
        event_overflow++;
        event_count = RACE_RAM_EVENT_CAPACITY;
    }
    if (summary_count > RACE_RAM_SUMMARY_CAPACITY) {
        summary_overflow++;
        summary_count = RACE_RAM_SUMMARY_CAPACITY;
    }

    lc_printf("RACE_RAM_BEGIN seq=%lu win=%u/%u win_ov=%u ev=%u/%u ev_ov=%u sum=%u/%u sum_ov=%u max_laps=%u log_stride=%u\r\n",
        (unsigned long)seq++,
        window_count,
        RACE_RAM_WINDOW_CAPACITY,
        window_overflow,
        event_count,
        RACE_RAM_EVENT_CAPACITY,
        event_overflow,
        summary_count,
        RACE_RAM_SUMMARY_CAPACITY,
        summary_overflow,
        RACE_RAM_LOG_MAX_LAPS,
        (unsigned int)RACE_RAM_WINDOW_LOG_STRIDE);
    race_ram_dump_line_pause();
    lc_printf("RACE_CFG_MAIN seq=%lu laps=%u mode=%u line_base=%ld arc_base=%ld ac_tgt=%ld bd_tgt=%ld gyro_to=%d ff_gain=%d\r\n",
        (unsigned long)seq++,
        (unsigned int)target_laps,
        (unsigned int)task4_mode,
        (long)line_base_pwm,
        (long)arc_base_pwm,
        (long)ac_target_cdeg,
        (long)bd_target_cdeg,
        RACE_GYRO_TURN_TIMEOUT_MS,
        RACE_DIFF_FF_GAIN);
    race_ram_dump_line_pause();
    lc_printf("RACE_CFG_ST seq=%lu gyro_st=%u ir_assist=%u h_div=%d h_max=%d h_gd=%d win_pre=%d win_start=%d\r\n",
        (unsigned long)seq++,
        RACE_STRAIGHT_GYRO_NAV_ENABLE,
        RACE_STRAIGHT_IR_ASSIST_ENABLE,
        RACE_STRAIGHT_HEADING_CORR_DIVISOR,
        RACE_STRAIGHT_HEADING_CORR_MAX,
        RACE_STRAIGHT_GYRO_DAMP_DIVISOR,
        RACE_RAM_WINDOW_BEFORE_COUNT,
        RACE_RAM_WINDOW_AFTER_START_COUNT);
    race_ram_dump_line_pause();
    lc_printf("RACE_CFG_ARC seq=%lu arc_yaw=%u arc_div=%d arc_max=%d arc_gd=%d arc_yaw_arm=%d\r\n",
        (unsigned long)seq++,
        RACE_ARC_YAW_NAV_ENABLE,
        RACE_ARC_YAW_CORR_DIVISOR,
        RACE_ARC_YAW_CORR_MAX,
        RACE_ARC_GYRO_DAMP_DIVISOR,
        RACE_ARC_POINT_YAW_ARM_CDEG);
    race_ram_dump_line_pause();
    lc_printf("RACE_CFG_TURN seq=%lu turn_slow=%u slow_yaw=%d yaw_stop=%u yaw_tol=%d yaw_gz=%d b_exit=%ld a_exit=%ld\r\n",
        (unsigned long)seq++,
        RACE_FAST_TURN_GYRO_SLOW_ENABLE,
        RACE_FAST_TURN_GYRO_SLOW_CDEG,
        RACE_EXIT_TURN_YAW_STOP_ENABLE,
        RACE_TURN_YAW_STOP_TOL_CDEG,
        RACE_TURN_YAW_STOP_GZLP_TOL_MDPS,
        (long)bd_target_cdeg,
        (long)ac_target_cdeg);
    race_ram_dump_line_pause();
    lc_printf("RACE_CFG_DUMP seq=%lu sum=%u evt=%u win=%u win_stride=%u line_delay=%u section_delay=%u\r\n",
        (unsigned long)seq++,
        (unsigned int)RACE_DUMP_SUM_ENABLE,
        (unsigned int)RACE_DUMP_EVT_ENABLE,
        (unsigned int)RACE_DUMP_WIN_ENABLE,
        (unsigned int)RACE_DUMP_WIN_STRIDE,
        (unsigned int)RACE_DUMP_LINE_DELAY_MS,
        (unsigned int)RACE_DUMP_SECTION_DELAY_MS);
    race_ram_dump_line_pause();

    lc_printf("RACE_DUMP_MARK seq=%lu step=sum_begin\r\n",
        (unsigned long)seq++);
    race_ram_dump_line_pause();
#if RACE_DUMP_SUM_ENABLE
    lc_printf("RACE_DUMP_SECTION seq=%lu name=SUM count=%u\r\n",
        (unsigned long)seq++,
        (unsigned int)summary_count);
    race_ram_dump_line_pause();
    for (i = 0U; i < summary_count; i++) {
        const race_summary_log_t *log = &g_race_summary_log[i];
        lc_printf("RACE_SEG seq=%lu idx=%u lap=%u ph=%u rsn=%u t0=%lu t1=%lu dist=%u n=%u\r\n",
            (unsigned long)seq++,
            (unsigned int)i,
            (unsigned int)log->lap,
            (unsigned int)log->phase,
            (unsigned int)log->reason,
            (unsigned long)log->start_ms,
            (unsigned long)log->end_ms,
            (unsigned int)log->dist_count,
            (unsigned int)log->sample_count);
        race_ram_dump_line_pause();
        lc_printf("RACE_SEG_LINE seq=%lu idx=%u seen=%u first=%u last=%u span=%u gap=%u lost=%u maxlost=%u endlost=%u pm=0x%02X pf=0x%02X\r\n",
            (unsigned long)seq++,
            (unsigned int)i,
            (unsigned int)log->line_seen_count,
            (unsigned int)log->first_line_dist_count,
            (unsigned int)log->last_line_dist_count,
            (unsigned int)log->line_span_count,
            (unsigned int)log->end_gap_count,
            (unsigned int)log->lost_count,
            (unsigned int)log->max_lost_streak_count,
            (unsigned int)log->end_lost_streak_count,
            (unsigned int)log->point_mask,
            (unsigned int)log->point_flags);
        race_ram_dump_line_pause();
        lc_printf("RACE_SEG_YAW seq=%lu idx=%u yaw0=%d yaw1=%d ypr=%d herr=%d avgh=%d maxh=%u turn=%d/%d/%d\r\n",
            (unsigned long)seq++,
            (unsigned int)i,
            (int)log->yaw_start_cdeg,
            (int)log->yaw_end_cdeg,
            (int)log->yaw_progress_cdeg,
            (int)log->end_heading_error_cdeg,
            (int)log->avg_abs_heading_error,
            (unsigned int)log->max_abs_heading_error,
            (int)log->avg_line_turn,
            (int)log->avg_nav_turn,
            (int)log->avg_turn);
        race_ram_dump_line_pause();
        lc_printf("RACE_SEG_NAV seq=%lu idx=%u nav=%u lost=%u fd=%u stale=%u upd=0x%02X gz=%d/%u gzl=%d/%u err=%d/%u\r\n",
            (unsigned long)seq++,
            (unsigned int)i,
            (unsigned int)log->nav_sample_count,
            (unsigned int)log->nav_lost_count,
            (unsigned int)log->nav_frame_count,
            (unsigned int)log->nav_stale_count,
            (unsigned int)log->nav_update_flags,
            (int)log->avg_gyro_z_x100_mdps,
            (unsigned int)log->max_abs_gyro_z_x100_mdps,
            (int)log->avg_gzlp_x100_mdps,
            (unsigned int)log->max_abs_gzlp_x100_mdps,
            (int)log->avg_abs_error,
            (unsigned int)log->max_abs_error);
        race_ram_dump_line_pause();
    }
    lc_printf("RACE_DUMP_SECTION_END seq=%lu name=SUM\r\n",
        (unsigned long)seq++);
    race_ram_dump_section_pause();
#endif

    lc_printf("RACE_DUMP_MARK seq=%lu step=evt_begin\r\n",
        (unsigned long)seq++);
    race_ram_dump_line_pause();
#if RACE_DUMP_EVT_ENABLE
    lc_printf("RACE_DUMP_SECTION seq=%lu name=EVT count=%u\r\n",
        (unsigned long)seq++,
        (unsigned int)event_count);
    race_ram_dump_line_pause();
    for (i = 0U; i < event_count; i++) {
        const race_event_log_t *log = &g_race_event_log[i];
        lc_printf("RACE_EVT seq=%lu idx=%u lap=%u ph=%u ev=%u rsn=%u t=%lu dist=%u pd=%u raw=0x%02X mask=0x%02X cnt=%u fl=0x%02X\r\n",
            (unsigned long)seq++,
            (unsigned int)i,
            (unsigned int)log->lap,
            (unsigned int)log->phase,
            (unsigned int)log->event,
            (unsigned int)log->reason,
            (unsigned long)log->t_ms,
            (unsigned int)log->dist_count,
            (unsigned int)log->phase_dist_count,
            (unsigned int)log->raw,
            (unsigned int)log->line_mask,
            (unsigned int)log->active_count,
            (unsigned int)log->flags);
        race_ram_dump_line_pause();
        lc_printf("RACE_EVT_YAW seq=%lu idx=%u yaw=%d ypr=%d yd=%d exp=%d herr=%d nav=%d gz100=%d err=%d B=%d A=%d\r\n",
            (unsigned long)seq++,
            (unsigned int)i,
            (int)log->yaw_cdeg,
            (int)log->yaw_progress_cdeg,
            (int)log->yaw_delta_cdeg,
            (int)log->expected_yaw_cdeg,
            (int)log->heading_error_cdeg,
            (int)log->nav_turn,
            (int)log->gzlp_x100_mdps,
            (int)log->line_error,
            (int)log->motor_b_total,
            (int)log->motor_a_total);
        race_ram_dump_line_pause();
    }
    lc_printf("RACE_DUMP_SECTION_END seq=%lu name=EVT\r\n",
        (unsigned long)seq++);
    race_ram_dump_section_pause();
#endif

    lc_printf("RACE_DUMP_MARK seq=%lu step=win_begin\r\n",
        (unsigned long)seq++);
    race_ram_dump_line_pause();
#if RACE_DUMP_WIN_ENABLE
    {
#if RACE_DUMP_WIN_STRIDE <= 1
        uint16_t window_stride = 1U;
#else
        uint16_t window_stride = (uint16_t)RACE_DUMP_WIN_STRIDE;
#endif
        uint16_t window_dump_count =
            (uint16_t)((window_count + window_stride - 1U) / window_stride);

        lc_printf("RACE_DUMP_SECTION seq=%lu name=WIN count=%u kept=%u stride=%u\r\n",
            (unsigned long)seq++,
            (unsigned int)window_dump_count,
            (unsigned int)window_count,
            (unsigned int)window_stride);
        race_ram_dump_line_pause();
        for (i = 0U; i < window_count; i++) {
            const race_window_log_t *log;

            if ((i % window_stride) != 0U) {
                continue;
            }

            log = &g_race_window_log[i];
            lc_printf("RACE_WIN seq=%lu idx=%u lap=%u ph=%u t=%lu dist=%u yaw=%d exp=%d herr=%d err=%d mask=0x%02X cnt=%u fl=0x%02X pwm=%d/%d\r\n",
                (unsigned long)seq++,
                (unsigned int)i,
                (unsigned int)log->lap,
                (unsigned int)log->phase,
                (unsigned long)log->t_ms,
                (unsigned int)log->dist_count,
                (int)log->yaw_cdeg,
                (int)log->expected_yaw_cdeg,
                (int)log->heading_error_cdeg,
                (int)log->line_error,
                (unsigned int)log->line_mask,
                (unsigned int)log->active_count,
                (unsigned int)log->flags,
                (int)log->motor_b_pwm,
                (int)log->motor_a_pwm);
            race_ram_dump_line_pause();
            lc_printf("RACE_WIN_NAV seq=%lu idx=%u rawy=%d py=%d ypr=%d nav=%d turn=%d gz=%d gzl=%d fd=%u upd=0x%02X\r\n",
                (unsigned long)seq++,
                (unsigned int)i,
                (int)log->yaw_raw_cdeg,
                (int)log->phase_yaw_cdeg,
                (int)log->yaw_progress_cdeg,
                (int)log->nav_turn,
                (int)log->control_turn,
                (int)log->gyro_z_x100_mdps,
                (int)log->gzlp_x100_mdps,
                (unsigned int)log->nav_frame_delta,
                (unsigned int)log->nav_update_flags);
            race_ram_dump_line_pause();
        }
    }
    lc_printf("RACE_DUMP_SECTION_END seq=%lu name=WIN\r\n",
        (unsigned long)seq++);
    race_ram_dump_section_pause();
#endif

    lc_printf("RACE_RAM_END seq=%lu\r\n", (unsigned long)seq++);
}
#else
#define race_post_point_context_begin(point_ms, phase_distance_count) ((void)0)
#define race_post_point_event_ms(local_ms) (local_ms)
#define race_ram_log_reset() ((void)0)
#define race_ram_log_set_context(lap, phase) ((void)0)
#define race_ram_log_segment_reset(lap, phase, start_ms, yaw_start_cdeg) ((void)0)
#define race_ram_log_segment_sample(ir_ok, nav_ok, sample, phase_distance_count, line_turn, nav_turn, control_turn, heading_error_cdeg, gyro_z_mdps, gyro_z_filtered_mdps, nav_frame_delta, nav_update_flags) ((void)0)
#define race_ram_log_segment_finish(reason, end_ms, dist_count, yaw_end_cdeg, yaw_progress_cdeg, heading_error_cdeg, ir_ok, sample, point_flags) ((void)0)
#define race_ram_window_should_log(lap, phase_distance_count, point_arm_count) (0U)
#define race_ram_log_window_sample(lap, phase, ir_ok, sample, edge_seen, extra_flags, elapsed_ms, phase_distance_count, yaw_cdeg, yaw_raw_cdeg, phase_yaw_cdeg, yaw_progress_cdeg, expected_yaw_cdeg, heading_error_cdeg, nav_turn, control_turn, motor_b_pwm, motor_a_pwm, gyro_z_mdps, gyro_z_filtered_mdps, roll_cdeg, pitch_cdeg, nav_frame_delta, nav_update_flags) ((void)0)
#define race_ram_log_event(event, reason, lap, phase, extra_flags, elapsed_ms, distance_count, phase_distance_count, yaw_cdeg, yaw_progress_cdeg, yaw_delta_cdeg, expected_yaw_cdeg, heading_error_cdeg, nav_turn, gyro_z_filtered_mdps, ir_ok, sample, motor_b_total, motor_a_total) ((void)0)
#define race_log_segment_start_snapshot(target_laps, lap, phase, elapsed_ms, nav_ok, yaw_cdeg, gyro_z_filtered_mdps) ((void)0)
#define race_ram_log_dump(target_laps) ((void)0)
#endif

#if TASK5_RAM_LOG_ENABLE && !RACE_RAM_LOG_ENABLE
static task5_ram_log_t g_task5_ram_log_storage[TASK5_RAM_LOG_CAPACITY];
task5_ram_log_t * const g_task5_ram_log = g_task5_ram_log_storage;
uint16_t g_task5_ram_log_count;
uint16_t g_task5_ram_log_overflow;
#endif

#endif /* RACE_LOG_H */
