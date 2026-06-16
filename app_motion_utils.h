#ifndef APP_MOTION_UTILS_H
#define APP_MOTION_UTILS_H

#include <stdint.h>
#include "app_control.h"

/**
 * @brief Normalize an angle in centi-degrees into [-18000, 18000].
 */
static inline int32_t normalize_cdeg(int32_t angle_cdeg)
{
    while (angle_cdeg > 18000L) {
        angle_cdeg -= 36000L;
    }

    while (angle_cdeg < -18000L) {
        angle_cdeg += 36000L;
    }

    return angle_cdeg;
}

/**
 * @brief Average absolute left/right encoder totals into forward distance.
 */
static inline int32_t motion_distance_count(int32_t motor_b_total,
    int32_t motor_a_total)
{
    return (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
}

/**
 * @brief Return signed B-A encoder distance error for straight balancing.
 */
static inline int32_t motion_distance_error(int32_t motor_b_total,
    int32_t motor_a_total)
{
    return motor_b_total - motor_a_total;
}

#endif /* APP_MOTION_UTILS_H */
