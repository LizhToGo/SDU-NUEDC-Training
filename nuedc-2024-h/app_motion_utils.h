#ifndef APP_MOTION_UTILS_H
#define APP_MOTION_UTILS_H

#include <stdint.h>
#include "app_control.h"

/**
 * @brief 将百分之一度角度归一化到 [-18000, 18000]。
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
 * @brief 用左右轮编码器绝对值平均得到前进距离。
 */
static inline int32_t motion_distance_count(int32_t motor_b_total,
    int32_t motor_a_total)
{
    return (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
}

/**
 * @brief 返回 B-A 有符号编码器距离误差，用于直线平衡。
 */
static inline int32_t motion_distance_error(int32_t motor_b_total,
    int32_t motor_a_total)
{
    return motor_b_total - motor_a_total;
}

#endif /* APP_MOTION_UTILS_H */
