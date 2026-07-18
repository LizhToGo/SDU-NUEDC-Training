#ifndef _BSP_IR_TRACKING_H
#define _BSP_IR_TRACKING_H

#include <stdint.h>
#include "board.h"

/*
 * 八路红外循迹模块驱动。
 *
 * 当前只封装 I2C 读取方式：
 * - 设备地址：0x12
 * - 数字量寄存器：0x30
 * - 原始字节：bit7 对应 X1，bit0 对应 X8
 * - 实测逻辑：黑线为 0，白底为 1
 *
 * 为了让后续循迹控制更好写，驱动会把原始字节转换为 line_mask：
 * - bit0 对应 X1，bit7 对应 X8
 * - 1 表示该路检测到黑线
 */
#define IR_TRACKING_I2C_ADDR       (0x12U)
#define IR_TRACKING_DATA_REG       (0x30U)
#define IR_TRACKING_SENSOR_COUNT   (8U)
#define IR_TRACKING_POSITION_SCALE (1000)

/**
 * @brief 一次解析后的 8 路红外循迹采样。
 */
typedef struct {
    uint8_t raw;          /* 模块原始字节：bit7=X1，bit0=X8。 */
    uint8_t line_mask;    /* 归一化黑线掩码：bit0=X1，bit7=X8，1 表示该路检测到黑线。 */
    uint8_t active_count; /* 当前检测到黑线的探头数量。 */
    uint8_t line_lost;    /* 1 表示 8 路都没有检测到黑线。 */
    int16_t position;     /* 加权线位置，中心为 0，左负右正。 */
    int16_t error;        /* 给循迹控制器使用的误差，当前等于 position。 */
} ir_tracking_sample_t;

/* 初始化驱动内部状态。I2C 外设本身由 SYSCFG_DL_init() 初始化。 */
/** 复位驱动内部状态；I2C 外设初始化由 SysConfig 负责。 */
void IRTracking_Init(void);

/* 从模块读取任意寄存器，成功返回 1，失败返回 0。 */
/** 从红外模块读取一个寄存器。 */
uint8_t IRTracking_ReadRegister(uint8_t reg, uint8_t *value);

/* 读取 0x30 寄存器中的原始 8 路数字状态。 */
/** 读取原始 8 路数字状态寄存器。 */
uint8_t IRTracking_ReadRaw(uint8_t *raw);

/* 读取并解析一次完整采样，推荐主程序优先使用这个函数。 */
/** 读取并解析一次完整红外循迹采样。 */
uint8_t IRTracking_ReadSample(ir_tracking_sample_t *sample);

/* 将模块原始字节转换成黑线掩码。 */
/** 将模块原始字节转换为归一化黑线掩码。 */
uint8_t IRTracking_RawToLineMask(uint8_t raw);

/* 读取最近一次有效的线位置误差；丢线时会保留这个方向。 */
/** 返回最近一次有效线误差；丢线时仍保留该值。 */
int16_t IRTracking_GetLastError(void);

/* 通过 UART0 打印采样结果，主要用于调试阶段。 */
/** 通过 UART0 打印一次解析后的采样，便于调试。 */
void IRTracking_PrintSample(const ir_tracking_sample_t *sample);

#endif /* 结束 _BSP_IR_TRACKING_H 头文件保护 */
