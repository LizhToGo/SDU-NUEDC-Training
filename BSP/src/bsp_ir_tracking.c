#include "bsp_ir_tracking.h"

/* 轮询等待 I2C 状态变化时使用的软件超时，避免接线错误时程序永久卡死。 */
#define IR_TRACKING_I2C_TIMEOUT (100000UL)

/* 最近一次有效线位置。丢线时继续返回它，方便后续按最后方向低速找线。 */
static int16_t g_ir_tracking_last_error;

/* 等待 I2C 控制器进入空闲状态。 */
static uint8_t IRTracking_WaitIdle(void)
{
    uint32_t timeout = IR_TRACKING_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout == 0U) {
            return 0U;
        }
        timeout--;
    }

    return 1U;
}

/* 等待 I2C 总线释放。SDA/SCL 被外部拉住或模块异常时可能超时。 */
static uint8_t IRTracking_WaitBusFree(void)
{
    uint32_t timeout = IR_TRACKING_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (timeout == 0U) {
            return 0U;
        }
        timeout--;
    }

    return 1U;
}

/* 检查 I2C 控制器是否报告错误，例如无应答或总线错误。 */
static uint8_t IRTracking_HasBusError(void)
{
    return ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) ? 1U : 0U;
}

/*
 * 根据黑线掩码计算线位置。
 *
 * X1 到 X8 从左到右排列，权重从 -3500 到 +3500。
 * 多个探头同时压线时取平均值，所以中心两个探头 X4/X5 同时压线时结果为 0。
 */
static int16_t IRTracking_CalculatePosition(uint8_t line_mask, uint8_t *active_count)
{
    static const int16_t weights[IR_TRACKING_SENSOR_COUNT] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    int32_t weighted_sum = 0;
    uint8_t count = 0U;
    uint8_t index;

    for (index = 0U; index < IR_TRACKING_SENSOR_COUNT; index++) {
        if ((line_mask & (1U << index)) != 0U) {
            weighted_sum += weights[index];
            count++;
        }
    }

    *active_count = count;

    if (count == 0U) {
        /* 没有任何探头看到黑线时，保留最后一次有效方向，后续找线会更自然。 */
        return g_ir_tracking_last_error;
    }

    return (int16_t)(weighted_sum / (int32_t)count);
}

void IRTracking_Init(void)
{
    /* 上电初始默认认为线在中心。 */
    g_ir_tracking_last_error = 0;
}

uint8_t IRTracking_ReadRegister(uint8_t reg, uint8_t *value)
{
    if (value == 0) {
        return 0U;
    }

    if (IRTracking_WaitIdle() == 0U) {
        return 0U;
    }

    /*
     * 读取流程：
     * 1. 先发送寄存器地址 reg；
     * 2. 等待写阶段完成；
     * 3. 再启动 1 字节读传输；
     * 4. 从 RX FIFO 中取回寄存器值。
     */
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(
        I2C_0_INST, IR_TRACKING_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    if ((IRTracking_WaitBusFree() == 0U) || (IRTracking_WaitIdle() == 0U) ||
        (IRTracking_HasBusError() != 0U)) {
        return 0U;
    }

    DL_I2C_startControllerTransfer(
        I2C_0_INST, IR_TRACKING_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1U);

    if ((IRTracking_WaitBusFree() == 0U) || (IRTracking_WaitIdle() == 0U) ||
        (IRTracking_HasBusError() != 0U)) {
        return 0U;
    }

    *value = DL_I2C_receiveControllerData(I2C_0_INST);
    return 1U;
}

uint8_t IRTracking_ReadRaw(uint8_t *raw)
{
    return IRTracking_ReadRegister(IR_TRACKING_DATA_REG, raw);
}

uint8_t IRTracking_RawToLineMask(uint8_t raw)
{
    uint8_t line_mask = 0U;
    uint8_t index;

    for (index = 0U; index < IR_TRACKING_SENSOR_COUNT; index++) {
        /*
         * raw 的 bit7 是 X1，bit0 是 X8；
         * line_mask 反过来用 bit0 表示 X1，bit7 表示 X8。
         */
        uint8_t raw_bit = (uint8_t)((raw >> (7U - index)) & 0x01U);

        /* 当前模块实测为黑线输出 0，所以 raw_bit 为 0 时置位 line_mask。 */
        if (raw_bit == 0U) {
            line_mask |= (uint8_t)(1U << index);
        }
    }

    return line_mask;
}

uint8_t IRTracking_ReadSample(ir_tracking_sample_t *sample)
{
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    int16_t position;

    if (sample == 0) {
        return 0U;
    }

    if (IRTracking_ReadRaw(&raw) == 0U) {
        return 0U;
    }

    line_mask = IRTracking_RawToLineMask(raw);
    position = IRTracking_CalculatePosition(line_mask, &active_count);

    /* 将一次采样的所有中间结果保存到结构体，方便主程序打印或控制。 */
    sample->raw = raw;
    sample->line_mask = line_mask;
    sample->active_count = active_count;
    sample->line_lost = (active_count == 0U) ? 1U : 0U;
    sample->position = position;
    sample->error = position;

    if (active_count != 0U) {
        /* 只有看到黑线时才更新最近有效误差，避免丢线时把方向清零。 */
        g_ir_tracking_last_error = position;
    }

    return 1U;
}

int16_t IRTracking_GetLastError(void)
{
    return g_ir_tracking_last_error;
}

void IRTracking_PrintSample(const ir_tracking_sample_t *sample)
{
    if (sample == 0) {
        return;
    }

    lc_printf("IR raw=0x%02X mask=0x%02X count=%u lost=%u pos=%d err=%d\r\n",
        sample->raw,
        sample->line_mask,
        sample->active_count,
        sample->line_lost,
        sample->position,
        sample->error);
}
