#ifndef _BSP_JY62_H
#define _BSP_JY62_H

#include <stdbool.h>
#include <stdint.h>
#include "board.h"

/*
 * JY62 串口驱动。
 *
 * 接线约定：
 * - JY62 RXD 接 MSPM0 PA08 / UART1_TX
 * - JY62 TXD 接 MSPM0 PA09 / UART1_RX
 * - 当前先按 115200 测试，8 数据位，无校验，1 停止位
 *
 * 协议来自商家 WIT/JY 系列例程：
 * - 帧头固定为 0x55
 * - 0x51 表示加速度帧
 * - 0x52 表示角速度帧
 * - 0x53 表示角度帧
 * - 每帧 11 字节，最后 1 字节为前 10 字节累加和低 8 位
 *
 * 为了方便当前工程直接编译，本驱动采用头文件内 static 实现。
 * 后续如果 CCS 工程文件重新生成并能自动纳入新增 .c 文件，可以再拆成 bsp_jy62.c。
 */

#define JY62_UART_BAUD_RATE      (115200U)
#define JY62_FRAME_HEADER        (0x55U)
#define JY62_FRAME_ACC           (0x51U)
#define JY62_FRAME_GYRO          (0x52U)
#define JY62_FRAME_ANGLE         (0x53U)
#define JY62_FRAME_LEN           (11U)
#define JY62_FLAG_ACC            (0x01U)
#define JY62_FLAG_GYRO           (0x02U)
#define JY62_FLAG_ANGLE          (0x04U)
#define JY62_NAV_REQUIRED_FLAGS  (JY62_FLAG_GYRO | JY62_FLAG_ANGLE)
#define JY62_RAW_DUMP_LEN        (24U)
#define JY62_GZ_FILTER_NUM       (3)
#define JY62_GZ_FILTER_DEN       (4)
#define JY62_UART_ERROR_INTERRUPTS \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR | \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | \
     DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR | \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)
#define JY62_UART_RX_INTERRUPTS \
    (DL_UART_MAIN_INTERRUPT_RX | JY62_UART_ERROR_INTERRUPTS)

/**
 * @brief Raw decoded JY62 frames plus parser/statistics counters.
 */
typedef struct {
    int16_t acc_raw[3];       /* 原始加速度：换算为 g 时 raw / 32768 * 16。 */
    int16_t gyro_raw[3];      /* 原始角速度：换算为 deg/s 时 raw / 32768 * 2000。 */
    int16_t angle_raw[3];     /* 原始角度：换算为 deg 时 raw / 32768 * 180。 */
    int16_t aux_raw;          /* 每帧第 4 个 int16：0x51 为温度，0x52 为电压，0x53 为版本/无效值。 */
    uint8_t update_flags;     /* 本次读到的新数据类型，见 JY62_FLAG_*。 */
    uint8_t received_flags;   /* 上电以来已经收到过的数据类型，不会被 JY62_Poll 清零。 */
    uint8_t last_frame_type;  /* 最近一次校验通过的帧类型。 */
    uint8_t raw_count;        /* recent_raw 中当前有效的字节数量。 */
    uint8_t raw_write_index;  /* recent_raw 的环形写入位置。 */
    uint8_t recent_raw[JY62_RAW_DUMP_LEN]; /* 最近收到的原始字节，用于排查波特率和协议。 */
    uint32_t header_count;    /* 收到 0x55 帧头的累计数量。 */
    uint32_t frame_count;     /* 校验通过且属于 0x51/0x52/0x53 的姿态帧累计数量。 */
    uint32_t unknown_frame_count; /* 校验通过但不是姿态帧的累计数量。 */
    uint32_t checksum_error;  /* 校验失败累计数量。 */
    uint32_t rx_byte_count;   /* UART1 收到的字节累计数量。 */
    uint32_t rx_irq_count;    /* UART1 接收中断累计次数，用于确认中断接收是否工作。 */
    uint32_t uart_error_count; /* UART1 硬件错误累计次数，常见原因是线序、波特率或 FIFO 溢出。 */
    uint32_t overrun_count;   /* UART1 FIFO 溢出累计次数，非 0 说明主循环轮询接收来不及。 */
} jy62_sample_t;

/**
 * @brief Navigation-friendly JY62 output used by car control code.
 */
typedef struct {
    int32_t yaw_cdeg;             /* 原始偏航角，单位 0.01 度，范围约为 -18000 到 +18000。 */
    int32_t yaw_relative_cdeg;    /* 以上电后第一次有效 yaw 为零点的相对偏航角，单位 0.01 度。 */
    int32_t yaw_zero_cdeg;        /* 软件零点，单位 0.01 度。 */
    int32_t gyro_z_mdps;          /* Z 轴角速度，单位 0.001 度/秒。 */
    int32_t gyro_z_filtered_mdps; /* 低通后的 Z 轴角速度，后续可作为转向阻尼项。 */
    int32_t roll_cdeg;            /* 滚转角，仅用于观察模块安装状态。 */
    int32_t pitch_cdeg;           /* 俯仰角，仅用于观察模块安装状态。 */
    uint8_t valid;                /* 已收到角速度帧和角度帧时为 1。 */
    uint8_t update_flags;         /* 本次读取周期内更新过的数据类型。 */
    uint32_t rx_byte_count;
    uint32_t header_count;
    uint32_t frame_count;
    uint32_t checksum_error;
    uint32_t uart_error_count;
    uint32_t overrun_count;
} jy62_navigation_t;

static uint8_t g_jy62_frame[JY62_FRAME_LEN];
static uint8_t g_jy62_frame_index;
static uint32_t g_jy62_last_poll_frame_count;
static int32_t g_jy62_yaw_zero_cdeg;
static int32_t g_jy62_gyro_z_filtered_raw;
static uint8_t g_jy62_yaw_zero_valid;
static uint8_t g_jy62_gyro_z_filter_valid;
static jy62_sample_t g_jy62_sample;

/**
 * @brief Convert little-endian frame bytes into signed int16.
 */
static int16_t JY62_MakeI16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/**
 * @brief Absolute value helper for JY62 calculations.
 */
static int32_t JY62_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/**
 * @brief Convert raw acceleration to milli-g.
 */
static int32_t JY62_RawToAccMg(int16_t raw)
{
    return (int32_t)(((int64_t)raw * 16000LL) / 32768LL);
}

/**
 * @brief Convert raw gyro reading to milli-degrees per second.
 */
static int32_t JY62_RawToGyroMdps(int16_t raw)
{
    return (int32_t)(((int64_t)raw * 2000000LL) / 32768LL);
}

/**
 * @brief Convert raw angle reading to centi-degrees.
 */
static int32_t JY62_RawToAngleCdeg(int16_t raw)
{
    return (int32_t)(((int64_t)raw * 18000LL) / 32768LL);
}

/**
 * @brief Normalize a centi-degree angle into +/-180 degrees.
 */
static int32_t JY62_NormalizeAngleCdeg(int32_t angle_cdeg)
{
    if (angle_cdeg > 18000L) {
        angle_cdeg -= 36000L;
    } else if (angle_cdeg < -18000L) {
        angle_cdeg += 36000L;
    }

    return angle_cdeg;
}

/**
 * @brief Print a signed fixed-point value without using floating point.
 */
static void JY62_PrintSignedFixed(int32_t value, uint16_t scale, uint8_t digits)
{
    int32_t abs_value = JY62_Abs32(value);
    int32_t integer = abs_value / scale;
    int32_t decimal = abs_value % scale;

    if (value < 0) {
        lc_printf("-");
    }

    if (digits == 3U) {
        lc_printf("%ld.%03ld", integer, decimal);
    } else {
        lc_printf("%ld.%02ld", integer, decimal);
    }
}

/**
 * @brief Verify the JY62 10-byte sum checksum.
 */
static uint8_t JY62_ChecksumOk(const uint8_t *frame)
{
    uint8_t sum = 0U;
    uint8_t index;

    for (index = 0U; index < (JY62_FRAME_LEN - 1U); index++) {
        sum = (uint8_t)(sum + frame[index]);
    }

    return (sum == frame[JY62_FRAME_LEN - 1U]) ? 1U : 0U;
}

/**
 * @brief Recover parser alignment after a bad or partial frame.
 */
static void JY62_ResyncFrame(void)
{
    uint8_t next_header_index;
    uint8_t index;

    for (next_header_index = 1U; next_header_index < JY62_FRAME_LEN; next_header_index++) {
        if (g_jy62_frame[next_header_index] == JY62_FRAME_HEADER) {
            break;
        }
    }

    if (next_header_index >= JY62_FRAME_LEN) {
        g_jy62_frame_index = 0U;
        return;
    }

    g_jy62_frame_index = (uint8_t)(JY62_FRAME_LEN - next_header_index);
    for (index = 0U; index < g_jy62_frame_index; index++) {
        g_jy62_frame[index] = g_jy62_frame[next_header_index + index];
    }
}

/**
 * @brief Store recent raw bytes for UART/protocol diagnostics.
 */
static void JY62_SaveRecentByte(uint8_t data)
{
    g_jy62_sample.recent_raw[g_jy62_sample.raw_write_index] = data;
    g_jy62_sample.raw_write_index++;
    if (g_jy62_sample.raw_write_index >= JY62_RAW_DUMP_LEN) {
        g_jy62_sample.raw_write_index = 0U;
    }

    if (g_jy62_sample.raw_count < JY62_RAW_DUMP_LEN) {
        g_jy62_sample.raw_count++;
    }
}

/**
 * @brief Decode one checksum-valid JY62 frame into the sample cache.
 */
static void JY62_ParseFrame(const uint8_t *frame)
{
    int16_t value0 = JY62_MakeI16(frame[2], frame[3]);
    int16_t value1 = JY62_MakeI16(frame[4], frame[5]);
    int16_t value2 = JY62_MakeI16(frame[6], frame[7]);
    int16_t value3 = JY62_MakeI16(frame[8], frame[9]);

    g_jy62_sample.last_frame_type = frame[1];

    switch (frame[1]) {
    case JY62_FRAME_ACC:
        g_jy62_sample.acc_raw[0] = value0;
        g_jy62_sample.acc_raw[1] = value1;
        g_jy62_sample.acc_raw[2] = value2;
        g_jy62_sample.aux_raw = value3;
        g_jy62_sample.update_flags |= JY62_FLAG_ACC;
        g_jy62_sample.received_flags |= JY62_FLAG_ACC;
        g_jy62_sample.frame_count++;
        break;

    case JY62_FRAME_GYRO:
        g_jy62_sample.gyro_raw[0] = value0;
        g_jy62_sample.gyro_raw[1] = value1;
        g_jy62_sample.gyro_raw[2] = value2;
        g_jy62_sample.aux_raw = value3;
        if (g_jy62_gyro_z_filter_valid == 0U) {
            g_jy62_gyro_z_filtered_raw = value2;
            g_jy62_gyro_z_filter_valid = 1U;
        } else {
            g_jy62_gyro_z_filtered_raw =
                ((g_jy62_gyro_z_filtered_raw * JY62_GZ_FILTER_NUM) +
                ((int32_t)value2 * (JY62_GZ_FILTER_DEN - JY62_GZ_FILTER_NUM))) /
                JY62_GZ_FILTER_DEN;
        }
        g_jy62_sample.update_flags |= JY62_FLAG_GYRO;
        g_jy62_sample.received_flags |= JY62_FLAG_GYRO;
        g_jy62_sample.frame_count++;
        break;

    case JY62_FRAME_ANGLE:
        g_jy62_sample.angle_raw[0] = value0;
        g_jy62_sample.angle_raw[1] = value1;
        g_jy62_sample.angle_raw[2] = value2;
        g_jy62_sample.aux_raw = value3;
        if (g_jy62_yaw_zero_valid == 0U) {
            g_jy62_yaw_zero_cdeg = JY62_RawToAngleCdeg(value2);
            g_jy62_yaw_zero_valid = 1U;
        }
        g_jy62_sample.update_flags |= JY62_FLAG_ANGLE;
        g_jy62_sample.received_flags |= JY62_FLAG_ANGLE;
        g_jy62_sample.frame_count++;
        break;

    default:
        g_jy62_sample.unknown_frame_count++;
        break;
    }
}

/**
 * @brief Feed one UART byte into the JY62 frame parser.
 */
static void JY62_PushByte(uint8_t data)
{
    g_jy62_sample.rx_byte_count++;
    JY62_SaveRecentByte(data);

    if (data == JY62_FRAME_HEADER) {
        g_jy62_sample.header_count++;
    }

    if (g_jy62_frame_index == 0U) {
        if (data != JY62_FRAME_HEADER) {
            return;
        }
        g_jy62_frame[g_jy62_frame_index++] = data;
        return;
    }

    g_jy62_frame[g_jy62_frame_index++] = data;

    if (g_jy62_frame_index >= JY62_FRAME_LEN) {
        if (JY62_ChecksumOk(g_jy62_frame) != 0U) {
            JY62_ParseFrame(g_jy62_frame);
            g_jy62_frame_index = 0U;
        } else {
            g_jy62_sample.checksum_error++;
            JY62_ResyncFrame();
        }
    }
}

/**
 * @brief Drain UART1 RX FIFO and push every byte into the parser.
 */
static void JY62_DrainRxFifo(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
        JY62_PushByte(DL_UART_Main_receiveData(UART_1_INST));
    }
}

/**
 * @brief Record UART1 hardware error counters.
 */
static void JY62_RecordUartError(uint8_t is_overrun)
{
    g_jy62_sample.uart_error_count++;
    if (is_overrun != 0U) {
        g_jy62_sample.overrun_count++;
    }
}

/**
 * @brief Enable UART1 RX and error interrupts for JY62.
 */
static void JY62_EnableRxInterrupt(void)
{
    /*
     * JY62 在 115200 下会连续输出二进制帧。
     * 如果只靠主循环轮询，UART0 打印较长日志时容易让 UART1 FIFO 溢出。
     * 因此这里把 RX 阈值设为 1 字节，并打开 UART1 中断，收到字节就立刻解析。
     */
    DL_UART_Main_disableInterrupt(UART_1_INST, JY62_UART_RX_INTERRUPTS);
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(UART_1_INST, 1U);
    DL_UART_Main_clearInterruptStatus(UART_1_INST, JY62_UART_RX_INTERRUPTS);
    JY62_DrainRxFifo();
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_1_INST, JY62_UART_RX_INTERRUPTS);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/**
 * @brief Reset parser/cache state and enable JY62 UART receiving.
 */
static void JY62_Init(void)
{
    uint8_t index;

    g_jy62_frame_index = 0U;
    g_jy62_last_poll_frame_count = 0U;
    g_jy62_yaw_zero_cdeg = 0;
    g_jy62_gyro_z_filtered_raw = 0;
    g_jy62_yaw_zero_valid = 0U;
    g_jy62_gyro_z_filter_valid = 0U;
    g_jy62_sample.update_flags = 0U;
    g_jy62_sample.received_flags = 0U;
    g_jy62_sample.last_frame_type = 0U;
    g_jy62_sample.raw_count = 0U;
    g_jy62_sample.raw_write_index = 0U;
    g_jy62_sample.header_count = 0U;
    g_jy62_sample.frame_count = 0U;
    g_jy62_sample.unknown_frame_count = 0U;
    g_jy62_sample.checksum_error = 0U;
    g_jy62_sample.rx_byte_count = 0U;
    g_jy62_sample.rx_irq_count = 0U;
    g_jy62_sample.uart_error_count = 0U;
    g_jy62_sample.overrun_count = 0U;
    g_jy62_sample.aux_raw = 0;

    for (index = 0U; index < 3U; index++) {
        g_jy62_sample.acc_raw[index] = 0;
        g_jy62_sample.gyro_raw[index] = 0;
        g_jy62_sample.angle_raw[index] = 0;
    }

    for (index = 0U; index < JY62_RAW_DUMP_LEN; index++) {
        g_jy62_sample.recent_raw[index] = 0U;
    }

    JY62_EnableRxInterrupt();
}

/**
 * @brief Poll accumulated parser state into a raw sample snapshot.
 */
static uint32_t JY62_Poll(jy62_sample_t *sample)
{
    uint32_t frame_delta;

    __disable_irq();
    if (sample != 0) {
        *sample = g_jy62_sample;
        g_jy62_sample.update_flags = 0U;
    }
    frame_delta = g_jy62_sample.frame_count - g_jy62_last_poll_frame_count;
    g_jy62_last_poll_frame_count = g_jy62_sample.frame_count;
    __enable_irq();

    return frame_delta;
}

/**
 * @brief Set the current yaw as software zero when angle data is valid.
 */
static void JY62_SetYawZeroToCurrent(void)
{
    __disable_irq();
    g_jy62_yaw_zero_cdeg = JY62_RawToAngleCdeg(g_jy62_sample.angle_raw[2]);
    g_jy62_yaw_zero_valid = 1U;
    __enable_irq();
}

/**
 * @brief Poll and convert JY62 data into navigation fields for controllers.
 */
static uint32_t JY62_GetNavigation(jy62_navigation_t *nav)
{
    jy62_sample_t sample;
    int32_t gyro_z_filtered_raw;
    int32_t yaw_zero_cdeg;
    uint32_t frame_delta;
    uint8_t yaw_zero_valid;

    __disable_irq();
    sample = g_jy62_sample;
    gyro_z_filtered_raw = g_jy62_gyro_z_filtered_raw;
    yaw_zero_cdeg = g_jy62_yaw_zero_cdeg;
    yaw_zero_valid = g_jy62_yaw_zero_valid;
    g_jy62_sample.update_flags = 0U;
    frame_delta = g_jy62_sample.frame_count - g_jy62_last_poll_frame_count;
    g_jy62_last_poll_frame_count = g_jy62_sample.frame_count;
    __enable_irq();

    if (nav != 0) {
        nav->yaw_cdeg = JY62_RawToAngleCdeg(sample.angle_raw[2]);
        nav->yaw_zero_cdeg = yaw_zero_cdeg;
        nav->yaw_relative_cdeg = JY62_NormalizeAngleCdeg(nav->yaw_cdeg - yaw_zero_cdeg);
        nav->gyro_z_mdps = JY62_RawToGyroMdps(sample.gyro_raw[2]);
        nav->gyro_z_filtered_mdps = JY62_RawToGyroMdps((int16_t)gyro_z_filtered_raw);
        nav->roll_cdeg = JY62_RawToAngleCdeg(sample.angle_raw[0]);
        nav->pitch_cdeg = JY62_RawToAngleCdeg(sample.angle_raw[1]);
        nav->valid = (((sample.received_flags & JY62_NAV_REQUIRED_FLAGS) == JY62_NAV_REQUIRED_FLAGS) &&
            (yaw_zero_valid != 0U)) ? 1U : 0U;
        nav->update_flags = sample.update_flags;
        nav->rx_byte_count = sample.rx_byte_count;
        nav->header_count = sample.header_count;
        nav->frame_count = sample.frame_count;
        nav->checksum_error = sample.checksum_error;
        nav->uart_error_count = sample.uart_error_count;
        nav->overrun_count = sample.overrun_count;
    }

    return frame_delta;
}

/**
 * @brief Read navigation fields without resetting update-frame accounting.
 */
static uint8_t JY62_PeekNavigation(jy62_navigation_t *nav)
{
    jy62_sample_t sample;
    int32_t gyro_z_filtered_raw;
    int32_t yaw_zero_cdeg;
    uint8_t yaw_zero_valid;

    __disable_irq();
    sample = g_jy62_sample;
    gyro_z_filtered_raw = g_jy62_gyro_z_filtered_raw;
    yaw_zero_cdeg = g_jy62_yaw_zero_cdeg;
    yaw_zero_valid = g_jy62_yaw_zero_valid;
    __enable_irq();

    if (nav != 0) {
        nav->yaw_cdeg = JY62_RawToAngleCdeg(sample.angle_raw[2]);
        nav->yaw_zero_cdeg = yaw_zero_cdeg;
        nav->yaw_relative_cdeg = JY62_NormalizeAngleCdeg(nav->yaw_cdeg - yaw_zero_cdeg);
        nav->gyro_z_mdps = JY62_RawToGyroMdps(sample.gyro_raw[2]);
        nav->gyro_z_filtered_mdps = JY62_RawToGyroMdps((int16_t)gyro_z_filtered_raw);
        nav->roll_cdeg = JY62_RawToAngleCdeg(sample.angle_raw[0]);
        nav->pitch_cdeg = JY62_RawToAngleCdeg(sample.angle_raw[1]);
        nav->valid = (((sample.received_flags & JY62_NAV_REQUIRED_FLAGS) == JY62_NAV_REQUIRED_FLAGS) &&
            (yaw_zero_valid != 0U)) ? 1U : 0U;
        nav->update_flags = sample.update_flags;
        nav->rx_byte_count = sample.rx_byte_count;
        nav->header_count = sample.header_count;
        nav->frame_count = sample.frame_count;
        nav->checksum_error = sample.checksum_error;
        nav->uart_error_count = sample.uart_error_count;
        nav->overrun_count = sample.overrun_count;
    }

    return (nav != 0) ? nav->valid : 0U;
}

/**
 * @brief UART1 ISR entry used by main.c to service JY62 bytes/errors.
 */
static void JY62_UART1_IRQHandler(void)
{
    DL_UART_IIDX pending;

    do {
        pending = DL_UART_Main_getPendingInterrupt(UART_1_INST);

        switch (pending) {
        case DL_UART_MAIN_IIDX_RX:
            g_jy62_sample.rx_irq_count++;
            JY62_DrainRxFifo();
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
            break;

        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            g_jy62_sample.rx_irq_count++;
            JY62_DrainRxFifo();
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            JY62_RecordUartError(1U);
            JY62_DrainRxFifo();
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
            break;

        case DL_UART_MAIN_IIDX_BREAK_ERROR:
            JY62_RecordUartError(0U);
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_BREAK_ERROR);
            break;

        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            JY62_RecordUartError(0U);
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
            break;

        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            JY62_RecordUartError(0U);
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
            break;

        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            JY62_RecordUartError(0U);
            DL_UART_Main_clearInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
            break;

        default:
            break;
        }
    } while (pending != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

/**
 * @brief Print raw decoded JY62 sample values for diagnostics.
 */
static void JY62_PrintSample(const jy62_sample_t *sample)
{
    int32_t acc_x = JY62_RawToAccMg(sample->acc_raw[0]);
    int32_t acc_y = JY62_RawToAccMg(sample->acc_raw[1]);
    int32_t acc_z = JY62_RawToAccMg(sample->acc_raw[2]);
    int32_t gyro_x = JY62_RawToGyroMdps(sample->gyro_raw[0]);
    int32_t gyro_y = JY62_RawToGyroMdps(sample->gyro_raw[1]);
    int32_t gyro_z = JY62_RawToGyroMdps(sample->gyro_raw[2]);
    int32_t roll = JY62_RawToAngleCdeg(sample->angle_raw[0]);
    int32_t pitch = JY62_RawToAngleCdeg(sample->angle_raw[1]);
    int32_t yaw = JY62_RawToAngleCdeg(sample->angle_raw[2]);

    lc_printf("acc_mg=%ld,%ld,%ld gyro_mdps=%ld,%ld,%ld angle_cdeg=%ld,%ld,%ld yaw=",
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw);
    JY62_PrintSignedFixed(yaw, 100U, 2U);
    lc_printf(" gz=");
    JY62_PrintSignedFixed(gyro_z, 1000U, 3U);
}

/**
 * @brief Print navigation-friendly JY62 values for diagnostics.
 */
static void JY62_PrintNavigation(const jy62_navigation_t *nav)
{
    lc_printf("ok=%u yaw=", nav->valid);
    JY62_PrintSignedFixed(nav->yaw_cdeg, 100U, 2U);
    lc_printf(" rel=");
    JY62_PrintSignedFixed(nav->yaw_relative_cdeg, 100U, 2U);
    lc_printf(" gz=");
    JY62_PrintSignedFixed(nav->gyro_z_mdps, 1000U, 3U);
    lc_printf(" gzlp=");
    JY62_PrintSignedFixed(nav->gyro_z_filtered_mdps, 1000U, 3U);
}

/**
 * @brief Print the recent raw UART byte ring buffer.
 */
static void JY62_PrintRecentRaw(const jy62_sample_t *sample)
{
    uint8_t index;
    uint8_t start;

    if (sample->raw_count == 0U) {
        lc_printf("none");
        return;
    }

    if (sample->raw_count < JY62_RAW_DUMP_LEN) {
        start = 0U;
    } else {
        start = sample->raw_write_index;
    }

    for (index = 0U; index < sample->raw_count; index++) {
        uint8_t raw_index = (uint8_t)((start + index) % JY62_RAW_DUMP_LEN);
        if (index != 0U) {
            lc_printf(" ");
        }
        lc_printf("%02X", sample->recent_raw[raw_index]);
    }
}

#endif /* _BSP_JY62_H */
