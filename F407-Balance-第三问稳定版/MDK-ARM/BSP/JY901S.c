#include "JY901S.h"

#include <string.h>

/*
 * JY901S 数据处理链路：
 * USART3 + ReceiveToIdle DMA -> UART_DMA.c 统一回调
 * -> JY901S_OnRxData() 逐字节送入状态机
 * -> 收齐 11 字节后校验
 * -> 按 0x51/0x52/0x53/0x54 解码并换算单位
 * -> g_jy901s_debug.data 提供给主循环或控制层读取。
 *
 * 逐字节状态机与 DMA 分块边界无关：即使一帧被拆进两次 DMA 回调，或者一次
 * DMA 回调包含多帧，解析仍然可以连续进行。
 */

/* Keil Watch 总观察变量；绝大部分字段在 USART3 中断上下文中更新。 */
volatile JY901S_Debug_t g_jy901s_debug = {0};

/* 当前正在组装的一帧原始数据。 */
static uint8_t s_frame[JY901S_FRAME_SIZE];
/* 已写入 s_frame 的字节数；0 表示正在等待 0x55 帧头。 */
static uint8_t s_frame_index = 0U;

/**
 * @brief 按 WIT 协议的小端顺序读取一个有符号 16 位整数。
 * @note data[0] 是低字节，data[1] 是高字节。
 */
static int16_t JY901S_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/**
 * @brief 计算 WIT 普通协议校验和。
 * @return Byte0~Byte9 累加结果的低 8 位，应与 Byte10 相同。
 */
static uint8_t JY901S_CalculateChecksum(const uint8_t *frame)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < (JY901S_FRAME_SIZE - 1U); i++) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    return checksum;
}

/**
 * @brief 保存最近一次凑满的 11 字节帧，便于在 Watch 中直接查看十六进制内容。
 * @note 无论校验是否通过都会保存，这样校验错误时也能看到问题原始帧。
 */
static void JY901S_CopyLastFrame(const uint8_t *frame)
{
    uint8_t i;

    for (i = 0U; i < JY901S_FRAME_SIZE; i++) {
        g_jy901s_debug.last_frame[i] = frame[i];
    }
}

/**
 * @brief 标记某类物理量刚刚更新，并刷新在线时间。
 * @param flag 一个或多个 JY901S_UPDATE_* 位。
 */
static void JY901S_MarkUpdated(uint32_t flag)
{
    g_jy901s_debug.data.update_flag |= flag;
    g_jy901s_debug.data.last_update_tick = HAL_GetTick();
    g_jy901s_debug.data.online = 1U;
}

/**
 * @brief 对一帧已经通过校验的数据进行类型判断、原始值保存和比例换算。
 *
 * @details 四个数据字均按小端 int16_t 提取。前三个字对应 X/Y/Z 或
 * Roll/Pitch/Yaw；第四个字的含义随帧类型变化，本工程只把 0x51 的第四字
 * 按温度 raw/100 ℃ 使用。
 */
static void JY901S_DecodeFrame(const uint8_t *frame)
{
    int16_t x = JY901S_ReadInt16(&frame[2]);
    int16_t y = JY901S_ReadInt16(&frame[4]);
    int16_t z = JY901S_ReadInt16(&frame[6]);
    int16_t fourth = JY901S_ReadInt16(&frame[8]);

    g_jy901s_debug.last_frame_type = frame[1];
    g_jy901s_debug.valid_frame_count++;

    switch (frame[1]) {
        case 0x51U:
            /* 加速度帧：raw/32768*16，单位 g；第四数据字为温度 raw/100 ℃。 */
            g_jy901s_debug.raw_acc[0] = x;
            g_jy901s_debug.raw_acc[1] = y;
            g_jy901s_debug.raw_acc[2] = z;
            g_jy901s_debug.raw_temperature = fourth;
            g_jy901s_debug.data.acc[0] = (float)x / 32768.0f * 16.0f;
            g_jy901s_debug.data.acc[1] = (float)y / 32768.0f * 16.0f;
            g_jy901s_debug.data.acc[2] = (float)z / 32768.0f * 16.0f;
            g_jy901s_debug.data.temperature = (float)fourth / 100.0f;
            g_jy901s_debug.data.acc_update_sequence++;
            g_jy901s_debug.acc_frame_count++;
            JY901S_MarkUpdated(JY901S_UPDATE_ACC | JY901S_UPDATE_TEMP);
            break;

        case 0x52U:
            /* 角速度帧：raw/32768*2000，单位 °/s。 */
            g_jy901s_debug.raw_gyro[0] = x;
            g_jy901s_debug.raw_gyro[1] = y;
            g_jy901s_debug.raw_gyro[2] = z;
            g_jy901s_debug.data.gyro[0] = (float)x / 32768.0f * 2000.0f;
            g_jy901s_debug.data.gyro[1] = (float)y / 32768.0f * 2000.0f;
            g_jy901s_debug.data.gyro[2] = (float)z / 32768.0f * 2000.0f;
            g_jy901s_debug.data.gyro_update_sequence++;
            g_jy901s_debug.gyro_frame_count++;
            JY901S_MarkUpdated(JY901S_UPDATE_GYRO);
            break;

        case 0x53U:
            /* 欧拉角帧：raw/32768*180，顺序为 Roll、Pitch、Yaw，单位 °。 */
            g_jy901s_debug.raw_angle[0] = x;
            g_jy901s_debug.raw_angle[1] = y;
            g_jy901s_debug.raw_angle[2] = z;
            g_jy901s_debug.data.angle[0] = (float)x / 32768.0f * 180.0f;
            g_jy901s_debug.data.angle[1] = (float)y / 32768.0f * 180.0f;
            g_jy901s_debug.data.angle[2] = (float)z / 32768.0f * 180.0f;
            g_jy901s_debug.angle_frame_count++;
            JY901S_MarkUpdated(JY901S_UPDATE_ANGLE);
            break;

        case 0x54U:
            /* 磁场帧：保留传感器原始 int16_t，不在 BSP 中擅自换算单位。 */
            g_jy901s_debug.raw_mag[0] = x;
            g_jy901s_debug.raw_mag[1] = y;
            g_jy901s_debug.raw_mag[2] = z;
            g_jy901s_debug.data.mag[0] = x;
            g_jy901s_debug.data.mag[1] = y;
            g_jy901s_debug.data.mag[2] = z;
            g_jy901s_debug.mag_frame_count++;
            JY901S_MarkUpdated(JY901S_UPDATE_MAG);
            break;

        default:
            /* 例如时间、端口状态等其他合法帧：保留总帧计数，但暂不更新 online。 */
            g_jy901s_debug.unsupported_frame_count++;
            break;
    }
}

/**
 * @brief 将一个字节送入固定 11 字节组帧状态机。
 *
 * @details
 * - index=0 时只接受 0x55，其余字节计入 discarded_byte_count；
 * - 收到帧头后直接按固定长度收集，不会把载荷中的 0x55 误判为新帧头；
 * - 收满后校验并立即将 index 清零，下一字节重新寻找帧头。
 *
 * 若线路丢失一个字节，本帧通常会校验失败；清零后会在后续 0x55 处重新同步。
 */
static void JY901S_PushByte(uint8_t byte)
{
    uint8_t checksum;

    if (s_frame_index == 0U) {
        /* 未同步状态：所有非帧头字节都只是噪声或上一帧残留。 */
        if (byte != 0x55U) {
            g_jy901s_debug.discarded_byte_count++;
            return;
        }
        s_frame[0] = byte;
        s_frame_index = 1U;
        g_jy901s_debug.parser_index = s_frame_index;
        return;
    }

    s_frame[s_frame_index++] = byte;
    g_jy901s_debug.parser_index = s_frame_index;

    if (s_frame_index < JY901S_FRAME_SIZE) {
        return;
    }

    /* 收满 11 字节后先保存原帧，再比较本地计算值与传感器携带值。 */
    checksum = JY901S_CalculateChecksum(s_frame);
    JY901S_CopyLastFrame(s_frame);
    g_jy901s_debug.last_checksum_calculated = checksum;
    g_jy901s_debug.last_checksum_received = s_frame[JY901S_FRAME_SIZE - 1U];

    if (checksum == s_frame[JY901S_FRAME_SIZE - 1U]) {
        JY901S_DecodeFrame(s_frame);
    } else {
        g_jy901s_debug.checksum_error_count++;
    }

    s_frame_index = 0U;
    g_jy901s_debug.parser_index = 0U;
}

/**
 * @brief 清空 JY901S 软件解析状态。
 * @note 本函数不配置传感器寄存器、不发送 WIT 指令，也不启动 USART3 DMA。
 *       调用顺序应为 JY901S_Init() -> UART_DMA_Init()。
 */
void JY901S_Init(void)
{
    memset((void *)&g_jy901s_debug, 0, sizeof(g_jy901s_debug));
    memset(s_frame, 0, sizeof(s_frame));
    s_frame_index = 0U;
}

/**
 * @brief 主循环在线状态维护。
 *
 * @details 只有校验正确且属于当前支持类型的帧才更新 last_update_tick。
 * 因此“USART3 有字节”不等于“JY901S 有有效数据”，可结合 rx_byte_count、
 * checksum_error_count 和 valid_frame_count 分层判断问题位置。
 */
void JY901S_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t last_update_tick = g_jy901s_debug.data.last_update_tick;

    if ((last_update_tick == 0U) ||
        ((uint32_t)(now - last_update_tick) > JY901S_ONLINE_TIMEOUT_MS)) {
        g_jy901s_debug.data.online = 0U;
    }
}

/**
 * @brief 消费一段 USART3 原始字节流。
 * @note 在中断上下文调用。这里只执行确定上限的循环和简单浮点换算，不进行
 *       阻塞串口发送、参数写入或延时。
 */
void JY901S_OnRxData(const uint8_t *data, uint16_t size)
{
    uint16_t i;

    if ((data == NULL) || (size == 0U)) {
        return;
    }

    g_jy901s_debug.rx_chunk_count++;
    g_jy901s_debug.rx_byte_count += size;
    g_jy901s_debug.last_rx_len = size;
    g_jy901s_debug.last_rx_tick = HAL_GetTick();

    for (i = 0U; i < size; i++) {
        JY901S_PushByte(data[i]);
    }
}

/**
 * @brief 保存 USART3 HAL 错误码和发生时间。
 * @note 实际的 HAL_UART_AbortReceive、清 ORE 和重新挂 DMA 在 UART_DMA.c 中统一完成。
 */
void JY901S_OnUartError(uint32_t error_code)
{
    g_jy901s_debug.uart_error_count++;
    g_jy901s_debug.last_uart_error_code = error_code;
    g_jy901s_debug.last_error_tick = HAL_GetTick();
}

/**
 * @brief 临界区复制一份一致的 JY901S 数据快照。
 *
 * @details 数据结构包含多个 float/uint32_t 字段，复制过程中若被 USART3 中断
 * 打断，可能出现不同帧字段混合。这里保存调用前 PRIMASK，短暂关中断复制，
 * 再按原状态恢复；不会错误地打开调用者原本已经关闭的中断。
 */
uint8_t JY901S_GetData(JY901S_Data_t *data)
{
    uint32_t primask;
    uint32_t now;

    if (data == NULL) {
        return 0U;
    }

    now = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_jy901s_debug.data.last_update_tick == 0U) ||
        ((uint32_t)(now - g_jy901s_debug.data.last_update_tick) > JY901S_ONLINE_TIMEOUT_MS)) {
        g_jy901s_debug.data.online = 0U;
    }
    *data = g_jy901s_debug.data;
    if (primask == 0U) {
        __enable_irq();
    }

    return data->online;
}

/**
 * @brief 查看哪些数据类型自上次清除后至少更新过一次。
 */
uint32_t JY901S_GetUpdateFlag(void)
{
    return g_jy901s_debug.data.update_flag;
}

/**
 * @brief 原子地清除指定更新位。
 * @note 该操作只表示“上层已经消费这些新数据”，不会清零 acc/gyro/angle 等值。
 */
void JY901S_ClearUpdateFlag(uint32_t flag)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_jy901s_debug.data.update_flag &= ~flag;
    if (primask == 0U) {
        __enable_irq();
    }
}
