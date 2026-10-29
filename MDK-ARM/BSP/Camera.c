#include "Camera.h"

#include <limits.h>
#include <string.h>

/*
 * 摄像头数据处理链路：
 * USART2 + DMA 收到一段字节 -> UART_DMA.c 的统一 HAL 回调
 * -> Camera_OnRxData() 按 \r/\n 重新拼成完整文本行
 * -> Camera_HandleLine() 判断坐标、目标丢失或非法格式
 * -> g_camera_debug.data 保存最新快照，供控制层读取。
 *
 * DMA 的“一个回调”不等于摄像头的“一帧”：一条文本可能被拆成两次回调，
 * 也可能一次回调中包含多条文本。因此这里必须维护跨回调的行缓存。
 */

/* Keil Watch 的摄像头总观察变量；中断和主循环都会访问，所以声明为 volatile。 */
volatile Camera_Debug_t g_camera_debug = {0};

/* 尚未遇到行结束符的文本内容。多出来的 1 字节专门留给字符串结尾 '\0'。 */
static char s_line_buffer[CAMERA_LINE_BUFFER_SIZE + 1U];
/* 当前已经写入 s_line_buffer 的有效字符数，不包含 '\0'。 */
static uint16_t s_line_length = 0U;
/* 单行过长后置 1；在遇到下一个换行符前丢弃剩余字符，重新建立帧边界。 */
static uint8_t s_line_overflow = 0U;

/**
 * @brief 判断字符是否是协议允许忽略的横向空白。
 * @note 这里只跳过空格和制表符，\r/\n 由流式组帧函数单独处理。
 */
static uint8_t Camera_IsSpace(char ch)
{
    return (uint8_t)((ch == ' ') || (ch == '\t'));
}

/**
 * @brief 从当前位置跳过连续空格/制表符并返回第一个有效字符。
 */
static const char *Camera_SkipSpace(const char *cursor)
{
    while ((*cursor != '\0') && Camera_IsSpace(*cursor)) {
        cursor++;
    }
    return cursor;
}

/**
 * @brief 从字符串当前位置解析一个有符号 int16_t 十进制整数。
 * @param cursor 输入/输出游标；成功后指向数字后的第一个字符。
 * @param value  输出解析结果。
 * @retval 1=解析成功；0=空参数、没有数字或数值越界。
 *
 * @details
 * 允许数字前存在空格和可选的 +/- 号。正数上限 32767，负数绝对值上限
 * 32768。乘 10 前先判断是否会越界，避免超长异常字符串造成 int32_t 溢出。
 */
static uint8_t Camera_ParseInt16(const char **cursor, int16_t *value)
{
    const char *p;
    int32_t magnitude = 0;
    int32_t signed_value;
    int32_t limit = INT16_MAX;
    int8_t sign = 1;
    uint8_t digit_count = 0U;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL)) {
        return 0U;
    }

    p = Camera_SkipSpace(*cursor);

    /* 负数比正数多一个可表示值，因此负号情况下允许 magnitude=32768。 */
    if ((*p == '+') || (*p == '-')) {
        if (*p == '-') {
            sign = -1;
            limit = 32768;
        }
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        int32_t digit = (int32_t)(*p - '0');

        /* 在执行 magnitude*10+digit 之前检查，保证运算本身不会溢出。 */
        if (magnitude > ((limit - digit) / 10)) {
            return 0U;
        }
        magnitude = magnitude * 10 + digit;
        digit_count++;
        p++;
    }

    if (digit_count == 0U) {
        return 0U;
    }

    signed_value = magnitude * sign;
    *value = (int16_t)signed_value;
    *cursor = p;
    return 1U;
}

/**
 * @brief 将最近一条完整文本行复制到 Watch 缓冲区并补字符串结尾。
 */
static void Camera_CopyLastFrame(const char *line, uint16_t length)
{
    uint16_t copy_length = length;

    if (copy_length > CAMERA_LINE_BUFFER_SIZE) {
        copy_length = CAMERA_LINE_BUFFER_SIZE;
    }
    memcpy((void *)g_camera_debug.last_frame, line, copy_length);
    g_camera_debug.last_frame[copy_length] = '\0';
}

/**
 * @brief 记录“摄像头串口链路仍有完整文本行到达”。
 * @note online 仅代表通信活跃，不代表本帧一定识别到了小球；target_found
 *       会由具体帧类型单独设置。
 */
static void Camera_MarkCompleteFrame(uint32_t now)
{
    g_camera_debug.data.online = 1U;
    g_camera_debug.data.last_frame_tick = now;
    g_camera_debug.data.frame_sequence++;
}

/**
 * @brief 解析一条不含行结束符的完整文本帧。
 *
 * @details 解析优先级：
 * 1. 精确的 N/n：链路在线，但目标丢失；
 * 2. 有符号 X，后面可选“逗号 + 有符号 Y”；
 * 3. 数字后只能存在空白，否则整行判为非法。
 *
 * 非法帧会立即清除 target_found，防止控制层继续使用过期目标状态；
 * 最近一次有效 x/y 数值仍保留，便于 Watch 对比故障前后的数据。
 */
static void Camera_HandleLine(char *line, uint16_t length)
{
    const char *cursor;
    int16_t x = 0;
    int16_t y = 0;
    uint32_t now = HAL_GetTick();

    line[length] = '\0';
    Camera_CopyLastFrame(line, length);
    g_camera_debug.completed_line_count++;
    Camera_MarkCompleteFrame(now);

    /* “N”表示视觉算法本帧没有找到球，并不表示串口掉线。 */
    cursor = Camera_SkipSpace(line);
    if (((*cursor == 'N') || (*cursor == 'n')) &&
        (*Camera_SkipSpace(cursor + 1) == '\0')) {
        g_camera_debug.data.target_found = 0U;
        g_camera_debug.lost_frame_count++;
        g_camera_debug.last_frame_type = CAMERA_FRAME_LOST;
        return;
    }

    /* 至少必须存在一个 X；没有逗号时按照一维轨道协议令 Y=0。 */
    if (Camera_ParseInt16(&cursor, &x) == 0U) {
        g_camera_debug.data.target_found = 0U;
        g_camera_debug.parse_error_count++;
        g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
        return;
    }

    /* 若出现逗号，则逗号后必须紧跟一个合法的有符号 Y。 */
    cursor = Camera_SkipSpace(cursor);
    if (*cursor == ',') {
        cursor++;
        if (Camera_ParseInt16(&cursor, &y) == 0U) {
            g_camera_debug.data.target_found = 0U;
            g_camera_debug.parse_error_count++;
            g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
            return;
        }
    }

    /* 坐标后出现非空白尾随字符通常意味着粘包格式或发送端协议不一致。 */
    cursor = Camera_SkipSpace(cursor);
    if (*cursor != '\0') {
        g_camera_debug.data.target_found = 0U;
        g_camera_debug.parse_error_count++;
        g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
        return;
    }

    /*
     * 第三问串口契约直接发送相对 O 点毫米值。超过轨道物理范围的整数虽然格式
     * 正确，但不得进入控制器，也不得刷新 last_valid_tick，否则错误大坐标会被
     * 当成“视觉仍有效”并产生危险输出。
     */
    if ((x < -CAMERA_X_PHYSICAL_LIMIT_MM) ||
        (x > CAMERA_X_PHYSICAL_LIMIT_MM)) {
        g_camera_debug.data.target_found = 0U;
        g_camera_debug.physical_reject_count++;
        g_camera_debug.parse_error_count++;
        g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
        return;
    }

    g_camera_debug.data.x = x;
    g_camera_debug.data.y = y;
    g_camera_debug.data.valid_sequence++;
    g_camera_debug.data.last_valid_tick = now;
    g_camera_debug.data.target_found = 1U;
    g_camera_debug.valid_coord_count++;
    g_camera_debug.last_frame_type = CAMERA_FRAME_COORD;
}

/**
 * @brief 初始化摄像头解析器的软件状态。
 * @note 本函数不访问 USART2 寄存器，也不启动 DMA；必须在 UART_DMA_Init()
 *       之前调用，防止 DMA 中断到来后又被这里清空计数和解析状态。
 */
void Camera_Init(void)
{
    memset((void *)&g_camera_debug, 0, sizeof(g_camera_debug));
    memset(s_line_buffer, 0, sizeof(s_line_buffer));
    s_line_length = 0U;
    s_line_overflow = 0U;
}

/**
 * @brief 在主循环中维护摄像头在线状态。
 *
 * @details last_frame_tick 由完整文本行更新，而不是任意噪声字节更新。
 * 因此只有真正形成合法帧边界的持续数据才能维持 online=1。
 */
void Camera_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t last_frame_tick = g_camera_debug.data.last_frame_tick;

    if ((last_frame_tick == 0U) ||
        ((uint32_t)(now - last_frame_tick) > CAMERA_LINK_TIMEOUT_MS)) {
        g_camera_debug.data.online = 0U;
        g_camera_debug.data.target_found = 0U;
    }
}

/**
 * @brief 接收并消费一段可能被拆分或粘连的摄像头字节流。
 * @note 该函数在 USART2 中断上下文执行，只做有限长度复制、字符扫描和整数
 *       解析，不调用阻塞串口发送或 HAL_Delay。
 */
void Camera_OnRxData(const uint8_t *data, uint16_t size)
{
    uint16_t i;

    if ((data == NULL) || (size == 0U)) {
        return;
    }

    g_camera_debug.rx_chunk_count++;
    g_camera_debug.rx_byte_count += size;
    g_camera_debug.last_rx_len = size;
    g_camera_debug.last_rx_tick = HAL_GetTick();

    for (i = 0U; i < size; i++) {
        char ch = (char)data[i];

        /* 同时兼容 \n、\r 和常见的 \r\n。第二个连续结束符因行长为 0 被忽略。 */
        if ((ch == '\n') || (ch == '\r')) {
            /* 过长帧必须一直丢弃到换行，避免中间任意字节被误当成新帧头。 */
            if (s_line_overflow != 0U) {
                /* 到换行才算形成了一条完整但过长的非法文本帧。 */
                g_camera_debug.completed_line_count++;
                Camera_MarkCompleteFrame(HAL_GetTick());
                g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
                s_line_overflow = 0U;
                s_line_length = 0U;
                continue;
            }
            if (s_line_length != 0U) {
                Camera_HandleLine(s_line_buffer, s_line_length);
                s_line_length = 0U;
            }
            continue;
        }

        /* 已进入溢出恢复状态时，换行前的所有字节都不再写入缓存。 */
        if (s_line_overflow != 0U) {
            continue;
        }

        if (s_line_length < CAMERA_LINE_BUFFER_SIZE) {
            s_line_buffer[s_line_length++] = ch;
        } else {
            /* 只在首次越界时累计一次错误，随后等待行结束符恢复同步。 */
            s_line_overflow = 1U;
            s_line_length = 0U;
            g_camera_debug.data.target_found = 0U;
            g_camera_debug.line_overflow_count++;
            g_camera_debug.parse_error_count++;
            g_camera_debug.last_frame_type = CAMERA_FRAME_INVALID;
        }
    }
}

/**
 * @brief 保存 USART2 最近一次 HAL 错误信息，供 Keil Watch 排查。
 * @note 清错误标志、中止旧 RX 和重新启动 DMA 的动作统一在 UART_DMA.c 完成。
 */
void Camera_OnUartError(uint32_t error_code)
{
    g_camera_debug.uart_error_count++;
    g_camera_debug.last_uart_error_code = error_code;
    g_camera_debug.last_error_tick = HAL_GetTick();
}

/**
 * @brief 将中断持续更新的数据复制成一致的本地快照。
 *
 * @details Cortex-M4 上单个 32 位字段读写通常是原子的，但整个结构体包含多个
 * 字段。若复制过程中恰好进入 USART2 中断，调用者可能拿到新旧字段混合的数据。
 * 因此这里短暂关闭中断完成结构体复制，并恢复调用前的 PRIMASK 状态。
 */
uint8_t Camera_GetData(Camera_Data_t *data)
{
    uint32_t primask;

    if (data == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *data = g_camera_debug.data;
    if (primask == 0U) {
        __enable_irq();
    }

    return data->online;
}
