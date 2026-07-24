#include "JY901S.h"

#include "Board.h"
#include "bsp_config.h"
#include "ti_msp_dl_config.h"

#define JY901S_FRAME_LENGTH 11U
#define JY901S_HEADER       0x55U
#define JY901S_FRAME_ACC    0x51U
#define JY901S_FRAME_GYRO   0x52U
#define JY901S_FRAME_ANGLE  0x53U

/* 解析结果由 UART ISR 更新、主循环读取，因此使用 volatile 快照。 */
static volatile JY901S_Data_t g_imu;
/* 固定 11 字节接收缓冲和当前写入位置，只在 UART ISR 中访问。 */
static uint8_t g_frame[JY901S_FRAME_LENGTH];
static uint8_t g_frame_index = 0U;

static int16_t ReadS16LE(const uint8_t *data)
{
    /* JY901S 多字节字段均为低字节在前的有符号 16 位整数。 */
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void DecodeFrame(void)
{
    uint8_t sum = 0U;
    uint8_t i;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temperature;

    /* 前 10 字节无符号累加后取低 8 位，应等于第 11 字节校验和。 */
    for (i = 0U; i < (JY901S_FRAME_LENGTH - 1U); i++) {
        sum = (uint8_t)(sum + g_frame[i]);
    }
    if (sum != g_frame[JY901S_FRAME_LENGTH - 1U]) {
        g_imu.checksum_error_count++;
        return;
    }

    x = ReadS16LE(&g_frame[2]);
    y = ReadS16LE(&g_frame[4]);
    z = ReadS16LE(&g_frame[6]);
    temperature = ReadS16LE(&g_frame[8]);

    /*
     * 比例系数来自 JY901S 标准协议：加速度 ±16 g、角速度 ±2000 deg/s、
     * 角度 ±180 deg。温度字段三个已支持帧类型均采用 0.01 ℃/LSB。
     */
    if (g_frame[1] == JY901S_FRAME_ACC) {
        g_imu.acc_x_g = (float)x * 16.0f / 32768.0f;
        g_imu.acc_y_g = (float)y * 16.0f / 32768.0f;
        g_imu.acc_z_g = (float)z * 16.0f / 32768.0f;
        g_imu.temperature_c = (float)temperature / 100.0f;
    } else if (g_frame[1] == JY901S_FRAME_GYRO) {
        g_imu.gyro_x_dps = (float)x * 2000.0f / 32768.0f;
        g_imu.gyro_y_dps = (float)y * 2000.0f / 32768.0f;
        g_imu.gyro_z_dps = (float)z * 2000.0f / 32768.0f;
        g_imu.temperature_c = (float)temperature / 100.0f;
    } else if (g_frame[1] == JY901S_FRAME_ANGLE) {
        g_imu.roll_deg = (float)x * 180.0f / 32768.0f;
        g_imu.pitch_deg = (float)y * 180.0f / 32768.0f;
        g_imu.yaw_deg = (float)z * 180.0f / 32768.0f;
        g_imu.temperature_c = (float)temperature / 100.0f;
    } else {
        /* 时间、磁场等其他 0x55 帧暂不解析，避免把基础驱动做得过重。 */
        return;
    }

    /* 只有已支持且校验正确的帧才刷新在线时间。 */
    g_imu.valid_frame_count++;
    g_imu.last_frame_ms = Board_Millis();
}

/**
 * @brief 接收一个字节并组装 JY901S 固定 11 字节帧。
 *
 * 这只是串口帧同步，不是整车运行状态机：只有首字节 0x55、固定长度和校验和
 * 三条规则。出现乱码时丢弃当前字节，等待下一个 0x55 自动重新同步。
 */
static void PushByte(uint8_t byte)
{
    if (g_frame_index == 0U) {
        if (byte != JY901S_HEADER) return;
        g_frame[0] = byte;
        g_frame_index = 1U;
        return;
    }

    /* 帧内不再把数据字节 0x55 误判成新帧头，固定收满 11 字节后统一校验。 */
    g_frame[g_frame_index++] = byte;
    if (g_frame_index >= JY901S_FRAME_LENGTH) {
        DecodeFrame();
        g_frame_index = 0U;
    }
}

void JY901S_Init(void)
{
    /* 明确逐字段清零，避免依赖 C 运行库 memset，并使初始化含义可直接审阅。 */
    g_imu.acc_x_g = 0.0f;
    g_imu.acc_y_g = 0.0f;
    g_imu.acc_z_g = 0.0f;
    g_imu.gyro_x_dps = 0.0f;
    g_imu.gyro_y_dps = 0.0f;
    g_imu.gyro_z_dps = 0.0f;
    g_imu.roll_deg = 0.0f;
    g_imu.pitch_deg = 0.0f;
    g_imu.yaw_deg = 0.0f;
    g_imu.temperature_c = 0.0f;
    g_imu.valid_frame_count = 0U;
    g_imu.checksum_error_count = 0U;
    g_imu.last_frame_ms = 0U;
    g_imu.online = false;
    g_frame_index = 0U;
}

void JY901S_GetData(JY901S_Data_t *data)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *data = g_imu;
    if (primask == 0U) {
        __enable_irq();
    }

    /* online 不在 ISR 中维护，读取时依据当前时间计算，掉线后会自然变为 false。 */
    data->online = (data->valid_frame_count != 0U) &&
                   ((Board_Millis() - data->last_frame_ms) <= JY901S_OFFLINE_TIMEOUT_MS);
}

void JY901S_UART_INST_IRQHandler(void)
{
    /* 一次中断尽量排空 RX FIFO，避免高输出频率下留下字节导致溢出。 */
    if (DL_UART_Main_getPendingInterrupt(JY901S_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(JY901S_UART_INST)) {
            PushByte(DL_UART_Main_receiveData(JY901S_UART_INST));
        }
    }
}
