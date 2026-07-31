#ifndef G3507_BASE_JY901S_H
#define G3507_BASE_JY901S_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file JY901S.h
 * @brief JY901S 标准 0x55 串口帧解析结果与快照接口。
 *
 * 驱动在 UART2 RX 中断中组帧并完成比例换算，APP 读取到的单位固定为
 * g、deg/s、deg 和摄氏度，不再接触原始 int16 数据。
 */
typedef struct {
    float acc_x_g;                 /* X 轴加速度，单位 g */
    float acc_y_g;                 /* Y 轴加速度，单位 g */
    float acc_z_g;                 /* Z 轴加速度，单位 g */
    float gyro_x_dps;              /* X 轴角速度，单位 deg/s */
    float gyro_y_dps;              /* Y 轴角速度，单位 deg/s */
    float gyro_z_dps;              /* Z 轴角速度，单位 deg/s */
    float roll_deg;                /* 横滚角，单位 deg */
    float pitch_deg;               /* 俯仰角，单位 deg */
    float yaw_deg;                 /* 航向角，单位 deg */
    float temperature_c;           /* 模块内部温度，单位摄氏度 */
    uint32_t valid_frame_count;    /* 0x51/0x52/0x53 有效帧累计数 */
    uint32_t checksum_error_count; /* 校验和错误帧累计数 */
    uint32_t last_frame_ms;        /* 最近有效帧到达时的系统毫秒时间 */
    bool online;                   /* GetData 根据最近帧时间动态计算的在线状态 */
} JY901S_Data_t;

/**
 * Keil Watch 镜像。override_enable=0 时 data 自动刷新真实传感器数据；
 * 写成 1 后驱动返回 Watch 中手工修改的 data，便于脱离传感器联调。
 */
typedef struct {
    uint8_t override_enable;
    JY901S_Data_t data;
} JY901S_Watch_t;

/** 非 static、volatile 全局变量，可直接添加到 Keil Watch。 */
extern volatile JY901S_Watch_t g_jy901s_watch;

/** 清空解析器、统计计数和所有物理量。 */
void JY901S_Init(void);

/** 在临界区复制完整快照，并根据最后有效帧时间计算 online。 */
void JY901S_GetData(JY901S_Data_t *data);

#endif
