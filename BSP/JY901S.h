#ifndef G3507_BASE_JY901S_H
#define G3507_BASE_JY901S_H

#include <stdbool.h>
#include <stdint.h>

/* 单位固定为 g、deg/s、deg 和摄氏度，避免 APP 再做比例换算。 */
typedef struct {
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float temperature_c;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t last_frame_ms;
    bool online;
} JY901S_Data_t;

/* GetData 会在临界区复制完整快照，并根据最后帧时间计算 online。 */
void JY901S_Init(void);
void JY901S_GetData(JY901S_Data_t *data);

#endif
