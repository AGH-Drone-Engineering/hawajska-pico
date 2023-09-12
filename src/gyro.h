#ifndef _APP_GYRO_H_
#define _APP_GYRO_H_

#include <stdbool.h>

enum gyro_fs
{
    GYRO_FS_245DPS = 0,
    GYRO_FS_500DPS = 1,
    GYRO_FS_2000DPS = 2,
};

enum gyro_out_sel
{
    GYRO_OUT_SEL_LPF1 = 0,
    GYRO_OUT_SEL_HPF = 1,
    GYRO_OUT_SEL_LPF2 = 2,
};

void gyro_init(int dr, int bw, enum gyro_fs fs, bool low_odr, bool hp_en, enum gyro_out_sel out_sel);

bool gyro_read(float *vel);

#endif
