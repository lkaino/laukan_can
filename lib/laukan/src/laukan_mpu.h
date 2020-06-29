#pragma once

#define X 0
#define Y 1
#define Z 2

typedef enum
{
    LAUKAN_MPU_RET_OK = 0,
    LAUKAN_MPU_RET_NOT_INITIALIZED,
    LAUKAN_MPU_RET_INVALID_PARAM,
    LAUKAN_MPU_RET_HW_FAILURE,
} ln_MPURet_e;

typedef struct
{
    float accel_mss[3];
    float gyro_rads[3];
    float mag_uT[3];
    float temp_C;
} ln_MPUSample_t;

ln_MPURet_e ln_MPUInit(void);
ln_MPURet_e ln_MPUSample(ln_MPUSample_t *pSample);