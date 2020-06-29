#include <Arduino.h>
#include "laukan_mpu.h"
#include "MPU9250.h"

static MPU9250 IMU(Wire, 0x68);
static bool gbInitialized = false;

ln_MPURet_e ln_MPUInit(void)
{
    ln_MPURet_e retVal = LAUKAN_MPU_RET_OK;
    int IMURet = IMU.begin();
    if (IMURet < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(IMURet);
        retVal = LAUKAN_MPU_RET_HW_FAILURE;
    }

    // setting the accelerometer full scale range to +/-8G
    if ((retVal == LAUKAN_MPU_RET_OK) &&
        (IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G) < 0))
    {
        retVal = LAUKAN_MPU_RET_HW_FAILURE;
        Serial.print("[MPU] Failed to set accelerometer range!\n");
    }
    if ((retVal == LAUKAN_MPU_RET_OK) &&
        (IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS) < 0))
    {
        retVal = LAUKAN_MPU_RET_HW_FAILURE;
        Serial.print("[MPU] Failed to set gyroscope range!\n");
    }
    if ((retVal == LAUKAN_MPU_RET_OK) &&
        (IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS) < 0))
    {
        retVal = LAUKAN_MPU_RET_HW_FAILURE;
        Serial.print("[MPU] Failed to set gyroscope range!\n");
    }
    if ((retVal == LAUKAN_MPU_RET_OK) &&
        (IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ) < 0))
    {
        retVal = LAUKAN_MPU_RET_HW_FAILURE;
        Serial.print("[MPU] Failed to set filter bandwidth!\n");
    }

    if ((retVal == LAUKAN_MPU_RET_OK) &&
        (IMU.setSrd(19) < 0))
    {
        retVal = LAUKAN_MPU_RET_HW_FAILURE;
        Serial.print("[MPU] Failed to set update rate!\n");
    }

    if (retVal == LAUKAN_MPU_RET_OK)
    {
        gbInitialized = true;
    }

    return retVal;
}

ln_MPURet_e ln_MPUSample(ln_MPUSample_t *pSample)
{
    ln_MPURet_e retVal = LAUKAN_MPU_RET_OK;

    if (!gbInitialized)
    {
        retVal = LAUKAN_MPU_RET_NOT_INITIALIZED;
    }
    else if (pSample == NULL)
    {
        retVal = LAUKAN_MPU_RET_INVALID_PARAM;
    }
    else
    {
        if (IMU.readSensor() < 0)
        {
            Serial.printf("[IMU] Failed to read data!\n");
            retVal = LAUKAN_MPU_RET_HW_FAILURE;
        }
        else
        {
            pSample->accel_mss[X] = IMU.getAccelX_mss();
            pSample->accel_mss[Y] = IMU.getAccelY_mss();
            pSample->accel_mss[Z] = IMU.getAccelZ_mss();
            pSample->gyro_rads[X] = IMU.getGyroX_rads();
            pSample->gyro_rads[Y] = IMU.getGyroY_rads();
            pSample->gyro_rads[Z] = IMU.getGyroZ_rads();
            pSample->mag_uT[X] = IMU.getMagX_uT();
            pSample->mag_uT[Y] = IMU.getMagY_uT();
            pSample->mag_uT[Z] = IMU.getMagZ_uT();
            pSample->temp_C = IMU.getTemperature_C();
        }
    }

    return retVal;
}
