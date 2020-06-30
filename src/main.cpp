#include <Arduino.h>
#include <driver/can.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "sd.h"
#include "laukan_can.h"
#include "laukan_mpu.h"
#include "laukan_kline.h"
#include "laukan_debug_print.h"
#include "OBDIICommunication.h"

#include "leaf_can.h"

void setup()
{
  ln_Dbg_init();
  //sd_init();
  ln_canInit(3);

  Serial.printf("after caninit\n");
  leafCANInit();
  //OBDIIInit();
  //ln_MPUInit();
  //ln_MPUInit();
  //ln_klInit();

  // put your setup code here, to run once:
}

void loop()
{ /*
  ln_klSetPinState(0, 200);
  ln_klSetPinState(1, 400);
  ln_klSetPinState(0, 400);
  ln_klSetPinState(1, 400);
  ln_klSetPinState(0, 200);
  ln_klSetPinState(1, 400);

  char data[6];
  data[0] = 1;
  data[1] = 2;
  data[2] = 3;
  data[3] = 4;
  data[4] = 5;
  data[5] = 6;
  ln_klSend(data, sizeof(data), 10400, 5);
  ln_klSend(data, sizeof(data), 9600, 5);

  ln_printf(NULL, LN_DBG_INFO, "portTICK_RATE_MS %d\n", portTICK_RATE_MS);*/
  //ln_canStart(LAUKAN_CAN_BAUD_500K, 1000);
  //vTaskDelay(pdMS_TO_TICKS(10000));
  //ln_canStop(1000);
  // put your main code here, to run repeatedly:
  /*ln_MPUSample_t mpuSample;
  if (ln_MPUSample(&mpuSample) == LAUKAN_MPU_RET_OK)
  {
    Serial.printf("acc (% 03.1f, % 03.1f, % 03.1f) gyro (% 03.1f, % 03.1f, % 03.1f) mag (% 03.1f, % 03.1f, % 03.1f) temp % 03.1f\n",
                  mpuSample.accel_mss[X],
                  mpuSample.accel_mss[Y],
                  mpuSample.accel_mss[Z],
                  mpuSample.gyro_rads[X],
                  mpuSample.gyro_rads[Y],
                  mpuSample.gyro_rads[Z],
                  mpuSample.mag_uT[X],
                  mpuSample.mag_uT[Y],
                  mpuSample.mag_uT[Z],
                  mpuSample.temp_C);
  }*/
}