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
#include "OBDIICommunication.h"

void setup()
{
  Serial.begin(115200);
  sd_init();
  laukan_canInit(3);
  OBDIIInit();

  // put your setup code here, to run once:
}

void loop()
{
  laukan_canStart(LAUKAN_CAN_BAUD_500K, 1000);
  vTaskDelay(pdMS_TO_TICKS(10000));
  laukan_canStop(1000);
  // put your main code here, to run repeatedly:
}