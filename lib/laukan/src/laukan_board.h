#pragma once
#include "esp_system.h"
#include "driver/gpio.h"

#define LAUKAN_CAN_IOPIN_TX GPIO_NUM_5
#define LAUKAN_CAN_IOPIN_RX GPIO_NUM_4

#define LAUKAN_I2C_IOPIN_SDA GPIO_NUM_21
#define LAUKAN_I2C_IOPIN_CLK GPIO_NUM_22

#define LAUKAN_KLINE_INVERT 1
#define LAUKAN_KLINE_IOPIN_RX GPIO_NUM_9
#define LAUKAN_KLINE_IOPIN_TX GPIO_NUM_10
