#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"

#include "laukan_can.h"
#include "laukan_debug_print.h"
#include "laukan_task_priorities.h"
#include "OBDII.h"
#include "OBDIICommunication.h"

// ****************************************************************************
// *                     CONSTANTS AND TYPE DEFINITIONS                       *
// ****************************************************************************
#define NUM_OF_RECEIVE_MESSAGES_TO_BUFFER 10
#define TRANSMIT_TIMEOUT_MS 50
#define RESPONSE_TIMEOUT_MS 100

// ****************************************************************************
// *                                GLOBALS                                   *
// ****************************************************************************
static bool gbInitialized = false;
static ln_canProtocolHandle gProtocolHandle;
static const char gModule[] = "LEAF";

static uint32_t gAliveTimestamp_ms = 0;
static bool gFirstTime = true;

// ****************************************************************************
// *                     INTERNAL FUNCTIONS PROTOTYPES                        *
// ****************************************************************************
static void ProtocolTask(void *arg);

// ****************************************************************************
// *                         FUNCTION DEFINITIONS                             *
// ****************************************************************************
ln_CANRet_e leafCANInit()
{
    gFirstTime = true;
    ln_CANRet_e ret = LAUKAN_CAN_RET_OK;

    Serial.printf("leafCANInit calling canStart\n");

    ret = ln_canStart(LAUKAN_CAN_BAUD_500K, 1000);

    const uint32_t CANIDMask = 0x00;
    const uint32_t CANIDValue = CANIDMask;
    ln_CANRet_e canRet = ln_canRegisterProtocol(&gProtocolHandle,
                                                NUM_OF_RECEIVE_MESSAGES_TO_BUFFER,
                                                CANIDMask,
                                                CANIDValue);
    if (canRet != LAUKAN_CAN_RET_OK)
    {
        ln_printf(gModule, LN_DBG_ERROR, "Failed to register Leaf protocol (%d)!\n", canRet);
    }
    else
    {
        if (xTaskCreatePinnedToCore(ProtocolTask,
                                    "KLineProtocol",
                                    4096,
                                    NULL,
                                    TASK_PRIORITY_LEAF_PROTOCOL,
                                    NULL,
                                    TASK_CORE_LEAF_PROTOCOL) != pdPASS)
        {
            ln_printf(gModule, LN_DBG_ERROR, "Failed to create OS task!\n");
            ret = LAUKAN_CAN_RET_LOW_LEVEL_ERROR;
        }
    }

    return ret;
}

static void ProtocolTask(void *arg)
{
    uint8_t counter = 0;
    (void)arg;
    while (1)
    {
        can_message_t rxMessage;
        ln_CANRet_e canRet = ln_canReceive(gProtocolHandle, &rxMessage, 1000);
        if (canRet == LAUKAN_CAN_RET_OK)
        {
            if (rxMessage.identifier == 0x601)
            {
                uint32_t now_ms = millis();
                uint32_t diff_ms = (now_ms - gAliveTimestamp_ms);
                if (gFirstTime || ((now_ms - gAliveTimestamp_ms) > 3000))
                {
                    vTaskDelay(pdMS_TO_TICKS(6300));
                    Serial.printf("Sending RADIO button, diff %d ms\n", diff_ms);
                    gFirstTime = false;
                    /* 
                    1	0	155	RX	329930750		0x00000681	8	0x04 20 40 0d a3 ff ff ff 	
                    1	0	157	RX	329930850		0x00000681	8	0x04 30 40 0d a3 ff ff ff 	
                    1	0	159	RX	329930900		0x00000681	8	0x04 40 40 0d 23 ff ff ff 	
*/
                    can_message_t txMessage;
                    txMessage.identifier = 0x681;
                    txMessage.flags = CAN_MSG_FLAG_NONE;
                    txMessage.data_length_code = 8;
                    txMessage.data[0] = 0x04;
                    txMessage.data[1] = 0x20;
                    txMessage.data[2] = 0x40;
                    txMessage.data[3] = 0x0D;
                    txMessage.data[4] = 0xA3;
                    txMessage.data[5] = 0xFF;
                    txMessage.data[6] = 0xFF;
                    txMessage.data[7] = 0xFF;

                    canRet = ln_canSend(&txMessage, 50);
                    if (canRet != LAUKAN_CAN_RET_OK)
                    {
                        ln_printf(gModule, LN_DBG_ERROR, "Failed to send first CAN message (%d)!\n", canRet);
                    }

                    vTaskDelay(pdMS_TO_TICKS(10));

                    txMessage.data[0] = 0x04;
                    txMessage.data[1] = 0x40;
                    txMessage.data[2] = 0x40;
                    txMessage.data[3] = 0x0D;
                    txMessage.data[4] = 0x23;

                    canRet = ln_canSend(&txMessage, 50);
                    if (canRet != LAUKAN_CAN_RET_OK)
                    {
                        ln_printf(gModule, LN_DBG_ERROR, "Failed to send second CAN message (%d)!\n", canRet);
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));

                    txMessage.data[0] = 0x04;
                    txMessage.data[1] = 0x30;
                    txMessage.data[2] = 0x40;
                    txMessage.data[3] = 0x0D;
                    txMessage.data[4] = 0xA3;

                    canRet = ln_canSend(&txMessage, 50);
                    if (canRet != LAUKAN_CAN_RET_OK)
                    {
                        ln_printf(gModule, LN_DBG_ERROR, "Failed to send second CAN message (%d)!\n", canRet);
                    }
                }
                gAliveTimestamp_ms = millis();
            }
        }
    }
}