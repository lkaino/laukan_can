// ****************************************************************************
// *                               INCLUDES                                   *
// ****************************************************************************
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"

#include "laukan_board.h"
#include "laukan_task_priorities.h"
#include "laukan_can.h"

// ****************************************************************************
// *                     CONSTANTS AND TYPE DEFINITIONS                       *
// ****************************************************************************
#define ENABLED_ALERTS (CAN_ALERT_BUS_RECOVERED | \
                        CAN_ALERT_BUS_OFF)

#define LAUKAN_CAN_EVENT_START BIT0
#define LAUKAN_CAN_EVENT_STOP BIT1
#define LAUKAN_CAN_EVENT_STARTED BIT2
#define LAUKAN_CAN_EVENT_STOPPED BIT3

typedef enum
{
    LAUKAN_CAN_STATE_UNITIALIZED = 0,
    LAUKAN_CAN_STATE_STOPPED,
    LAUKAN_CAN_STATE_OPERATIONAL,
    LAUKAN_CAN_STATE_BUSOFF
} laukan_CANState_e;

typedef struct
{
    uint32_t CANIDAcceptanceMask;
    uint32_t CANIDAcceptanceValue;
    QueueHandle_t RXQueue;
} laukan_CANProtocol_t;

// ****************************************************************************
// *                                GLOBALS                                   *
// ****************************************************************************
static can_general_config_t gCANConfig;
static can_timing_config_t gCANTimingConfig;
static const can_filter_config_t gCANFilterConfig = CAN_FILTER_CONFIG_ACCEPT_ALL();

static SemaphoreHandle_t gCANMutex;
static EventGroupHandle_t gCANControlEvent;

static laukan_CANState_e gCANState = LAUKAN_CAN_STATE_UNITIALIZED;

static laukan_CANProtocol_t *gpProtocols;
static uint32_t gMaxNumOfProcotols;
static uint32_t gNumOfProtocols;

// ****************************************************************************
// *                     INTERNAL FUNCTIONS PROTOTYPES                        *
// ****************************************************************************
static void ReceiveTask(void *arg);
static void ControlTask(void *arg);
static void StatusTask(void *arg);

// ****************************************************************************
// *                         FUNCTION DEFINITIONS                             *
// ****************************************************************************
static void ReceiveTask(void *arg)
{
    (void)arg;
    while (1)
    {
        can_message_t message;
        esp_err_t err = can_receive(&message, portMAX_DELAY);
        if (err == ESP_OK)
        {
            for (uint32_t i = 0; i < gNumOfProtocols; i++)
            {
                laukan_CANProtocol_t *pProtocol = &gpProtocols[i];
                if ((message.identifier & pProtocol->CANIDAcceptanceMask) == pProtocol->CANIDAcceptanceValue)
                {
                    if (xQueueSend(gpProtocols[i].RXQueue, &message, 0) == errQUEUE_FULL)
                    {
                        Serial.printf("Receive buffer full for protocol with handle %u\n", i);
                    }
                }
            }
        }
        else
        {
            Serial.printf("can_receive failed (%d).\n", err);
        }
    }
    vTaskDelete(NULL);
}

static void ControlTask(void *arg)
{
    (void)arg;
    while (1)
    {
        EventBits_t events = xEventGroupWaitBits(gCANControlEvent,
                                                 LAUKAN_CAN_EVENT_START |
                                                     LAUKAN_CAN_EVENT_STOP,
                                                 pdTRUE, pdFALSE, portMAX_DELAY);
        if (xSemaphoreTake(gCANMutex, portMAX_DELAY))
        {
            if ((events & LAUKAN_CAN_EVENT_STOP) &&
                (gCANState >= LAUKAN_CAN_STATE_OPERATIONAL))
            {
                Serial.printf("Stopping CAN.\n");
                if (can_stop() != ESP_OK)
                {
                    Serial.printf("Failed to stop the CAN driver!\n");
                }
                // Empty the receive buffer
                Serial.printf("Empty CAN receive buffer.\n");
                can_message_t message;
                while (can_receive(&message, 0) == ESP_OK)
                {
                }

                Serial.printf("Uninstall CAN driver.\n");
                if (can_driver_uninstall() != ESP_OK)
                {
                    Serial.printf("Failed to uninstall the CAN driver!\n");
                }

                gCANState = LAUKAN_CAN_STATE_STOPPED;
                xEventGroupSetBits(gCANControlEvent, LAUKAN_CAN_EVENT_STOPPED);
            }
            if ((events & LAUKAN_CAN_EVENT_START) &&
                (gCANState == LAUKAN_CAN_STATE_STOPPED))
            {
                Serial.printf("Installing CAN driver.\n");
                if (can_driver_install(&gCANConfig, &gCANTimingConfig, &gCANFilterConfig) != ESP_OK)
                {
                    Serial.printf("Failed to install CAN driver!\n");
                }
                else
                {
                    Serial.printf("Starting CAN.\n");
                    if (can_start() != ESP_OK)
                    {
                        Serial.printf("Failed to start CAN driver!\n");
                        can_driver_uninstall();
                    }
                    else
                    {
                        Serial.printf("CAN started.\n");
                        gCANState = LAUKAN_CAN_STATE_OPERATIONAL;
                        xEventGroupSetBits(gCANControlEvent, LAUKAN_CAN_EVENT_STARTED);
                    }
                }
            }

            if (xSemaphoreGive(gCANMutex) == pdFALSE)
            {
                Serial.printf("Failed to give CAN mutex!\n");
            }
        }
        else
        {
            Serial.printf("Failed to take CAN mutex!\n");
        }
    }

    vTaskDelete(NULL);
}

static void StatusTask(void *arg)
{
    (void)arg;
    while (1)
    {
        uint32_t alerts = 0;
        esp_err_t err = can_read_alerts(&alerts, portMAX_DELAY);
        if (err == ESP_OK)
        {
            if (xSemaphoreTake(gCANMutex, portMAX_DELAY))
            {
                if (alerts & CAN_ALERT_BUS_OFF)
                {
                    Serial.printf("CAN in Bus Off state.\n");
                    gCANState = LAUKAN_CAN_STATE_BUSOFF;
                    can_initiate_recovery();
                }
                else if (alerts & CAN_ALERT_BUS_RECOVERED)
                {
                    Serial.printf("CAN bus recovered, restarting.\n");
                    gCANState = LAUKAN_CAN_STATE_OPERATIONAL;
                    can_start();
                }
                xSemaphoreGive(gCANMutex);
            }
        }
        else
        {
            Serial.printf("can_read_alerts failed (%d)\n", err);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

laukan_CANRet_e laukan_canInit(uint32_t maxNumOfProtocols)
{
    laukan_CANRet_e ret = LAUKAN_CAN_RET_OK;
    if (gCANState != LAUKAN_CAN_STATE_UNITIALIZED)
    {
        ret = LAUKAN_CAN_RET_ALREADY_INITIALIZED;
    }

    if (ret == LAUKAN_CAN_RET_OK)
    {
        memset(&gpProtocols, 0, sizeof(gpProtocols));
        gpProtocols = (laukan_CANProtocol_t *)malloc(maxNumOfProtocols * sizeof(laukan_CANProtocol_t));
        if (gpProtocols == NULL)
        {
            ret = LAUKAN_CAN_RET_NO_MEM;
        }
        else
        {
            gMaxNumOfProcotols = maxNumOfProtocols;
        }
    }

    if (ret == LAUKAN_CAN_RET_OK)
    {
        gCANConfig.mode = CAN_MODE_NORMAL;
        gCANConfig.tx_io = LAUKAN_CAN_IOPIN_TX;
        gCANConfig.rx_io = LAUKAN_CAN_IOPIN_RX;
        gCANConfig.clkout_io = (gpio_num_t)CAN_IO_UNUSED;
        gCANConfig.bus_off_io = (gpio_num_t)CAN_IO_UNUSED;
        gCANConfig.tx_queue_len = 5;
        gCANConfig.rx_queue_len = 5;
        gCANConfig.alerts_enabled = ENABLED_ALERTS;
        gCANConfig.clkout_divider = 0;

        gCANControlEvent = xEventGroupCreate();
        gCANMutex = xSemaphoreCreateMutex();

        xTaskCreatePinnedToCore(StatusTask, "CANStatus", 4096, NULL, TASK_PRIORITY_CAN_STATUS, NULL, TASK_CORE_CAN);
        xTaskCreatePinnedToCore(ControlTask, "CANControl", 4096, NULL, TASK_PRIORITY_CAN_CONTROL, NULL, TASK_CORE_CAN);
        xTaskCreatePinnedToCore(ReceiveTask, "CANReceive", 4096, NULL, TASK_PRIORITY_CAN_RX, NULL, TASK_CORE_CAN);
        gCANState = LAUKAN_CAN_STATE_STOPPED;
    }

    return ret;
}

laukan_CANRet_e laukan_canStart(laukan_CANBaud_e baud, uint32_t wait_ms)
{
    laukan_CANRet_e ret = LAUKAN_CAN_RET_OK;
    if (gCANState != LAUKAN_CAN_STATE_UNITIALIZED)
    {
        //can_timing_config_t newConfig = CAN_TIMING_CONFIG_25KBITS();
        switch (baud)
        {
        case LAUKAN_CAN_BAUD_25K:
            gCANTimingConfig = CAN_TIMING_CONFIG_25KBITS();
            break;
        case LAUKAN_CAN_BAUD_50K:
            gCANTimingConfig = CAN_TIMING_CONFIG_50KBITS();
            break;
        case LAUKAN_CAN_BAUD_100K:
            gCANTimingConfig = CAN_TIMING_CONFIG_100KBITS();
            break;
        case LAUKAN_CAN_BAUD_125K:
            gCANTimingConfig = CAN_TIMING_CONFIG_125KBITS();
            break;
        case LAUKAN_CAN_BAUD_250K:
            gCANTimingConfig = CAN_TIMING_CONFIG_250KBITS();
            break;
        case LAUKAN_CAN_BAUD_500K:
            gCANTimingConfig = CAN_TIMING_CONFIG_500KBITS();
            break;
        case LAUKAN_CAN_BAUD_800K:
            gCANTimingConfig = CAN_TIMING_CONFIG_800KBITS();
            break;
        case LAUKAN_CAN_BAUD_1M:
            gCANTimingConfig = CAN_TIMING_CONFIG_1MBITS();
            break;
        default:
            ret = LAUKAN_CAN_RET_INVALID_PARAM;
            break;
        }

        if (ret == LAUKAN_CAN_RET_OK)
        {
            xEventGroupClearBits(gCANControlEvent, LAUKAN_CAN_EVENT_STARTED);
            xEventGroupSetBits(gCANControlEvent, LAUKAN_CAN_EVENT_START);
            if (wait_ms)
            {
                EventBits_t events = xEventGroupWaitBits(gCANControlEvent, LAUKAN_CAN_EVENT_STARTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(wait_ms));
                if (!(events & LAUKAN_CAN_EVENT_STARTED))
                {
                    ret = LAUKAN_CAN_RET_TIMEOUT;
                }
            }
        }
    }
    else
    {
        ret = LAUKAN_CAN_RET_NOT_INITIALIZED;
    }

    return ret;
}

laukan_CANRet_e laukan_canStop(uint32_t wait_ms)
{
    laukan_CANRet_e ret = LAUKAN_CAN_RET_OK;
    if (gCANState == LAUKAN_CAN_STATE_UNITIALIZED)
    {
        ret = LAUKAN_CAN_RET_NOT_INITIALIZED;
    }
    else if (gCANState != LAUKAN_CAN_STATE_STOPPED)
    {
        xEventGroupClearBits(gCANControlEvent, LAUKAN_CAN_EVENT_STOPPED);
        xEventGroupSetBits(gCANControlEvent, LAUKAN_CAN_EVENT_STOP);
        if (wait_ms)
        {
            EventBits_t events = xEventGroupWaitBits(gCANControlEvent, LAUKAN_CAN_EVENT_STOPPED, pdTRUE, pdFALSE, pdMS_TO_TICKS(wait_ms));
            if (!(events & LAUKAN_CAN_EVENT_STOPPED))
            {
                Serial.printf("CAN stop timed out.");
                ret = LAUKAN_CAN_RET_TIMEOUT;
            }
        }
    }
    return ret;
}

laukan_CANRet_e laukan_canRegisterProtocol(laukan_canProtocolHandle *pHandle,
                                           uint32_t numOfMessagesToBuffer,
                                           uint32_t CANIDAcceptanceMask,
                                           uint32_t CANIDAcceptanceValue)
{
    laukan_CANRet_e ret = LAUKAN_CAN_RET_OK;

    if ((pHandle == NULL) || (numOfMessagesToBuffer == 0))
    {
        ret = LAUKAN_CAN_RET_INVALID_PARAM;
    }
    else if (gCANState == LAUKAN_CAN_STATE_UNITIALIZED)
    {
        ret = LAUKAN_CAN_RET_NOT_INITIALIZED;
    }
    else if (xSemaphoreTake(gCANMutex, portMAX_DELAY))
    {
        if (gNumOfProtocols == gMaxNumOfProcotols)
        {
            *pHandle = gNumOfProtocols;
            ret = LAUKAN_CAN_RET_NO_SLOTS;
        }
        else
        {
            QueueHandle_t queueHandle = xQueueCreate(numOfMessagesToBuffer, sizeof(can_message_t));
            if (queueHandle == NULL)
            {
                // Set to max num so the handle can't be used
                *pHandle = gMaxNumOfProcotols;
                ret = LAUKAN_CAN_RET_NO_MEM;
            }
            else
            {
                *pHandle = gNumOfProtocols;
                gpProtocols[gNumOfProtocols].RXQueue = queueHandle;
                gNumOfProtocols++;
            }
        }
        xSemaphoreGive(gCANMutex);
    }

    return ret;
}

laukan_CANRet_e laukan_canReceive(laukan_canProtocolHandle handle,
                                  can_message_t *pMessage,
                                  uint32_t timeout_ms)
{
    laukan_CANRet_e ret = LAUKAN_CAN_RET_OK;

    if ((pMessage == NULL) ||
        (handle >= gNumOfProtocols))
    {
        ret = LAUKAN_CAN_RET_INVALID_PARAM;
    }
    else if (gCANState == LAUKAN_CAN_STATE_UNITIALIZED)
    {
        ret = LAUKAN_CAN_RET_NOT_INITIALIZED;
    }
    else
    {
        if (!xQueueReceive(gpProtocols[handle].RXQueue,
                           pMessage,
                           pdMS_TO_TICKS(timeout_ms)))
        {
            ret = LAUKAN_CAN_RET_TIMEOUT;
        }
    }

    return ret;
}

laukan_CANRet_e laukan_canSend(const can_message_t *pMessage,
                               uint32_t timeout_ms)
{
    laukan_CANRet_e ret = LAUKAN_CAN_RET_OK;

    if (pMessage == NULL)
    {
        ret = LAUKAN_CAN_RET_INVALID_PARAM;
    }
    else if (gCANState == LAUKAN_CAN_STATE_UNITIALIZED)
    {
        ret = LAUKAN_CAN_RET_NOT_INITIALIZED;
    }
    else
    {
        esp_err_t espErr = can_transmit(pMessage, pdMS_TO_TICKS(timeout_ms));
        switch (espErr)
        {
        case ESP_OK:
            break;
        case ESP_ERR_TIMEOUT:
            ret = LAUKAN_CAN_RET_TIMEOUT;
            break;
        default:
            ret = LAUKAN_CAN_RET_LOW_LEVEL_ERROR;
            break;
        }
    }

    return ret;
}