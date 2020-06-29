#include <Arduino.h>
#include <stdarg.h>
#include "laukan_debug_print.h"
#include "laukan_task_priorities.h"

// Would be better to use message buffers, but they are available on FreeFRTOS v10.0.0
#include "freertos/queue.h"

#define DEBUG_MAX_LENGTH 256
#define DEBUG_QUEUE_LEN 10

static const char gModeStrings[][5] = {
    "ERR ", "WARN", "INFO", "VERB"};

static ln_Debug_level_e gLevel = LN_DBG_INFO;
static QueueHandle_t gMessageQueue = NULL;
static bool gbInitialized = false;

static void dbgTask(void *arg);

ln_Debug_ret_e ln_Dbg_init(void)
{
    ln_Debug_ret_e ret = LN_DBG_RET_OK;

    Serial.begin(115200);

    gMessageQueue = xQueueCreate(DEBUG_QUEUE_LEN, DEBUG_MAX_LENGTH);
    if (gMessageQueue == NULL)
    {
        ret = LN_DBG_RET_OS_ERR;
    }

    if (ret == LN_DBG_RET_OK)
    {
        BaseType_t os_ret = xTaskCreate(dbgTask, "DBGTask", 2048, NULL, TASK_PRIORITY_DEBUG, NULL);
        if (os_ret != pdPASS)
        {
            ret = LN_DBG_RET_OS_ERR;
        }
    }

    if (ret == LN_DBG_RET_OK)
    {
        gbInitialized = true;
    }

    return ret;
}

static void dbgTask(void *arg)
{
    (void)arg;

    while (1)
    {
        char buffer[DEBUG_MAX_LENGTH];
        if (xQueueReceive(gMessageQueue, buffer, portMAX_DELAY))
        {
            Serial.print(buffer);
        }
    }
    vTaskDelete(NULL);
}

void ln_printf(const char *module, ln_Debug_level_e level, const char *format, ...)
{
    if (!gbInitialized)
    {
        return;
    }
    else if (level > gLevel)
    {
        va_list args;
        va_start(args, format);
        va_end(args);
        return;
    }

    char buffer[DEBUG_MAX_LENGTH];

    char *pBuffer = buffer;
    size_t bufferSize = sizeof(buffer);
    if (module != NULL)
    {
        int n = snprintf(pBuffer, bufferSize, "[%-10s %s] ", module, gModeStrings[level]);
        bufferSize -= n;
        pBuffer += n;
    }

    va_list args;
    va_start(args, format);

    vsnprintf(pBuffer, bufferSize, format, args);

    xQueueSend(gMessageQueue, buffer, 0);

    va_end(args);
}

void ln_DbgSetLevel(ln_Debug_level_e newLevel)
{
    if (newLevel < LN_NUM_OF_DEBUG_LEVELS)
    {
        gLevel = newLevel;
    }
}
