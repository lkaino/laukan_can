// ****************************************************************************
// *                               INCLUDES                                   *
// ****************************************************************************
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "laukan_debug_print.h"
#include "laukan_kline.h"
#include "laukan_kline_protocol.h"
#include "laukan_task_priorities.h"

// ****************************************************************************
// *                     CONSTANTS AND TYPE DEFINITIONS                       *
// ****************************************************************************
#define EVENT_START_AUTO BIT0
#define FRAME_TIME_ON_BAUD(_bytes, _baud) (((90000 / _baud) * _bytes + 10) / 10)
#define ISO14230_MAX_HEADER_LEN 4
#define ISO14230_CHECKSUM_LEN 1
#define ISO14230_MAX_FRAME_SIZE(_payloadLength) (ISO14230_MAX_HEADER_LEN + _payloadLength + ISO14230_CHECKSUM_LEN)

#define ISO14230_START_COM_SERVICE_ID 0x81
#define ISO14230_TESTER_ADDRESS 0xF1
#define ISO14230_ECU_ADDRESS 0x33

// ****************************************************************************
// *                                GLOBALS                                   *
// ****************************************************************************
static ln_kl_prot_e gActiveProtocol = LN_KLINE_PROT_NUM_OF;
static EventGroupHandle_t gProtocolEvent;
static bool gbInitialized = false;
static const char gModule[] = "KLINEPROT";

// ****************************************************************************
// *                     INTERNAL FUNCTIONS PROTOTYPES                        *
// ****************************************************************************
static void ProtocolTask(void *arg);
static ln_kl_ret_e ProtocolInit(void);
static char CalculateChecksum(char *pData, uint32_t len);
static uint32_t ISO14230ConstructFrame(char *pData,
                                       uint32_t maxLen,
                                       char source,
                                       char target,
                                       const char *pPayload,
                                       uint32_t payloadLen);

// ****************************************************************************
// *                         FUNCTION DEFINITIONS                             *
// ****************************************************************************
ln_kl_ret_e ln_klProtocolStart(void)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (!gbInitialized)
    {
        ret = ProtocolInit();
    }

    return ret;
}

ln_kl_ret_e ProtocolInit(void)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    gProtocolEvent = xEventGroupCreate();
    if (gProtocolEvent == NULL)
    {
        ln_printf(gModule, LN_DBG_ERROR, "Failed to create OS event!\n");
        ret = LAUKAN_KL_RET_OS_ERROR;
    }

    if (ret == LAUKAN_KL_RET_OK)
    {
        if (xTaskCreatePinnedToCore(ProtocolTask,
                                    "KLineProtocol",
                                    4096,
                                    NULL,
                                    TASK_PRIORITY_KLINE_PROTOCOL,
                                    NULL,
                                    TASK_CORE_KLINE_PROTOCOL) != pdPASS)
        {
            ln_printf(gModule, LN_DBG_ERROR, "Failed to create OS task!\n");
            ret = LAUKAN_KL_RET_OS_ERROR;
        }
    }

    return ret;
}

static void ProtocolTask(void *arg)
{
    (void)arg;
    while (1)
    {
        EventBits_t events = xEventGroupWaitBits(gProtocolEvent,
                                                 EVENT_START_AUTO,
                                                 pdTRUE, pdFALSE, portMAX_DELAY);

        if (events & EVENT_START_AUTO)
        {
        }
    }
}

//! Bus "high" idle time prior to transmission of address byte
#define ISO9141_W0_MIN_MS 2
//! Time from the end of the address byte to the start of synchronization
#define ISO9141_W1_MAX_MS 300
//! Time from the end of the sync pattern to start start of key word 1
#define ISO9141_W2_MAX_MS 20
//! Time between key word 1 and 2
#define ISO9141_W3_MAX_MS 20
//! Time between key word 2 received and the inversion being returned by tester
#define ISO9141_W4_MIN_MS 25
#define ISO9141_W4_MAX_MS 50
//! Time that bus must remain idle before retransmitting the address byte
#define ISO9141_W5_MIN_MS 300

#define ISO9141_INIT_ADDRESS 0x33
#define ISO9141_SYNC 0x55

#define ISO9141_BAUD_INIT 5
#define ISO9141_BAUD_NORMAL 10400
#define ISO14230_BAUD_NORMAL 10400

//! Interbyte time for messages from the vehicle to the tester
#define ISO9141_TIMING_P1_MIN_MS 0
#define ISO9141_TIMING_P1_MAX_MS 20
//! Intermessage time for vehicles with key word 0x94
#define ISO9141_TIMING_P2_94H_MIN_MS 0
#define ISO9141_TIMING_P2_94H_MAX_MS 50
//! Intermessage time for vehicles with key word 0x08
#define ISO9141_TIMING_P2_08H_MIN_MS 25
#define ISO9141_TIMING_P2_08H_MAX_MS 50
//! Intermessage time between the end of all vehicle-sourced responses and the start of the the next tester request
#define ISO9141_TIMING_P3_MIN_MS 55
#define ISO9141_TIMING_P3_MAX_MS 5000
//! Interbyte time for messages from the tester to the vehicle
#define ISO9141_TIMING_P4_MIN_MS 5
#define ISO9141_TIMING_P4_MAX_MS 20

static ln_kl_ret_e ISO9141InitReceiveByte(const uint32_t baud,
                                          const uint32_t time_ms,
                                          char *pReceivedByte,
                                          const char *pNameOfByte,
                                          const char *pExpectedValue)
{
    uint32_t len = 1;
    char byte;
    if (pReceivedByte == NULL)
    {
        pReceivedByte = &byte;
    }
    ln_kl_ret_e ret = ln_klReceive(pReceivedByte, &len, baud,
                                   time_ms + FRAME_TIME_ON_BAUD(len, baud));
    if (ret == LAUKAN_KL_RET_OK)
    {
        if (len != 1)
        {
            ln_printf(gModule, LN_DBG_INFO, "ISO9141 Initialization: did not receive %s.\n", pNameOfByte);
            ret = LAUKAN_KL_RET_COMM_ERROR;
        }
        else if ((pExpectedValue != NULL) && (*pExpectedValue != *pReceivedByte))
        {
            ln_printf(gModule, LN_DBG_INFO, "ISO9141 Initialization: Value for received %s was not expected 0x%0X != 0x%0X.\n",
                      pNameOfByte,
                      *pReceivedByte,
                      *pExpectedValue);
            ret = LAUKAN_KL_RET_COMM_ERROR;
        }
    }

    return ret;
}

static ln_kl_ret_e StartCommunication(ln_kl_prot_e protocol)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;
    if (protocol == LN_KLINE_PROT_ISO9141)
    {
        ln_printf(gModule, LN_DBG_INFO, "ISO9141: Starting communication.\n");
        {
            char byte = ISO9141_INIT_ADDRESS;
            ret = ln_klSend(&byte, sizeof(byte), ISO9141_BAUD_INIT, 0);
        }

        if (ret == LAUKAN_KL_RET_OK)
        {
            const char expectedValue = ISO9141_SYNC;
            ret = ISO9141InitReceiveByte(ISO9141_BAUD_NORMAL, ISO9141_W1_MAX_MS, NULL, "SYNC", &expectedValue);
        }

        char keyWord;
        if (ret == LAUKAN_KL_RET_OK)
        {
            ret = ISO9141InitReceiveByte(ISO9141_BAUD_NORMAL, ISO9141_W2_MAX_MS, &keyWord, "Key word 1", NULL);
            if (ret == LAUKAN_KL_RET_OK)
            {
                ret = ISO9141InitReceiveByte(ISO9141_BAUD_NORMAL, ISO9141_W3_MAX_MS, NULL, "Key word 2", &keyWord);
            }
        }

        if (ret == LAUKAN_KL_RET_OK)
        {
            ln_printf(gModule, LN_DBG_INFO, "ISO9141: Received Key words 0x%0X.\n", keyWord);
            vTaskDelay(pdMS_TO_TICKS(ISO9141_W4_MIN_MS));
            const char byte = ~keyWord;
            ret = ln_klSend(&byte, sizeof(byte), ISO9141_BAUD_NORMAL, 0);
        }

        if (ret == LAUKAN_KL_RET_OK)
        {
            const char expectedReply = ~ISO9141_INIT_ADDRESS;
            ret = ISO9141InitReceiveByte(ISO9141_BAUD_NORMAL, ISO9141_W4_MAX_MS, NULL, "Ready to communicate", &expectedReply);
        }
    }
    else if (protocol == LN_KLINE_PROT_ISO14230_KWP_FAST)
    {
        ln_printf(gModule, LN_DBG_INFO, "ISO14230_KWP_FAST: Starting communication.\n");
        ret = ln_klSetPinState(0, 25);
        if (ret == LAUKAN_KL_RET_OK)
        {
            ret = ln_klSetPinState(1, 25);
        }
        if (ret == LAUKAN_KL_RET_OK)
        {

            const char payload = ISO14230_START_COM_SERVICE_ID;
            char data[ISO14230_MAX_FRAME_SIZE(sizeof(payload))];
            const uint32_t len = ISO14230ConstructFrame(data, sizeof(data), ISO14230_TESTER_ADDRESS, ISO14230_ECU_ADDRESS, payload, sizeof(payload));
            if (len == 0)
            {
                ret = LAUKAN_KL_RET_COMM_ERROR;
            }
            else
            {
                ret = ln_klSend(data, len, ISO14230_BAUD_NORMAL, ISO9141_TIMING_P4_MIN_MS);
            }
        }

        if (ret == LAUKAN_KL_RET_OK)
        {
            //const char resoibse;
            //ret = ln_klReceive();
        }
    }
}

static uint32_t ISO14230ConstructFrame(char *pData,
                                       uint32_t maxLen,
                                       char source,
                                       char target,
                                       const char *pPayload,
                                       uint32_t payloadLen)
{
    uint32_t bytes = 0;
    bool bSuccess = true;

    if ((pData == NULL) || (pPayload == NULL))
    {
        bSuccess = false;
    }

    if (bSuccess)
    {
        const uint32_t bufferSizeRequired = ISO14230_MAX_FRAME_SIZE(payloadLen);
        if (maxLen < bufferSizeRequired)
        {
            bSuccess = false;
            ln_printf(gModule, LN_DBG_ERROR, "Too small buffer for ISO14230 frame (%d / %d bytes).\n", maxLen, bufferSizeRequired);
        }
    }

    if (bSuccess)
    {
        // Format byte
        char byte = (BIT0 | BIT1) << 6;
        bool bUseLengthByte = false;
        if (payloadLen < 64)
        {
            byte |= payloadLen;
        }
        else
        {
            bUseLengthByte = true;
        }

        pData[bytes++] = byte;
        pData[bytes++] = target;
        pData[bytes++] = source;

        if (bUseLengthByte)
        {
            pData[bytes++] = payloadLen;
        }

        memcpy(pData[bytes], pPayload, payloadLen);
        bytes += payloadLen;
        pData[bytes++] = CalculateChecksum(pData, bytes);
    }

    return bytes;
}

static char CalculateChecksum(char *pData, uint32_t len)
{
    char checksum = 0;
    if (pData != NULL)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            checksum += pData[i];
        }
    }
    return checksum;
}