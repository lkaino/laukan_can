// ****************************************************************************
// *                               INCLUDES                                   *
// ****************************************************************************
#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "laukan_board.h"
#include "laukan_kline.h"
#include "laukan_debug_print.h"

// ****************************************************************************
// *                     CONSTANTS AND TYPE DEFINITIONS                       *
// ****************************************************************************
#define KLINE_TX_BUF_SIZE 0
#define KLINE_RX_BUF_SIZE 1024
#define KLINE_DEFAULT_BAUD 10400
#define KLINE_UART UART_NUM_1

typedef struct
{
    bool bWithAddress;
    bool bFunctionalAddressing;
    bool bExceptionMode;
    bool bWithLengthField;
    uint8_t target;
    uint8_t source;
    uint8_t payloadLen;
    uint8_t payloadOffset;
    bool parseSuccess;
} ISO14230_header_t;

typedef enum
{
    KL_STATE_UNITIALIZED = 0,
    KL_STATE_UART_STOPPED,
    KL_STATE_UART_STARTED,
} ln_kl_state_e;

// ****************************************************************************
// *                                GLOBALS                                   *
// ****************************************************************************
static const char gModule[] = "KLINE";
static ln_kl_state_e gState = KL_STATE_UNITIALIZED;

// ****************************************************************************
// *                     INTERNAL FUNCTIONS PROTOTYPES                        *
// ****************************************************************************
static void KLineTask(void *arg);
static ln_kl_ret_e startUART(uint32_t baud);
static ln_kl_ret_e stopUART();
static ln_kl_ret_e checkBaud(uint32_t baud);
static ln_kl_ret_e configureGPIO();

// ****************************************************************************
// *                         FUNCTION DEFINITIONS                             *
// ****************************************************************************
ln_kl_ret_e ln_klInit(void)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;
    /*if (ret == LAUKAN_KL_RET_OK)
    {
        BaseType_t os_ret = xTaskCreate(KLineTask, "KLineTask", 2048, NULL, 10, NULL);
        if (os_ret != pdPASS)
        {
            ln_printf(gModule, LN_DBG_ERROR, "xTaskCreate failed %d\n", os_ret);
            ret = LAUKAN_KL_RET_OS_ERROR;
        }
    }*/
    gState = KL_STATE_UART_STOPPED;

    ret = configureGPIO();
    if (ret != LAUKAN_KL_RET_OK)
    {
        gState = KL_STATE_UNITIALIZED;
    }

    return ret;
}

ln_kl_ret_e ln_klSend(const char *pData, size_t len, uint32_t baud, uint32_t timeBetweenBytes_ms)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (gState == KL_STATE_UNITIALIZED)
    {
        ln_printf(gModule, LN_DBG_ERROR, "%s called but not initialized\n", __FUNCTION__);
        return LAUKAN_KL_RET_NOT_INITIALIZED;
    }
    else if (pData == NULL)
    {
        ln_printf(gModule, LN_DBG_ERROR, "%s called with NULL data\n", __FUNCTION__);
        ret = LAUKAN_KL_RET_INVALID_PARAM;
    }
    else if (len <= 0)
    {
        ln_printf(gModule, LN_DBG_ERROR, "%s called with invalid length: %d bytes\n", __FUNCTION__, len);
        ret = LAUKAN_KL_RET_INVALID_PARAM;
    }

    ret = checkBaud(baud);
    if (ret == LAUKAN_KL_RET_OK)
    {
        if (timeBetweenBytes_ms == 0)
        {
            const int nymOfBytesWritten = uart_write_bytes(KLINE_UART, pData, len);
            if (nymOfBytesWritten < 0)
            {
                ln_printf(gModule, LN_DBG_ERROR, "%s: uart_write_bytes failed: %d\n", __FUNCTION__, nymOfBytesWritten);
                ret = LAUKAN_KL_RET_HW_ERROR;
            }
        }
        else
        {
            while ((ret == LAUKAN_KL_RET_OK) && (len > 0))
            {
                if (uart_write_bytes(KLINE_UART, pData, 1) < 0)
                {
                    ln_printf(gModule, LN_DBG_ERROR, "%s: uart_write_bytes with delay failed\n", __FUNCTION__);
                    ret = LAUKAN_KL_RET_HW_ERROR;
                }
                else
                {
                    pData++;
                    len--;
                    if (len > 0)
                    {
                        vTaskDelay(timeBetweenBytes_ms / portTICK_RATE_MS + 1);
                    }
                }
            }
        }
    }

    return ret;
}

ln_kl_ret_e ln_klReceive(const char *pData, size_t *pLen, uint32_t baud, uint32_t readUntil_ms)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (gState == KL_STATE_UNITIALIZED)
    {
        ln_printf(gModule, LN_DBG_ERROR, "%s called but not initialized.\n", __FUNCTION__);
        return LAUKAN_KL_RET_NOT_INITIALIZED;
    }
    else if ((pData == NULL) || (pLen == NULL))
    {
        ln_printf(gModule, LN_DBG_ERROR, "%s called with NULL pointer.\n", __FUNCTION__);
        ret = LAUKAN_KL_RET_INVALID_PARAM;
    }
    else if (*pLen <= 0)
    {
        ln_printf(gModule, LN_DBG_ERROR, "%s called with invalid length: %d bytes.\n", __FUNCTION__, *pLen);
        ret = LAUKAN_KL_RET_INVALID_PARAM;
    }

    ret = checkBaud(baud);
    if (ret == LAUKAN_KL_RET_OK)
    {
        const uint32_t start_ms = millis();

        uint32_t dataOffset = 0;
        const uint32_t bufferLen = *pLen;
        uint32_t now_ms = millis();
        while ((ret == LAUKAN_KL_RET_OK) &&
               (dataOffset < bufferLen) &&
               ((now_ms - start_ms) < readUntil_ms))
        {
            const uint32_t left_ms = now_ms - start_ms + 1;
            const uint32_t maxLen = bufferLen - dataOffset;
            int numOfBytesRead = uart_read_bytes(KLINE_UART, pData[dataOffset], maxLen, left_ms / portTICK_RATE_MS);
            if (numOfBytesRead < 0)
            {
                ln_printf(gModule, LN_DBG_ERROR, "%s: Failed to read bytes: maximum length %d, ret %d.\n", __FUNCTION__, maxLen, numOfBytesRead);
                ret = LAUKAN_KL_RET_HW_ERROR;
            }
            else
            {
                dataOffset += (uint32_t)numOfBytesRead;
            }

            now_ms = millis();
        }

        if (ret == LAUKAN_KL_RET_OK)
        {
            *pLen = dataOffset;
        }
    }
}

ln_kl_ret_e ln_klSetPinState(uint32_t state, uint32_t delayAfter_ms)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (gState == KL_STATE_UNITIALIZED)
    {
        ret = LAUKAN_KL_RET_NOT_INITIALIZED;
    }
    else if (gState == KL_STATE_UART_STARTED)
    {
        ret = stopUART();
    }

    if (ret == LAUKAN_KL_RET_OK)
    {
        state = state ? 1 : 0;
        if (LAUKAN_KLINE_INVERT)
        {
            state ^= 1;
        }

        esp_err_t esp_err = gpio_set_level(LAUKAN_KLINE_IOPIN_TX, state);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "gpio_set_level failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
        else
        {
            if (delayAfter_ms > 0)
            {
                vTaskDelay(delayAfter_ms / portTICK_RATE_MS);
            }
        }
    }

    return ret;
}

static ln_kl_ret_e checkBaud(uint32_t baud)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (gState == KL_STATE_UNITIALIZED)
    {
        ret = LAUKAN_KL_RET_NOT_INITIALIZED;
    }

    uint32_t currentBaud = baud;
    if (gState == KL_STATE_UART_STOPPED)
    {
        ret = startUART(baud);
    }
    else if (ret == LAUKAN_KL_RET_OK)
    {
        esp_err_t esp_err = uart_get_baudrate(KLINE_UART, &currentBaud);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_get_baudrate failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    if ((ret == LAUKAN_KL_RET_OK) && (currentBaud != baud))
    {
        esp_err_t esp_err = uart_set_baudrate(KLINE_UART, baud);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_set_baudrate failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    return ret;
}

static ln_kl_ret_e startUART(uint32_t baud)
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (ret == LAUKAN_KL_RET_OK)
    {
        esp_err_t esp_err = uart_driver_install(KLINE_UART,
                                                KLINE_RX_BUF_SIZE,
                                                KLINE_TX_BUF_SIZE,
                                                0,
                                                NULL,
                                                0);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_driver_install failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    if (ret == LAUKAN_KL_RET_OK)
    {
        uart_config_t uart_config;
        uart_config.baud_rate = (int)baud;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.use_ref_tick = false;
        uart_config.rx_flow_ctrl_thresh = 122;

        esp_err_t esp_err = uart_param_config(KLINE_UART, &uart_config);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_param_config failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    if (ret == LAUKAN_KL_RET_OK)
    {
        esp_err_t esp_err = uart_set_pin(KLINE_UART,
                                         LAUKAN_KLINE_IOPIN_TX,
                                         LAUKAN_KLINE_IOPIN_RX,
                                         UART_PIN_NO_CHANGE,
                                         UART_PIN_NO_CHANGE);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_set_pin failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    if ((ret == LAUKAN_KL_RET_OK) && (LAUKAN_KLINE_INVERT == 1))
    {
        esp_err_t esp_err = uart_set_line_inverse(KLINE_UART, (UART_INVERSE_RXD | UART_INVERSE_TXD));
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_set_line_inverse failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    if (ret == LAUKAN_KL_RET_OK)
    {
        gState = KL_STATE_UART_STARTED;
    }

    return ret;
}

static ln_kl_ret_e stopUART()
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;

    if (gState == KL_STATE_UNITIALIZED)
    {
        ret = LAUKAN_KL_RET_NOT_INITIALIZED;
    }
    else if (gState == KL_STATE_UART_STARTED)
    {
        esp_err_t esp_err = uart_driver_delete(KLINE_UART);
        if (esp_err == ESP_OK)
        {
            gState = KL_STATE_UART_STOPPED;
            ret = configureGPIO();
            if (ret != LAUKAN_KL_RET_OK)
            {
                gState = KL_STATE_UNITIALIZED;
            }
        }
        else
        {
            ln_printf(gModule, LN_DBG_ERROR, "uart_driver_delete failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }
    return ret;
}

static ln_kl_ret_e configureGPIO()
{
    ln_kl_ret_e ret = LAUKAN_KL_RET_OK;
    if (gState != KL_STATE_UART_STOPPED)
    {
        ret = LAUKAN_KL_RET_INVALID_STATE;
    }

    if (ret == LAUKAN_KL_RET_OK)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << LAUKAN_KLINE_IOPIN_TX;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        esp_err_t esp_err = gpio_config(&io_conf);
        if (esp_err != ESP_OK)
        {
            ln_printf(gModule, LN_DBG_ERROR, "gpio_config failed %d\n", esp_err);
            ret = LAUKAN_KL_RET_HW_ERROR;
        }
    }

    return ret;
}

static void KLineTask(void *arg)
{
    (void)arg;

    uint32_t len = 0;
    const char data[3] = {1, 2, 3};
    while (1)
    {
        const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
        ln_printf(gModule, LN_DBG_INFO, "KLineTask wrote %d bytes\n", txBytes);
        len = (len % 3) + 1;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

static ISO14230_header_t parseHeader(const uint8_t *pData, uint32_t len)
{
    ISO14230_header_t ret;
    const uint8_t const *pDataStart = pData;

    ret.parseSuccess = true;
    if (pData == NULL)
    {
        ret.parseSuccess = false;
        ln_printf(gModule, LN_DBG_ERROR, "%s: NULL pData\n", __FUNCTION__);
        return ret;
    }
    else if (len == 0)
    {
        ret.parseSuccess = false;
        ln_printf(gModule, LN_DBG_WARNING, "Parsing header with zero data\n");
        return ret;
    }

    ret.bWithAddress = false;
    ret.bFunctionalAddressing = false;
    ret.bExceptionMode = false;
    ret.bWithLengthField = false;

    const uint8_t byte = *pData++;
    len--;
    const uint8_t A = byte >> 6;
    const uint8_t L = byte & ~(BIT7 | BIT8);
    if (A & BIT1)
    {
        // With address information
        ret.bWithAddress = true;
        if (A & BIT0)
        {
            ret.bFunctionalAddressing = true;
        }
    }
    else if (A & BIT0)
    {
        ret.bExceptionMode = true;
    }

    if (L == 0)
    {
        ret.bWithLengthField = false;
    }
    else
    {
        ret.payloadLen = L;
    }

    if (ret.bWithAddress && (len >= 2))
    {
        ret.target = *pData++;
        ret.source = *pData++;
        len -= 2;
    }
    else
    {
        ln_printf(gModule, LN_DBG_WARNING, "Not enough data in the header for for source and target address\n");
        ret.parseSuccess = false;
    }

    if (ret.parseSuccess && ret.bWithLengthField)
    {
        if (len >= 1)
        {
            ret.payloadLen = *pData++;
            len--;
            return ret;
        }
        else
        {
            ln_printf(gModule, LN_DBG_WARNING, "Header indicates a length field, but data is not long enough\n");
            ret.parseSuccess = false;
        }
    }

    return ret;
}

//NOTE: snprintf_P and PSTR are used extensive together in this program.
//      The combination allows using an snprintf function with an
//      argument string that is stored in FLASH memory without using
//      valuable SRAM space.

// Use FLASH memory to store strings instead of SRAM
// (Software USART) AltSoftSerial is used because it can handle non-standard baud rates
// (Software Timer) Used to keep the connection alive

//#define LED_BUILTIN 13

// Must use the same UART pins from AltSoftSerial
// #define rxPin 8
// #define txPin 9
// AltSoftSerial softSerial; // Uses Tx = 9, Rx = 8

// // The max number of ECUs that will be read from (determines how much RAM will be used)
// #define MAX_RESPONSES 15

// // Hold a list of globally supported PIDs. A specific PID is supported if
// // any ECU connected to the system supports the PID.
// // Stored in reverse bit order
// //   e.g. PID 0x01 = bit 32, PID 0x02 = bit 31, ... PID 0x20 = bit 1
// // For now just store 0x20 PIDs at a time
// // TODO: Store all PIDs at once if we can find room in SRAM
// uint32_t gPidSupport = 0;

// // A buffer to hold all responses to any request messages
// // Response messages are always 11 bytes or less:
// //   Header1, Header2, Header3 (Source), Data (up to 7 bytes), Checksum
// uint8_t responseBuffer[MAX_RESPONSES][11] = {0};

// // A buffer to hold strings that will be displayed to the user via UART
// #define BUFLEN 64
// //TODO: Change to uint8_t once we can find an alternative to Serial.println(buffer)
// //      Try Serial.println((char*)buffer)
// char buffer[BUFLEN] = {0};

// // Used to send a 'ping' request every few seconds to keep the connection alive
// RBD::Timer timerKeepAlive;

// /*
//    Send the 5-baud Initialization sequence to open communications
// */
// void send5BaudInit()
// {
//     // ISO 9141-2 uses a 5 baud initialization
//     // Neither Hardware nor Software UARTs can transmit at 5 baud,
//     // so we must bit bang the init sequence to the OBD port.
//     pinMode(txPin, OUTPUT); // Set the txPin to idle HIGH
//     digitalWrite(txPin, HIGH);
//     // Indicator LED
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(2000); // Leave Tx HIGH for 2 seconds before starting the actual init routine

//     Serial.println(F("Starting Init"));

//     // Send 0x33 (LSB first) via K-Line to the OBD port (0x33 is address of OBD)
//     digitalWrite(txPin, LOW); // start bit
//     digitalWrite(LED_BUILTIN, LOW);
//     delay(200);                // 5 baud means 200ms per bit
//     digitalWrite(txPin, HIGH); //11
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(400);
//     digitalWrite(txPin, LOW); //00
//     digitalWrite(LED_BUILTIN, LOW);
//     delay(400);
//     digitalWrite(txPin, HIGH); //11
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(400);
//     digitalWrite(txPin, LOW); //00
//     digitalWrite(LED_BUILTIN, LOW);
//     delay(400);
//     digitalWrite(txPin, HIGH); // stop bit
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(200);

//     // We have (min) 60ms to setup our serial port before the
//     // ECU starts sending back the SYNC byte (0x55)

//     // Set baud rate for the Software UART defined by ISO 9141-2 (10.4 Kbaud)
//     softSerial.begin(10400);

//     // The rest of the timing is a horrible mess of nested if statements
//     // Should we have used the dreaded 'goto'?
//     // Should we have refactored into smaller functions for such a simple timing segment?
//     // I don't know.
//     // Just try not to get lost in the jungle of if/else below...

//     bool bInitError = false; // If there is an error, set bInitError to true
//     String strError = "";    // and store the error description in strError.
//     bool iso9141 = false;    // Will either be ISO 9141 or ISO 14230
//     // Wait for SYNC byte (0x55)
//     uint8_t sync;
//     if (!getSoftSerial(sync, 300))
//     {
//         //DEBUG
//         snprintf_P(buffer, BUFLEN, PSTR("SYNC Byte: %2X"), sync);
//         Serial.println(buffer);

//         // Continue Init if we got the correct SYNC Byte
//         if (sync == 0x55)
//         {
//             uint8_t key1;
//             uint8_t key2;
//             uint8_t invKey2;
//             uint8_t invAddress;
//             // Get key1
//             if (!getSoftSerial(key1, 20))
//             {
//                 // Get key2
//                 if (!getSoftSerial(key2, 20))
//                 {
//                     //DEBUG
//                     snprintf_P(buffer, BUFLEN, PSTR("KEY1: %X    KEY2: %X"), key1, key2);
//                     Serial.println(buffer);

//                     delay(25); // 25ms <= W4 <= 50ms (Time between finished reciept of key2 and start send of inverted key2)
//                     //DEBUG
//                     invKey2 = ~key2;
//                     snprintf_P(buffer, BUFLEN, PSTR("Sending ~KEY2: %X"), invKey2);
//                     Serial.println(buffer);

//                     //TODO: Use sendSoftSerial instead so we don't need to duplicate the junk read-back
//                     softSerial.write(invKey2); // Send back inverted key2
//                     uint8_t junk;
//                     getSoftSerial(junk, 1); // TX and RX share the same line so we recieve back the byte we just sent
//                     //DEBUG
//                     snprintf_P(buffer, BUFLEN, PSTR("Received ~KEY2: %X"), junk);
//                     Serial.println(buffer);

//                     if (!getSoftSerial(invAddress, 50))
//                     {
//                         //DEBUG
//                         snprintf_P(buffer, BUFLEN, PSTR("Inverted Address: %X"), invAddress);
//                         Serial.println(buffer);

//                         //HACK: Arduino has a BUG where it cannot compare a char with a constant if
//                         //      the char is negative (e.g. 0xCC becomes 0xFFCC)
//                         //TODO: Check that this is 0xCC. If we use anything other than char Arduino
//                         //      uses significantly more SRAM and we are unable to compile for the
//                         //      2K available to use in a Pro Mini
//                         if (invAddress == 0xCC)
//                         {
//                             // Check if we are using ISO 9141-2 (otherwise it is ISO 14230)
//                             if ((key1 == 0x08) && (key2 == 0x08))
//                             {
//                                 iso9141 = true;
//                                 Serial.println(F("Protocol: ISO 9141-2"));
//                                 Serial.println(F("5-baud Init SUCCESS"));
//                             }
//                             else
//                             {
//                                 //ERROR: Only ISO 9141-2 is supported
//                                 bInitError = true;
//                                 strError = F("ISO 14230-4 is not supported");
//                             }
//                         }
//                         else
//                         { // if( invAddress == 0xCC ) {
//                             //ERROR: Unexpected Inverted Address received
//                             bInitError = true;
//                             strError = F("Bad Inverted Address Byte");
//                         }
//                     }
//                     else
//                     { // if( !getSoftSerial( invAddress, 50 ) ) {
//                         //ERROR: invAddress timeout
//                         bInitError = true;
//                         strError = F("Did not receive invAddress before timeout occured");
//                     }
//                 }
//                 else
//                 { // if( !getSoftSerial( key2, 20 ) ) {
//                     //ERROR: key2 timeout
//                     bInitError = true;
//                     strError = F("Did not receive key1 before timeout occured");
//                 }
//             }
//             else
//             { // if( !getSoftSerial( key1, 20 ) ) {
//                 //ERROR: key1 timeout
//                 bInitError = true;
//                 strError = F("Did not receive key1 before timeout occured");
//             }
//         }
//         else
//         { // if( sync == 0x55 ) {
//             //ERROR: SYNC Byte is not correct
//             bInitError = true;
//             strError = F("Bad SYNC Byte");
//         }
//     }
//     else
//     { // if( !getSoftSerial( sync, 300 ) ) {
//         //ERROR: sync timeout
//         bInitError = true;
//         strError = F("Did not receive SYNC byte before Timeout occured");
//     }

//     // Report any errors that occured during Init and enter inf loop
//     if (bInitError)
//     {
//         Serial.println(F("5-baud Init FAILED"));
//         Serial.println(strError);
//         Serial.println(F("RESET to Retry"));
//         // Go into inf loop to prevent WDT from being setup
//         while (true)
//         {
//         }
//     }

//     delay(55); // 55ms <= P3 (wait between messages) <= 5s
//     // Init is over. Start reading data from the ECU

//     // 55ms has passed since the last message. We now have (5sec - 55ms) to send another
//     timerKeepAlive.restart();
// }

// /*
//   Send a request to system as a the 'Diagnostic Tool/External Test Equipment'

//   @param mode The Mode of the PID
//   @param pid The PID to request (or default of 0 if requesting Mode 3 or 4)
//   @param ping If true, we won't save any response data in the global responseBuffer
//   @return The number of response messages received
// */
// uint8_t sendPid(uint8_t mode, uint8_t pid = 0, boolean ping = false)
// {
//     // Flash LED to show we are sending data
//     digitalWrite(LED_BUILTIN, LOW);

//     // Message format: Header1, Header2, Source Address, Mode, (PID), Checksum   (see SAE J1979)
//     uint8_t request[6] = {0x68, 0x6A, 0xF1, mode, 0x00, 0x00};
//     uint8_t crc = (0x68 + 0x6A + 0xF1 + mode);

//     // Format the request based on which mode we are requesting
//     if ((mode == 0x03) || (mode == 0x04))
//     {
//         // Mode 3 and 4 require no PID
//         request[4] = crc;
//         // request[5] is unused, leave it as 0x00
//         sendSoftSerial(request, 5); // Send the request
//     }
//     else
//     {
//         // Modes 1, 2, 5, and 9 require a PID
//         crc += pid;
//         request[4] = pid;
//         request[5] = crc;
//         sendSoftSerial(request, 6); // Send the request
//     }
//     // The ECU(s) will now wait 50ms (P2) to ensure that we are done transmitting before responding
//     delay(50);

//     // Get all responses and store them in the responseBuffer
//     // Start loop to get a response from each ECU that wants to reply
//     boolean bMsgTimeout = false;
//     uint8_t messageNum = 0;

//     while (!bMsgTimeout)
//     {
//         boolean bByteTimeout = false;
//         uint8_t trash;

//         //TODO: Define constants for timeouts
//         uint8_t byteNum = 0;
//         while (!bByteTimeout)
//         {
//             if (ping)
//             {
//                 bByteTimeout = getSoftSerial(trash, 20); // Store response
//             }
//             else
//             {
//                 bByteTimeout = getSoftSerial(responseBuffer[messageNum][byteNum], 20); // Store response
//                 if (bByteTimeout)
//                 {
//                     // Set the current byte to 0 if the timeout occured before it could be written
//                     responseBuffer[messageNum][byteNum] = 0;
//                 }
//             }

//             byteNum++;
//         }

//         // Wait for another ECU to respond or for message timeout to occur
//         // 55ms must pass before we know that no other ECU will respond
//         // The previous message timed out for the last byte recieved so add
//         // the byteTimeout to the start time, and wait for messageTimeout
//         unsigned long startTime = millis() - 20;

//         while (!softSerial.available())
//         {
//             // Timeout is 55ms
//             if ((millis() - startTime) >= 55)
//             {
//                 bMsgTimeout = true; // Leave the outter 'while' loop
//                 break;              // Leave this 'while' loop
//             }
//         }

//         // Fill the rest of the current message with zeros if this is not a ping
//         if (!ping)
//         {
//             for (int i = byteNum; byteNum < 11; byteNum++)
//             {
//                 responseBuffer[messageNum][byteNum] = 0;
//             }
//         }

//         // Move to the next message if timeout was BETWEEN 20ms and 55ms
//         messageNum++;
//     }

//     // 55ms has passed since the last message. We now have (5sec - 55ms) to send another
//     timerKeepAlive.restart();

//     // Flash LED to show we are sending data
//     digitalWrite(LED_BUILTIN, HIGH);

//     return messageNum;
// }

// /*
//    A function that will wait for data to become available on the
//    Hardware Serial port, then return the data.
// */
// uint8_t getSerial()
// {
//     while (!Serial.available())
//     {
//         //Check the keep-alive timer while we wait for a byte to arrive
//         if (timerKeepAlive.isExpired())
//         {
//             // Send a 'keep-alive' message
//             sendPid(1, 0, true);
//             timerKeepAlive.restart();
//         }
//     }
//     // Return the (16-bit) int as a single (8-bit) byte
//     // We always check for data first with Serial.available()
//     // so no need to check for an empty buffer (e.g. 0xFFFF)
//     return Serial.read();
// }

// /*
//    A function that will wait for a byte to become available on the
//    Software Serial port OR for timeout to occur. If data is available
//    before the timeout, then return the data.

//    @param retVal The character read on the software serial port
//    @param timeout The number of milliseconds to wait for a byte to be read on the port
//    @return Did the timeout occur before reading? False: read is good, True: retVal was not filled in time (failed)
// */
// boolean getSoftSerial(uint8_t &retVal, uint32_t timeout)
// {
//     uint32_t start = millis();
//     while (!softSerial.available())
//     {
//         //Wait for a byte to arrive
//         if ((millis() - start) > timeout)
//         {
//             return true;
//         }
//     }
//     // Return the (16-bit) int as a single (8-bit) byte
//     // We always check for data first with Serial.available()
//     // so no need to check for an empty buffer (e.g. 0xFFFF)
//     retVal = softSerial.read();
//     return false;
// }

// /*
//    A function that will send (Tx) an array of bytes to the Software Serial port
// */
// void sendSoftSerial(uint8_t *bytes, unsigned size)
// {
//     //DEBUG
//     /*
//     Serial.println();
//     Serial.print("Sending: ");
//     for( int i = 0; i < size; i++ ) {
//       snprintf(buffer, BUFLEN, "%02X ", bytes[i]);
//       Serial.print(buffer);
//     }
//     Serial.println();
//   */

//     for (int i = 0; i < size; i++)
//     {
//         // Send the next byte
//         softSerial.write(bytes[i]);
//         // TX and RX share the same line so we recieve back the byte we just sent
//         uint8_t trash;
//         getSoftSerial(trash, 1);
//         //TODO: This can be adjusted depending on how long the above
//         //      function takes to execute. 5ms <= P4 <= 20ms
//         delay(5); // P4 (Inter-byte spacing)
//     }
// }

// // Run once after Arduino start-up/reset
// void setup()
// {
//     // Open (hardware) serial port now while we aren't restricted by timing
//     Serial.begin(57600);

//     // Use on-board LED for indication
//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, LOW);

//     // If more than 5 seconds passes between messages then we need to resend the init sequence
//     // so use this timer to send 'ping' messages every 4 seconds to keep the connection alive.
//     timerKeepAlive.setTimeout(4000);

//     // Send Init sequence once to open communications
//     send5BaudInit();
// }

// /*
//    Our main program loop will display a menu to the user
// */
// void loop()
// {
//     // Display a menu to select which Mode the user wants to use
//     Serial.println(F("MENU - Mode Selection"));
//     Serial.println(F("  1) Show Current Data"));
//     Serial.println(F("  2) Show Freeze Frame Data"));
//     Serial.println(F("  3) Show Diagnostic Trouble Codes"));
//     Serial.println(F("  4) Clear Diagnostic Trouble Codes and stored values"));
//     Serial.println(F("  5) Test results, oxygen sensor monitoring (non CAN only)"));
//     Serial.println(F("  6) Test results, other component/system monitoring (Test results, oxygen sensor monitoring for CAN only)"));
//     Serial.println(F("  7) Show pending Diagnostic Trouble Codes (detected during current or last driving cycle)"));
//     Serial.println(F("  8) Control operation of on-board component/system"));
//     Serial.println(F("  9) Request vehicle information"));
//     Serial.println(F("  A) Permanent Diagnostic Trouble Codes (DTCs) (Cleared DTCs)"));
//     Serial.print(F(" Select Mode (1-9,A): "));
//     uint8_t chMode = getSerial();
//     // Check and convert user input from ASCII
//     if ((chMode >= '0') && (chMode <= '9'))
//     {
//         chMode -= 0x30;
//     }
//     else if ((chMode == 'A') || (chMode == 'a'))
//     {
//         chMode = 0x0A;
//     }
//     else
//     {
//         Serial.println(F("Bad Input"));
//         return;
//     }
//     // Display the input back to the user
//     snprintf_P(buffer, BUFLEN, PSTR("%02X"), chMode);
//     Serial.println(buffer);

//     // Modes where a PID is required
//     if ((chMode == 0x01) || (chMode == 0x02) || (chMode == 0x05) || (chMode == 0x08) || (chMode == 0x09))
//     {
//         // Request which PIDs are supported for this Mode
//         uint8_t numMsg = sendPid(chMode, 0);
//         // Display formatted table of PIDs available
//         formatResponse(chMode, 0, numMsg);

//         // Now that the user can see supported PIDs, ask for their input
//         Serial.print(F("Select PID (00 - FF): "));
//         uint8_t chPid;
//         // Convert from ASCII to actual hex value
//         uint8_t c1 = getSerial();
//         if ((c1 >= '0') && (c1 <= '9'))
//         {
//             c1 -= 0x30;
//         }
//         else if ((c1 >= 'A') && (c1 <= 'F'))
//         {
//             c1 -= 0x41;
//         }
//         else if ((c1 >= 'a') && (c1 <= 'f'))
//         {
//             c1 -= 0x61;
//         }
//         else
//         {
//             Serial.println(F("Bad Input"));
//             return;
//         }

//         uint8_t c2 = getSerial();
//         if ((c2 >= '0') && (c2 <= '9'))
//         {
//             c2 -= 0x30;
//         }
//         else if ((c2 >= 'A') && (c2 <= 'F'))
//         {
//             c2 -= 0x41;
//         }
//         else if ((c2 >= 'a') && (c2 <= 'f'))
//         {
//             c2 -= 0x61;
//         }
//         else if (c2 != '\n')
//         {
//             Serial.println(F("Bad Input"));
//             return;
//         }

//         //DEBUG
//         Serial.println(c1, HEX);
//         Serial.println(c2, HEX);

//         // Check if we received 1 or 2 hex digits (convert to nibbles)
//         if (c2 == '\n')
//         {
//             chPid = c1;
//         }
//         else
//         {
//             chPid = (c1 << 4) | c2;
//         }
//         // Display the input back to the user
//         snprintf_P(buffer, BUFLEN, PSTR("%02X"), chPid);
//         Serial.println(buffer);

//         // Send the actual request
//         snprintf_P(buffer, BUFLEN, PSTR("Sending Mode %X PID %02X Request"), chMode, chPid);
//         Serial.println(buffer);
//         numMsg = sendPid(chMode, chPid);
//         // Format the response and send to Serial
//         formatResponse(chMode, chPid, numMsg);
//     }

//     // Request DTCs from all ECUs
//     if ((chMode == 0x03) || (chMode == 0x07))
//     {
//         uint8_t numMsg = sendPid(chMode);
//         //DEBUG
//         dumpRawData(numMsg);

//         // Display all the DTCs
//         Serial.println(F("DTCs:"));
//         //DEBUG
//         //Serial.print("Num Msg: ");
//         //Serial.println(numMsg);
//         for (uint8_t curMsg = 0; curMsg < numMsg; curMsg++)
//         {
//             // Display the three DTCs
//             for (uint8_t k = 0; k < 3; k++)
//             {
//                 uint16_t dtc = (responseBuffer[curMsg][4 + (2 * k)] << 8) | responseBuffer[curMsg][5 + (2 * k)];
//                 if (dtc != 0)
//                 { // Don't print blank DTCs
//                     snprintf_P(buffer, BUFLEN, PSTR("  %c%04X  (Raw Data: %04X)"), firstDTCChar(dtc), (dtc & 0x3FFF), dtc);
//                     Serial.println(buffer);
//                 }
//             }
//         }
//     }

//     // Prompt before erasing DTCs (Mode 4)
//     if (chMode == 0x04)
//     {
//         Serial.println(F("Delete all stored DTCs (y or n)? "));
//         char resp = getSerial();
//         if ((resp == 'y') || (resp == 'Y'))
//         {
//             sendPid(chMode);
//             Serial.println(F("All stored DTCs have been erased"));
//         }
//     }

//     if (chMode == 0x06)
//     {
//         Serial.println(F("Mode 6 is used by CAN bus Test Results. Use Mode 5 for ISO 9141 Test Results"));
//     }

//     if (chMode == 0x0A)
//     {
//         Serial.println(F("Mode A is only supported by ISO 15765-4"));
//     }
// }

// /*
//   Format the response for the specified Mode and PID combination
// */
// //TODO: Read 'source' address [2] and display data for each ECU that responds. Currently, only the first ECU is displayed.
// //TODO: Read Mode and PID from response data ([3]=mode, [4]=pid)
// //TODO: Allow cont. read on input (remove numMsg and just print something for each response received)
// //TODO: Fix all map()ed variables to allow decimal numbers by using math instead of map()
// void formatResponse(uint8_t chMode, uint8_t chPid, uint8_t numMsg)
// {
//     //DEBUG
//     dumpRawData(numMsg);

//     if (chMode == 0x01)
//     {
//         // Mode 1 - Show Current Data
//         switch (chPid)
//         {
//         case 0x00:
//         {
//             displaySupportedPids(chMode, chPid, numMsg);
//             break;
//         }
//         case 0x01: // Monitor status since DTCs cleared
//         {
//             uint8_t numDTCs = 0;
//             for (uint8_t i = 0; i < numMsg; i++)
//             {
//                 // Display ECU address
//                 snprintf_P(buffer, BUFLEN, PSTR("ECU %2X "), responseBuffer[i][2]);
//                 Serial.println(buffer);

//                 // Display the MIL indicator status
//                 if (responseBuffer[i][5] & 0x80)
//                 {
//                     Serial.println(F("  MIL: ON"));
//                 }
//                 else
//                 {
//                     Serial.println(F("  MIL: OFF"));
//                 }

//                 // Display the number of DTCs returned
//                 Serial.print(F("  DTC_CNT: "));
//                 Serial.println(responseBuffer[i][5] & 0x7F, DEC);

//                 numDTCs += responseBuffer[i][5] & 0x7F;
//             }

//             // Print the total number of DTCs
//             Serial.print(F("Total DTCs: "));
//             Serial.println(numDTCs, DEC);
//             break;
//         }
//         case 0x02: // DTC that caused required freeze frame data storage
//         {
//             uint16_t dtc = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
//             if (dtc == 0)
//             {
//                 Serial.println(F("DTCFRZF: No Freeze Frame Data"));
//             }
//             else
//             {
//                 snprintf_P(buffer, BUFLEN, PSTR("DTCFRZF: %c%04X"), firstDTCChar(dtc & 0xC0), (dtc & 0x3F));
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         case 0x03: // Fuel system 1/2 status
//         {
//             for (int i = 5; i <= 6; i++)
//             {
//                 snprintf_P(buffer, BUFLEN, PSTR("FUELSYS%d "), i - 4);
//                 Serial.println(buffer);
//                 switch (responseBuffer[0][i])
//                 {
//                 case 0x01:
//                     Serial.println(F("Open loop - has not yet satisfied conditions to go closed loop"));
//                     break;
//                 case 0x02:
//                     Serial.println(F("Closed loop - using oxygen sensor(s) as feedback for fuel control"));
//                     break;
//                 case 0x04:
//                     Serial.println(F("Open loop due to driving conditions (e.g. power enrichment, deceleration enleanment)"));
//                     break;
//                 case 0x08:
//                     Serial.println(F("Open loop - due to detected system fault"));
//                     break;
//                 case 0x10:
//                     Serial.println(F("Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"));
//                     break;
//                 }
//             }
//             break;
//         }
//         case 0x04: // Calculated LOAD Value
//         {
//             uint32_t load = map(responseBuffer[0][5], 0, 255, -40, 215);
//             snprintf_P(buffer, BUFLEN, PSTR("LOAD_PCT: %d%%"), load);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x05: // Engine Coolant Temperature
//         {
//             uint32_t temp = map(responseBuffer[0][5], 0, 255, 0, 100);
//             snprintf_P(buffer, BUFLEN, PSTR("ECT: %d C"), temp);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x06:
//         {
//             uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("SHRTFT1: %d%%"), fuel);
//             Serial.println(buffer);
//             fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("SHRTFT3: %d%%"), fuel);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x07:
//         {
//             uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("LONGFT1: %d%%"), fuel);
//             Serial.println(buffer);
//             fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("LONGFT3: %d%%"), fuel);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x08:
//         {
//             uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("SHRTFT2: %d%%"), fuel);
//             Serial.println(buffer);
//             fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("SHRTFT4: %d%%"), fuel);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x09:
//         {
//             uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("LONGFT2: %d%%"), fuel);
//             Serial.println(buffer);
//             fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("LONGFT4: %d%%"), fuel);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0A: // Fuel Rail Pressure (gauge)
//         {
//             uint32_t kpa = map(responseBuffer[0][5], 0, 255, 0, 765);
//             snprintf_P(buffer, BUFLEN, PSTR("FRP: %d kPa"), kpa);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0B: // Intake Manifold Absolute Pressure
//         {
//             uint32_t kpa = map(responseBuffer[0][5], 0, 255, 0, 255);
//             snprintf_P(buffer, BUFLEN, PSTR("MAP: %d kPa"), kpa);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0C: // Engine RPM
//         {
//             uint32_t rpm = map((responseBuffer[0][5] << 8) | responseBuffer[0][6], 0, 65535, 0, 16383.75);
//             snprintf_P(buffer, BUFLEN, PSTR("RPM: %d/min"), rpm);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0D: // Vehicle Speed Sensor
//         {
//             uint32_t vss = map(responseBuffer[0][5], 0, 255, 0, 255);
//             snprintf_P(buffer, BUFLEN, PSTR("VSS: %d kPa"), vss);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0E: // Ignition Timing Advance for #1 Cylinder
//         {
//             uint32_t adv = map(responseBuffer[0][5], 0, 255, -64, 63.5);
//             snprintf_P(buffer, BUFLEN, PSTR("SPARKADV: %d deg"), adv);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0F: // Intake Air Temperature
//         {
//             uint32_t iat = map(responseBuffer[0][5], 0, 255, -40, 215);
//             snprintf_P(buffer, BUFLEN, PSTR("IAT: %d C"), iat);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x10: // Air Flow Rate from Mass Air Flow Sensor
//         {
//             uint32_t maf = map((responseBuffer[0][5] << 8) | responseBuffer[0][6], 0, 65535, 0, 655.35);
//             snprintf_P(buffer, BUFLEN, PSTR("MAF: %d g/s"), maf);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x11: // Absolute Throttle Position
//         {
//             uint32_t tp = map(responseBuffer[0][5], 0, 255, 0, 100);
//             snprintf_P(buffer, BUFLEN, PSTR("TP: %d%%"), tp);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x12: // Commanded Secondary Air Status
//         {
//             switch (responseBuffer[0][5])
//             {
//             case 0x01:
//                 Serial.println(F("AIR_STAT: UPS (upstream of first catalytic converter)"));
//                 break;
//             case 0x02:
//                 Serial.println(F("AIR_STAT: DNS (downstream of first catalytic converter inlet)"));
//                 break;
//             case 0x04:
//                 Serial.println(F("AIR_STAT: OFF (atmosphere / off)"));
//                 break;
//             }
//             break;
//         }
//         case 0x13: // Location of Oxygen Sensors
//                    // PID 0x13 shall only be supported by a given vehicle if PID 0x1D is not supported.
//         {
//             uint8_t o2loc = responseBuffer[0][5];
//             Serial.print(F("O2SLOC: "));
//             if (o2loc & 0x01)
//             {
//                 Serial.print(F("O2S11 "));
//             }
//             if (o2loc & 0x02)
//             {
//                 Serial.print(F("O2S12 "));
//             }
//             if (o2loc & 0x04)
//             {
//                 Serial.print(F("O2S13 "));
//             }
//             if (o2loc & 0x08)
//             {
//                 Serial.print(F("O2S14 "));
//             }
//             if (o2loc & 0x10)
//             {
//                 Serial.print(F("O2S21 "));
//             }
//             if (o2loc & 0x20)
//             {
//                 Serial.print(F("O2S22 "));
//             }
//             if (o2loc & 0x40)
//             {
//                 Serial.print(F("O2S23 "));
//             }
//             if (o2loc & 0x80)
//             {
//                 Serial.print(F("O2S24 "));
//             }
//             Serial.println();
//             break;
//         }
//         case 0x14: // [0x14 - 0x1B] The Bank/Sensor number depends on if PID 0x13 or 0x1D is supported.
//         case 0x15: // Oxygen Sensor Output Voltage
//         case 0x16: // and Short Term Fuel Trim
//         case 0x17: // for each sensor in the
//         case 0x18: // 4 banks (2 sensors each)
//         case 0x19:
//         case 0x1A:
//         case 0x1B:
//         {
//             // Get which Bank and Sensor we are reading based on the PID
//             char bank;
//             char sensor;
//             // First check whether PID 0x13 or 0x1D is supported to
//             // determine how many banks/sensors are used/supported
//             if (gPidSupport & 0x2000)
//             {
//                 // PID 0x13 uses 2 banks with 4 sensor positions each
//                 if (chPid < 0x18)
//                 {
//                     bank = 1;
//                 }
//                 else
//                 {
//                     bank = 2;
//                 }
//                 sensor = (chPid % 4) + 1;
//             }
//             else
//             { // else i gPidSupport[0] & 0x0008 )
//                 // PID 0x1D uses 4 banks with 2 sensor positions each
//                 if (chPid < 0x16)
//                 {
//                     bank = 1;
//                 }
//                 else if (chPid < 0x18)
//                 {
//                     bank = 2;
//                 }
//                 else if (chPid < 0x1A)
//                 {
//                     bank = 3;
//                 }
//                 else
//                 {
//                     bank = 4;
//                 }
//                 sensor = (chPid % 2) + 1;
//             }

//             // Display O2 sensor voltage
//             // Arduino has a hard time printing double/float so do the math by hand
//             // Scale = 0.005V per bit
//             uint16_t o2 = responseBuffer[0][5] * 5; // number in millivolts
//             snprintf_P(buffer, BUFLEN, PSTR("O2S%d%d: %d.%dV"), bank, sensor, o2 / 1000, o2 % 1000);
//             Serial.println(buffer);

//             // Display fuel trim
//             uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
//             snprintf_P(buffer, BUFLEN, PSTR("SHRTFT%d%d: %d%%"), bank, sensor, fuel);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x1C: // OBD requirements to which vehicle is designed
//         {
//             uint8_t obd = responseBuffer[0][5];
//             Serial.print(F("OBDSUP: "));
//             if (obd == 0x01)
//             {
//                 Serial.println(F("OBD II"));
//             }
//             else if (obd == 0x02)
//             {
//                 Serial.println(F("OBD"));
//             }
//             else if (obd == 0x03)
//             {
//                 Serial.println(F("OBD and OBD II"));
//             }
//             else if (obd == 0x04)
//             {
//                 Serial.println(F("OBD I"));
//             }
//             else if (obd == 0x05)
//             {
//                 Serial.println(F("NO OBD"));
//             }
//             else if (obd == 0x06)
//             {
//                 Serial.println(F("EOBD"));
//             }
//             else if (obd == 0x07)
//             {
//                 Serial.println(F("EOBD and OBD II"));
//             }
//             else if (obd == 0x08)
//             {
//                 Serial.println(F("EOBD and OBD"));
//             }
//             else if (obd == 0x09)
//             {
//                 Serial.println(F("EOBD, OBD and OBD II"));
//             }
//             else if (obd == 0x0A)
//             {
//                 Serial.println(F("JOBD"));
//             }
//             else if (obd == 0x0B)
//             {
//                 Serial.println(F("JOBD and OBD II"));
//             }
//             else if (obd == 0x0C)
//             {
//                 Serial.println(F("JOBD and EOBD"));
//             }
//             else if (obd == 0x0D)
//             {
//                 Serial.println(F("JOBD, EOBD, and OBD II"));
//             }
//             else if (obd == 0x0E)
//             {
//                 Serial.println(F("EURO IV B1"));
//             }
//             else if (obd == 0x0F)
//             {
//                 Serial.println(F("EURO V B2"));
//             }
//             else if (obd == 0x10)
//             {
//                 Serial.println(F("EURO C"));
//             }
//             else if (obd == 0x11)
//             {
//                 Serial.println(F("EMD"));
//             }
//             else if (obd <= 0xFA)
//             {
//                 Serial.println(F("ISO/SAE reserved"));
//             }
//             else
//             {
//                 Serial.println(F("SAE J1939 special meaning"));
//             }
//             break;
//         }
//         case 0x1D: // Location of Oxygen Sensors
//                    // PID 0x1D shall only be supported by a given vehicle if PID 0x13 is not supported.
//         {
//             uint8_t o2loc = responseBuffer[0][5];
//             Serial.print(F("O2SLOC: "));
//             if (o2loc & 0x01)
//             {
//                 Serial.print(F("O2S11 "));
//             }
//             if (o2loc & 0x02)
//             {
//                 Serial.print(F("O2S12 "));
//             }
//             if (o2loc & 0x04)
//             {
//                 Serial.print(F("O2S21 "));
//             }
//             if (o2loc & 0x08)
//             {
//                 Serial.print(F("O2S22 "));
//             }
//             if (o2loc & 0x10)
//             {
//                 Serial.print(F("O2S31 "));
//             }
//             if (o2loc & 0x20)
//             {
//                 Serial.print(F("O2S32 "));
//             }
//             if (o2loc & 0x40)
//             {
//                 Serial.print(F("O2S41 "));
//             }
//             if (o2loc & 0x80)
//             {
//                 Serial.print(F("O2S42 "));
//             }
//             Serial.println();
//             break;
//         }
//         case 0x1E: // Auxiliary Input Status
//         {
//             if (responseBuffer[0][5] & 0x01)
//             {
//                 Serial.println(F("PTO_STAT: ON"));
//             }
//             else
//             {
//                 Serial.println(F("PTO_STAT: OFF"));
//             }
//             // Bits 1-7 are Reserved and should be reported as '0'
//         }
//         case 0x1F: // Time Since Engine Start
//         {
//             uint16_t uptime = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
//             snprintf_P(buffer, BUFLEN, PSTR("RUNTM: %d sec."), uptime);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x20: // PIDs supported [21 - 40] (same format as PID 0x00)
//         {
//             displaySupportedPids(chMode, chPid, numMsg);
//             break;
//         }
//         default: // PIDs 0x84-0xFF ISO/SAE Reserved
//             dumpRawData(numMsg);
//             break;
//         }
//     }
//     else if (chMode == 0x02)
//     {
//         //TODO: Mode 2 - Show Freeze Frame Data (Not Yet Implemented. Similar to Mode 1)
//     }
//     else if (chMode == 0x05)
//     {
//         //TODO: Mode 5 - (Non-CAN) Test Results (Not Yet Implemented)
//     }
//     else if (chMode == 0x08)
//     {
//         //TODO: Mode 8 (SMOG Testing?) Probably Won't Be Implemented
//     }
//     else if (chMode == 0x09)
//     {
//         // Mode 9 - Vehicle Information
//         switch (chPid)
//         {
//         case 0x00:
//         {
//             displaySupportedPids(chMode, chPid, numMsg);
//             break;
//         }
//         case 0x01: // MessageCount VIN (Mode 9 PID 2)
//         {
//             snprintf_P(buffer, BUFLEN, PSTR("MC_VIN: %d"), responseBuffer[0][5]);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x02: // Vehicle Identification Number (VIN)
//         {
//             if (numMsg == 5)
//             {
//                 char vin[18]; // 17 ASCII chars + '\0'
//                 // Nested loops were hard to read, so just copy the chars over one by one
//                 vin[0] = responseBuffer[0][8];
//                 vin[1] = responseBuffer[1][5];
//                 vin[2] = responseBuffer[1][6];
//                 vin[3] = responseBuffer[1][7];
//                 vin[4] = responseBuffer[1][8];
//                 vin[5] = responseBuffer[2][5];
//                 vin[6] = responseBuffer[2][6];
//                 vin[7] = responseBuffer[2][7];
//                 vin[8] = responseBuffer[2][8];
//                 vin[9] = responseBuffer[3][5];
//                 vin[10] = responseBuffer[3][6];
//                 vin[11] = responseBuffer[3][7];
//                 vin[12] = responseBuffer[3][8];
//                 vin[13] = responseBuffer[4][5];
//                 vin[14] = responseBuffer[4][6];
//                 vin[15] = responseBuffer[4][7];
//                 vin[16] = responseBuffer[4][8];
//                 vin[17] = '\0';
//                 // Print the VIN "string"
//                 snprintf_P(buffer, BUFLEN, PSTR("VIN: %s"), vin);
//                 Serial.println(buffer);
//             }
//             else
//             {
//                 snprintf_P(buffer, BUFLEN, PSTR("ERROR: Expecting 5 messages and received %d"), numMsg);
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         case 0x03: // MessageCount CALID
//         {
//             snprintf_P(buffer, BUFLEN, PSTR("MC_CALID: %d"), responseBuffer[0][5]);
//             Serial.println(buffer);
//             if ((responseBuffer[0][5] % 4) != 0)
//             {
//                 Serial.println(F("ERROR: MC_CALID was not a multiple of 4"));
//             }
//             break;
//         }
//         case 0x04: // Calibration Identifications
//         {
//             for (int calNum = 0; calNum < numMsg; calNum += 4)
//             {
//                 char calid[17]; // 16 ASCII chars + '\0'
//                 // Nested loops were hard to read, so just copy the chars over one by one
//                 calid[0] = responseBuffer[calNum][5];
//                 calid[1] = responseBuffer[calNum][6];
//                 calid[2] = responseBuffer[calNum][7];
//                 calid[3] = responseBuffer[calNum][8];
//                 calid[4] = responseBuffer[calNum + 1][5];
//                 calid[5] = responseBuffer[calNum + 1][6];
//                 calid[6] = responseBuffer[calNum + 1][7];
//                 calid[7] = responseBuffer[calNum + 1][8];
//                 calid[8] = responseBuffer[calNum + 2][5];
//                 calid[9] = responseBuffer[calNum + 2][6];
//                 calid[10] = responseBuffer[calNum + 2][7];
//                 calid[11] = responseBuffer[calNum + 2][8];
//                 calid[12] = responseBuffer[calNum + 3][5];
//                 calid[13] = responseBuffer[calNum + 3][6];
//                 calid[14] = responseBuffer[calNum + 3][7];
//                 calid[15] = responseBuffer[calNum + 3][8];
//                 calid[16] = '\0';
//                 // Print the CALID "string"
//                 snprintf_P(buffer, BUFLEN, PSTR("CALID: %s"), calid);
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         case 0x05: // MessageCount CVN
//         {
//             snprintf_P(buffer, BUFLEN, PSTR("MC_CVN: %d"), responseBuffer[0][5]);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x06: // Calibration Verification Numbers
//         {
//             for (int numCvn = 0; numCvn < numMsg; numCvn++)
//             {
//                 // CVN uses 4 bytes of HEX data
//                 snprintf_P(buffer, BUFLEN, PSTR("CVN: %X%X%X%X"), responseBuffer[numCvn][5], responseBuffer[numCvn][6], responseBuffer[numCvn][7], responseBuffer[numCvn][8]);
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         case 0x07: // MessageCount IPT
//         {
//             snprintf_P(buffer, BUFLEN, PSTR("MC_IPT: %d"), responseBuffer[0][5]);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x08: // In-use Performance Tracking
//         {
//             // We are expecting either 16 or 20 messages (4 data bytes each)
//             // in the specific order definded in SAE 1979/ISO 9141-2
//             Serial.println(F("IPT: "));
//             for (int ipt = 0; ipt < numMsg; ipt++)
//             {
//                 uint16_t counts = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
//                 // Format the data depending on the message number we are on
//                 if (ipt == 0)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("OBDCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 1)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("IGNCNTR: %d cnts"), counts);
//                 }
//                 else if (ipt == 2)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("CATCOMP1: %d cnts"), counts);
//                 }
//                 else if (ipt == 3)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("CATCOND1: %d cnts"), counts);
//                 }
//                 else if (ipt == 4)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("CATCOMP2: %d cnts"), counts);
//                 }
//                 else if (ipt == 5)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("CATCOND2: %d cnts"), counts);
//                 }
//                 else if (ipt == 6)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("O2SCOMP1: %d cnts"), counts);
//                 }
//                 else if (ipt == 7)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("O2SCOND1: %d cnts"), counts);
//                 }
//                 else if (ipt == 8)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("O2SCOMP2: %d cnts"), counts);
//                 }
//                 else if (ipt == 9)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("O2SCOND2: %d cnts"), counts);
//                 }
//                 else if (ipt == 10)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EGRCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 11)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EGRCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 12)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("AIRCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 13)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("AIRCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 14)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EVAPCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 15)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EVAPCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 16)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("SO2SCOMP1: %d cnts"), counts);
//                 }
//                 else if (ipt == 17)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("SO2SCOND1: %d cnts"), counts);
//                 }
//                 else if (ipt == 18)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("SO2SCOMP2: %d cnts"), counts);
//                 }
//                 else if (ipt == 19)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("SO2SCOND2: %d cnts"), counts);
//                 } /* else ERROR */
//                 // Print the data we formatted
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         case 0x09: // MessageCount ECUNAME
//         {
//             // For ISO 9141-2 the message count in the response should be 0x05
//             snprintf_P(buffer, BUFLEN, PSTR("MC_ECUNM: %d"), responseBuffer[0][5]);
//             Serial.println(buffer);
//             break;
//         }
//         case 0x0A: // ECUNAME
//         {
//             for (int ecuNum = 0; ecuNum < numMsg; ecuNum += 5)
//             {
//                 char ecuAbbrv[5]; // 4 ASCII chars + '\0'
//                 char ecuFull[16]; // 15 ASCII chars + '\0'
//                 // Nested loops were hard to read, so just copy the chars over one by one
//                 ecuAbbrv[0] = responseBuffer[ecuNum][5];
//                 ecuAbbrv[1] = responseBuffer[ecuNum][6];
//                 ecuAbbrv[2] = responseBuffer[ecuNum][7];
//                 ecuAbbrv[3] = responseBuffer[ecuNum][8];
//                 ecuAbbrv[4] = '\0';

//                 ecuFull[0] = responseBuffer[ecuNum + 1][6];
//                 ecuFull[1] = responseBuffer[ecuNum + 1][7];
//                 ecuFull[2] = responseBuffer[ecuNum + 1][8];
//                 ecuFull[3] = responseBuffer[ecuNum + 2][5];
//                 ecuFull[4] = responseBuffer[ecuNum + 2][6];
//                 ecuFull[5] = responseBuffer[ecuNum + 2][7];
//                 ecuFull[6] = responseBuffer[ecuNum + 2][8];
//                 ecuFull[7] = responseBuffer[ecuNum + 3][5];
//                 ecuFull[8] = responseBuffer[ecuNum + 3][6];
//                 ecuFull[9] = responseBuffer[ecuNum + 3][7];
//                 ecuFull[10] = responseBuffer[ecuNum + 3][8];
//                 ecuFull[11] = responseBuffer[ecuNum + 4][5];
//                 ecuFull[12] = responseBuffer[ecuNum + 4][6];
//                 ecuFull[13] = responseBuffer[ecuNum + 4][7];
//                 ecuFull[14] = responseBuffer[ecuNum + 4][8];
//                 ecuFull[15] = '\0';
//                 // Print the CALID "string"
//                 snprintf_P(buffer, BUFLEN, PSTR("ECU: %s"), ecuAbbrv);
//                 Serial.println(buffer);
//                 snprintf_P(buffer, BUFLEN, PSTR("ECUNAME: %s"), ecuFull);
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         case 0x0B: // In-use Performance Tracking
//         {
//             // We are expecting either 16 (4 data bytes each)
//             // in the specific order definded in SAE 1979/ISO 9141-2
//             Serial.println(F("IPT: "));
//             for (int ipt = 0; ipt < numMsg; ipt++)
//             {
//                 uint16_t counts = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
//                 // Format the data depending on the message number we are on
//                 if (ipt == 0)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("OBDCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 1)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("IGNCNTR: %d cnts"), counts);
//                 }
//                 else if (ipt == 2)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("HCCATCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 3)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("HCCATCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 4)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("NCATCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 5)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("NCATCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 6)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("NADSCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 7)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("NADSCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 8)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("PMCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 9)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("PMCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 10)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EGSCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 11)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EGSCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 12)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EGRCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 13)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("EGRCOND: %d cnts"), counts);
//                 }
//                 else if (ipt == 14)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("BPCOMP: %d cnts"), counts);
//                 }
//                 else if (ipt == 15)
//                 {
//                     snprintf_P(buffer, BUFLEN, PSTR("BPCOND: %d cnts"), counts);
//                 } /* else ERROR */
//                 // Print the data we formatted
//                 Serial.println(buffer);
//             }
//             break;
//         }
//         default: // PIDs 0x0C-0xFF ISO/SAE Reserved
//             dumpRawData(numMsg);
//             break;
//         }
//     }
// }

// void dumpRawData(uint8_t numMsg)
// {
//     Serial.print("Raw Data: ");
//     for (uint8_t i = 0; i < numMsg; i++)
//     {
//         for (uint8_t j = 0; j < 11; j++)
//         {
//             snprintf(buffer, BUFLEN, "%02X ", responseBuffer[i][j]);
//             Serial.print(buffer);
//         }
//         Serial.println();
//     }
// }

// /*
//    Get the first character of the DTC.  It is determined by the first
//    two bits in the first byte of the DTC returned in a Mode 3 message.
// */
// uint8_t firstDTCChar(uint16_t dtc)
// {
//     // Get the first two bits of the high byte
//     uint8_t hidtc = highByte(dtc & 0xC000);
//     switch (hidtc)
//     {
//     case 0x00:
//         return 'P';
//     case 0x40:
//         return 'C';
//     case 0x80:
//         return 'B';
//     case 0xC0:
//         return 'U';
//     }
//     // ERROR - Bad input
//     return 0;
// }

// /*
//    Function to display a list of PIDs which may or may not be supported
//    chPid determines which PIDs are displayed (e.g. PID 0x20 will display 0x21 through 0x40)
// */
// void displaySupportedPids(uint8_t chMode, uint8_t chPid, uint8_t numEcu)
// {
//     gPidSupport = 0; // Reset global PID support
//     // Create a list of globally supported PIDs supported by any ECU in the system
//     for (uint8_t curEcu = 0; curEcu < numEcu; curEcu++)
//     {
//         // Store the PIDs that are supported for this specific ECU packed into 4 bytes
//         uint32_t ecuPidSupport = (responseBuffer[curEcu][5] << 24) | (responseBuffer[curEcu][6] << 16) | (responseBuffer[curEcu][7] << 8) | responseBuffer[curEcu][8];
//         // OR support with the list of globally supported PIDs for this system
//         gPidSupport = gPidSupport | ecuPidSupport;

//         //DEBUG: Print which PIDs are supported for this ECU
//         //snprintf(buffer, BUFLEN, "ECU %2X: %8X", responseBuffer[ecuNum][2], ecuPidSupport);
//         //Serial.println(buffer);
//     }

//     // Display a second Menu with available PIDs
//     snprintf_P(buffer, BUFLEN, PSTR("Mode %X PIDs Supported:"), chMode);
//     Serial.println(buffer);

//     // Show 0x20 PIDs in 2 rows. First PID determined by chPid (e.g. chPid + 1)
//     for (uint8_t row = 0; row < 2; row++)
//     {
//         Serial.print(F("  "));
//         for (uint8_t col = 1; col <= 16; col++)
//         {
//             snprintf_P(buffer, BUFLEN, PSTR("%02X "), (16 * row) + col + chPid);
//             Serial.print(buffer);
//         }
//         // Move below the numbers to print support (Y, N, or ?)
//         Serial.println();
//         Serial.print(F("  "));

//         // Now print if the previous PIDs are supported or not
//         for (uint8_t bitNum = (16 * row); bitNum < (16 * row) + 16; bitNum++)
//         {
//             // Invert the bits to get supported PIDs in numerical order
//             if ((gPidSupport >> (31 - bitNum)) & 0x01)
//             {
//                 Serial.print(F(" Y "));
//             }
//             else
//             {
//                 // If there was no response then we don't know if there
//                 // is support for PIDs with 0s. If there was a reply
//                 // then 0s in gPidSupport means that PID is unsupported.
//                 if (numEcu > 0)
//                 {
//                     Serial.print(F(" N "));
//                 }
//                 else
//                 {
//                     Serial.print(F(" ? "));
//                 }
//             }
//         }
//         Serial.println();
//     }

//     // Print the number of ECUs that responded to the request
//     Serial.print(F("Number of responding ECUs: "));
//     Serial.println(numEcu, DEC);
// }
