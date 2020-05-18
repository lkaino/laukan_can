#pragma once
#include <stdint.h>

typedef enum
{
    LAUKAN_CAN_BAUD_25K = 0,
    LAUKAN_CAN_BAUD_50K,
    LAUKAN_CAN_BAUD_100K,
    LAUKAN_CAN_BAUD_125K,
    LAUKAN_CAN_BAUD_250K,
    LAUKAN_CAN_BAUD_500K,
    LAUKAN_CAN_BAUD_800K,
    LAUKAN_CAN_BAUD_1M
} laukan_CANBaud_e;

typedef enum
{
    LAUKAN_CAN_RET_OK = 0,
    LAUKAN_CAN_RET_NOT_INITIALIZED,
    LAUKAN_CAN_RET_ALREADY_INITIALIZED,
    LAUKAN_CAN_RET_INVALID_PARAM,
    LAUKAN_CAN_RET_LOW_LEVEL_ERROR,
    LAUKAN_CAN_RET_TIMEOUT,
    LAUKAN_CAN_RET_NO_MEM,
    LAUKAN_CAN_RET_NO_SLOTS,
} laukan_CANRet_e;

typedef uint32_t laukan_canProtocolHandle;

laukan_CANRet_e laukan_canInit(uint32_t maxNumOfProtocols);

laukan_CANRet_e laukan_canStart(laukan_CANBaud_e baud, uint32_t wait_ms);

laukan_CANRet_e laukan_canStop(uint32_t wait_ms);

/**
 * @brief   Register a protocol for receiving CAN messages.
 *
 * @param[out] pHandle              Allocated protocol handle is written here.
 *                                  Use this handle when receiving messages.
 * @param[in]  bufferLength         Maximum number of frames to buffer.
 * @param[in]  CANIDAcceptanceMask  CAN ID acceptance mask for incoming messages.
 * @param[in]  CANIDAcceptanceValue CAN ID must match this after applying the mask.
 *                                  i.e. (CANID & CANIDAcceptanceMask) == CANIDAcceptanceValue
 *
 * @return
 *      - LAUKAN_CAN_RET_OK: Successfully registered the protocol.
 *      - LAUKAN_CAN_RET_NOT_INITIALIZED: The interface is not initialized, call laukan_canInit().
 *      - LAUKAN_CAN_RET_NO_SLOTS: All protocol slots taken.
 *      - LAUKAN_CAN_RET_NO_MEM: Out of memory for creating the receive buffer.
 */
laukan_CANRet_e laukan_canRegisterProtocol(laukan_canProtocolHandle *pHandle,
                                           uint32_t numOfMessagesToBuffer,
                                           uint32_t CANIDAcceptanceMask,
                                           uint32_t CANIDAcceptanceValue);

/**
 * @brief   Receive message for a registered protocol handle. This is a blocking call.
 *
 * @param[in]  handle         A protocol handle registered with laukan_canRegisterProtocol().
 * @param[out] pMessage       Received CAN message is written here.
 * @param[in]  timeout_ms     Maximum time to wait for messages before returning.
 *
 * @return
 *      - LAUKAN_CAN_RET_OK: Received a message.
 *      - LAUKAN_CAN_RET_NOT_INITIALIZED: The interface is not initialized, call laukan_canInit().
 *      - LAUKAN_CAN_RET_TIMEOUT: No message received before timeout.
 */
laukan_CANRet_e laukan_canReceive(laukan_canProtocolHandle handle,
                                  can_message_t *pMessage,
                                  uint32_t timeout_ms);

/**
 * @brief   Send a CAN message.
 *
 * @param[in]  pMessage       Pointer to the CAN message to be sent.
 * @param[in]  timeout_ms     Maximum time to wait for space in send buffer before returning.
 *
 * @return
 *      - LAUKAN_CAN_RET_OK: Message was put to send buffer.
 *      - LAUKAN_CAN_RET_NOT_INITIALIZED: The interface is not initialized, call laukan_canInit().
 *      - LAUKAN_CAN_RET_TIMEOUT: There was no space in send buffer before timing out.
 */
laukan_CANRet_e laukan_canSend(const can_message_t *pMessage,
                               uint32_t timeout_ms);