#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        LAUKAN_KL_RET_OK = 0,
        LAUKAN_KL_RET_NOT_INITIALIZED,
        LAUKAN_KL_RET_ALREADY_INITIALIZED,
        LAUKAN_KL_RET_INVALID_PARAM,
        LAUKAN_KL_RET_INVALID_STATE,
        LAUKAN_KL_RET_HW_ERROR,
        LAUKAN_KL_RET_COMM_ERROR,
        LAUKAN_KL_RET_OS_ERROR,
        LAUKAN_KL_RET_OUT_OF_MEM,
    } ln_kl_ret_e;

    ln_kl_ret_e ln_klInit(void);

    ln_kl_ret_e ln_klSend(const char *pData, size_t len, uint32_t baud, uint32_t timeBetweenBytes_ms);

    ln_kl_ret_e ln_klReceive(const char *pData, size_t *pLen, uint32_t baud, uint32_t readUntil_ms);

    ln_kl_ret_e ln_klSetPinState(uint32_t state, uint32_t delayAfter_ms);

#ifdef __cplusplus
}
#endif