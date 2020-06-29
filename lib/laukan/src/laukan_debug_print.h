#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        LN_DBG_ERROR = 0,
        LN_DBG_WARNING,
        LN_DBG_INFO,
        LN_DBG_VERBOSE,
        LN_NUM_OF_DEBUG_LEVELS,
    } ln_Debug_level_e;

    typedef enum
    {
        LN_DBG_RET_OK = 0,
        LN_DBG_RET_OS_ERR,
    } ln_Debug_ret_e;

    ln_Debug_ret_e ln_Dbg_init(void);
    void ln_printf(const char *module, ln_Debug_level_e level, const char *format, ...);

#ifdef __cplusplus
}
#endif
