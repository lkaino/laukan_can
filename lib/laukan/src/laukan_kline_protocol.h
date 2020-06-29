#pragma once
#include "laukan_kline.h"

typedef enum
{
    LN_KLINE_PROT_ISO14230_KWP_5BAUD = 0,
    LN_KLINE_PROT_ISO14230_KWP_FAST,
    LN_KLINE_PROT_ISO9141,
    LN_KLINE_PROT_AUTO,
    LN_KLINE_PROT_NUM_OF
} ln_kl_prot_e;
