/**
 * @brief: typedefs.h
 *
 *
*/


#ifndef TYPEDEFS_H
#define TYPEDEFS_H
#include "ysizet.h"
#ifdef __cplusplus
extern "C" {
#endif

enum {
    X_AXIS   =  0U,
    Y_AXIS   =  1U,
    Z_AXIS   =  2U,
    NUM_AXIS =  3U
};


/* raw sensor order  (from xbowsp_algorithm.h) */
enum {
    XACCEL     = 0U,
    YACCEL     = 1U,
    ZACCEL     = 2U,
    XRATE      = 3U,
    YRATE      = 4U,
    ZRATE      = 5U,
    XMAG       = 6U,
    CHIP_TEMP  = 6U,
    YMAG       = 7U,
    ZMAG       = 8U,
    XATEMP     = 9U,
    YATEMP     = 10U,
    ZATEMP     = 11U,
    XRTEMP     = 12U,
    YRTEMP     = 13U,
    ZRTEMP     = 14U,
    BTEMP      = 15U,
    N_RAW_SENS = 16U // size
};

/// raw sensor order (from dmu.h)
enum {
    ACCEL_START         = 0U,
    RATE_START          = 3U,
    MAG_START           = 6U,
    NUM_SENSOR_IN_AXIS  = 9U,
    TEMP_START          = 9U,
    GYRO_TEMP           = 9U,
    TEMP_SENSOR         = 10U,
    NUM_SENSOR_READINGS = 11U
};

enum {
    NUM_SENSOR_CHIPS    = 3U,
    NUM_SENSORS         = 3U,
};

#define CONST          const    // defines a constant item

/*lint -e(9023,9024) Multiple use of '#/##' operators [MISRA 2012 Rule 20.10, advisory]*/
#define DEFINE_TYPES(name, base)        \
 typedef base              t ## name;    \
 typedef base *            tP  ## name;  \
 typedef volatile base     tV  ## name;  \
 typedef volatile base *   tVP ## name;

//-----------------------------------------------------------------------
// typedefs (scope: global)
//-----------------------------------------------------------------------

DEFINE_TYPES(       Bool, unsigned char)
DEFINE_TYPES(         U8, unsigned char)
DEFINE_TYPES(         S8, signed char)
DEFINE_TYPES(        U16, unsigned short)
DEFINE_TYPES(        S16, signed short)
DEFINE_TYPES(        U32, unsigned long)
DEFINE_TYPES(        S32, signed long)
DEFINE_TYPES(       Char, char)
DEFINE_TYPES(      UChar, unsigned char)
DEFINE_TYPES(      SChar, signed char)
DEFINE_TYPES(     UShort, unsigned short)
DEFINE_TYPES(      Short, signed short)
DEFINE_TYPES(      ULong, unsigned long)
DEFINE_TYPES(       Long, signed long)
DEFINE_TYPES(  ULongLong, unsigned long long)
DEFINE_TYPES(   LongLong, long long)
DEFINE_TYPES(       UInt, unsigned int)
DEFINE_TYPES(        Int, signed int)
DEFINE_TYPES(      Float, float)
DEFINE_TYPES(     Double, double)
DEFINE_TYPES(    LDouble, long double)
DEFINE_TYPES(    Unicode, unsigned short)
DEFINE_TYPES(       Size, size_t)

DEFINE_TYPES(  UBitField, unsigned int)
DEFINE_TYPES(      SEnum, signed int)
DEFINE_TYPES(      UEnum, unsigned int)

typedef char *            tString;
typedef const char*       tCString;

typedef unsigned int      uint;
typedef tS8               int8;
typedef tS16              int16;
typedef tS32              int32;
typedef tU8               uint8;
typedef tU16              uint16;
typedef tU32              uint32;
typedef tS8               sint8;
typedef tS16              sint16;
typedef tS32              sint32;

typedef tBool             Bool;
typedef tBool             boolean_t;


#ifdef __cplusplus
}
#endif

#endif /* TYPEDEFS_H */
