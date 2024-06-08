#ifndef _HELPER_H_
#define _HELPER_H_
#include <stdint.h>

#define MAKE_UINT16(hb, lb) ((uint16_t)((hb) << 8) | (lb))
#define MAKE_INT16(hb, lb) ((int16_t)((hb) << 8) | (lb))
#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define clamp(x, min, max) ((x) < (min) ? (min) : (x > (max) ? (max) : (x)))
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif