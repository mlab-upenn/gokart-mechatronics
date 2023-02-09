#ifndef _ENDIANNESS_H
#define _ENDIANNESS_H

#include <stdint.h>

#define ntohs(x)        __builtin_bswap16(x)
#define htons(x)        __builtin_bswap16(x)

#define ntohl(x)        __builtin_bswap32(x)
#define htonl(x)        __builtin_bswap32(x)

#endif // _ENDIANNESS_H
