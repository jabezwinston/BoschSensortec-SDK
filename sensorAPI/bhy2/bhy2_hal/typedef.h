#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
typedef uint8_t u8;/**< used for unsigned 8bit */
typedef uint16_t u16;/**< used for unsigned 16bit */
typedef uint32_t u32;/**< used for unsigned 32bit */
typedef uint64_t u64;/**< used for unsigned 64bit */

/*signed integer types*/
typedef int8_t s8;/**< used for signed 8bit */
typedef int16_t s16;/**< used for signed 16bit */
typedef int32_t s32;/**< used for signed 32bit */
typedef int64_t s64;/**< used for signed 64bit */

/* Exported macro ------------------------------------------------------------*/
#define MAX_UINT16                  65535               /**< @brief Maximum value in a 16 but unsigned integer */
#define MIN_UINT16                  0                   /**< @brief Minimum value in a 16 but unsigned integer */
#define MAX_SINT16                  32767               /**< @brief Maximum value in a 16 but signed integer */
#define MIN_SINT16                  -32768              /**< @brief Minimum value in a 16 but signed integer */

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

#endif
