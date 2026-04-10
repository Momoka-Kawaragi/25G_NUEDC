/**************************************************************************/
/*!
    @file     si5351_asserts.h
    @author   K. Townsend (Adafruit Industries)

    @section LICENSE

    软件许可协议 (BSD License)
    
    (版权声明保留英文以确保法律效力，此处省略...)
*/
/**************************************************************************/
#ifndef _ASSERTS_H_
#define _ASSERTS_H_

#include "si5351_errors.h"

/**************************************************************************/
/*!
    @brief 检查条件，如果断言失败，则在调用函数中返回提供的 returnValue。

    @code
    // 确保 'addr' 在范围内
    ASSERT(addr <= MAX_ADDR, ERROR_ADDRESSOUTOFRANGE);
    @endcode
*/
/**************************************************************************/
#define ASSERT(condition, returnValue) \
        do{\
          if (!(condition)) {\
            return (returnValue);\
          }\
        }while(0)

/**************************************************************************/
/*!
    @brief  检查提供的 \ref err_t 值 (sts)，如果它不等于 \ref ERROR_NONE，
            则返回该 sts 值。

    @details
    此宏用于检查函数是否返回错误，而无需用无休止的 "if (error) {...}" 
    使代码膨胀。

    @code
    // 如果 si5351a 返回除 ERROR_NONE 以外的任何内容，
    // 此宏将记录错误并退出函数，返回 error_t 值。
    ASSERT_STATUS(si5351_Init());
    @endcode
*/
/**************************************************************************/
#define ASSERT_STATUS(sts) \
        do{\
          err_t status = (sts);\
          if (ERROR_NONE != status) {\
            return status;\
          }\
        } while(0)

#endif
				
				