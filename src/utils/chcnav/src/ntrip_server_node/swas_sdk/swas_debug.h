/**
 * @file        swas_debug.h
 * @brief       swas_debug
 * @author      CHCNAV
 * @version     v1.0
 * @date        2022-02-18
 * @copyright CHCNAV
 */

#ifndef SWAS_DEBUG_H__
#define SWAS_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "swas_types.h"

/*******************************************************************************
 * swas debug common macros definition
 *******************************************************************************/
typedef enum
{
    SWAS_DEBUG_CLOSE = 0,
    SWAS_DEBUG_OPENE,
} swas_debug_mask_type_e;

/**
  * Swas debug printing
  *
  * @param[in] value: Swas Debugging the printing function[swas_debug_mask_type_e]
  * 
  */
swas_int32_t swas_debug_set_debug_mask(swas_uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* SWAS_DEBUG_H__ */
