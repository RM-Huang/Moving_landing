/**
 * @file      swas_types.h
 * @brief     swas sdk types
 * @author    CHCNAV
 * @version   v1.0
 * @date      2021-08-11
 * @copyright CHCNAV
 */

#ifndef SWAS_TYPES_H__
#define SWAS_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h> /* NULL */

/*
 * basic types definition
 */
/* character */
#ifndef SWAS_CHAR_T
#define SWAS_CHAR_T
typedef char swas_char_t;
#endif

/* 8bits signed integer */
#ifndef SWAS_INT8_T
#define SWAS_INT8_T
typedef signed char swas_int8_t;
#endif

/* 8bits unsigned integer */
#ifndef SWAS_UINT8_T
#define SWAS_UINT8_T
typedef unsigned char swas_uint8_t;
#endif

/* 16bits signed integer */
#ifndef SWAS_INT16_T
#define SWAS_INT16_T
typedef signed short swas_int16_t;
#endif

/* 16bits unsigned integer */
#ifndef SWAS_UINT16_T
#define SWAS_UINT16_T
typedef unsigned short swas_uint16_t;
#endif

/* 32bits signed integer */
#ifndef SWAS_INT32_t
#define SWAS_INT32_t
typedef signed int swas_int32_t;
#endif

/* 32bits unsigned integer */
#ifndef SWAS_UINT32_T
#define SWAS_UINT32_T
typedef unsigned int swas_uint32_t;
#endif

#ifndef SWAS_ULONG32_T
#define SWAS_ULONG32_T
typedef unsigned long swas_ulong32_t;
#endif

/* 64bits signed integer */
#ifndef SWAS_INT64_T
#define SWAS_INT64_T
typedef signed long long swas_int64_t;
#endif

/* 64bits unsigned integer */
#ifndef SWAS_UINT64_T
#define SWAS_UINT64_T
typedef unsigned long long swas_uint64_t;
#endif

/* single precision float number */
#ifndef SWAS_FLOAT32_T
#define SWAS_FLOAT32_T
typedef float swas_float32_t;
#endif

/* double precision float number */
#ifndef SWAS_FLOAT64_T
#define SWAS_FLOAT64_T
typedef double swas_float64_t;
#endif

/* void */
#ifndef SWAS_VOID_T
#define SWAS_VOID_T
typedef void swas_void_t;
#endif

/* socket */
#ifndef SWAS_SOCKET_T
#define SWAS_SOCKET_T
typedef void swas_socket_t;
#endif

/* NULL */
#ifndef SWAS_NULL
#define SWAS_NULL (void*)0
#endif

/* boolean representation */
#ifndef SWAS_BOOT_T
#define SWAS_BOOT_T
typedef enum
{
    /* FALSE value */
    SWAS_FALSE,
    /* TRUE value */
    SWAS_TRUE
} SWAS_bool_t;
#endif

/* MISRA-C[pm098] */
#ifndef NULL
#define NULL ((void*)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* SWAS_TYPES_H__ */
