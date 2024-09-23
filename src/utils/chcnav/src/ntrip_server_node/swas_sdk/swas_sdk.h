/**
 * @file        swas_sdk.h
 * @brief       swas_sdk
 * @author      CHCNAV
 * @version     v1.0
 * @date        2022-02-18
 * @copyright CHCNAV
 */

#ifndef SWAS_SDK_H__
#define SWAS_SDK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "swas_types.h"
#include "swas_socket.h"

/*******************************************************************************
 * SDK status Definitions
 *******************************************************************************/

/**********************************
 * status code, 1 - 999
 **********************************/
/*
 * interface related, 1 - 100
 */
#define SWAS_SDK_STA_OK (0)

/*
 * network related, 101 - 200
 */
#define SWAS_SDK_NETWORK_AVAILABLE                          101

/*
 * calib related, 201 - 300
 */
#define SWAS_SDK_CALIB_SUCCESS                              201
#define SWAS_SDK_CALIB_ING                                  202

/*
 * auth related, 301 - 400
 */
#define SWAS_SDK_AUTH_SUCCESS                               301
#define SWAS_SDK_AUTH_ING                                   302

/**********************************
 * error code, from -1 to -999
 **********************************/
/*
 * interface related, -1 - -100
 */
#define SWAS_SDK_STA_ENOINIT                                -1
#define SWAS_SDK_NOTSUP                                     -2
#define SWAS_SDK_INPROGRESS                                 -3
#define SWAS_SDK_ALREADY                                    -4
#define SWAS_SDK_NOT_READY                                  -5
#define SWAS_SDK_STA_ENOMEM                                 -6
#define SWAS_SDK_STA_EINVAL                                 -7

/*
 * network related, -101 - -200
 */
#define SWAS_SDK_ERR_NETWORK_UNAVAILABLE                    -101

/*
 * calib related, -201 - -300
 */
#define SWAS_SDK_CALIB_IDLE                                 -201 //Uncalibrated, reset state
#define SWAS_SDK_CALIB_FAIL                                 -202
#define SWAS_SDK_CALIB_ALREADY                              -203

/*
 * auth related, -301 - -400
 */
#define SWAS_SDK_AUTH_IDLE                                  -301 //Unauthenticated, reset status
#define SWAS_SDK_AUTH_FAIL                                  -302
#define SWAS_SDK_AUTH_ALREADY                               -303

/**
 * Define the SDK status or error code for user.
*/
typedef enum {
    SWAS_SDK_CODE_OK                         = SWAS_SDK_STA_OK, /* OK */
    SWAS_SDK_ERR_STATUS_PASSED                = SWAS_SDK_STA_OK, /* auth passed */
    SWAS_SDK_ERR_STATUS_INVALID_PARAM         = -1000, /* invalid parameter */
    SWAS_SDK_ERR_STATUS_PERMISSION_ERR        = -1001, /* permission error, please contact your administrator for more information */
    SWAS_SDK_ERR_STATUS_INTERNAL_ERR          = -1002, /* service inernal error, please contact your administrator for more information */
    SWAS_SDK_ERR_STATUS_ID_AGE_ERR            = -1003, /* the age of the ID number is illegal, please correct and retry */
}swas_sdk_status_code_e;

/*******************************************************************************
 * SDK common macros definition
 *******************************************************************************/

/*
 * limitations
 */
#define SWAS_SDK_MAX_KEY_LEN                                2048
#define SWAS_SDK_MAX_USERNAME_LEN                           10
#define SWAS_SDK_MAX_PASSWORD_LEN                           10
#define SWAS_SDK_MAX_SN_LEN                                 16
#define SWAS_SDK_MAX_APP_ID_LEN                             8

#define SWAS_SDK_MAX_THIRD_MOUNTPOINT_LEN                   16
#define SWAS_SDK_MAX_THIRD_NAME_LEN                         32
#define SWAS_SDK_MAX_THIRD_PASSWORD_LEN                     32

#define SWAS_SDK_MAX_GGA_BUF_LEN                            256
#define SWAS_SDK_MAX_DIFF_BUF_LEN                           2048

typedef enum
{
    SWAS_SDK_LOGIN_TYPE_USER_PASSWORD = 0, /* use user and password login */
    SWAS_SDK_LOGIN_TYPE_SN_DEVICE_ID  = 1, /* use sn and device id login */
    SWAS_SDK_LOGIN_TYPE_THIRD_CORS    = 2, /* use NTRIP accesses third-party CORS */
} swas_sdk_login_type_e;

typedef enum
{
    SWAS_SDK_COORDINATE_CGCS2000 = 1,      /* CGCS2000 */
    SWAS_SDK_COORDINATE_WGS84 = 2,         /* WGS84 */
    SWAS_SDK_COORDINATE_ITRF14 = 3,        /* ITRF14 */
} swas_sdk_coordinate_value_e;

typedef enum {
    SWAS_SDK_DATA_TYPE_SWAS_DIFF            = 0,             /* diff data */
    SWAS_SDK_DATA_TYPE_NONE                 = 0x7fffffff,    /* type guard */
} swas_sdk_data_type_e;

/*
 * persistent callbacks
 */

/**
 * The user registers to get the timestamp callback
 *
 *  @return:
 *       swas_uint64_t: timestamp;
 *
 * Notice:
 *      The user needs to register the timestamp function to obtain the current time, which is verified by SWAS
 */
typedef swas_uint64_t (*swas_sdk_time_get_func)();

/**********************************
 * The calib coordinate system results
 *
 * @param[in]  status: see calib related;
 *
 * Notice:
 *    Third-party integrated manufacturer devices can be configured with coordinate system permissions;  We solve this problem by accessing swager service and delivering IP + port
 *    The developer needs to pay attention to the value of status_code[see calib related] in the calib callback interface and decide
 *    whether to retry if the calib fails
 *
 **********************************/
typedef swas_void_t (*swas_sdk_calib_callback_t)(swas_int32_t status);

/**********************************
 * The authentication results
 *
 * @param[in]  status: see auth related;
 *
 * Notice:
 *      After an authentication failure, the SDK does not retry. The developer needs to pay attention to the value
 * of status_code[see auth related] in the authentication callback interface and decide whether to retry if the authentication fails
 *
 **********************************/
typedef swas_void_t (*swas_sdk_auth_callback_t)(swas_int32_t status);

/**********************************
 * Return data, etc
 *
 * @param[in] type: data type
 * @param[in] pdata: data address
 * @param[in] len: data length
 *
 * Notice:
 *      The obtained SWAS service difference data and other data are transmitted to developers through the data
 * callback function, and users can use the relevant data in the data callback function
 *
 **********************************/
typedef swas_void_t (*swas_sdk_data_callback_t)(swas_uint32_t type, swas_void_t* pdata, swas_uint32_t len);

/**********************************
 * Continuous status update
 *
 * @param[in]  status_code: Refer to swas_sdk_status_code_e for common error codes;
 *
 * Notice:
 *      Report the status code generated by the SDK to the developer
 *
 **********************************/
typedef swas_void_t (*swas_sdk_status_callback_t)(swas_int32_t status_code);

typedef struct swas_sdk_sn_land_s
{
    swas_uint8_t sn[SWAS_SDK_MAX_SN_LEN];
} swas_sdk_sn_land_t;

typedef struct swas_sdk_username_password_s
{
    swas_uint8_t username[SWAS_SDK_MAX_USERNAME_LEN];
    swas_uint8_t password[SWAS_SDK_MAX_PASSWORD_LEN];
} swas_sdk_username_password_t;

typedef struct swas_sdk_third_cors_s
{
    swas_sdk_host_info_t third_addr;

    swas_uint8_t mountpoint[SWAS_SDK_MAX_THIRD_MOUNTPOINT_LEN];
    swas_uint8_t username[SWAS_SDK_MAX_THIRD_NAME_LEN];
    swas_uint8_t password[SWAS_SDK_MAX_THIRD_PASSWORD_LEN];
} swas_sdk_third_cors_t;

typedef struct swas_sdk_config_s
{
    swas_sdk_login_type_e login_type;
    swas_sdk_coordinate_value_e coordinate_type;
    swas_uint8_t key[SWAS_SDK_MAX_KEY_LEN];
    swas_uint8_t appID[SWAS_SDK_MAX_APP_ID_LEN];   //Manufacturer's identification
    swas_sdk_username_password_t username_password;
    swas_sdk_sn_land_t sn_land;
    swas_sdk_third_cors_t third_cors;

    /*register by the user
        then called by the swas sdk*/
    swas_sdk_time_get_func time_get_func;
    swas_sdk_calib_callback_t calib_cb;
    swas_sdk_auth_callback_t auth_cb;
    swas_sdk_data_callback_t data_cb;
    swas_sdk_status_callback_t status_cb;
    swas_socket_func_t socket_operations;
} swas_sdk_config_t;

/**********************************
 * initialize SDK
 *
 * @param[in]  config: collect of account and callbacks;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 * Notice:
 *      If the SDK has an error, check the input parameters and socket interface and call the function again
 *
 **********************************/
swas_int32_t swas_sdk_init(swas_sdk_config_t* config_param);

/**
  * The socket start
  *
  */
extern swas_int32_t swas_socket_start();

/**********************************
 * Configuration coordinate system
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 **********************************/
swas_int32_t swas_sdk_calib_coordinate();

/**********************************
 * connect ntrip caster
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 **********************************/
swas_int32_t swas_sdk_auth();

/**********************************
 * upload gga
 *
 * @param[in]  gga: pointer of GGA string, such as["$GNGGA,072632.30,3100.0000000,N,12100.0000000,E,1,00,1.0,-9.442,M,9.442,M,0.0,0000*41"];
 * @param[in]  len: length of the string;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 **********************************/
swas_int32_t swas_sdk_upload_gga(const swas_char_t* gga, swas_uint32_t len);

/**********************************
 * This interface is used to obtain the SDK version number. This interface does
 * not depend on the initialization interface
 *
 * @return:
 *      const swas_char_t*: Current version number;
 **********************************/
const swas_char_t* swas_sdk_version();

/**********************************
 * cleanup sdk
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 **********************************/
swas_int32_t swas_sdk_cleanup();

#ifdef __cplusplus
}
#endif

#endif /* SWAS_SDK_H__ */
