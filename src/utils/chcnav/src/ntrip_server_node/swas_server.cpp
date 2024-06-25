#include "swas_server.hpp"

#ifndef SWAS_TRY_TIME
#define SWAS_TRY_TIME 20
#endif

swas_int32_t swas_server::connect(swas_void_t)
{
    if (this->sdk_config.auth_cb == NULL || this->sdk_config.calib_cb == NULL ||
        this->sdk_config.data_cb == NULL || this->sdk_config.status_cb == NULL ||
        this->sdk_config.time_get_func == NULL)
    {
        DEMO_LOG("callback func null \r\n");
        return SWAS_SDK_NOT_READY;
    }

    if (this->swas_status == SWAS_TRUE)
    {
        DEMO_LOG("connect already \r\n");
        return SWAS_SDK_STA_EINVAL;
    }

    this->auth_success_flag = SWAS_FALSE;
    this->calib_success_flag = SWAS_FALSE;
    this->socket_connect_flag = SWAS_FALSE;

    // init sdk
    swas_int32_t ret = swas_sdk_init(&this->sdk_config);
    if (ret < 0)
        DEMO_LOG("sdk init failed\r\n");

#if SWAS_DEBUG == 1
    swas_debug_set_debug_mask(SWAS_DEBUG_OPENE);
#else
    swas_debug_set_debug_mask(SWAS_DEBUG_CLOSE);
#endif

    /* Start the socket */
    ret = swas_socket_start();
    if (ret < 0)
    {
        DEMO_LOG("swas_socket_start failed ret=%d\r\n", ret);
        return SWAS_SDK_NOT_READY;
    }

    /* waiting for socket connect */
    int try_time = SWAS_TRY_TIME;
    bool stat = false;
    do
    {
        /* Wait for the calibration coordinate or authentication socket to connect successfully */
        if (this->socket_connect_flag == SWAS_TRUE)
        {
            DEMO_LOG("calibration coordinate or authentication socket connect success\r\n");
            this->socket_connect_flag = SWAS_FALSE;
            stat = true;
            break;
        }
        usleep(1000 * 1000);
        DEMO_LOG("waiting for calibration coordinate or authentication socket connect result\r\n");

    } while (try_time-- > 0);

    if (stat == false)
    {
        DEMO_LOG("calibration coordinate or authentication socket connect failed\r\n");
        return SWAS_SDK_NOT_READY;
    }

    if (this->sdk_config.login_type != SWAS_SDK_LOGIN_TYPE_THIRD_CORS)
    {
        /* do calib */
        ret = swas_sdk_calib_coordinate();
        if (ret < 0)
        {
            DEMO_LOG("call sdk calib failed ret=%d\r\n", ret);
        }

        try_time = SWAS_TRY_TIME;
        stat = false;
        do
        {
            /* Wait for the calibration coordinate result, and wait for the authentication socket connection to succeed */
            if (this->calib_success_flag == SWAS_TRUE && this->socket_connect_flag == SWAS_TRUE)
            {
                DEMO_LOG("calibration coordinate success and authentication socket connect success\r\n");
                this->calib_success_flag = SWAS_FALSE;
                this->socket_connect_flag = SWAS_FALSE;

                stat = true;
                break;
            }
            usleep(1000 * 1000);
            DEMO_LOG("waiting for calibration coordinate result and wait authentication socket connect result\r\n");

        } while (try_time-- > 0);

        if (stat == false)
        {
            DEMO_LOG("calibration coordinate result or wait authentication socket connect failed\r\n");
            return SWAS_SDK_NOT_READY;
        }
    }

    /* do auth */
    ret = swas_sdk_auth();
    if (ret < 0)
    {
        DEMO_LOG("call sdk auth failed ret=%d\r\n", ret);
        return SWAS_SDK_AUTH_FAIL;
    }

    /* Wait for the authentication result */
    try_time = SWAS_TRY_TIME;
    stat = false;
    do
    {
        if (this->auth_success_flag == SWAS_TRUE)
        {
            DEMO_LOG("authentication success\r\n");
            this->auth_success_flag = SWAS_FALSE;
            stat = true;
            break;
        }
        usleep(1000 * 1000);
        DEMO_LOG("waiting for authentication result\r\n");

    } while (try_time-- > 0);

    if (stat == false)
    {
        DEMO_LOG("authentication failed\r\n");
        return SWAS_SDK_NOT_READY;
    }

    this->swas_status = SWAS_TRUE;

    return SWAS_SDK_STA_OK;
}