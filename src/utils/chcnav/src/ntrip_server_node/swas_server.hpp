#ifndef __CHCNAV_ROS_SAWS_SERVER_HPP__
#define __CHCNAV_ROS_SAWS_SERVER_HPP__

#include "swas_sdk/swas_debug.h"
#include "swas_sdk/swas_sdk.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

#if SWAS_DEBUG == 1
#define DEMO_LOG(fmt, ...) printf("[SWAS][%s:%d]" fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define DEMO_LOG(fmt, ...)
#endif

class swas_server
{
protected:
    swas_sdk_config_t sdk_config;

    swas_char_t swas_status = SWAS_FALSE;
    swas_char_t auth_success_flag = SWAS_FALSE;
    swas_char_t calib_success_flag = SWAS_FALSE;
    swas_char_t socket_connect_flag = SWAS_FALSE;

public:
    swas_server(){};
    ~swas_server()
    {
        disconnect();
    }

    /**
     * @brief sn snkey login type
     *
     * @param sn
     * @param snkey
     * @param app_id
     * @param coordinate
     * */
    void set_sn_device(string sn, string snkey, string app_id, swas_sdk_coordinate_value_e coordinate)
    {
        this->sdk_config.login_type = SWAS_SDK_LOGIN_TYPE_SN_DEVICE_ID;
        memcpy(this->sdk_config.sn_land.sn, sn.c_str(), SWAS_SDK_MAX_SN_LEN);
        memcpy(this->sdk_config.key, snkey.c_str(), SWAS_SDK_MAX_KEY_LEN);
        memcpy(this->sdk_config.appID, app_id.c_str(), SWAS_SDK_MAX_APP_ID_LEN);
        this->sdk_config.coordinate_type = coordinate;
    }

    /**
     * @brief username password login type
     *
     * @param username
     * @param password
     * @param snkey
     * @param app_id
     * @param coordinate
     * */
    void set_user_password(string username, string password, string snkey, string app_id, swas_sdk_coordinate_value_e coordinate)
    {
        this->sdk_config.login_type = SWAS_SDK_LOGIN_TYPE_USER_PASSWORD;
        memcpy(this->sdk_config.username_password.username, username.c_str(), SWAS_SDK_MAX_USERNAME_LEN);
        memcpy(this->sdk_config.username_password.password, password.c_str(), SWAS_SDK_MAX_PASSWORD_LEN);
        memcpy(this->sdk_config.key, snkey.c_str(), SWAS_SDK_MAX_KEY_LEN);
        memcpy(this->sdk_config.appID, app_id.c_str(), SWAS_SDK_MAX_APP_ID_LEN);
        this->sdk_config.coordinate_type = coordinate;
    }

    /**
     * @brief third cors
     *
     * @param host
     * @param port
     * @param mountpoint
     * @param username
     * @param password
     * */
    void set_third_cors(string host, swas_uint16_t port, string mountpoint, string username, string password)
    {
        this->sdk_config.login_type = SWAS_SDK_LOGIN_TYPE_THIRD_CORS;
        memcpy(this->sdk_config.third_cors.third_addr.sin_addr, host.c_str(), SWAS_SOCKET_MAX_ADDR_LEN);
        this->sdk_config.third_cors.third_addr.sin_port = port;
        memcpy(this->sdk_config.third_cors.mountpoint, mountpoint.c_str(), SWAS_SDK_MAX_THIRD_MOUNTPOINT_LEN);
        memcpy(this->sdk_config.third_cors.username, username.c_str(), SWAS_SDK_MAX_THIRD_NAME_LEN);
        memcpy(this->sdk_config.third_cors.password, password.c_str(), SWAS_SDK_MAX_THIRD_PASSWORD_LEN);
    }

    /**
     * @brief Set callback funcs
     *
     * @param calib_cb
     * @param auth_cb
     * @param data_cb
     * @param status_cb
     * @param time_get
     * */
    void set_func_cb(swas_sdk_calib_callback_t calib_cb,
                     swas_sdk_auth_callback_t auth_cb,
                     swas_sdk_data_callback_t data_cb,
                     swas_sdk_status_callback_t status_cb,
                     swas_sdk_time_get_func time_get)
    {
        this->sdk_config.calib_cb = calib_cb;
        this->sdk_config.auth_cb = auth_cb;
        this->sdk_config.data_cb = data_cb;
        this->sdk_config.status_cb = status_cb;
        this->sdk_config.time_get_func = time_get;
    }

    /**
     * @brief init swas sdk and connect to swas server
     *
     * @return swas_int32_t
     * */
    swas_int32_t connect(swas_void_t);

    /**
     * @brief deinit swas sdk
     *
     * @return swas_int32_t
     * */
    swas_int32_t disconnect(swas_void_t)
    {
        DEMO_LOG("swas_sdk_leeanup()\n");

        this->swas_status = SWAS_FALSE;
        this->auth_success_flag = SWAS_FALSE;
        this->calib_success_flag = SWAS_FALSE;
        this->socket_connect_flag = SWAS_FALSE;

        if (swas_sdk_cleanup() == 0)
            return SWAS_SDK_STA_OK;
        else
            return SWAS_SDK_NOT_READY;
    }

    /*********************************************************
     * class member set
     *********************************************************/
    void set_socket_connect_flag(swas_char_t status)
    {
        this->socket_connect_flag = status;
    }
    void set_calib_success_flag(swas_char_t status)
    {
        this->calib_success_flag = status;
    }
    void set_auth_success_flag(swas_char_t status)
    {
        this->auth_success_flag = status;
    }
    void set_swas_status(swas_char_t status)
    {
        this->swas_status = status;
    }

    /*********************************************************
     * class member get
     *********************************************************/
    swas_char_t get_socket_connect_flag(void)
    {
        return this->socket_connect_flag;
    }
    swas_char_t get_calib_success_flag(void)
    {
        return this->calib_success_flag;
    }
    swas_char_t get_auth_success_flag(void)
    {
        return this->auth_success_flag;
    }
    swas_char_t get_swas_status(void)
    {
        return this->swas_status;
    }
    swas_sdk_config_t get_sdk_config(void)
    {
        return this->sdk_config;
    }
};

#endif