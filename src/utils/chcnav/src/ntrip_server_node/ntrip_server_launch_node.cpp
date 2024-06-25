#include "ros/ros.h"

#include "swas_server.hpp"

// msgs include
#include "chcnav/int8_array.h"
#include "chcnav/string.h"
#include "std_msgs/String.h"

#include <pthread.h>
#include <signal.h>
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

static swas_server gs_ntrip_server;
static ros::Publisher gs_raw_data_outputer;

static map<int, string> gs_status_msg = {
    // status code
    {101, "The coordinate system is successfully calibrated"},
    {102, "Calibration set in the coordinate system"},
    {201, "Authentication success"},
    {202, "Authentication is underway. Procedure"},
    // error status code
    {-1, "Interface parameters are not initialized or registered"},
    {-2, "Authentication mode is not supported"},
    {-3, "Calibration or authentication has not yet been completed"},
    {-4, "The operation has been completed"},
    {-5, "The operation environment is not ready"},
    {-6, "Insufficient heap memory"},
    {-7, "Parameters of illegal"},
    {-101, "The operation environment is not ready"},
    {-201, "The network connection is unavailable"},
    {-202, "The reset state of the configured calibration coordinate system is not performed"},
    {-203, "The calibration coordinate system has been configured"},
    {-301, "Authentication is not reset"},
    {-302, "Authentication failed"},
    {-303, "The authentication is successful. Procedure"},
    {-1000, "Authentication parameter error"},
    {-1001, "The account secret or SN permission is incorrect"},
    {-1002, "Authentication internal service error"},
    {-1003, "Authentication validity period error"},
    // HTTP status
    {100, "Continue"},
    {101, "Switching Protocols"},
    {200, "OK"},
    {201, "Created"},
    {202, "SC_CREATED"},
    {203, "SC_NON_AUTHORITATIVE_INFORMATION"},
    {204, "No Content"},
    {205, "Reset Content"},
    {206, "Partial Content"},
    {300, "Multiple Choices"},
    {301, "Moved Permanently"},
    {302, "Found"},
    {303, "See Other"},
    {304, "Not Modified"},
    {305, "Use Proxy"},
    {307, "Temporary Redirect"},
    {400, "Bad Request"},
    {401, "Unauthorized"},
    {403, "Forbidden"},
    {404, "Not Found"},
    {405, "Method Not Allowed"},
    {406, "Not Acceptable"},
    {407, "Proxy Authentication Required"},
    {408, "Request Timeout"},
    {409, "Conflict"},
    {410, "Gone"},
    {411, "Length Required"},
    {412, "Precondition Failed"},
    {413, "Request Entity Too Large"},
    {414, "Request URI Too Long"},
    {415, "Unsupported Media Type"},
    {416, "Requested Range Not Satisfiable"},
    {417, "Expectation Failed"},
    {500, "Internal Server Error"},
    {501, "Not Implemented"},
    {502, "Bad Gateway"},
    {503, "Service Unavailable"},
    {504, "Gateway Timeout"},
    {505, "HTTP Version Not Supported"},
};

/**
 * The user registers to get the timestamp callback
 *
 *  @return:
 *       swas_uint64_t: timestamp;
 *
 * Notice:
 *      The user needs to register the timestamp function to obtain the current time, which is verified by SWAS
 */
static swas_uint64_t unix_time_get(swas_void_t)
{
    swas_uint64_t tm_now = time(nullptr);

    return tm_now;
}

/**
 * Return the calib coordinate system results
 *
 * @param[in]  status: see calib related;
 *
 * Notice:
 *    Third-party integrated manufacturer devices can be configured with coordinate system permissions;  We solve this problem by accessing swager service and delivering IP + port
 *    The developer needs to pay attention to the value of status_code[see calib related] in the calib callback interface and decide
 *    whether to retry if the calib fails
 *
 */
static swas_void_t calib_callback(swas_int32_t status)
{
    if (status == SWAS_SDK_CALIB_SUCCESS)
    {
        gs_ntrip_server.set_calib_success_flag(SWAS_TRUE);
        ROS_INFO("calib success");
    }
    else if (status == SWAS_SDK_CALIB_ING)
    {
        ROS_INFO("calib ing");
    }
    else
    {
        ROS_WARN("calib failed, status[%d][%s]\r\n", status, gs_status_msg[status].c_str());
        gs_ntrip_server.set_calib_success_flag(SWAS_FALSE);
        gs_ntrip_server.set_swas_status(SWAS_FALSE);
    }
}

/**
 * Return the swas authentication result
 *
 * @param[in] status: The authentication result is success or failure. If the
 * authentication fails, you need to call function [swas_sdk_auth] authenticate
 * again
 */
static swas_void_t auth_callback(swas_int32_t status)
{
    if (status == SWAS_SDK_AUTH_SUCCESS)
    {
        gs_ntrip_server.set_auth_success_flag(SWAS_TRUE);
        ROS_INFO("auth success");
    }
    else if (status == SWAS_SDK_AUTH_ING)
    {
        ROS_INFO("auth ing");
    }
    else
    {
        ROS_WARN("auth failed, status[%d][%s]\r\n", status, gs_status_msg[status].c_str());
        gs_ntrip_server.set_auth_success_flag(SWAS_FALSE);
        gs_ntrip_server.set_swas_status(SWAS_FALSE);
    }
}

/**
 * Return data, etc
 *
 * @param[in] type: data type
 * @param[in] pdata: data address
 * @param[in] len: data length
 */
static swas_void_t data_callback(swas_uint32_t type, swas_void_t *pdata, swas_uint32_t len)
{
    swas_char_t *diff_buf = (swas_char_t *)pdata;

    chcnav::int8_array raw_data;
    raw_data.data.resize(len);

    for (int i = 0; i < len; i++)
        raw_data.data[i] = diff_buf[i];

    gs_raw_data_outputer.publish(raw_data);
}

/**
 * Report the status code generated by the SDK to the developer
 *
 * @param[in] status_code: Refer to swas_sdk_status_code_e for common error
 * codes;
 *
 */
static swas_void_t status_callback(swas_int32_t status_code)
{
    const swas_char_t *sdk_version = swas_sdk_version();

    if (status_code == SWAS_SDK_NETWORK_AVAILABLE)
        gs_ntrip_server.set_socket_connect_flag(SWAS_TRUE);

    ROS_INFO("status_callback, status[%d][%s] sdk_version=[%s]\r\n", status_code, gs_status_msg[status_code].c_str(), sdk_version);
}

/**
 * @brief upload gpgga msg to cors server
 *
 * @param msg
 * */
static void client_gpgga_callback(const chcnav::string::ConstPtr &msg);

/**
 * @brief Safety exit
 *
 * @param sigo
 * */
static void signal_exit(int sigo); // exit func

static string login_type, username, password;    // common params
static string snkey, sn, app_id, str_coordinate; // swas server params
static string host, mountpoint, port;            // thire cors server params
static string frame_id;                          // frame_id
static swas_sdk_coordinate_value_e coordinate;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ntrip_server");

    ros::NodeHandle private_nh("~");

    signal(SIGTERM, signal_exit); // signal to eixt
    signal(SIGINT, signal_exit);  // signal to eixt
    signal(SIGKILL, signal_exit); // signal to eixt

    private_nh.param<string>("frame_id", frame_id, "None");
    private_nh.param<string>("login_type", login_type, "user_password");

    ROS_INFO("login type [%s]", login_type.c_str());
    if (login_type.compare("user_password") == 0)
    {
        private_nh.param<string>("snkey", snkey, "");
        private_nh.param<string>("app_id", app_id, "");
        private_nh.param<string>("username", username, "");
        private_nh.param<string>("password", password, "");
        private_nh.param<string>("coordinate", str_coordinate, "");

        if (str_coordinate.compare("CGCS2000") == 0)
            coordinate = SWAS_SDK_COORDINATE_CGCS2000;
        else if (str_coordinate.compare("ITRF14") == 0)
            coordinate = SWAS_SDK_COORDINATE_WGS84;
        else if (str_coordinate.compare("WGS84") == 0)
            coordinate = SWAS_SDK_COORDINATE_ITRF14;
        else
            coordinate = SWAS_SDK_COORDINATE_CGCS2000;

        ROS_INFO("username:[%s] password:[%s]", username.c_str(), password.c_str());
        ROS_INFO("snkey:[%s] app_id:[%s]", snkey.c_str(), app_id.c_str());

        gs_ntrip_server.set_user_password(username, password, snkey, app_id, coordinate);
    }
    else if (login_type.compare("sn_device") == 0)
    {
        private_nh.param<string>("sn", sn, "");
        private_nh.param<string>("snkey", snkey, "");
        private_nh.param<string>("app_id", app_id, "");
        private_nh.param<string>("coordinate", str_coordinate, "");

        if (str_coordinate.compare("CGCS2000") == 0)
            coordinate = SWAS_SDK_COORDINATE_CGCS2000;
        else if (str_coordinate.compare("ITRF14") == 0)
            coordinate = SWAS_SDK_COORDINATE_WGS84;
        else if (str_coordinate.compare("WGS84") == 0)
            coordinate = SWAS_SDK_COORDINATE_ITRF14;
        else
            coordinate = SWAS_SDK_COORDINATE_CGCS2000;

        gs_ntrip_server.set_sn_device(sn, snkey, app_id, coordinate);

        ROS_INFO("sn:[%s] snkey:[%s]", sn.c_str(), snkey.c_str());
        ROS_INFO("app_id:[%s] coordinate:[%s]", app_id.c_str(), str_coordinate.c_str());
    }
    else if (login_type.compare("third_cors") == 0)
    {
        private_nh.param<string>("host", host, "");
        private_nh.param<string>("port", port, "");
        private_nh.param<string>("mountpoint", mountpoint, "");
        private_nh.param<string>("username", username, "");
        private_nh.param<string>("password", password, "");

        gs_ntrip_server.set_third_cors(host, (uint16_t)atoi(port.c_str()), mountpoint, username, password);

        ROS_INFO("host:[%s] port:[%s] mountpoint:[%s] username:[%s] password:[%s]",
                 host.c_str(), port.c_str(), mountpoint.c_str(), username.c_str(), password.c_str());
    }
    else
    {
        ROS_ERROR("Please specifiy correct login type");
        return -1;
    }

    ros::Subscriber ntrip_source = private_nh.subscribe("ntrip_source", 1000, client_gpgga_callback);

    gs_raw_data_outputer = private_nh.advertise<chcnav::int8_array>("differential_data", 1000);

    gs_ntrip_server.set_func_cb(calib_callback, auth_callback, data_callback, status_callback, unix_time_get);

    if (gs_ntrip_server.connect() != SWAS_SDK_STA_OK)
        ROS_WARN("ntrip server connect failed");

    ros::spin();

    return 0;
}

static void client_gpgga_callback(const chcnav::string::ConstPtr &msg)
{
    if (gs_ntrip_server.get_swas_status() != SWAS_TRUE)
    {
        ROS_INFO("cors not connect! try reconnect ...");

        gs_ntrip_server.disconnect();

        if (gs_ntrip_server.connect() != SWAS_SDK_STA_OK)
        {
            ROS_WARN("cors reconnect overtime");
            return;
        }

        ROS_INFO("cors reconnect success");
    }

    string prefix(msg->sentence.c_str(), 0, 6);

    if (prefix.compare("$GPGGA") == 0 && frame_id.compare(msg->header.frame_id) == 0)
        swas_sdk_upload_gga(msg->sentence.c_str(), strlen(msg->sentence.c_str()));
}

static void signal_exit(int sigo)
{
    ROS_WARN("ntrip server exit");
    gs_ntrip_server.disconnect();
    exit(0);
}