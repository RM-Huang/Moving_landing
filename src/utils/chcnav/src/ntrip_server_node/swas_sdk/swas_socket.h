/**
 * @file        swas_socket.h
 * @brief       swas_socket
 * @author      CHCNAV
 * @version     v1.0
 * @date        2022-02-18
 * @copyright CHCNAV
 */

#ifndef SWAS_SOCKET_H__
#define SWAS_SOCKET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "swas_types.h"

/*
 * limitations
 */
#define SWAS_SOCKET_MAX_ADDR_LEN 32

/*******************************************************************************
 * swas socket common macros definition
 *******************************************************************************/
typedef enum
{
    SWAS_SOCKET_ERROR_NONE = 0,
    SWAS_SOCKET_ERROR_CONNECT,
    SWAS_SOCKET_ERROR_SEND,
    SWAS_SOCKET_ERROR_RECV,
    SWAS_SOCKET_ERROR_UNKNOW,
} swas_socket_status_error_e;

typedef struct swas_sdk_host_info_s
{
    swas_uint16_t sin_port; /* Port number */
    swas_uint8_t sin_addr[SWAS_SOCKET_MAX_ADDR_LEN]; /* internet address */
} swas_sdk_host_info_t;

/**********swas socket standard callback functions type defined*************/
typedef swas_int32_t (*swas_socket_connect_cb)(swas_socket_t* sock_fd);
typedef swas_int32_t (*swas_socket_send_cb)(swas_socket_t* sock_fd, size_t len);
typedef swas_int32_t (*swas_socket_recv_cb)(swas_socket_t* sock_fd, const swas_void_t* buf, size_t len);
typedef swas_int32_t (*swas_socket_status_cb)(swas_socket_t* sock_fd, swas_int32_t status);

/**************************swas socket operator interface type*************************/
typedef swas_int32_t (*swas_socket_init_func)();
typedef swas_socket_t* (*swas_socket_create_func)();
/*You only need to call the callback[cbs] when the connection is successful. Socket exceptions are returned through the status callback*/
typedef swas_int32_t (*swas_socket_connect_func)(swas_socket_t* sock_fd, const swas_sdk_host_info_t *host, swas_socket_connect_cb cbs);
typedef swas_int32_t (*swas_socket_send_func)(swas_socket_t* sock_fd, const swas_void_t* buf, size_t len);
typedef swas_int32_t (*swas_socket_close_func)(swas_socket_t* sock_fd);
typedef swas_int32_t (*swas_socket_clean_func)();

/**********socket************/
typedef struct swas_socket_func_s
{
    /*register by the user
        then called by the swas sdk*/
    swas_socket_init_func init_func;
    swas_socket_create_func create_func;
    swas_socket_connect_func connect_func;
    swas_socket_send_func send_func;
    swas_socket_close_func close_func;
    swas_socket_clean_func clean_func;
} swas_socket_func_t;

typedef struct swas_socket_cb_s
{
    /*register by the swas sdk
        then called by the user*/
    swas_socket_send_cb send_cb_func;        /* After the SDK is initialized[swas_sdk_init], users can call swas_socket_send_cb */
    swas_socket_recv_cb recv_cb_func;        /* After the SDK is initialized[swas_sdk_init], users can call swas_socket_recv_cb */
    swas_socket_status_cb status_cb_func;    /* After the SDK is initialized[swas_sdk_init], users can call swas_socket_status_cb */
} swas_socket_cb_t;

extern swas_socket_cb_t* swas_socket_get_socket_cb_operations();

/**
 * The socket sends a message to the upper layer through the callback function
 *
 * @param[in] sock_fd: Socket handle
 * @param[in] len: data length
 *
 */
#define swas_socket_send_cb(sock_fd, len) (swas_socket_get_socket_cb_operations()->send_cb_func(sock_fd, len))

/**
  * Socket asynchronous callback mode, which must be called upon successful receipt of data The call function notifies
  * the upper layer
  *
  * @param[in] sock_fd: Socket handle
  * @param[in] pbuf: the buffer receiving data
  * @param[in] len: The length of the buffer receiving data
  *
  */
#define swas_socket_recv_cb(sock_fd, buf, len) (swas_socket_get_socket_cb_operations()->recv_cb_func(sock_fd, buf, len))

/**
  * This callback must be called when the network socket status is abnormal to notify the upper layer developer,
  * such as connection timeout, send failure, Failed to receive, closed link, etc
  *
  * @param[in] sock_fd: Socket handle
  * @param[in] status: error state[swas_socket_status_error_e]
  *
  */
#define swas_socket_status_cb(sock_fd, status) (swas_socket_get_socket_cb_operations()->status_cb_func(sock_fd, status))

#ifdef __cplusplus
}
#endif

#endif /* SWAS_SOCKET_H__ */
