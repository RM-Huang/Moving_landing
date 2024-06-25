#ifndef __HC_CGI_PROTOCOL_H_
#define __HC_CGI_PROTOCOL_H_

#include <stdio.h>
#include <sys/types.h>

__BEGIN_DECLS

/**
 * @brief 定义 CGI HEADER 协议消息号
 * */
typedef enum _hc_cgi_protocol_e
{
    RAWGNSSPVATB = 0x0101,
    RAWIMUIB = 0x0102,
    RAWODOB = 0x0103,
    RAWRTCMPB = 0x0104,
    RAWRTCMSB = 0x0105,
    RAWRTCMB = 0x0106,
    RAWIMUB = 0x0107,
    RAWIMUVB = 0x0108,
    RAWGSVB = 0x0109,
    RAWNMEAB = 0x010A,
    INSPVATB = 0x0201,
    INSPVATZCB = 0x1201,
    PINFOLTSB = 0x1301,
    PINFONZB = 0x1302,
} hc_cgi_protocol_e;

typedef struct _hc_cgi_header
{
    unsigned char sync[4];         // CGI_HEADER 为: 0xaa 0xcc 0x48 0x43
    unsigned short message_length; // 消息实际长度，不包含头和校验
    unsigned short message_id;     // 消息号
    unsigned short gps_week;       // GPS周
    unsigned int gps_week_ms;      // GPS周内秒
    unsigned int sn;               // 设备序列号
    char receiver[4];              // 预留
} hc_cgi_header;

typedef struct _hc_cgi_short_header
{
    unsigned char sync[4];         // CGI_HEADER 为: 0xaa 0x55 0x48 0x43
    unsigned short message_length; // 消息实际长度，不包含头和校验
    unsigned short message_id;     // 消息号
} hc_cgi_short_header;

/**
 * @brief 校验crc结果
 *
 * @param ucBuffer 存储协议内容
 * @param len 协议总长度（header+crc32+内容）
 * @return int @c 0 校验成功， @c -1 校验失败
 * */
int hc__cgi_check_crc32(unsigned char *ucBuffer, unsigned int len);

__END_DECLS

#endif