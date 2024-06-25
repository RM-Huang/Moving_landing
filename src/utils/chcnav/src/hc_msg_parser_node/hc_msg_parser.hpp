#ifndef __HC_MSG_PARSER_NODE_HPP_
#define __HC_MSG_PARSER_NODE_HPP_

#include "ros/ros.h"

#include <string>
#include <unistd.h>

#include "chcnav/hc_sentence.h"
#include "chcnav/int8_array.h"
#include "chcnav/string.h"
#include "device_connector.hpp"
#include "hc_msg_parser.h"

using namespace std;

// 标识华测协议信息类型
typedef enum _hc__msg_type_e
{
    HC__MSG_NMEA = 0,         // NMEA协议
    HC__MSG_CGI_SHORT_HEADER, // CGI SHORT HEADER 自定义协议
    HC__MSG_CGI_HEADER,       // CGI HEADER 自定义协议
} hc__msg_type_e;

// 单条华测协议结构体
typedef struct _hc__single_msg_t
{
    hc__msg_type_e msg_type;

    union msg_value
    {
        struct nmea
        {
            // 常规nmea head（指第一个字段不包含$的值，例如：GPGGA）只有5位，定义为6位是因为华测自定义协议中存在6位。再 +1 保存 '\0'
            char header[6 + 1];
        } nmea;

        struct cgi_header
        {
            unsigned short msg_id; // header协议消息号
        } cgi_header;

        struct cgi_short_header
        {
            unsigned short msg_id; // short协议消息号
        } cgi_short_header;
    } type_value;

    char value[HC_PROTOCOL_MAX_LEN]; // 数据存储空间
    unsigned int value_len;          // 存储的数据长度
} hc__single_msg_t;

/**
 * @brief 供解析器使用的回调函数
 * */
static int parser_read_handler(void *buffer, size_t size, size_t *size_read, void *parser_id);

/**
 * @brief 声明节点处理类，该类发布 nmea_sentence、hc_sentence 两个话题，初始化时传入一个已经初始化过的device_connector对象。
 *        并从该对象中获取数据并将解析后的协议发布。
 * */
class hc__msg_parser_node;

/**
 * @brief 消息接受时的回调函数，
 *
 * @param single_msg
 * @return int
 * */
static int msg_recv_callback(hc__single_msg_t single_msg, hc__msg_parser_node *node);

class hc__msg_parser_node
{
private:
    int state = -1;          // 标识初始化状态 -1 初始化失败，1 初始化成功
    unsigned int hz = 0;     // 消息发布频率（每秒最大发布消息数量）
    hc__msg_parser_t parser; // 数据解析器
    hc__msg_token_t token;   // token
    string node_name;        // 节点名称
public:
    ros::NodeHandle *nh;         // 节点handle
    ros::NodeHandle *private_nh; // private 节点handle
    ros::Publisher nmea_puber;   // nmea topic
    ros::Publisher hc_puber;     // hc topic
    ros::Subscriber write_suber; // sub topic

    chcnav::string nmea_msg;    // nmea msg
    chcnav::hc_sentence hc_msg; // hc msg

    hc__device_connector *device_connector; // 设备连接器

    /**
     * @brief 构建一个节点
     *
     * @param hz 消息发布频率（每秒最大发布消息数量）
     * @param device_connector 已经初始化的设备连接器
     * @param node_name 节点名称
     * */
    hc__msg_parser_node(unsigned int hz, hc__device_connector *device_connector, ros::NodeHandle *nh, ros::NodeHandle *private_nh, string node_name)
    {
        if (NULL != nh && NULL != private_nh && NULL != device_connector)
        {
            this->nh = nh;
            this->private_nh = private_nh;

            // 初始化nmea协议话题
            this->nmea_puber = this->nh->advertise<chcnav::string>("nmea_sentence", 1000); // a public topic
            this->nmea_msg.header.seq = 0;
            this->nmea_msg.header.frame_id = node_name;

            // 初始化华测自定义协议话题
            this->hc_puber = this->nh->advertise<chcnav::hc_sentence>("hc_sentence", 1000); // a public topic
            this->hc_msg.header.seq = 0;
            this->hc_msg.header.frame_id = node_name;

            // 订阅写话题
            this->write_suber = this->private_nh->subscribe("write", 20, &hc__msg_parser_node::node_write, this);

            // 初始化解析器
            hc__init_msg_parser(&this->parser, HC_PROTOCOL_MAX_LEN, parser_read_handler, this);

            this->device_connector = device_connector;
            this->hz = hz;
            this->node_name = node_name;

            this->state = 1;
        }
        else
        {
            this->state = -1;
        }
    };

    /**
     * @brief 开始运行节点，解析并发布话题(节点是阻塞式运行的，可自行创建多进程实现并行)
     *
     * @param hc__single_msg_t 返回的消息结构体
     * */
    void node_start(void)
    {
        hc__single_msg_t single_msg;

        if (this->state != 1)
        {
            ROS_ERROR("node init failed !");
            return;
        }

        ros::AsyncSpinner spinner(1);
        spinner.start();

        usleep(300 * 1000); // 此处延时才能保证正常实现文件读取

        ros::Rate loop_rate(this->hz);
        while (ros::ok()) // 此处循环表示直到成功取到一次数据（指通过校验，hc cgi 自定义协议或者 nmea 协议）后才 break 该层循环进入 sleep
        {
            while (1)
            {
                hc__msg_parser_scan(&this->parser, &this->token);
                switch (token.type)
                {
                    case HC__TOKERN_HC_SHORT_MSG:
                    
                        single_msg.msg_type = HC__MSG_CGI_SHORT_HEADER;
                        single_msg.type_value.cgi_header.msg_id = (this->token.token_value.value[7] << 8) + this->token.token_value.value[6];
                        memcpy(single_msg.value, this->token.token_value.value, this->token.token_value.value_len);
                        single_msg.value_len = this->token.token_value.value_len;
                        break;
                    case HC__TOKERN_HC_MSG:
                        single_msg.msg_type = HC__MSG_CGI_HEADER;
                        single_msg.type_value.cgi_header.msg_id = (this->token.token_value.value[7] << 8) + this->token.token_value.value[6];
                        memcpy(single_msg.value, this->token.token_value.value, this->token.token_value.value_len);
                        single_msg.value_len = this->token.token_value.value_len;
                        break;
                    case HC__TOKERN_NMEA_MGS:
                        single_msg.msg_type = HC__MSG_NMEA;

                        if (this->token.token_value.value[6] == ',')
                            memcpy(single_msg.type_value.nmea.header, this->token.token_value.value + 1, 5);
                        else
                            memcpy(single_msg.type_value.nmea.header, this->token.token_value.value + 1, 6);

                        memcpy(single_msg.value, this->token.token_value.value, this->token.token_value.value_len);
                        single_msg.value_len = this->token.token_value.value_len;
                        break;

                    case HC__TOKERN_NONE:
                    case HC__TOKERN_ERROR:
                    default:
                        // ROS_WARN("[S%d] %s", parser.error.error_code, parser.error.description);
                        parser.error.error_code = HC__STATE_NONE;
                        memset(parser.error.description, 0x00, sizeof(parser.error.description));
                        parser.state = HC__STATE_S0;
                        break;
                }

                // 直到取到了一条合法消息才会休眠
                if (token.type == HC__TOKERN_HC_MSG || token.type == HC__TOKERN_NMEA_MGS || token.type == HC__TOKERN_HC_SHORT_MSG)
                {
                    msg_recv_callback(single_msg, this);
                    memset(&single_msg, 0, sizeof(hc__single_msg_t));
                    break;
                }
            }
            loop_rate.sleep();
        };
    };

    /**
     * @brief 获取私有对象节点名称
     * */
    string get_node_name(void) { return this->node_name; }

    /**
     * @brief 设备写入信息话题回调函数
     * */
    void node_write(const chcnav::int8_array::ConstPtr &msg)
    {
        this->device_connector->write((char *)&msg->data[0], msg->data.size());
    };

    ~hc__msg_parser_node() { this->device_connector->disconnect(); }
};

int parser_read_handler(void *buffer, size_t size, size_t *size_read, void *parser_id)
{
    hc__msg_parser_node *parser_node = (hc__msg_parser_node *)parser_id;

    int read_size = parser_node->device_connector->read((char *)buffer, size);
    if (read_size <= size && read_size > 0)
        *size_read = read_size;
    else
        *size_read = 0;

    return 0;
};

int msg_recv_callback(hc__single_msg_t single_msg, hc__msg_parser_node *node)
{
    int index;
    char raw_xor_string[5];
    unsigned char xor_check_uchar;
    // 由此函数发布的数据都是未经过 crc、xor 校验的协议数据

    // 但是 nmea 在此处经过校验再发布
    // 按设计功能划分方式来说不应该在此处校验，应该在 nmea 对应的 process_node 中校验。
    // 但考虑到目前 nmea 并不需要做后续处理（已经是ASCII字符串），为节省话题及节点资源，在此处校验
    switch (single_msg.msg_type)
    {
        case HC__MSG_NMEA:
            if (single_msg.value_len < 5)
                return -1;

            node->nmea_msg.header.seq++;
            node->nmea_msg.header.stamp = ros::Time::now();
            node->nmea_msg.sentence.clear();

            char nmea_end_2[2];
            nmea_end_2[0] = single_msg.value[single_msg.value_len - 2];
            nmea_end_2[1] = single_msg.value[single_msg.value_len - 1];

            // 去除nmea最后的\r\n，如需要则注释这两行
            single_msg.value[single_msg.value_len - 1] = '\0';
            single_msg.value[single_msg.value_len - 2] = '\0';

            node->nmea_msg.sentence.append(single_msg.value);

            // 校验 nmea
            xor_check_uchar = single_msg.value[0];
            for (index = 0; index < single_msg.value_len - 5; index++)
            {
                xor_check_uchar ^= single_msg.value[index]; // xor calculation
            }

            // 获取原始的异或校验值
            raw_xor_string[0] = '0';
            raw_xor_string[1] = 'x';
            raw_xor_string[2] = single_msg.value[single_msg.value_len - 4];
            raw_xor_string[3] = single_msg.value[single_msg.value_len - 3];
            raw_xor_string[4] = '\0';

            // 如果校验值与原始值一致则发布
            if (strtol(raw_xor_string, NULL, 16) != xor_check_uchar) 
            {
                fprintf(stderr, "[%s] xor check not pass\n", single_msg.value);
            }
            else if((nmea_end_2[0] != '\r') || (nmea_end_2[1] != '\n'))
            {
                fprintf(stderr, "[%s] tail check not pass\n", single_msg.value);
            }
            else
            {
                node->nmea_puber.publish(node->nmea_msg);                         
            }
            break;

        case HC__MSG_CGI_HEADER:
            // 赋值msg id
            node->hc_msg.msg_id = single_msg.type_value.cgi_header.msg_id;

            // 发布时间
            node->hc_msg.header.seq++;
            node->hc_msg.header.stamp = ros::Time::now();

            // 赋值data
            node->hc_msg.data.resize(single_msg.value_len);
            memcpy(&node->hc_msg.data[0], single_msg.value, single_msg.value_len);

            // 发布
            node->hc_puber.publish(node->hc_msg);
            break;

        case HC__MSG_CGI_SHORT_HEADER:
                    // 赋值msg id
            node->hc_msg.msg_id = single_msg.type_value.cgi_short_header.msg_id;

            // 发布时间
            node->hc_msg.header.seq++;
            node->hc_msg.header.stamp = ros::Time::now();

            // 赋值data
            node->hc_msg.data.resize(single_msg.value_len);
            memcpy(&node->hc_msg.data[0], single_msg.value, single_msg.value_len);

            // 发布
            node->hc_puber.publish(node->hc_msg);
            break;
        default:
            ROS_WARN("protocol unsupport");
            break;
    }

    return 0;
}

#endif