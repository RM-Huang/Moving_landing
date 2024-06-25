#ifndef __HC_TCP_COMMON_HPP_
#define __HC_TCP_COMMON_HPP_

#include "device_connector.hpp"

#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class tcp_common : public hc__device_connector
{
private:
    int status = -1;                // 连接状态。 -1 未连接，1 已连接
    std::string host;               // ip
    int port;                       // 端口号
    int socketfd;                   // socket fd
    struct sockaddr_in client_addr; // 客户端信息结构体

public:
    tcp_common(std::string host, int port) : hc__device_connector()
    {
        this->host = host;
        this->port = port;

        this->client_addr.sin_family = AF_INET;
        this->client_addr.sin_port = htons(this->port);
        this->client_addr.sin_addr.s_addr = inet_addr(this->host.c_str());
    }

    int connect(void)
    {
        if (this->status != -1)
            this->disconnect();

        // 建立套接字
        this->socketfd = socket(AF_INET, SOCK_STREAM, 0);

        int ret = ::connect(this->socketfd, (struct sockaddr *)&this->client_addr, sizeof(struct sockaddr_in));
        if (ret < 0)
        {
            this->status = -1;
            fprintf(stderr, "tcp connect fail!\n");
            return -1;
        }

        fprintf(stdout, "tcp connect success!\n");
        this->status = 1;

        int flag = fcntl(this->socketfd, F_GETFL, 0);
        fcntl(this->socketfd, F_SETFL, flag | O_NONBLOCK);

        return 0;
    }

    int write(char *data, unsigned int len)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "tcp disconnect, reconnect!\n");
            this->connect();
            sleep(1);
        }

        int write_bytes = ::write(this->socketfd, data, len);
        if (write_bytes == -1)
        {
            printf("socker [%d] send failed\n", this->socketfd);
            this->disconnect();
            return -1;
        }

        return write_bytes;
    }

    int read(char *data, unsigned int maxsize)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "tcp disconnect, reconnect!\n");
            this->connect();
            sleep(1);
        }

        int read_bytes = ::read(this->socketfd, data, maxsize);
        if (read_bytes <= 0)
        {
            if (errno == EWOULDBLOCK) // nothing sended by client sock
            {
                errno = 0;
            }
            else
            {
                printf("socker [%d] read failed\n", this->socketfd);
                this->disconnect();
                return -1;
            }
        }

        return read_bytes;
    }

    int disconnect(void)
    {
        close(socketfd);
        this->status = -1;

        return 0;
    }
};

#endif