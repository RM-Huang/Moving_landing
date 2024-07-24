#ifndef __HC_UDP_COMMON_HPP_
#define __HC_UDP_COMMON_HPP_

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

class udp_common : public hc__device_connector
{
private:
    int status = -1;              // 连接状态。 -1 未连接，1 已连接
    int port;                     // 端口号
    int socketfd;                 // socket fd
    struct sockaddr_in sock_addr; // ip信息结构体

public:
    udp_common(int port) : hc__device_connector()
    {
        this->port = port;

        this->sock_addr.sin_family = AF_INET;
        this->sock_addr.sin_port = htons(this->port);
        this->sock_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    }

    int connect(void)
    {
        if (this->status != -1)
            this->disconnect();

        // 建立套接字
        this->socketfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (bind(this->socketfd, (struct sockaddr *)&this->sock_addr, sizeof(this->sock_addr)) == -1)
        {
            this->status = -1;
            return -1;
        }
        else
        {
            this->status = 1;
            return 0;
        }
    }

    int write(char *data, unsigned int len)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "udp socket not created! recreate now\n");
            this->connect();
            sleep(1);
        }

        int write_bytes = sendto(this->socketfd, data, len, 0, (struct sockaddr *)(&this->sock_addr), sizeof(this->sock_addr));
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
            fprintf(stderr, "udp socket not created! recreate now\n");
            this->connect();
            sleep(1);
        }

        socklen_t len = sizeof(this->sock_addr);
        int read_bytes = recvfrom(this->socketfd, data, maxsize, 0, (struct sockaddr *)(&this->sock_addr), &len);
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