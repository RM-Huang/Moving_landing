#ifndef __CHCNAV_ROS_FILE_HPP__
#define __CHCNAV_ROS_FILE_HPP__

#include "device_connector.hpp"
#include "serial/serial.h"
#include <pthread.h>
#include <stdio.h>
#include <string>
#include <termios.h>

using namespace serial;

class file_common : public hc__device_connector
{
private:
    std::string path = ""; // serial port
    FILE *fp = NULL;
    int status = -1; // -1 not connected. 1 connected

public:
    /**
     * @brief default Construct
     * */
    file_common(void) : hc__device_connector()
    {
        this->status = -1;
    };

    /**
     * @brief serial_common object
     *
     * @param port      // serial port
     * @param speed     // bound rate. 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
     * @param bits      // data bits,such 7/8 etc
     * @param stop      // stop bits, 1/2 etc
     * @param parity    // checkout bits. None Odd Even.
     *
     * @note this constructor will not open the serial port.
     */
    file_common(std::string path) : hc__device_connector()
    {
        this->status = -1;
        this->path = path;
    }

    /**
     * @brief open a serial port
     *
     * @note if a object call this function which has opened a serial port, this func will do nothing.
     *
     * @return int < 0 failed. 0 success. 1 already opened.
     */
    int connect(void)
    {
        if (this->status != -1)
            this->disconnect();

        if (access(this->path.c_str(), F_OK) != 0)
        {
            fprintf(stderr, "access [%s] failed\n", path.c_str());
            return -1;
        }

        this->fp = fopen(this->path.c_str(), "r");
        if (NULL != this->fp)
            this->status = 1;

        return 0;
    }

    /**
     * @brief write data to device
     *
     * @param data data block
     * @param len length of data
     * @return int nums of written. -1 means write fail.
     */
    int write(char *data, unsigned int len)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "file open failed, reopen!\n");
            this->connect();
            sleep(1);
        }

        return fwrite((unsigned char *)data, 1, len, this->fp);
    }

    /**
     * @brief read data from device
     *
     * @param data storage of data
     * @param maxsize max size of the data memory
     * @return int nums of read. -1 means failed.
     */
    int read(char *data, unsigned int maxsize)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "file open failed, reopen!\n");
            this->connect();
            sleep(1);
        }
        return fread((unsigned char *)data, 1, maxsize, this->fp);
    }

    /**
     * @brief close this object
     *
     * @return int 0 means succeed. -1 failed.
     */
    int disconnect(void)
    {
        this->status = -1;

        if (this->fp != NULL)
        {
            fclose(fp);
            this->fp = NULL;
        }

        return 0;
    }

    /**
     * @brief Destroy object and close serial
     */
    ~file_common() { this->disconnect(); }
};

#endif