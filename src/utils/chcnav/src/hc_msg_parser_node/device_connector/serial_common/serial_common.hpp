#ifndef __CHCNAV_ROS_SERIAL_HPP__
#define __CHCNAV_ROS_SERIAL_HPP__

#include "device_connector.hpp"
#include "serial/serial.h"
#include <pthread.h>
#include <string>
#include <termios.h>

using namespace serial;

class serial_common : public hc__device_connector
{
private:
    std::string port = "NULL";          // serial port
    Serial *serial_device;              // point of serial::Serial obj
    int status = -1;                    // -1 not connected. 1 connected
    uint32_t baudrate = 115200;         // bound rate. 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
    bytesize_t bytesize = eightbits;    // data bits,such 7/8 etc
    parity_t parity = parity_none;      // checkout bits. None Odd Even.
    stopbits_t stopbits = stopbits_one; // stop bits, 1/2 etc

public:
    /**
     * @brief default Construct
     * */
    serial_common(void) : hc__device_connector()
    {
        this->status = -1;
    };

    /**
     * @brief serial_common object
     *
     * @param port      // serial port
     * @param baudrate     // bound rate. 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
     * @param bits      // data bits,such 7/8 etc
     * @param stop      // stop bits, 1/2 etc
     * @param parity    // checkout bits. None Odd Even.
     *
     * @note this constructor will not open the serial port.
     */
    serial_common(std::string port, int baudrate, short bits, short stop, std::string parity) : hc__device_connector()
    {
        this->status = -1;
        this->set(port, baudrate, bits, stop, parity);
    }

    int set(std::string port, int baudrate, short bits, short stop, std::string parity)
    {
        int baudrate_attr[] = {300, 600, 1200, 2400, 4800, 9600,
                            19200, 38400, 57600, 115200, 230400, 460800, 921600};

        // disconnect first if already connnect
        if (this->status == 1)
            this->disconnect();

        // try access the port
        if (access(port.c_str(), F_OK) != 0)
        {
            fprintf(stderr, "access [%s] failed\n", port.c_str());
            return -1;
        }
        this->port = port;

        // set baudrate
        this->baudrate = -1;
        for (int temp_i = 0; temp_i < sizeof(baudrate_attr) / sizeof(int); temp_i++)
        {
            if (baudrate == baudrate_attr[temp_i])
                this->baudrate = baudrate_attr[temp_i];
        }
        if (this->baudrate == -1)
        {
            fprintf(stderr, "error baudrate\n");
            return -1;
        }

        // set databits
        switch (bits)
        {
            case 5:
                this->bytesize = fivebits;
                break;
            case 6:
                this->bytesize = sixbits;
                break;
            case 7:
                this->bytesize = sevenbits;
                break;
            case 8:
                this->bytesize = eightbits;
                break;
            default:
                fprintf(stderr, "error databits\n");
                break;
        }

        // set stopbits
        switch (stop)
        {
            case 1:
                this->stopbits = stopbits_one;
                break;
            case 2:
                this->stopbits = stopbits_two;
                break;
            default:
                fprintf(stderr, "error stopbits\n");
                break;
        }

        // set parity
        if (parity.compare("None") == 0)
        {
            this->parity = parity_none;
        }
        else if (parity.compare("Odd") != 0)
        {
            this->parity = parity_odd;
        }
        else if (parity.compare("Even") != 0)
        {
            this->parity = parity_even;
        }
        else if (parity.compare("Mark") != 0)
        {
            this->parity = parity_mark;
        }
        else if (parity.compare("Space") != 0)
        {
            this->parity = parity_space;
        }
        else
        {
            fprintf(stderr, "error parity [%s]\n", parity.c_str());
            return -1;
        }

        return 0;
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
        this->serial_device = new Serial(this->port, this->baudrate, Timeout::simpleTimeout(0), this->bytesize, this->parity, this->stopbits, flowcontrol_none);
        this->serial_device->setRTS(true);
        this->serial_device->setDTR(true);

        if (this->serial_device->isOpen())
            this->status = 1;
        else
            this->status = -1;

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
            fprintf(stderr, "serial disconnect, reconnect!\n");
            this->connect();
            sleep(1);
        }

        return this->serial_device->write((unsigned char *)data, len);
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
            fprintf(stderr, "serial disconnect, reconnect!\n");
            this->connect();
            sleep(1);
        }
        return this->serial_device->read((unsigned char *)data, maxsize);
    }

    /**
     * @brief close this object
     *
     * @return int 0 means succeed. -1 failed.
     */
    int disconnect(void)
    {
        this->status = -1;

        if (this->serial_device->isOpen())
            this->serial_device->close();

        return 0;
    }

    /**
     * @brief Destroy object and close serial
     */
    ~serial_common() { this->disconnect(); }
};

#endif