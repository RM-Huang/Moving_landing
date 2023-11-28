#pragma once
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*POSIX 终端控制定义*/
#include <stdio.h>   /*标准输入输出定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <unistd.h>  /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <thread>
#include <memory.h>
#include <iostream>

class SerialPort
{

public:
    SerialPort(const std::string &path);
    SerialPort();
    ~SerialPort();
    enum BaudRate	//波特率 枚举 
    {
        BR0 = 0000000,
        BR50 = 0000001,
        BR75 = 0000002,
        BR110 = 0000003,
        BR134 = 0000004,
        BR150 = 0000005,
        BR200 = 0000006,
        BR300 = 0000007,
        BR600 = 0000010,
        BR1200 = 0000011,
        BR1800 = 0000012,
        BR2400 = 0000013,
        BR4800 = 0000014,
        BR9600 = 0000015,
        BR19200 = 0000016,
        BR38400 = 0000017,
        BR57600 = 0010001,
        BR115200 = 0010002,
        BR230400 = 0010003,
        BR460800 = 0010004,
        BR500000 = 0010005,
        BR576000 = 0010006,
        BR921600 = 0010007,
        BR1000000 = 0010010,
        BR1152000 = 0010011,
        BR1500000 = 0010012,
        BR2000000 = 0010013,
        BR2500000 = 0010014,
        BR3000000 = 0010015,
        BR3500000 = 0010016,
        BR4000000 = 0010017
    };

    enum DataBits //数据位
    {
        DataBits5,
        DataBits6,
        DataBits7,
        DataBits8,
    };

    enum StopBits	//停止位
    {
        StopBits1,
        StopBits2
    };

    enum Parity 	//校验位
    {
        ParityNone,
        ParityEven,
        PariteMark,
        ParityOdd,
        ParitySpace
    };

    struct OpenOptions	//串口参数结构体
    {
        bool autoOpen;
        BaudRate baudRate;
        DataBits dataBits;
        StopBits stopBits;
        Parity parity;
        bool xon;
        bool xoff;
        bool xany;
        int vmin;
        int vtime;
    };
    static BaudRate BaudRateMake(unsigned long baudrate);
    static const OpenOptions defaultOptions;
	
	//串口打开
    bool open();
    bool open(const std::string &path, const OpenOptions &options);

    bool isOpen() const;
    bool _is_open = false;      //open state
	
	//写数据
    int write(const void *data, int length);
    //读数据
    int read(void *data, int length);
	//串口及监听线程关闭
    void close();
	//串口监听线程
    void openThread();
    void openThread(void (*pf)(char *buff,uint16_t len));

protected:
    void termiosOptions(termios &tios, const OpenOptions &options);
    int _tty_fd;                //opened port
private:
    std::string _path;          //port address
    OpenOptions _open_options;  //port data
    // int _tty_fd;                //opened port
};
//重载操作符
bool operator==(const SerialPort::OpenOptions &lhs, const SerialPort::OpenOptions &rhs);
bool operator!=(const SerialPort::OpenOptions &lhs, const SerialPort::OpenOptions &rhs);

SerialPort::SerialPort()
{
    _open_options = SerialPort::defaultOptions;
}
SerialPort::SerialPort(const std::string &path)
{
    _path = path;
    _open_options = SerialPort::defaultOptions;
}
SerialPort::~SerialPort()
{
    close();
}

const SerialPort::OpenOptions SerialPort::defaultOptions = {
    true,             //    bool autoOpen;
    SerialPort::BR115200,     //    BaudRate baudRate;
    SerialPort::DataBits8,  //    DataBits dataBits;
    SerialPort::StopBits1,  //    StopBits stopBits;
    SerialPort::ParityNone, //    Parity parity;
    false,            //    input xon       允许输入时对XON/XOFF流进行控制
    false,            //    input xoff      允许输入时对XON/XOFF流进行控制
    false,            //    input xany      输入任何字符将重启停止的输出
    0,                //    c_cc vmin       设置非规范模式下的超时时长和最小字符数：阻塞模式起作用
    50,               //    c_cc vtime      VTIME与VMIN配合使用，是指限定的传输或等待的最长时间 单位：0.1S
};

SerialPort::BaudRate SerialPort::BaudRateMake(unsigned long baudrate)
{
    switch (baudrate)
    {
    case 50:
        return BR50;
    case 75:
        return BR75;
    case 134:
        return BR134;
    case 150:
        return BR150;
    case 200:
        return BR200;
    case 300:
        return BR300;
    case 600:
        return BR600;
    case 1200:
        return BR1200;
    case 1800:
        return BR1800;
    case 2400:
        return BR2400;
    case 4800:
        return BR4800;
    case 9600:
        return BR9600;
    case 19200:
        return BR19200;
    case 38400:
        return BR38400;
    case 57600:
        return BR57600;
    case 115200:
        return BR115200;
    case 230400:
        return BR230400;
    case 460800:
        return BR460800;
    case 500000:
        return BR500000;
    case 576000:
        return BR576000;
    case 921600:
        return BR921600;
    case 1000000:
        return BR1000000;
    case 1152000:
        return BR1152000;
    case 1500000:
        return BR1500000;
    case 2000000:
        return BR2000000;
    case 2500000:
        return BR2500000;
    case 3000000:
        return BR3000000;
    case 3500000:
        return BR3500000;
    case 4000000:
        return BR4000000;
    default:
        break;
    }
    return BR0;
}

bool SerialPort::open(const std::string &path, const OpenOptions &options)
{
    if (_path != path)
        _path = path;
    if (_open_options != options)
        _open_options = options;
    /* O_RDWR 读写方式打开；
     * O_NOCTTY 不允许进程管理串口（不太理解，一般都选上）；
     * O_NDELAY 非阻塞（默认为阻塞，打开后也可以使用fcntl()重新设置）
     * O_NONBLOCK 如果路径名指向 FIFO/块文件/字符文件，则把文件的打开和后继 I/O设置为非阻塞模式
     * O_RDWR | O_NOCTTY                阻塞模式
     * O_RDWR | O_NOCTTY | O_NONBLOCK   非阻塞模式
     */
    _tty_fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_tty_fd < 0)
    {
        return false;
    }

    struct termios tios;
    termiosOptions(tios, options);
    tcsetattr(_tty_fd, TCSANOW, &tios); // TCSANOW立刻对值进行修改
    tcflush(_tty_fd, TCIOFLUSH);        // 清除所有正在发生的I/O数据。
    _is_open = true;
    return _is_open;
}

bool SerialPort::open()
{
    _is_open = open(_path, _open_options);
    return _is_open;
}

void SerialPort::termiosOptions(termios &tios, const OpenOptions &options)
{

    tcgetattr(_tty_fd, &tios);

    cfmakeraw(&tios);
    tios.c_cflag &= ~(CSIZE | CRTSCTS);
    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR);
    tios.c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    tios.c_oflag &= ~(OPOST | ONLCR);

    cfsetispeed(&tios, options.baudRate);
    cfsetospeed(&tios, options.baudRate);

    tios.c_iflag |= (options.xon ? IXON : 0) | (options.xoff ? IXOFF : 0) | (options.xany ? IXANY : 0);

    // data bits

    int databits[] = {CS5, CS6, CS7, CS8};
    tios.c_cflag &= ~0x30;
    tios.c_cflag |= databits[options.dataBits];

    // stop bits
    if (options.stopBits == StopBits2)
    {
        tios.c_cflag |= CSTOPB;
    }
    else
    {
        tios.c_cflag &= ~CSTOPB;
    }

    // parity
    if (options.parity == ParityNone)
    {
        tios.c_cflag &= ~PARENB;
    }
    else
    {
        tios.c_cflag |= PARENB;

        if (options.parity == PariteMark)
        {
            tios.c_cflag |= PARMRK;
        }
        else
        {
            tios.c_cflag &= ~PARMRK;
        }

        if (options.parity == ParityOdd)
        {
            tios.c_cflag |= PARODD;
        }
        else
        {
            tios.c_cflag &= ~PARODD;
        }
    }

    tios.c_cc[VMIN] = options.vmin;
    tios.c_cc[VTIME] = options.vtime;
}

bool SerialPort::isOpen() const
{
    return _is_open;
}

int SerialPort::write(const void *data, int length)
{
    return ::write(_tty_fd, data, length);
}

int SerialPort::read(void *data, int length)
{
    return ::read(_tty_fd, data, length);
}

void SerialPort::close()
{
    if(_is_open){
        ::close(_tty_fd);
        _is_open = false;
    } 
}
void SerialPort::openThread()
{
    std::thread serverThread(
        [&]()->void
        {
            char buffer[1024];
            int ret = 0;
            // signalInt.emit(10);
            std::cout<<" ID is" << std::this_thread::get_id() << std::endl;
            while (_is_open)
            {
                memset(buffer, 0, 1024);
                ret = this->read(buffer, 1024);
                if (ret > 0)
                {
                	std::cout << "success: buffer=\t"<<std::endl;
                }
            }
            return;
        });
    serverThread.detach();
}
void SerialPort::openThread(void (*pf)(char *buff,uint16_t len)) //带有回调函数
{
    std::thread serverThread(
        [&,pf]()->void //注意这里的pf，必须这么写，不然会报错
        {
            char buffer[512];
            int ret = 0;
            while (_is_open)
            {
                memset(buffer, 0, 512);
                ret = this->read(buffer, 512);
                if (ret > 0 && pf!=nullptr)
                {
                	// std::cout << "success: buffer=\t" << ret << std::endl;
                    (*pf)(buffer,ret);
                }
            }
            return;
        });
    serverThread.detach();
}

bool operator==(const SerialPort::OpenOptions &lhs, const SerialPort::OpenOptions &rhs)
{
    return lhs.autoOpen == rhs.autoOpen && lhs.baudRate == rhs.baudRate && lhs.dataBits == rhs.dataBits && lhs.parity == rhs.parity && lhs.stopBits == rhs.stopBits && lhs.vmin == rhs.vmin && lhs.vtime == rhs.vtime && lhs.xon == rhs.xon && lhs.xoff == rhs.xoff && lhs.xany == rhs.xany;
}

bool operator!=(const SerialPort::OpenOptions &lhs, const SerialPort::OpenOptions &rhs)
{
    return !(lhs == rhs);
}