/**
 * @file serial_port.hpp
 * @author luoyebai(2112216825@qq.com),davi(1224802565@qq.com),donghao
 * @brief 这份是我抄学长的懒得改了
 * 传数据的话在第400行左右传给vdata
 * data_file 的 data文件里有个serial_port文件记得复制
 * 修复了接受电控数据缓慢的问题
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _SERIAL_PORT_HPP_
#define _SERIAL_PORT_HPP_

// std
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <queue>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include"../detector/detector.hpp"

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)
#define DATA_LENGTH 16      // 接受的数据位数
#define ONCE_READ_LENGTH 31 // 一次读多少数据

struct timeval tv;

/**
 * @brief 串口类
 *
 */
class SerialPort
{
private:
    const speed_t baud_rate_ = 115200;              // 波特率
    const std::string uart_device_ = "/dev/ttyUSB"; // 串口/名
    // const char *uart_device_ = "/dev/pts/17"; // 串口名
    int serial_port_number_; // 串口号
    char parit_ = 'N';
    char data_bit_ = 8;      // 数据位
    char stop_bit_ = 1;      // 停止位
    bool is_synchronizable_; // 是否同步
    // unsigned char rdata[255]; // raw_data 原始数据
    unsigned char t_data_[30]; // transfrom data 转换数据
    bool SetBaudRate();
    bool SetBit();

public:
    SerialPort() = default;
    ~SerialPort() = default;

    /**
     * @brief 打开串口
     *
     * @return true
     * @return false
     */
    bool OpenPort();

    /**
     * @brief 数据换算
     *
     * @param data 数据
     */
    void TransformData(const VisionData data);

    void Send()
    {
        write(serial_port_number_, t_data_, 18);
        return;
    };
    void ClosePort()
    {
        close(serial_port_number_);
        return;
    };
};

bool SerialPort::OpenPort()
{
    // serial_port_number_ = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
    // if(serial_port_number_==-1)
    //{
    //         puts("Can't open Serial\n");
    //         return false;
    // }
    for (size_t i = 0; i < 6; ++i)
    {
        const std::string uart_path = std::string(uart_device_) + std::to_string(i);
        serial_port_number_ = open(uart_path.c_str(), O_RDWR | O_NOCTTY |
                                                          O_NDELAY); // O_NDELAY 不关心另一端是否在使用串口

        if (serial_port_number_ == -1)
        {
            if (i == 5)
            {
                puts("Can't open Serial\n");

                return false;
            }
            // perror(uart_path.c_str());
            continue;
        }
        break;
    }

    puts("Opening...\n");
    SetBaudRate(); // 设置波特率

    // if (SetBit() == false)
    // {
    //     puts("Set Parity Error\n");
    //     exit(0);
    // }
    return true;
}
bool SerialPort::SetBaudRate()
{
    int speed_arr[] = {
#include "../data/init/serial_port/speed_arr"
    };
    int name_arr[] = {
#include "../data/init/serial_port/name_arr"
    };
    size_t i;
    int status;
    struct termios Opt;
    tcgetattr(serial_port_number_, &Opt);

    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (baud_rate_ == name_arr[i])
        {
            tcflush(serial_port_number_, TCIOFLUSH); // 清空缓冲区的内容
            cfsetispeed(&Opt, speed_arr[i]);         // 设置接受和发送的波特率
            cfsetospeed(&Opt, speed_arr[i]);
            status =
                tcsetattr(serial_port_number_, TCSANOW, &Opt); // 使设置立即生效

            if (status != 0)
            {
                perror("tcsetattr fd1");
                return false;
            }

            tcflush(serial_port_number_, TCIOFLUSH);
        }
    }
    return true;
}

bool SerialPort::SetBit()
{
    struct termios termios_p;

    if (tcgetattr(serial_port_number_, &termios_p) != 0)
    {
        perror("SetupSerial 1");
        return false;
    }

    termios_p.c_cflag |= (CLOCAL | CREAD); // 接受数据
    termios_p.c_cflag &= ~CSIZE;           // 设置数据位数

    switch (data_bit_)
    {
    case 7:
        termios_p.c_cflag |= CS7;
        break;

    case 8:
        termios_p.c_cflag |= CS8;
        break;

    default:
        fprintf(stderr, "Unsupported data size\n");
        return false;
    }

    // 设置奇偶校验位double
    switch (parit_)
    {
    case 'n':
    case 'N':
        termios_p.c_cflag &= ~PARENB; /* Clear parity enable 无校验位 */
        termios_p.c_iflag &= ~INPCK;  /* Enable parity checking */
        break;

    case 'o':
    case 'O':
        termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
        termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
        break;

    case 'e':
    case 'E':
        termios_p.c_cflag |= PARENB;  /* Enable parity */
        termios_p.c_cflag &= ~PARODD; /* 转换为偶效验*/
        termios_p.c_iflag |= INPCK;   /* Disnable parity checking */
        break;

    case 'S':
    case 's': /*as no parity*/
        termios_p.c_cflag &= ~PARENB;
        termios_p.c_cflag &= ~CSTOPB;
        break;

    default:
        fprintf(stderr, "Unsupported parity\n");
        return false;
    }

    /* 设置停止位*/
    switch (stop_bit_)
    {
    case 1:
        termios_p.c_cflag &= ~CSTOPB; // 一位停止位
        break;

    case 2:
        termios_p.c_cflag |= CSTOPB; // 两位停止位
        break;

    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return false;
    }

    /* Set input parity option */
    if (parit_ != 'n')
        termios_p.c_iflag |= INPCK;

    tcflush(serial_port_number_, TCIFLUSH);               // 清除输入缓存区
    termios_p.c_cc[VTIME] = 150;                          /* 设置超时15 seconds*/
    termios_p.c_cc[VMIN] = 0;                             // 最小接收字符
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input原始输入*/
    termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios_p.c_iflag &= ~(ICRNL | IGNCR);
    termios_p.c_oflag &= ~OPOST; /*Output禁用输出处理*/

    if (tcsetattr(serial_port_number_, TCSANOW, &termios_p) !=
        0) /* Update the options and do it NOW */
    {
        perror("SetupSerial 3");
        return false;
    }

    return true;
}

void SerialPort::TransformData(const VisionData data)
{
    t_data_[0] = 0xA5;

    t_data_[1] = data.fixed_derection;
    t_data_[2] = data.signal;

    t_data_[3] = 0xFF;
    return;
}



#endif //! _SERIAL_PORT_HPP_
