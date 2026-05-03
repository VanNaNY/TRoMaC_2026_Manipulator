#include "Serial.h"

namespace TRoMaC
{
    Uart::Uart() {}

    Uart::~Uart()
    {
        if (serial_id >= 0) close(serial_id);
    }

    void Uart::Close()
    {
        if (serial_id >= 0) {
            close(serial_id);
            serial_id = -1;
        }
    }

    int Uart::open_port(int comport)
    {
        int fd = 0;
        if (comport == 1)
        {
            fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
            if (-1 == fd)
            {
                perror("Can't Open Serial Port");
                return(-1);
            }
        }
        else if(comport == 2)
        {
            fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
            if (-1 == fd)
            {
                perror("Can't Open Serial Port");
                return(-1);
            }
        }
        else if (comport == 3)
        {
            fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
            if (-1 == fd)
            {
                perror("Can't Open Serial Port");
                return(-1);
            }
        }
        if(fcntl(fd, F_SETFL, 0) < 0)
        {
            printf("fcntl failed!\n");
        }
        else
        {
            printf("fcntl = %d\n", fcntl(fd, F_SETFL, 0));
        }
        if(isatty(STDIN_FILENO) == 0)
        {
            printf("standard input is not a terminal device\n");
        }
        else
        {
            printf("isatty success!\n");
        }
        printf("fd-open = %d\n", fd);
        return fd;
    }

    int Uart::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
    {
        struct termios newtio, oldtio;
        if (tcgetattr(fd, &oldtio) != 0)
        {
            perror("SetupSerial 1");
            printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio));
            return -1;
        }
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag  |=  CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch(nBits)
        {
            case 7:
                newtio.c_cflag |= CS7;
                break;
            case 8:
                newtio.c_cflag |= CS8;
                break;
        }
        switch(nEvent)
        {
            case 'o':
            case 'O':
                newtio.c_cflag |= PARENB;
                newtio.c_cflag |= PARODD;
                newtio.c_iflag |= (INPCK | ISTRIP);
                break;
            case 'e':
            case 'E':
                newtio.c_iflag |= (INPCK | ISTRIP);
                newtio.c_cflag |= PARENB;
                newtio.c_cflag &= ~PARODD;
                break;
            case 'n':
            case 'N':
                newtio.c_cflag &= ~PARENB;
                break;
            default:
                break;
        }
        switch(nSpeed)
        {
            case 2400:
                cfsetispeed(&newtio, B2400);
                cfsetospeed(&newtio, B2400);
                break;
            case 4800:
                cfsetispeed(&newtio, B4800);
                cfsetospeed(&newtio, B4800);
                break;
            case 9600:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
                break;
            case 115200:
                cfsetispeed(&newtio, B115200);
                cfsetospeed(&newtio, B115200);
                break;
            case 460800:
                cfsetispeed(&newtio, B460800);
                cfsetospeed(&newtio, B460800);
                break;
            case 921600:
                cfsetispeed(&newtio, B921600);
                cfsetospeed(&newtio, B921600);
                break;
            case 1000000:
                cfsetispeed(&newtio, B1000000);
                cfsetospeed(&newtio, B1000000);
                break;
            default:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
                break;
        }
        if(nStop == 1)
        {
            newtio.c_cflag &=  ~CSTOPB;
        }
        else if (nStop == 2)
        {
            newtio.c_cflag |=  CSTOPB;
        }
        newtio.c_cc[VTIME] = 1;
        newtio.c_cc[VMIN]  = 1;    // 逐字节返回，配合状态机
        tcflush(fd, TCIFLUSH);

        if((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        {
            perror("com set error");
            return -1;
        }
        printf("set done!\n");
        return 0;
    }

    int Uart::open_port(const std::string& device_path)
    {
        int fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0)
        {
            perror("Can't Open Serial Port");
            return -1;
        }
        if(fcntl(fd, F_SETFL, 0) < 0)
        {
            printf("fcntl failed!\n");
        }
        printf("fd-open = %d (%s)\n", fd, device_path.c_str());
        return fd;
    }

    bool Uart::Open(const std::string& device_path, int _speed)
    {
        last_read_disconnected_ = false;
        // 重连入口需要重置帧解析状态机和缓冲区，避免上一帧的残留 payload 干扰新会话
        rx_state_      = RxState::WAIT_HEADER;
        rx_payload_idx_ = 0;
        rx_ring_pos_   = 0;
        rx_ring_len_   = 0;

        int fd = open_port(device_path);
        if(fd < 0)
        {
            // 频繁失败时（重连循环每秒一次）保持静默，避免 perror 刷屏
            return false;
        }
        if((set_opt(fd, _speed, 8, 'N', 1)) < 0)
        {
            perror("set_opt error");
            close(fd);
            return false;
        }
        serial_id = fd;
        return true;
    }

    bool Uart::Open(int comport, int _speed)
    {
        int fd = open_port(comport);
        if(fd < 0)
        {
            perror("open_port error");
            return false;
        }
        if((set_opt(fd, _speed, 8, 'N', 1)) < 0)
        {
            perror("set_opt error");
            return false;
        }
        serial_id = fd;
        return true;
    }

    bool Uart::checkSerial()
    {
        struct termios oldtio;
        if(tcgetattr(serial_id, &oldtio) == 0)
        {
            return true;
        }
        serial_id = -1;
        return false;
    }

    // 每次调用尝试从串口读取并解析出一帧完整数据。
    // 返回 true 表示成功解析一帧，数据已写入 read_data。
    bool Uart::ReadData()
    {
        // 处理缓冲区中的每个字节
        for (;;)
        {
            // 缓冲区耗尽，批量读取一次
            if (rx_ring_pos_ >= rx_ring_len_)
            {
                rx_ring_pos_ = 0;
                rx_ring_len_ = read(serial_id, rx_ring_, sizeof(rx_ring_));
                if (rx_ring_len_ <= 0)
                {
                    // select 已报 ready 仍读不到任何数据：USB 拔除导致 EOF（read=0）
                    // 或设备移除（read=-1, errno=EIO/ENXIO/EBADF）。两种都视为断开。
                    rx_ring_len_ = 0;
                    last_read_disconnected_ = true;
                    return false;
                }
            }

            uint8_t byte = rx_ring_[rx_ring_pos_++];

            switch (rx_state_)
            {
            case RxState::WAIT_HEADER:
                if (byte == FRAME_HEADER)
                {
                    rx_payload_idx_ = 0;
                    rx_state_ = RxState::READ_PAYLOAD;
                }
                // 非帧头字节直接丢弃，继续扫描
                break;

            case RxState::READ_PAYLOAD:
                if (byte == FRAME_HEADER)
                {
                    // payload 中间又遇到帧头 → 之前的帧已损坏，
                    // 把这个字节当作新帧的帧头，重新开始收 payload
                    rx_payload_idx_ = 0;
                    break;
                }
                rx_payload_buf_[rx_payload_idx_++] = byte;
                if (rx_payload_idx_ >= RX_PAYLOAD_SIZE)
                {
                    rx_state_ = RxState::WAIT_TAIL;
                }
                break;

            case RxState::WAIT_TAIL:
                if (byte == FRAME_TAIL)
                {
                    // 完整帧！解析 payload → read_data
                    std::memcpy(&read_data, rx_payload_buf_, RX_PAYLOAD_SIZE);
                    rx_state_ = RxState::WAIT_HEADER;
                    return true;
                }
                else if (byte == FRAME_HEADER)
                {
                    // 帧尾位置不对，但这是新帧的帧头
                    rx_payload_idx_ = 0;
                    rx_state_ = RxState::READ_PAYLOAD;
                }
                else
                {
                    // 帧尾校验失败，丢弃，回到等待帧头
                    rx_state_ = RxState::WAIT_HEADER;
                }
                break;
            }
        }
    }

    void Uart::send(const VisionData& data)
    {
        // 串口未打开（重连期间 hw_iface 已关闭 fd）：静默丢弃，避免 write(-1,…) 刷屏
        if (serial_id < 0) return;

        TXunion.VisionFrameTX.Joint_1 = data.Joint_1;
        TXunion.VisionFrameTX.Joint_2 = data.Joint_2;
        TXunion.VisionFrameTX.Joint_3 = data.Joint_3;
        TXunion.VisionFrameTX.Joint_4 = data.Joint_4;
        TXunion.VisionFrameTX.Joint_5 = data.Joint_5;
        TXunion.VisionFrameTX.Joint_6 = data.Joint_6;

        uint8_t frame[TX_FRAME_SIZE];
        frame[0] = FRAME_HEADER;
        std::memcpy(&frame[1], TXunion.u8arr, TX_PAYLOAD_SIZE);
        frame[TX_FRAME_SIZE - 1] = FRAME_TAIL;

        auto written = write(serial_id, frame, TX_FRAME_SIZE);
        tcflush(serial_id, TCOFLUSH);
        if (written != TX_FRAME_SIZE)
        {
            std::cout << "send error! expected " << TX_FRAME_SIZE
                      << " bytes, wrote: " << written << std::endl;
        }
    }
}
