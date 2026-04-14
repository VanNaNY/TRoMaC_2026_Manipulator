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
            fd = open( "/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY); 
            if (-1 == fd)
            { 
                perror("Can't Open Serial Port"); 
                return(-1); 
            } 
        } 
        else if(comport == 2)
        {     
            fd = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); 
            if (-1 == fd)
            { 
                perror("Can't Open Serial Port"); 
                return(-1); 
            } 
        } 
        else if (comport == 3)
        { 
            fd = open( "/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY); 
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
        newtio.c_cc[VMIN] = READ_DATA_NUM;
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
        int fd = open_port(device_path);
        if(fd < 0)
        {
            perror("open_port error");
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
    
	bool Uart::ReadData() 
    {
		int readsize = read(serial_id, data, READ_DATA_NUM);
        tcflush(serial_id, TCIFLUSH);

        if (readsize != READ_DATA_NUM && readsize != 0) 
        {
			std::cout << "READ ERROR:" << readsize << std::endl;
			return false;
		}
		else if(readsize == 0)
        {
            std::cout << "get the data error" << std::endl;
            return false;
        }

        read_data.Arm_Pos_x     = lptmp->Arm_Pos_x;
        read_data.Arm_Pos_y     = lptmp->Arm_Pos_y;
        read_data.Arm_Pos_z     = lptmp->Arm_Pos_z;
        read_data.Arm_Pos_Pitch = lptmp->Arm_Pos_Pitch;
        read_data.Arm_Pos_Roll  = lptmp->Arm_Pos_Roll;
        read_data.Button        = lptmp->Button;
        read_data.EndFrame      = lptmp->EndFrame;

        return true;
	}
//我给下位机发什么东西
	void Uart::send(const VisionData& data)
	{
        TXunion.VisionFrameTX.Joint_1 = data.Joint_1;//data.x;
        TXunion.VisionFrameTX.Joint_2 = data.Joint_2;
        TXunion.VisionFrameTX.Joint_3 = data.Joint_3;
        TXunion.VisionFrameTX.Joint_4 = data.Joint_4;
        TXunion.VisionFrameTX.Joint_5 = data.Joint_5;
        TXunion.VisionFrameTX.Joint_6 = data.Joint_6;

		auto write_stauts = write(serial_id, &TXunion.u8arr[0], SEND_DATA_NUM);
        tcflush(serial_id, TCOFLUSH);
		if (write_stauts != SEND_DATA_NUM) 
        { 
			std::cout << "send error! the length of data is: " << write_stauts << std::endl;
		}
	}
}
