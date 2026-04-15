#ifndef SERIAL_H_
#define SERIAL_H_

#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>
#include     <termios.h>
#include     <errno.h>
#include     <string.h>
#include     <iostream>
#include     <memory>
#include     <cstring>

// ---- 帧协议常量 ----
#define FRAME_HEADER     (0x78)
#define FRAME_TAIL       (0x91)

#define TX_PAYLOAD_SIZE  (12) 
#define RX_PAYLOAD_SIZE  (23)  
#define TX_FRAME_SIZE    (TX_PAYLOAD_SIZE + 2) 
#define RX_FRAME_SIZE    (RX_PAYLOAD_SIZE + 2) 

namespace TRoMaC
{
    #pragma pack(1)

    typedef struct
    {
        int16_t     Arm_Pos_x;
        int16_t     Arm_Pos_y;
        int16_t     Arm_Pos_z;
        int16_t     Arm_Pos_Pitch;
        int16_t     Arm_Pos_Roll;
        uint8_t     Button;

        int16_t     Real_Joint_1;
        int16_t     Real_Joint_2;
        int16_t     Real_Joint_3;
        int16_t     Real_Joint_4;
        int16_t     Real_Joint_5;
        int16_t     Real_Joint_6;
    } VisionFrameRX_structTypedef;

    typedef struct
    {
        int16_t       Joint_1;
        int16_t       Joint_2;
        int16_t       Joint_3;
        int16_t       Joint_4;
        int16_t       Joint_5;
        int16_t       Joint_6;
    } VisionFrameTX_structTypedef;

    typedef union
    {
        VisionFrameTX_structTypedef     VisionFrameTX;
        uint8_t                         u8arr[sizeof(VisionFrameTX_structTypedef)];
    } VisionFrameTX_unionTypeDef;

    typedef struct
    {
        int16_t       Joint_1;
        int16_t       Joint_2;
        int16_t       Joint_3;
        int16_t       Joint_4;
        int16_t       Joint_5;
        int16_t       Joint_6;
    } VisionData;

    typedef struct
    {
        int16_t     Arm_Pos_x;
        int16_t     Arm_Pos_y;
        int16_t     Arm_Pos_z;
        int16_t     Arm_Pos_Pitch;
        int16_t     Arm_Pos_Roll;
        uint8_t     Button;

        int16_t     Real_Joint_1;
        int16_t     Real_Joint_2;
        int16_t     Real_Joint_3;
        int16_t     Real_Joint_4;
        int16_t     Real_Joint_5;
        int16_t     Real_Joint_6;
    } ReceiveData;

    #pragma pack()

    class Uart
    {
    public:
        Uart();
        ~Uart();
        void Close();

        int open_port(int comport);
        int open_port(const std::string& device_path);
        int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

        bool Open(int comport, int _speed);
        bool Open(const std::string& device_path, int _speed);
        bool checkSerial();
        bool ReadData();
        void send(const VisionData& data);

        int serial_id = -1;
        int speed;
        const char* uart_path;

        VisionFrameRX_structTypedef read_data;
        VisionFrameTX_unionTypeDef TXunion;

    private:
        // RX 状态机
        enum class RxState { WAIT_HEADER, READ_PAYLOAD, WAIT_TAIL };
        RxState  rx_state_        = RxState::WAIT_HEADER;
        uint8_t  rx_payload_buf_[RX_PAYLOAD_SIZE]{};
        int      rx_payload_idx_  = 0;

        // 内部读缓冲 (批量读取，减少系统调用次数)
        uint8_t  rx_ring_[256]{};
        int      rx_ring_len_     = 0;
        int      rx_ring_pos_     = 0;
    };
}

#endif
