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
#include   	 <string.h>       
#include 	 <iostream>
#include 	 <memory>


// #define SEND_DATA_NUM (16)
// #define READ_DATA_NUM (19)

#define SEND_DATA_NUM (12) 
#define READ_DATA_NUM (12)

namespace TRoMaC
{
	#pragma pack(1)

    typedef struct //这里预留写按钮兑矿或者啥的
    {   
        int16_t     Arm_Pos_x;        // 机器人ID
        int16_t     Arm_Pos_y;       // 视觉模式
        int16_t     Arm_Pos_z;      // 丢包率   
        int16_t     Arm_Pos_Pitch;         // 预留调节曝光的一个字节
        int16_t     Arm_Pos_Roll;
        uint8_t     Button;       // 帧尾
        uint8_t     EndFrame;
    } VisionFrameRX_structTypedef;
    
    /* ==== 云台接收帧结构体&联合体 ==== */

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
    
    typedef union
 	{
    	VisionFrameRX_structTypedef     VisionFrameRX;
    	uint8_t                         u8arr[sizeof(VisionFrameRX_structTypedef)];
    }  VisionFrameRX_unionTypeDef;
    
    //用于保存目标相关角度和距离信息及瞄准情况
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
        int16_t     Arm_Pos_x;       // 机器人ID
        int16_t     Arm_Pos_y;       // 视觉模式
        int16_t     Arm_Pos_z;       // 丢包率   
        int16_t     Arm_Pos_Pitch;   // 预留调节曝光的一个字节
        int16_t     Arm_Pos_Roll;
        uint8_t     Button;          // 帧尾
        uint8_t     EndFrame;				
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

        int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);

		bool Open(int comport, int _speed);
		bool Open(const std::string& device_path, int _speed);

        bool checkSerial();

        bool ReadData();
        
		void send(const VisionData& data);
		
        int serial_id = -1;
        int speed;
		const char* uart_path;
        unsigned char data[30];

        VisionFrameRX_structTypedef read_data;
        VisionFrameRX_structTypedef *lptmp = (VisionFrameRX_structTypedef *)data;
        VisionFrameTX_unionTypeDef TXunion;
        
	private:
	};
}

#endif
