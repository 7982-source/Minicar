#include "config.h"
#include "pid.h"
#include "imu901.h"
extern ringbuffer_t uart3TxFifo;
// 外部变量声明
extern int16_t Speed_L;
extern int16_t Speed_R;
extern double Left_Target_Speed;
extern double Right_Target_Speed;
extern IncrementalPID Left_PID;
extern IncrementalPID Right_PID;

//extern float attitude_pitch;
// 定义数据包结构体
typedef struct {
    uint8_t header1;
    //uint8_t header2;
    //uint8_t type;
    //uint8_t length;
	  //4byte
    int8_t speed_L;
    int8_t speed_R;
	  //uint8_t deltaTime;
	  //4byte
    //uint8_t checksum;
    uint8_t footer1;
    //uint8_t footer2;
	  //3byte
} __attribute__((packed)) DataPacket_t; // 使用__attribute__((packed))来确保结构体不填充，内存连续
void Serial_Write(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        Serial_SendByte(data[i]);
    }
}
// 假设这是在你的主函数 main() 或其他处理函数中

void Data_Processing(void)
{
    if (Serial_GetRxFlag() == 1) // 检查是否接收到一个完整的数据包
    {

        
        // 显示"RX"标识，表示接收到数据
        //OLED_ShowString(3, 5, "RX");
        
        // ------------------------------------
        // 1. 重组第一个 int16 (N1)
        // 字节来自 Serial_RxPacket[0] 和 Serial_RxPacket[1]
        // ------------------------------------
        Left_Target_Speed = (int16_t)(
            ((uint16_t)Serial_RxPacket[0] << 8) | 
            (uint16_t)Serial_RxPacket[1]
        );
        
        Right_Target_Speed = (int16_t)(
            ((uint16_t)Serial_RxPacket[2] << 8) | 
            (uint16_t)Serial_RxPacket[3]
        );

        // 至此，received_data1 和 received_data2 就是 ROS 端发送过来的两个 int16 数据
        OLED_ShowSignedNum(4, 1, Left_Target_Speed, 5);	//不断刷新显示编码器测得的最新速度
	      OLED_ShowSignedNum(4, 7, Right_Target_Speed, 5);	//不断刷新显示编码器测得的最新速度
    }
}
// 修改后的 Send_Data_To_ROS 函数
void Send_Data_To_ROS(void) {
    DataPacket_t packet;
    uint8_t *p = (uint8_t*)&packet;
    uint8_t sum = 0;

    packet.header1 = 0x55;
//    packet.header2 = 0xAA;
//    packet.type = 0x01;
//    packet.length = 40;
    packet.speed_L = Speed_L;
    packet.speed_R = Speed_R;

// 计算校验和
//    for (int i = 0; i < sizeof(DataPacket_t) - 3; i++) {
//        sum += p[i];
//    }
//    packet.checksum = sum;
    packet.footer1 = 0x0D;
//    packet.footer2 = 0x0A;

    // 直接使用 ringbuffer_in 发送原始二进制数据
    Serial_Write(p, sizeof(DataPacket_t));
}

void Duty_1ms(void);
void Duty_2ms(void);
void Duty_5ms(void);
void Duty_10ms(void);
void Duty_15ms(void); 
void Duty_20ms(void);
void Duty_50ms(void);


loop_t loop;



void Loop_Init(void) {
    loop.check_flag = 0;
    loop.error_flag = 0;
    
    // 初始化所有计数器为0
    loop.cnt_2ms = 0;
    loop.cnt_5ms = 0;
    loop.cnt_10ms = 0;
    loop.cnt_15ms = 0;
    loop.cnt_20ms = 0;
    loop.cnt_50ms = 0;
}


void Loop_check(void) {
    loop.cnt_2ms++;
    loop.cnt_5ms++;
    loop.cnt_10ms++;
	  loop.cnt_15ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;

    if (loop.check_flag) {
        loop.error_flag++; 
    }
    
    loop.check_flag = 1; 
		
}


void Duty_Loop(void) 
{ 
    if (loop.check_flag) 
    {    
			  Duty_1ms();

        if (loop.cnt_2ms >= 2  ) {loop.cnt_2ms = 0;   Duty_2ms();  }
        if (loop.cnt_5ms >= 5  ) {loop.cnt_5ms = 0;   Duty_5ms();  }  
        if (loop.cnt_10ms >= 10) {loop.cnt_10ms = 0;  Duty_10ms(); }
				if (loop.cnt_15ms >= 15) {loop.cnt_15ms = 0;  Duty_15ms(); }
        if (loop.cnt_20ms >= 20) {loop.cnt_20ms = 0;  Duty_20ms(); }
        if (loop.cnt_50ms >= 50) {loop.cnt_50ms = 0;  Duty_50ms(); }
        
        loop.check_flag = 0;				
              
    }
}


void Duty_1ms(void) 
{

}

void Duty_2ms(void) 
{
 	
}
void Duty_5ms(void) 
{ 

}
void Duty_10ms(void) 
{

}

void Duty_15ms(void)
{
		
		
		// 计算左轮新的电机功率
//		double left_motor_output = compute_pid(&Left_PID, Left_Target_Speed, (double)Speed_L);

//		// 计算右轮新的电机功率
//		double right_motor_output = compute_pid(&Right_PID, Right_Target_Speed, (double)Speed_R);

//		if (left_motor_output > 100) left_motor_output = 100;
//		if (left_motor_output < -100) left_motor_output = -100;
//		if (right_motor_output > 100) right_motor_output = 100;
//		if (right_motor_output < -100) right_motor_output = -100;

//		// 应用新的电机功率
//		L_SetSpeed((int8_t)left_motor_output);
//		R_SetSpeed((int8_t)right_motor_output);
	
}	
    
void Duty_20ms(void) 
{
	Speed_L = Encoder_Left_Get();               
	Speed_R = Encoder_Right_Get();  
  Send_Data_To_ROS();
  Data_Processing();  

  		// 计算左轮新的电机功率
	double left_motor_output = compute_pid(&Left_PID, Left_Target_Speed, (double)Speed_L);

	// 计算右轮新的电机功率
	double right_motor_output = compute_pid(&Right_PID, Right_Target_Speed, (double)Speed_R);

	if (left_motor_output > 100) left_motor_output = 100;
	if (left_motor_output < -100) left_motor_output = -100;
	if (right_motor_output > 100) right_motor_output = 100;
	if (right_motor_output < -100) right_motor_output = -100;

	// 应用新的电机功率
	L_SetSpeed((int8_t)left_motor_output);
	R_SetSpeed((int8_t)right_motor_output);  
} 
void Duty_50ms(void) 
{
    	
		//Serial_Printf("%d,%d,%f\n",Speed_L,Speed_R , attitude.pitch);
		
		//每隔固定时间段读取一次编码器计数增量值，即为速度值

		OLED_ShowSignedNum(1, 4, Speed_L, 5);	//不断刷新显示编码器测得的最新速度
		OLED_ShowSignedNum(2, 4, Speed_R, 5);	//不断刷新显示编码器测得的最新速度
}



