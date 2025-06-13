//*****************************************************************************
//
// 头文件
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"        	
#include "inc/hw_types.h"         	
#include "inc/hw_ints.h"          	
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"      	
#include "driverlib/gpio.h"       	
#include "driverlib/pin_map.h"    	
#include "driverlib/sysctl.h"		
#include "driverlib/systick.h"    
#include "driverlib/interrupt.h"  	
#include "driverlib/uart.h"       
#include "driverlib/timer.h"		
#include "driverlib/i2c.h"			
#include "driverlib/pwm.h"   		
#include "driverlib/hibernate.h"	


#define SYSTICK_FREQUENCY		1000		
#define V_T1ms			1
#define V_T2ms			2
#define V_T20ms   		20				// 20ms软件定时器溢出值
#define V_T100ms		100             // 0.1s软件定时器溢出值
#define V_T300ms		300				// 0.3s软件定时器溢出值
#define V_T500ms		500             // 0.5s软件定时器溢出值
#define V_T900ms		900				// 0.9s软件定时器溢出值
#define V_T10s   		10000           // 10s软件定时器溢出值
#define V_T5s			5000			// 5s软件定时器溢出值		
#define V_T1s			1000			// 1s软件定时器溢出值
#define V_T3s			3000			// 3s软件定时器溢出值
#define V_T2s			2000			// 2s软件定时器溢出值
#define BUTTON			50

// 按键功能定义


// 系统状态定义
#define MODE_DATE   0
#define MODE_TIME   1
#define MODE_ALARM  2


//*****************************************************************************
//
//I2C GPIO chip addresecond and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 				0x22
#define PCA9557_I2CADDR					0x18	

#define PCA9557_INPUT					0x00
#define	PCA9557_OUTPUT					0x01
#define PCA9557_POLINVERT				0x02
#define PCA9557_CONFIG					0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06




void GPIOInit(void);
void SysTickInit(void);
void DevicesInit(void);
void S800_UART_Init(void);
void UARTStringPut(uint32_t,uint8_t *);
void PWMInit(void);
void PWMStart(uint32_t);
void PWMStop(void);
uint8_t I2C0_WriteByte(uint8_t, uint8_t, uint8_t);
uint8_t I2C0_ReadByte(uint8_t, uint8_t);
void S800_I2C0_Init(void);
void HibernateInit(void);
void handleUARTCommand(void);
void handleKeyPress(void);
void updateDisplay(void);
void updateTime(void);
void handleAlarm(void);
void handleInitialization(void);
void handleShiftControl(void);

// 系统时钟频率
uint32_t ui32SysClock;	

// 用户时钟
int8_t hour = 0;	// 时
int8_t minute = 0;	// 分
int8_t second = 0;	// 秒
int8_t alm_hour = 25;	//闹钟
int8_t alm_minute = 0;
int8_t alm_second = 0;
int8_t temp_hour = 0;
int8_t temp_minute = 0;
int8_t temp_second = 0;
uint16_t year = 2025;	// 年
uint8_t month = 6;		// 月
uint8_t day = 6;		// 日

uint16_t clock1s = 0, clock500ms = 0,clock300ms = 0, clock900ms = 0;
uint8_t clock20ms = 0, clock2ms = 0,clock1ms = 0;

bool overflow_1s_flag = 0;
bool overflow_20ms_flag = 0;
bool overflow_2ms_flag = 0;
bool overflow_1ms_flag = 0;
bool overflow_500ms_flag = 0;
bool overflow_300ms_flag = 0;
bool overflow_900ms_flag = 0;

//UART相关
uint8_t uart_receive_buffer[100];	// 接收缓冲区
uint8_t uart_receive_len = 0;
uint8_t cmd_state = 0;
uint8_t time_transmit_buffer[] = "00:00:00";	// 用户时钟
uint8_t alarm_transmit_buffer[] = "xx:xx:xx";	// 用户闹钟
uint8_t date_transmit_buffer[] = "2025-06-06";	// 用户日期

//数码管显示相关
uint8_t result, cnt;
int8_t	shift_left = 0, shift_right = 0, shift = 0;
uint8_t rightshift = 0x01;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x79,0x71,0x5c,0x00};	//需要用的数字和字母
uint8_t seg7_1[16] = {0x5b,0x3f,0x5b,0x6d,0x3f,0x7d,0x3f,0x7d,0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00};	//初始日期和时间
uint8_t seg7_2[8] = {0x4f,0x06,0x6f,0x06,0x3f,0x4f,0x7f,0x66};	//学号31910384
uint8_t seg7_3[8] = {0x38,0x30, 0x76, 0x30, 0x77, 0x37,0x7D,0x00};	//姓名lixiang
uint8_t seg7_4[8] = {0x5b,0x3f,0x5b,0x6d,0x3f,0x7d,0x3f,0x7d};//日期
uint8_t seg7_5[8] = {0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00};//时间
uint8_t seg7_6[8] = {0x77,0x38,0x01,0x00,0x01,0x00,0x01,0x00};//闹钟
bool shift_mode = 0;	//0-左流水 1-右流水
bool shift_speed = 0;	//0-慢速   1-快速
bool shifting = 1;		//1-流水   2-暂停
uint8_t state_shift = 0; //0-流水 1-单独日期 2-单独时间 3-闹钟

static uint8_t blink_state = 0;
static uint16_t blink_counter = 0;
uint8_t keyValue, lastKeyValue;
uint8_t key = 0;
bool button_flag = 0;
uint8_t button_presecond = 0;
bool init_flag = 1;
uint8_t init_procedure = 0;
uint8_t i=0;
uint8_t head = 0, tail = 0;
uint8_t help_doc[] = "Digital Clock Help Documentation\r\nCommand: HELP\r\nFunction: Call Help Documentation\r\nCommand: INIT\r\nFunction: Initialize Device\r\nCommand: SET TIME hour:minute:second\r\nFunction: Set Time\r\nCommand: SET ALARM hour:minute:second\r\nFunction: Set Alarm\r\nCommand: GET TIME\r\nFunction: Get Current Time\r\nCommand: GET DATE\r\nFunction: Get Current Date\r\nCommand: GET ALARM\r\nFunction: Get Current Alarm\r\n";
uint8_t uart_handle_buffer[3][20];
uint8_t uart_handle_len = 0;
uint8_t handlelen[3];
uint8_t handlecnt = 0;
bool space_flag = 0;

uint32_t storedRTC[4];
uint32_t currentRTC;
uint32_t fetchRTC[4];	
uint32_t pastSec;
uint8_t edit_digit = 0;  // 正在编辑哪一位（范围0-2）
bool editing = false;    // 是否进入编辑状态
uint8_t current_mode = MODE_TIME;       // 当前模式


uint32_t last_key_time = 0;             // 上次按键时间
uint8_t key_press_count = 0;            // 按键按下次数
uint32_t key_press_start = 0;           // 第一次按键时间

main(void)
{
	 DevicesInit();
   UARTStringPut(UART0_BASE, (uint8_t*)"Input Help to view the help documentation.");

    while(1)
    {
        handleAlarm();
        handleInitialization();
        handleKeyPress();
        updateDisplay();
        handleShiftControl();
        updateTime();
        handleUARTCommand();
    }
}

void DevicesInit(void)
{
	// 使用外部25MHz主时钟源，经过PLL，然后分频为20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | 
	                                   SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 
	                                   20000000);
	GPIOInit();		// GPIO初始化
	S800_UART_Init();		// UART初始化
	SysTickInit();
	PWMInit();
	S800_I2C0_Init();
	HibernateInit();
	
	IntMasterEnable();	// 总中断允许
}

void GPIOInit(void)
{

}

void HibernateInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_HIBERNATE));
	
	HibernateEnableExpClk(ui32SysClock);
	HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
	HibernateRTCEnable();
	//HibernateRTCSet(0);
}

void SysTickInit(void)
{
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms
	SysTickEnable();  			// SysTick使能
	SysTickIntEnable();			// SysTick中断允许
}


void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
 
	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX7_8);
  IntEnable(INT_UART0); 
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); 
}


void UARTStringPut(uint32_t ui32Base,uint8_t *cMessage)
{
    bool TXFIFO_free = 0;
    
	while(*cMessage != '\0')
    {
        TXFIFO_free = UARTCharPutNonBlocking(ui32Base, *(cMessage));	// 如果成功发送，返回true
        if(TXFIFO_free) // 成功发送
        {  
            cMessage++;	//下一位
        }
        TXFIFO_free = 0;
    }
}


void PWMInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);     // PWM0使能
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true); // 使能(允许)PWM0_7的输出
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);    // 使能GPIOK
	//while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    GPIOPinConfigure(GPIO_PK5_M0PWM7);              // 配置引脚复用
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);    // 引脚映射
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);   //配置PWM发生器
}

void PWMStart(uint32_t ui32Freq_Hz)
{
	PWMGenDisable(PWM0_BASE, PWM_GEN_3);     //使能PWM0模块的3号发生器(因为7号PWM是3号发生器产生的)   
    
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / ui32Freq_Hz); 					// 根据Freq_Hz设置PWM周期
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,(PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)/ 2)); 	//设置占空比为50%
    
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);     //使能PWM0模块的2号发生器(因为7号PWM是3号发生器产生的)   
}

void PWMStop(void)
{
	PWMGenDisable(PWM0_BASE, PWM_GEN_3);   // M0PWM7(PK5)停止产生PWM信号
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}



uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};	//等待总线处理
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);	//从机地址配置，false表示写入，true表示读入
	I2CMasterDataPut(I2C0_BASE, RegAddr);				//发送数据
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);	//配置模式
	while(I2CMasterBusy(I2C0_BASE)){};					
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);				//接收报错

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(5);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);		//配置成读入
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	Delay(5);
	return value;
}

void SysTick_Handler(void)       // 定时周期为20ms
{
	// 1秒钟软定时器计数
	if (++clock1s >= V_T1s)
	{
		overflow_1s_flag = 1;
		clock1s = 0;
		++second;
	}
	
	if (++clock2ms >= V_T2ms)
	{
		overflow_2ms_flag = 1;
		clock2ms = 0;
	}
	
	if (++clock500ms >= V_T500ms)
	{
		overflow_500ms_flag = 1;
		clock500ms = 0;
	}
	
	if (++clock300ms >= V_T300ms)
	{
		overflow_300ms_flag = 1;
		clock300ms = 0;
	}
	
	if (++clock900ms >= V_T900ms)
	{
		overflow_900ms_flag = 1;
		clock900ms = 0;
	}
	
	if (keyValue != 0xff)
	{
		++button_presecond;
	}
	else
	{
		button_presecond = 0;
	}
	
	if (button_presecond >= BUTTON)
	{
		button_flag = 1;
	}
	
	overflow_1ms_flag = 1;
	
	// 处理用户时钟进位
	if (second >= 60) {	++minute; 	second = 0;}
	if (minute >= 60) {	++hour; 	minute = 0;}
	if (hour >= 24) {	++day;	hour = 0;}
	
//	if (temp)
//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
//	else
//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0x00);
//	
//	temp = !temp;
}

void UART0_Handler(void)
{
    int32_t uart0_int_status;
    uint8_t uart_receive_char;
    volatile uint32_t i;
    
    uart0_int_status = UARTIntStatus(UART0_BASE, true); // 取中断状态
    UARTIntClear(UART0_BASE, uart0_int_status); // 清中断标志
  
    while(UARTCharsAvail(UART0_BASE)) // 重复从接收FIFO 读取字符
    {
        uart_receive_char = UARTCharGetNonBlocking(UART0_BASE); // 读入一个字符
        
        if(uart_receive_char != '\r' && uart_receive_char != '\n') // 命令未结束
            uart_receive_buffer[uart_receive_len++] = uart_receive_char;
        else
        {
            cmd_state = 1;
            uart_receive_buffer[uart_receive_len] = '\0';
        }
    }        
} 

void handleAlarm(void)
{
    if (hour == alm_hour && minute == alm_minute && second == alm_second)
    {
        PWMStart(500);
    }
}

void handleInitialization(void)
{
    if(init_flag)
    {
        currentRTC = HibernateRTCGet();
        HibernateDataGet(fetchRTC, 4);
        hour = fetchRTC[0];
        minute = fetchRTC[1];
        second = fetchRTC[2] + (currentRTC - fetchRTC[3]);

        while(second >= 60)
        {
            ++minute;
            if(minute >= 60)
            {
                ++hour;
                if(hour >=24)
                {
                    ++day;
                }
                second -= 60;
            }
        }

        if(overflow_2ms_flag == 1)
        {
            overflow_2ms_flag = 0;
            switch(init_procedure)
            {
                case 0:
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_2[cnt]);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, rightshift);
                    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00);
                    break;
                case 1:
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, rightshift);
                    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff);
                    break;
                case 2:
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_3[cnt]);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, rightshift);
                    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00);
                    break;
                case 3:
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00);
                    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, rightshift);
                    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff);
                    break;
                default:
                    init_flag = 0;
                    shift = 0;
                    break;
            }
            cnt++;
            rightshift = rightshift << 1;
            if (cnt >= 0x8)
            {
                rightshift = 0x01;
                cnt = 0;
            }
        }

        if(overflow_900ms_flag == 1)
        {
            overflow_900ms_flag = 0;
            ++init_procedure;
        }
    }
}
void handleKeyPress(void)
{   static uint8_t debounce_counter = 0;
    static bool key_pressed = false;
    if (overflow_1ms_flag == 1)
    {
        overflow_1ms_flag = 0;
        keyValue = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
    }

    // 检测按键释放
    if (keyValue == 0xff && lastKeyValue != 0xff)
    {
        // 按键释放处理
        button_flag = 0;
        button_presecond = 0;
        
        // 短按处理
        switch(key)
        {
            case 1: // 原KEY1功能
                // 原有功能：切换流水方向
                break;
                
						case 2:
              if (!editing) {
								
                current_mode = (current_mode + 1) % 3;  // 切换日期、时间、闹钟
                state_shift = current_mode;
              }
                break;
                
            case 3: // 原KEY3功能 (SAVE键)
                // 原有功能：停止蜂鸣器
                PWMStop();
                break;
                
          case 4:
						
               if (!editing) {
               editing = true;
               edit_digit = 0;
               } 
							 else {
               edit_digit = (edit_digit + 1) % 3;
               }
               break;
        
                
            case 5: // 原KEY5功能
                // 原有功能：暂停/继续流水
                shifting = !shifting;
                break;
                
           case 6:
							if (editing) {
								if (current_mode == MODE_DATE) {
									if (edit_digit == 0) year++;
									if (edit_digit == 1) month = (month % 12) + 1;
									if (edit_digit == 2) day = (day % 31) + 1;
								} else if (current_mode == MODE_TIME) {
									if (edit_digit == 0) hour = (hour + 1) % 24;
									if (edit_digit == 1) minute = (minute + 1) % 60;
									if (edit_digit == 2) second = (second + 1) % 60;
									} else if (current_mode == MODE_ALARM) {
									if (edit_digit == 0) alm_hour = (alm_hour + 1) % 24;
									if (edit_digit == 1) alm_minute = (alm_minute + 1) % 60;
									if (edit_digit == 2) alm_second = (alm_second + 1) % 60;
								}
							}
								break;

                
						 case 7:
								if (editing) {
								editing = false;
							}
								break;

                
            case 8: // 原KEY8功能
                // 原有功能：切换流水速度
                shift_speed = !shift_speed;
                break;
                
        
        
        key = 0;
    }
	}

    // 检测按键按下
    if (keyValue != 0xff && lastKeyValue == 0xff)
    {
        switch(keyValue)
        {
            case 0xfe: key = 1; break;    // KEY1
            case 0xfd: key = 2; break;    // KEY2 (FUNC)
            case 0xfb: key = 3; break;    // KEY3 (SAVE)
            case 0xf7: key = 4; break;    // KEY4
            case 0xef: key = 5; break;    // KEY5
            case 0xdf: key = 6; break;   // KEY6 (SHIFT)
            case 0xbf: key =7; break;     // KEY7 (ADD)
            case 0x7f: key = 8; break;    // KEY8
            default: key = 0; break;
        }
        
      
    }

    // 检测长按
    if (keyValue != 0xff)
    {
        button_presecond++;
        if (button_presecond >= BUTTON)
        {
            button_flag = 1;
            switch(key)
            {
                case 1: // 原KEY1功能长按
                    shift_mode = !shift_mode;
                    state_shift = 0;
                    break;
                    
                case 2: // FUNC键长按
                    editing = 0; // 保存并退出
                    break;
                    
                case 3: // 原KEY3功能长按
                    PWMStop(); // 停止蜂鸣器
                    break;
                    
                case 4: // 原KEY4功能长按
                    editing = 0;
                    break;
                    
                case 5: // 原KEY5功能长按
                    shifting = !shifting; // 暂停/继续流水
                    break;
                    
                case 6: // SHIFT键长按
                   
                    break;
                    
                case 7: // ADD键长按
         
                    break;
                    
                case 8: // 原KEY8功能长按
                    shift_speed = !shift_speed; // 切换流水速度
                    break;
            }
            key = 0;
        }
    }
    
    // 5秒超时处理
    if (editing && (clock1s - last_key_time) > 5000)
    {
        editing = 0;
    }
    
    // 更新最后按键时间
    if (keyValue != 0xff) {
        last_key_time = clock1s;
    }

    lastKeyValue = keyValue;
}
void updateDisplay(void)
{
    if (overflow_2ms_flag == 1 && init_flag == 0)
    {
        overflow_2ms_flag = 0;
        
        static uint8_t display_data[8]; // 存储当前要显示的数据
        uint8_t seg_index = cnt;        // 当前要更新的数码管位置
        
        // 设置模式下的闪烁效果
        if (editing)
        {
            blink_counter++;
            if (blink_counter >= 250) { // 250*2ms = 500ms
                blink_counter = 0;
                blink_state = !blink_state;
            }
        }
        else
        {
            blink_state = 0; // 非设置模式不闪烁
        }
        
        // 根据当前模式准备显示数据
        if (!editing)
        {
            // 非设置模式 - 使用原有的状态显示逻辑
            switch(state_shift)
            {
                case 0: // 流水显示
                    for (int i = 0; i < 8; i++) {
                        display_data[i] = seg7_1[(i + shift) % 16];
                    }
                    break;
                    
                case 1: // 日期显示
                    for (int i = 0; i < 8; i++) {
                        display_data[i] = seg7_4[i];
                    }
                    break;
                    
                case 2: // 时间显示
                    for (int i = 0; i < 8; i++) {
                        display_data[i] = seg7_5[i];
                    }
                    break;
                    
                case 3: // 闹钟显示
                    for (int i = 0; i < 8; i++) {
                        display_data[i] = seg7_6[i];
                    }
                    break;
            }
        }
        else
        {
            // 设置模式 - 根据当前编辑模式准备数据
            switch(current_mode)
            {
                case MODE_DATE:
                    for (int i = 0; i < 8; i++) {
                        // 设置模式下闪烁当前编辑字段
                        if (blink_state && (i == edit_digit * 2 || i == edit_digit * 2 + 1)) {
                            display_data[i] = 0;
                        } else {
                            display_data[i] = seg7_4[i];
                        }
                    }
                    break;
                    
                case MODE_TIME:
                    for (int i = 0; i < 8; i++) {
                        // 设置模式下闪烁当前编辑字段
                        if (blink_state && (i == edit_digit * 2 || i == edit_digit * 2 + 1)) {
                            display_data[i] = 0;
                        } else {
                            display_data[i] = seg7_5[i];
                        }
                    }
                    break;
                    
                case MODE_ALARM:
                    for (int i = 0; i < 8; i++) {
                        // 设置模式下闪烁当前编辑字段
                        if (blink_state && (i == edit_digit * 2 || i == edit_digit * 2 + 1)) {
                            display_data[i] = 0;
                        } else {
                            display_data[i] = seg7_6[i];
                        }
                    }
                    break;
            }
        }
        
        // 更新数码管显示 - 使用I2C0_WriteByte确保正确控制
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00); // 关闭所有位选
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, display_data[seg_index]); // 设置段选数据
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, rightshift); // 设置位选
        result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~rightshift); // 设置LED
        
        // 更新扫描位置
        cnt++;
        rightshift = rightshift << 1;
        if (cnt >= 8)
        {
            cnt = 0;
            rightshift = 0x01;
        }
    }
}

void handleShiftControl(void)
{
    if (overflow_500ms_flag == 1 && shift_speed == 0 && shifting == 1)
    {
        overflow_500ms_flag = 0;
        if (shift_mode == 0)
        {
            ++shift;
            shift %= 16;
        }
        else
        {
            if(--shift < 0)
            {
                shift = 15;
            }
        }
    }

    if (overflow_300ms_flag == 1 && shift_speed == 1 && shifting == 1)
    {
        overflow_300ms_flag = 0;
        if (shift_mode == 0)
        {
            ++shift;
            shift %= 16;
        }
        else
        {
            if(--shift < 0)
            {
                shift = 15;
            }
        }
    }
}

void updateTime(void)
{
    if (overflow_1s_flag == 1)
    {
        overflow_1s_flag = 0;
        time_transmit_buffer[0] = (hour / 10) + '0';
        time_transmit_buffer[1] = (hour % 10) + '0';
        time_transmit_buffer[3] = (minute / 10) + '0';
        time_transmit_buffer[4] = (minute % 10) + '0';
        time_transmit_buffer[6] = (second / 10) + '0';
        time_transmit_buffer[7] = (second % 10) + '0';
        alarm_transmit_buffer[0] = (alm_hour / 10) + '0';
        alarm_transmit_buffer[1] = (alm_hour % 10) + '0';
        alarm_transmit_buffer[3] = (alm_minute / 10) + '0';
        alarm_transmit_buffer[4] = (alm_minute % 10) + '0';
        alarm_transmit_buffer[6] = (alm_second / 10) + '0';
        alarm_transmit_buffer[7] = (alm_second % 10) + '0';
        seg7_1[9] = seg7[hour / 10];
        seg7_1[10] = seg7[hour % 10] | 0x80;
        seg7_1[11] = seg7[minute / 10];
        seg7_1[12] = seg7[minute % 10] | 0x80;
        seg7_1[13] = seg7[second / 10];
        seg7_1[14] = seg7[second % 10];
        seg7_5[2] = seg7_1[9];
        seg7_5[3] = seg7_1[10];
        seg7_5[4] = seg7_1[11];
        seg7_5[5] = seg7_1[12];
        seg7_5[6] = seg7_1[13];
        seg7_5[7] = seg7_1[14];
        seg7_6[2] = seg7[alm_hour / 10];
        seg7_6[3] = seg7[alm_hour % 10] | 0x80;
        seg7_6[4] = seg7[alm_minute / 10];
        seg7_6[5] = seg7[alm_minute % 10] | 0x80;
        seg7_6[6] = seg7[alm_second / 10];
        seg7_6[7] = seg7[alm_second % 10] | 0x80;
        storedRTC[0] = hour;
        storedRTC[1] = minute;
        storedRTC[2] = second;
        storedRTC[3] = HibernateRTCGet();
        HibernateDataSet(storedRTC, 4);
    }
}

void handleUARTCommand(void)
{
    if(cmd_state == 1)
    {
        cmd_state = 0;
        head = 0;
        tail = uart_receive_len;
        handlecnt = 0;
        uart_receive_len = 0;

        while(head <= tail)
        {
            if(uart_receive_buffer[head] != ' ' && uart_receive_buffer[head] != '\0' && uart_receive_buffer[head] != '\r' && uart_receive_buffer[head] != '\n')
            {
                uart_handle_buffer[handlecnt][uart_handle_len++] = uart_receive_buffer[head];
                space_flag = 0;
            }
            else
            {
                if(space_flag == 0)
                {
                    space_flag = 1;
                    handlelen[handlecnt] = uart_handle_len;
                    ++handlecnt;
                }
                uart_handle_buffer[handlecnt][uart_handle_len] = '\0';
                uart_handle_len = 0;
            }
            ++head;
        }

        if(handlelen[0] == 4)
        {
            if( (uart_handle_buffer[0][0] == 'I' || uart_handle_buffer[0][0] == 'i') &&
                (uart_handle_buffer[0][1] == 'N' || uart_handle_buffer[0][1] == 'n') &&
                (uart_handle_buffer[0][2] == 'I' || uart_handle_buffer[0][2] == 'i') &&
                (uart_handle_buffer[0][3] == 'T' || uart_handle_buffer[0][3] == 't'))
            {
                SysCtlReset();
            }
            else if( (uart_handle_buffer[0][0] == 'H' || uart_handle_buffer[0][0] == 'h') &&
                     (uart_handle_buffer[0][1] == 'E' || uart_handle_buffer[0][1] == 'e') &&
                     (uart_handle_buffer[0][2] == 'L' || uart_handle_buffer[0][2] == 'l') &&
                     (uart_handle_buffer[0][3] == 'P' || uart_handle_buffer[0][3] == 'p'))
            {
                UARTStringPut(UART0_BASE, help_doc);
            }
            else
            {
							UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
            }
        }
        else if(handlelen[0] == 3)
        {
            if( (uart_handle_buffer[0][0] == 'S' || uart_handle_buffer[0][0] == 's') &&
                (uart_handle_buffer[0][1] == 'E' || uart_handle_buffer[0][1] == 'e') &&
                (uart_handle_buffer[0][2] == 'T' || uart_handle_buffer[0][2] == 't'))
            {
                if( (uart_handle_buffer[1][0] == 'T' || uart_handle_buffer[1][0] == 't') &&
                    (uart_handle_buffer[1][1] == 'I' || uart_handle_buffer[1][1] == 'i') &&
                    (uart_handle_buffer[1][2] == 'M' || uart_handle_buffer[1][2] == 'm') &&
                    (uart_handle_buffer[1][3] == 'E' || uart_handle_buffer[1][3] == 'e') && handlelen[1] == 4)
                {
                    if(handlelen[2] == 8 && uart_handle_buffer[2][2] == ':' && uart_handle_buffer[2][5] == ':')
                    {
                        uint8_t temp_hour = (uart_handle_buffer[2][0] - '0') * 10 + (uart_handle_buffer[2][1] - '0');
                        uint8_t temp_minute = (uart_handle_buffer[2][3] - '0') * 10 + (uart_handle_buffer[2][4] - '0');
                        uint8_t temp_second = (uart_handle_buffer[2][6] - '0') * 10 + (uart_handle_buffer[2][7] - '0');

                        if(temp_hour >= 0 && temp_hour < 24 && temp_minute >= 0 && temp_minute < 60 && temp_second >= 0 && temp_second < 60)
                        {
                            hour = temp_hour;
                            minute = temp_minute;
                            second = temp_second;
                            UARTStringPut(UART0_BASE, (uint8_t*)"Successfully setting the time\r\n");
                        }
                        else
                        {
                            UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
                        }
                    }
                    else
                    {
											UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
                    }
                }
                else if( (uart_handle_buffer[1][0] == 'A' || uart_handle_buffer[1][0] == 'a') &&
                         (uart_handle_buffer[1][1] == 'L' || uart_handle_buffer[1][1] == 'l') &&
                         (uart_handle_buffer[1][2] == 'R' || uart_handle_buffer[1][2] == 'r') &&
                         (uart_handle_buffer[1][3] == 'M' || uart_handle_buffer[1][3] == 'm') && handlelen[1] == 4)
                {
                    if(handlelen[2] == 8 && uart_handle_buffer[2][2] == ':' && uart_handle_buffer[2][5] == ':')
                    {
                        uint8_t temp_hour = (uart_handle_buffer[2][0] - '0') * 10 + (uart_handle_buffer[2][1] - '0');
                        uint8_t temp_minute = (uart_handle_buffer[2][3] - '0') * 10 + (uart_handle_buffer[2][4] - '0');
                        uint8_t temp_second = (uart_handle_buffer[2][6] - '0') * 10 + (uart_handle_buffer[2][7] - '0');

                        if(temp_hour >= 0 && temp_hour < 24 && temp_minute >= 0 && temp_minute < 60 && temp_second >= 0 && temp_second < 60)
                        {
                            alm_hour = temp_hour;
                            alm_minute = temp_minute;
                            alm_second = temp_second;
                            UARTStringPut(UART0_BASE, (uint8_t*)"Successfully set the alarm clock\r\n");
                        }
                        else
                        {
													UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
                        }
                    }
                    else
                    {
                        UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
                    }
                }
            }
            else if( (uart_handle_buffer[0][0] == 'G' || uart_handle_buffer[0][0] == 'g') &&
                     (uart_handle_buffer[0][1] == 'E' || uart_handle_buffer[0][1] == 'e') &&
                     (uart_handle_buffer[0][2] == 'T' || uart_handle_buffer[0][2] == 't'))
            {
                if( (uart_handle_buffer[1][0] == 'T' || uart_handle_buffer[1][0] == 't') &&
                    (uart_handle_buffer[1][1] == 'I' || uart_handle_buffer[1][1] == 'i') &&
                    (uart_handle_buffer[1][2] == 'M' || uart_handle_buffer[1][2] == 'm') &&
                    (uart_handle_buffer[1][3] == 'E' || uart_handle_buffer[1][3] == 'e') && handlelen[1] == 4)
                {
                    UARTStringPut(UART0_BASE, (uint8_t*)"The current time is");
                    UARTStringPut(UART0_BASE, time_transmit_buffer);
                    UARTStringPut(UART0_BASE, (uint8_t*)"\r\n");
                }
                else if( (uart_handle_buffer[1][0] == 'A' || uart_handle_buffer[1][0] == 'a') &&
                         (uart_handle_buffer[1][1] == 'L' || uart_handle_buffer[1][1] == 'l') &&
                         (uart_handle_buffer[1][2] == 'R' || uart_handle_buffer[1][2] == 'r') &&
                         (uart_handle_buffer[1][3] == 'M' || uart_handle_buffer[1][3] == 'm') && handlelen[1] == 4)
                {
                    if(alm_hour == 25)
                    {
                        UARTStringPut(UART0_BASE,(uint8_t*)"You haven't set the alarm yet.");
                    }
                    else
                    {
											UARTStringPut(UART0_BASE,(uint8_t*)"The current alarm is");
                        UARTStringPut(UART0_BASE,alarm_transmit_buffer);
                        UARTStringPut(UART0_BASE,(uint8_t*)"\r\n");
                    }
                }
                else
                {
									UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
                }
            }
        }
        else
        {
					UARTStringPut(UART0_BASE, (uint8_t*)"This instruction is an illegal one.\r\n");
        }
    }
}






