//*******************
//*******************
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "string.h"
#include <stdio.h>
//#include "hibernate.h"
#define SYSTICK_FREQUENCY		1000			//1000Hz
#define flow_flash_time1		1000			//1s
#define flow_flash_time2		2000			//2s

//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06


void UARTStringPut(const char *cMessage);
void UARTStringPutNonBlocking(const char *cMessage);
void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);



//systick software counter define
volatile uint16_t systick_1ms_couter=0,systick_10ms_couter=0,systick_20ms_couter=0,systick_100ms_couter=0,systick_1s_couter=0,systick_flash_couter=0;
volatile uint8_t	systick_1ms_status=0,systick_10ms_status=0,systick_20ms_status=0,systick_100ms_status=0,systick_1s_status=0;

volatile uint8_t flash_place = 0,flash_flag = 0,startup_flag = 0;                 //���������˸λ����˸��־���Ƿ����¼�ʱ������86400s��
uint8_t position = 0;                 //�����ѡͨ��λ��
volatile uint16_t position_flash_cnt = 0;     //ˢ������ܼ�������3msˢ��һ��
volatile uint8_t state=0, set_flag=0;   //�������ʾ״̬���Ƿ���Ҫ��������ʱ��

uint32_t ui32SysClock;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
uint8_t led7[] = {0x7f,0xbf,0xdf,0xef,0xf7,0xfb,0xfd,0xfe};
uint8_t name[] = {0x38,0x30, 0x76, 0x30, 0x77, 0x37,0x7D,0x00}; //����



//-------------------------SW------------------------------//
volatile uint8_t	status_SW[8] = {0}, last_status_SW[8] = {0}, keyflag[8] = {0}, delayflag[8] = {0};   //0-7��ӦSW1-8,���ºͳ�����־
volatile uint16_t keydelay[8] = {0};  //ÿ������������ʱ��
void	checkbutton(void);



//----------------------start up--------------------------//
void display_studentcode(void);
uint8_t start_position=0;
void start_up(void);              //��������
void display_name(void);


//-----------------------flow_display-------------------------//
void flow_display(void);   //��ˮ��ʾʱ������
uint8_t flow16[16]={0};  //��ˮ����
uint16_t flow_cnt=0;  //��ˮ״̬
uint16_t flow_flash_cnt=0;      //��ˮˢ�¼�ʱ
uint16_t flow_flash_time=flow_flash_time1;   //��ˮ�ٶ�




//-----------------------uart receive---------------------//
char receive_order[50];           //���յ�������

char settime_order[9]="T12:00:00";
char setalarmtime_order[9]="A12:00:00";
char setdate_order[11]="D2024-06-12";

bool settime_flag=0,setdate_flag=0,setalarm_flag=0,gettime_flag=0,getdate_flag=0,getalarm_flag=0,initclock_flag=0,help_flag=0;     // �����־



//-----------------------time-----------------------------//
volatile uint32_t second=43200;        // �ܼ�ʱ��������ʼΪ12-00-00
char currenttime[] = "TIME12:00:00";     //���͵�ǰʱ�����
void settime(uint8_t flash_place,bool direction);          //��������ʱ��     
void display_time(uint8_t flash_place);         //�������ʾʱ��
uint8_t time_flash[7] = {8,7,5,4,2,1,0};         //0�����˳���˸���



//-----------------------date-------------------------------//
volatile uint32_t year=2024,month=6,day=12,total_days=164;        //��ʼ��2024-6-12
char currentdate[]="DATE2024-06-12";      //��������
void date_transform(void);                               //total_daysת��Ϊ����
void check_year(void);                                   //����Ƿ�Ϊ���꣬���޸�2��������������
void setdate(uint8_t flash_place,bool direction);         //������������
void display_date(uint8_t flash_place);                 //�������ʾ����


uint16_t days_mon[13] = {31,28,31,30,31,30,31,31,30,31,30,31,365};  //�·��������������
uint8_t date_flash[9] = {8,7,6,5,4,3,2,1,0};



//------------------------alarm-----------------//
uint32_t alarm_time=43500;              //���ӳ�ʼ12-05-00
char currentalarm[]="ALARM12-05-00";
void setalarm(uint8_t flash_place,bool direction);     //Ӳ������
void display_alarm(uint8_t flash_place);            //�������ʾ����
volatile uint16_t alarm_flash_cnt=0;
uint8_t alarm_position=0;                   
uint8_t alarm_ring_flag=0; 
uint16_t ringtime=0,ringcnt=0;    //��������ʱ��







int main(void)
{
	uint8_t j=0;     //ѭ������
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);

  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);
	SysTickEnable();
	SysTickIntEnable();																		//Enable Systick interrupt


	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();

	IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
  IntMasterEnable();
	start_up();
	
	while (1)
	{
		if(settime_flag)   //��������ʱ��
		{
		 settime_flag=0;
	   second=(settime_order[1]-'0')*36000+(settime_order[2]-'0')*3600+(settime_order[4]-'0')*600+(settime_order[5]-'0')*60+(settime_order[7]-'0')*10+(settime_order[8]-'0');
		}
		
		
		if(setdate_flag)   //������������
		{
		 setdate_flag=0;
	   year=(setdate_order[1]-'0')*1000+(setdate_order[2]-'0')*100+(setdate_order[3]-'0')*10+(setdate_order[4]-'0');
		 year=(year)%10000;
		 month=(setdate_order[6]-'0')*10 + (setdate_order[7]-'0');
		 day=(setdate_order[9]-'0')*10+(setdate_order[10]-'0');
			
		 total_days=0;
		 check_year();
			
		 	
		 for(;j<month;j++){ 
			 total_days+=days_mon[j];
		 }
		 j=0;
		 total_days+=day;
		 total_days-=days_mon[month-1];
		 
		}
		
		
		
	if(setalarm_flag)   //��������ʱ��
		{
		 setalarm_flag= 0;
	   alarm_time=(setalarmtime_order[1]-'0')*36000+(setalarmtime_order[2]-'0')*3600+(setalarmtime_order[4]-'0')*600+(setalarmtime_order[5]-'0')*60+(setalarmtime_order[7]-'0')*10+(setalarmtime_order[8]-'0');
		 alarm_time=alarm_time%86400;
		}

		
		
		
		
		
		
	if(gettime_flag)  //��ȡʱ��
		{
		 gettime_flag = 0;
		  currenttime[4] = second/3600/10+'0';currenttime[5] = second/3600%10+'0';
      currenttime[7] = second%3600/60/10+'0';currenttime[8] = second%3600/60%10+'0';
      currenttime[10] = second%60/10+'0';currenttime[11] = second%60%10+'0';
	    UARTStringPutNonBlocking(currenttime);
			UARTStringPutNonBlocking("\n");
	  }
		
		
		
		if(getdate_flag)  //��ȡ����
		{
			getdate_flag = 0;
		 currentdate[4]= year/1000+'0';currentdate[5]=year%1000/100+'0';currentdate[6]=year%100/10+'0';currentdate[7] = year%10+'0';
     currentdate[9] = month/10+'0';currentdate[10] = month%10+'0';
	   currentdate[12] = day/10+'0';currentdate[13] = day%10+'0';
			UARTStringPutNonBlocking(currentdate);
			UARTStringPutNonBlocking("\n");
		}
		
		
		if(getalarm_flag)     //��ȡ����ʱ��
		{
			getalarm_flag = 0;
		  currentalarm[5] = alarm_time/3600/10+'0';currentalarm[6] = alarm_time/3600%10+'0';
	    currentalarm[8] = alarm_time%3600/60/10+'0';currentalarm[9] = alarm_time%3600/60%10+'0';
      currentalarm[11]=alarm_time%60/10+'0';currentalarm[12]=alarm_time%60%10+'0';
	    UARTStringPutNonBlocking(currentalarm);
			UARTStringPutNonBlocking("\n");
			
		}
		
		
		if(initclock_flag)   //��ʼ��ʱ��
		{
		  initclock_flag = 0;
			second = 43200;      //��ʼ��Ϊ12-00-00
		}
		
		
		if(help_flag)    //��ʾ��������ĸ�ʽ
		{
		  help_flag = 0;
			UARTStringPut("INITCLOCK\nT12:30:00\nA12:30:00\nD2024-06-18\nGETTIME\nGETDATE\nGETALARM\n");
		}
		
		

		
				//-----------------------state 0----------------//
		
		
    if(state==0)        //��ˮ��ʾ
		{


				if(keyflag[5])
				{		          // SW6 ���£��л���ˮ�ٶ�
					 keyflag[5] = 0;
					
					 if(flow_flash_time==flow_flash_time1)
					   {flow_flash_time=flow_flash_time2;}
					 else 
					   {flow_flash_time=flow_flash_time1;}
				}

				flow_display();

			
		}
		
		
		
		
		//-----------------------state 1----------------//
		
		
    if(state==1)     //��ʾ������ʱ��
		{
			if(set_flag == 0)
			{		                     // display
				flash_place = 6;
			}
		 	else{								         // set
				flash_place = flash_place % 6;
				if(keyflag[3])
				{		          // SW4 ���£�����
					 keyflag[3] = 0;
					flash_place = (flash_place + 5) % 6;
				}
				if(keyflag[2])
				{		          // SW3 ���£�����
					 keyflag[2] = 0;
					flash_place = (flash_place + 1) % 6;
				}
				if(keyflag[6]||delayflag[6])
				{		// SW7���£�ʱ���
					 keyflag[6] = delayflag[6] = 0;
					settime(flash_place,1);
				}
				if(keyflag[7]||delayflag[7])
				{		// SW8 ���£�ʱ���
					 keyflag[7] = delayflag[7] = 0;
					settime(flash_place,0);
				}
			}
			display_time(time_flash[flash_place]);
		}
		
		
		
		//-----------------state 2--------------//
		if(state == 2)   //��ʾ����������
		{
		 if(set_flag == 0)
		 {
		  flash_place = 8;
		 }
		 
		 else{     //set
		    flash_place = flash_place %8;
			 
		   if(keyflag[3])
				{		          // SW4 ���£�����
					 keyflag[3] = 0;
					flash_place = (flash_place + 7) % 8;
				}
				
				if(keyflag[2])
				{		          // SW3 ���£�����
					 keyflag[2] = 0;
					flash_place = (flash_place + 1) % 8;
				}
				
				if(keyflag[6] || delayflag[6])
				{                            //sw7 ���£����ڼ�
				 keyflag[6] = delayflag[6] = 0;
					if(total_days < 31)
					{
					  year = (year +9999)%10000;   //year --
						total_days += days_mon[12];
					}
					setdate(flash_place,1);
				}
				
				if(keyflag[7] || delayflag[7])
				{                               //sw8 ���£����ڼ�
				 keyflag[7] = delayflag[7] = 0;

					setdate(flash_place,0);
				}
				
		 }
		 display_date(date_flash[flash_place]);
		}
		
		
		
		
		//-------------state 3---------------//
		if(state == 3)  //��ʾ����������
		{
		 if(set_flag == 0)
			{		                     // display
				flash_place = 6;
			}
			
		 	else{								         // set
				flash_place = flash_place % 6;
				if(keyflag[3])
				{		          // SW4 ���£�����
					 keyflag[3] = 0;
					flash_place = (flash_place + 5) % 6;
				}
				if(keyflag[2])
				{		          // SW3 ���£�����
					 keyflag[2] = 0;
					flash_place = (flash_place + 1) % 6;
				}
				if(keyflag[6]||delayflag[6])
				{		           // SW7���£�����ʱ���
					 keyflag[6] = delayflag[6] = 0;
					setalarm(flash_place,1);
				}
				if(keyflag[7]||delayflag[7])
				{		           // SW8 ���£�����ʱ���
					 keyflag[7] = delayflag[7] = 0;
					setalarm(flash_place,0);
				}
			}
			display_alarm(time_flash[flash_place]);           //���Ӻ�ʱ���ʽһ��
		}
		
		

		
		
		
		//-----------alarmring----------------//
		if(second == alarm_time)           //�ﵽ�趨ʱ��
		{
		 alarm_ring_flag=1;
			state = 1;      //�ع�ʱ�ӽ���
		}
		if(alarm_ring_flag&&systick_100ms_status )    //���÷�����Ƶ��
		{
		 systick_100ms_status = 0;
			if(++ringcnt>=5)  //ÿ500ms
			{
			  ringcnt = 0;
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, ~GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_5));
			}
			
			if(++ringtime>=100||keyflag[4])   //10s����߰���sw5��ֹͣ����
			{ 
				keyflag[4] = 0;
				alarm_ring_flag = 0;
				ringtime = 0;
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0x00);
			}
		}

		
		

		
		
  }
	
	
}





void start_up(void)
{
   uint8_t systick_500ms_couter= 0,flash_couter=0;
	 startup_flag = 1;                       //����ʱֹͣʱ��
	 while(flash_couter<16){    //8s
	   if(systick_100ms_status)    //ÿ100ms                
	   {
	     systick_100ms_status = 0;
		    if(++systick_500ms_couter>= 5)  //ÿ500ms 
		     {
		       systick_500ms_couter=0;
			     flash_couter++;
		   }
	}
		 
	   if(flash_couter== 1)
	     {
	        I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0);		//LEDȫ��
	        I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x0FF);		//�����ȫ��  
	        I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0FF);			
    }
			 
	    if(flash_couter== 2){
	         I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);		    //LEDȫ��
	         I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x0);		//�����ȫ��
	         I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0FF);				
	   }
			

	
	if((flash_couter%2 ==0)&&(flash_couter>=3)&&(flash_couter<=8))  //��ʾѧ��,3s
	{
		I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0);		//LEDȫ��
		display_studentcode();
	}
	
	
  if((flash_couter%2 ==0)&&(flash_couter>=9))       //��ʾ����,4s
	{
		   I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0);		//LEDȫ��
		   display_name();
	}
	
	
	
	if((flash_couter%2==1)&&(flash_couter>=3))
	{
	   I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);		    //LEDȫ��
	   I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x0);		  //�����ȫ��
	   I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x0FF);				
	}
	
	
}
	

  I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);		    //LEDȫ��
	startup_flag = 0; //ʱ�ӿ�ʼ
	
}








void setalarm(uint8_t flash_place,bool direction)  //������������ʱ��
{
 if(direction == 0)       //����ʱ���
	{
	 switch(flash_place)
			{
				case 0:alarm_time = (alarm_time + 1) % 86400;break;
				case 1:alarm_time = (alarm_time + 10) % 86400;break;
				case 2:alarm_time = (alarm_time + 60) % 86400;break;
				case 3:alarm_time = (alarm_time + 600) % 86400;break;
				case 4:alarm_time = (alarm_time + 3600) % 86400;break;
				case 5:alarm_time = (alarm_time + 36000) % 86400;break;
			}
	}
  else{                     //����ʱ���
	  switch(flash_place)
		{
			case 0:alarm_time = (alarm_time + 86400 - 1) % 86400;break;
			case 1:alarm_time = (alarm_time + 86400 - 10) % 86400;break;
			case 2:alarm_time = (alarm_time + 86400 - 60) % 86400;break;
			case 3:alarm_time = (alarm_time + 86400 - 600) % 86400;break;
			case 4:alarm_time = (alarm_time + 86400 - 3600) % 86400;break;
			case 5:alarm_time = (alarm_time + 86400 - 36000) % 86400;break;
		}
	}
}



void setdate(uint8_t flash_place,bool direction)
{
 if(direction == 0)     //���ڼ�
 {
  switch(flash_place)
		{
			case 0:total_days+=1;	  break;
			case 1:total_days+=10;   break;
			case 2:total_days+=days_mon[month-1];  break;
      case 3:total_days+=days_mon[month-1];  break; 			
			case 4:year = (year+1) % 10000;break;
			case 5:year = (year+10) % 10000;break;
			case 6:year = (year+100) % 10000;break;
			case 7:year = (year+1000) % 10000;break;
		}
 }
 
 else {
    switch(flash_place)
			{
				case 0:total_days -= 1;break;
				case 1:total_days -= 10;break;
				case 2:total_days -= days_mon[month-1];break;
				case 3:total_days -= days_mon[month-1];break;
				case 4:year = (year + 9999) % 10000;break;
				case 5:year = (year + 9990) % 10000;break;
				case 6:year = (year + 9900) % 10000;break;
				case 7:year = (year + 9000) % 10000;break;
			}
 }
 
}



void settime(uint8_t flash_place,bool direction)
{
  if(direction == 0)       //ʱ���
	{
	 switch(flash_place)
			{
				case 0:second = (second + 1) % 86400;break;
				case 1:second = (second + 10) % 86400;break;
				case 2:second = (second + 60) % 86400;break;
				case 3:second = (second + 600) % 86400;break;
				case 4:second = (second + 3600) % 86400;break;
				case 5:second = (second + 36000) % 86400;break;
			}
	}
	
  else{                     //ʱ���
	  switch(flash_place)
		{
			case 0:second = (second + 86400 - 1) % 86400;break;
			case 1:second = (second + 86400 - 10) % 86400;break;
			case 2:second = (second + 86400 - 60) % 86400;break;
			case 3:second = (second + 86400 - 600) % 86400;break;
			case 4:second = (second + 86400 - 3600) % 86400;break;
			case 5:second = (second + 86400 - 36000) % 86400;break;
		}
	
	}
}	



void display_studentcode(void)
{
   uint8_t num=0,pos=0;    //
	  if (systick_1ms_status) // timing 1ms
		{
			systick_1ms_status=0; //	clear the status of 1ms
			
			if (++position_flash_cnt>= 3)      // �����
			{
				position_flash_cnt=0;
				start_position=(start_position+1) % 8;
			}
		}
		
		switch(start_position)  //ѧ�ź��λ31910384  0x4f,0x06,0x6f,0x06,0x3f,0x4f,0x7f,0x66
		{
			case 0:num = seg7[1]; pos = 0x4f;break;
			case 1:num = seg7[2];	pos = 0x06;break;
			case 2:num = seg7[3];	pos = 0x6f;break;
			case 3:num = seg7[4];	pos = 0x06;break;
			case 4:num = seg7[5];	pos = 0x3f;break;
			case 5:num = seg7[6];	pos = 0x4f;break;
			case 6:num = seg7[7];	pos = 0x7f;break;
			case 7:num = seg7[8]; pos = 0x66;break;
		}
		
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);				//�����ȫ��
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num);		      //write port 1
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,pos);				  //write port 2
}




void display_name(void)
{
      uint8_t num=0,pos=0;   
	  if (systick_1ms_status) 
		{
			systick_1ms_status=0;
			
			if (++position_flash_cnt>= 3)     
			{
				position_flash_cnt=0;
				start_position=(start_position+1) % 8;
			}
		}
		
		switch(start_position)    //����
		{
			case 0:num = name[0]; pos = 0x01;break;
			case 1:num = name[1];	pos = 0x02;break;
			case 2:num = name[2];	pos = 0x04;break;
			case 3:num = name[3];	pos = 0x08;break;
			case 4:num = name[4];	pos = 0x10;break;
			case 5:num = name[5];	pos = 0x20;break;
			case 6:num = name[6];	pos = 0x40;break;
			case 7:num = name[7]; pos = 0x80;break;
		}
		
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);				//�����ȫ��
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num);		      //write port 1
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,pos);				  //write port 2


}




void check_year(void)
{
  if(year%100){
       if(year%4==0){
            days_mon[1] = 29;
	           days_mon[12] = 366;
              }
       else{
           days_mon[1] = 28;
	         days_mon[12] =365;
         } 
}
 else{
     if(year%400==0){
		 days_mon[1] = 29;
			 days_mon[12] = 366;
		 }
		 else{
		 days_mon[1] = 28;
			 days_mon[12] = 365;
		 }
 }
}






void display_alarm(uint8_t flash_place)
{
 uint8_t num = 0,pos = 0;
	if (systick_1ms_status)          // timing 1ms
		{
			systick_1ms_status	= 0;       //	clear the status of 1ms
			if (++alarm_flash_cnt>= 3)
			{
				alarm_flash_cnt=0;
				alarm_position = (alarm_position+1) % 8;               
			}
		}

		switch(alarm_position)
		{
			case 0:num = seg7[alarm_time/3600/10];     pos = 0x01;break;
			case 1:num = seg7[alarm_time/3600%10];	   pos = 0x02;break;
			case 2:num = 0x40;	                       pos = 0x04;break;
			case 3:num = seg7[alarm_time%3600/60/10];	 pos = 0x08;break;
			case 4:num = seg7[alarm_time%3600/60%10];	 pos = 0x10;break;
			case 5:num = 0x40;                         pos = 0x20;break;
			case 6:num = seg7[alarm_time%60/10];		   pos = 0x40;break;
			case 7:num = seg7[alarm_time%60%10];	     pos = 0x80;break;
		}
		
		if(flash_place &&flash_flag)  //������˸
		{
		 if((alarm_position+1)==flash_place)
		 {
		  num = 0x00;
		 }
		}
		
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);			//�����ȫ��
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num);		    //write port 1
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,pos);				//write port 2
		
}



void display_date(uint8_t flash_place)
{
   uint8_t num=0,pos=0;
	
	  date_transform();
	
	if (systick_1ms_status) // ÿ3ms
		{
			systick_1ms_status	= 0; 
			if (++position_flash_cnt >= 3)    // ÿ3ms
			{
				position_flash_cnt = 0;
				position=(position + 1) % 8;

			}
		}
		switch(position)
		{
			case 0:num = seg7[year/1000];        pos = 0x01;break;
			case 1:num = seg7[year%1000/100];    pos = 0x02;break;
			case 2:num = seg7[year%100/10];      pos = 0x04;break;
			case 3:num = seg7[year%10] | 0x80;   pos = 0x08;break;
			case 4:num = seg7[month/10];         pos = 0x10;break;
			case 5:num = seg7[month%10] | 0x80;  pos = 0x20;break;
			case 6:num = seg7[day/10];           pos = 0x40;break;
			case 7:num = seg7[day%10];           pos = 0x80;break;
		}
		
		
		if(flash_place&&flash_flag)   //��˸
		{
		 if((position+1)== flash_place)
			 num = 0x00;
		}
		
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);				//�����ȫ��
	  I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num);				//write port 1
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,pos);				//write port 2
		
}




void display_time(uint8_t flash_place)   //�������ʾʱ��
{
   uint8_t num=0,pos=0;
	  if (systick_1ms_status) // ÿ1ms
		 {
			systick_1ms_status	= 0;
			if (++position_flash_cnt>=3)  // ÿ3ms
			{
				position_flash_cnt=0;
				position=(position+1)%8;
			}
		 }

		switch(position)            //��ʾʱ��
		{
			case 0:num = seg7[second/3600/10];    pos= 0x01;break;
			case 1:num = seg7[second/3600%10];    pos= 0x02;break;
			case 2:num = 0x40;	                  pos= 0x04;break;
			case 3:num = seg7[second%3600/60/10]; pos= 0x08;break;
			case 4:num = seg7[second%3600/60%10]; pos= 0x10;break;
			case 5:num = 0x40;                    pos= 0x20;break;
			case 6:num = seg7[second%60/10];		  pos= 0x40;break;
			case 7:num = seg7[second%60%10];	    pos= 0x80;break;
		}
		
		if(flash_place&&flash_flag)  //������˸
		{
		 if((position+1)==flash_place)
		 {
		  num= 0x00;   //Ϩ��
		 }
		}
		
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);				//�����ȫ��
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num);		     //write port 1
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,pos);				//write port 2
		
}




void date_transform(void)
{	
	uint8_t tmp_month = 1,i = 0;
	
	check_year();
	
  while(total_days>=days_mon[12])
	{
	   year=(year+1) %10000;
		 total_days-=days_mon[12];
		 check_year();
	}
	day=total_days;
	
	while(day>days_mon[i])
	{
	    tmp_month++;
		  day-=days_mon[i];
		  i++;
	}
	month=tmp_month;
//	day++;
	
}








void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}



void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}



void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}



void	checkbutton(void)   //20ms���һ��
{
 uint8_t i = 0,key_sw=~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	
	for(;i<8;i++)  //��ⰴ��״̬
	{
		 last_status_SW[i] = status_SW[i];
		 status_SW[i] = key_sw&1<<i;
		
		if(last_status_SW[i]==0&&status_SW[i]==(1<<i))     //SW(i+1)������
			keyflag[i] = 1;
		
		if(last_status_SW[i]==(1<<i)&&status_SW[i]==(1<<i))   //��������SW(i+1)
		{
			keydelay[i]++;
			if(keydelay[i]>=9&&i!=3)  //�����ǰ���3����1.8s
			{
				keydelay[i]=0;
				delayflag[i]=1;
			}
		}
		
		else keydelay[i]=0;
		
	 if(last_status_SW[i]==(1<<i)&&status_SW[i]==0)   //�����ɿ�
			 keyflag[i]=0;
	 
	}
	
	
	
	if(keyflag[0]){		      // SW1������
		keyflag[0] = 0;
		state=(state+1) % 4;
		set_flag=0;
		flow_cnt=0;
	}
	
	if(keyflag[1]){		      //SW2������
		keyflag[1]=0;
		set_flag=1-set_flag;
	}
	
}



void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready
	
	// Set GPIO A0 and A1 as UART pins.
	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	
	// Set TxFIFO to 1/4, RxFIFO to 1/2.
	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX2_8,UART_FIFO_RX7_8);//set FIFO Level
	
	// Enable UART0 RX, TX interrupt
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
	IntEnable(INT_UART0);

	UARTStringPut("\r\nHello, world!\r\n"); 
}




void S800_GPIO_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);						//Enable PortK	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));			//Wait for the GPIO moduleK ready	
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			          //Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);			//Set PN0,PN1 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);			         //Set PK5 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
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
  result = 	I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
}




uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}




uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
uint8_t value;

	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
		return 0;
	Delay(100);

	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
		return 0;
	Delay(100);

	return value;
}






//SysTick
void SysTick_Handler(void)
{
	systick_1ms_status = 1;
	if (systick_1s_couter == 0) // Generate a 1s timer 
	{
		systick_1s_couter = 1000;
		if(startup_flag!=1)         //����������ʾ����
		  second = (second + 1) % 86400;
		systick_1s_status = 1;
	}
	else
		systick_1s_couter--;
	
	
	if(systick_flash_couter == 0)  //ˢ������300ms
	{
	 systick_flash_couter = 300;
		flash_flag = 1 - flash_flag ;         //ȡ��
	}
	else systick_flash_couter--;
	
	
	if (systick_100ms_couter == 0) // Generate a 100ms timer 
	{
		systick_100ms_couter = 100;
		systick_100ms_status = 1;
	}
	else
		systick_100ms_couter--;
	
	
  if (systick_20ms_couter	== 0) // Generate a 20ms timer 
	{
		systick_20ms_couter	 = 20;
		systick_20ms_status  = 1;
		checkbutton();        //��ⰴ��״̬
	}
	else
		systick_20ms_couter--;
	
	
	if (systick_10ms_couter	== 0) // Generate a 10ms timer 
	{
		systick_10ms_couter	 = 10;
		systick_10ms_status  = 1;
	}
	else
		systick_10ms_couter--;
	
}




//UART SysTick
void UART0_Handler(void)
{ 
	uint8_t cnt=0;
	int32_t uart0_int_status;
	
  uart0_int_status=UARTIntStatus(UART0_BASE, true);		  // Get the interrrupt status.

  UARTIntClear(UART0_BASE, uart0_int_status);								  //Clear the asserted interrupts

	if (uart0_int_status&(UART_INT_RX | UART_INT_RT)) 			  // receive or out of second
	{
    while(UARTCharsAvail(UART0_BASE))
	   {
          receive_order[cnt]=UARTCharGetNonBlocking(UART0_BASE);
		      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,GPIO_PIN_1 );  //����D1
          cnt++;
	 }
	       receive_order[cnt] = '\0';
	       cnt = 0;
	 
	 //ʶ������
	    if(receive_order[0]!='?')
	     {
	       if(receive_order[0] == 'I')
		       initclock_flag = 1;

	          if(receive_order[0] == 'T'){
		         settime_flag = 1;
						  settime_order[1]=receive_order[1];
							settime_order[2]=receive_order[2];
							settime_order[4]=receive_order[4];
							settime_order[5]=receive_order[5];
							settime_order[7]=receive_order[7];
							settime_order[8]=receive_order[8];
							
						}
	          if(receive_order[0] == 'A'){
		          setalarm_flag = 1;
							setalarmtime_order[1]=receive_order[1];
							setalarmtime_order[2]=receive_order[2];
							setalarmtime_order[4]=receive_order[4];
							setalarmtime_order[5]=receive_order[5];
							setalarmtime_order[7]=receive_order[7];
							setalarmtime_order[8]=receive_order[8];
						}
						if(receive_order[0] == 'D'){
		          setdate_flag =1;
							setdate_order[1]=receive_order[1];
							setdate_order[2]=receive_order[2];
							setdate_order[3]=receive_order[3];
							setdate_order[4]=receive_order[4];
							setdate_order[6]=receive_order[6];
							setdate_order[7]=receive_order[7];
							setdate_order[9]=receive_order[9];
							setdate_order[10]=receive_order[10];
						}
						

	       if(receive_order[0] == 'G')
	        {
	          if(receive_order[3] == 'T')
		        gettime_flag = 1;
	          if(receive_order[3] == 'D')
		         getdate_flag = 1;
	           if(receive_order[3] == 'A')
		         getalarm_flag = 1;
	        }
       }
			 
	else help_flag = 1;  //��Ҫ��������ʾ��ǰ�����ʽ
			 
  }
	
}


void flow_display(void)
{
	   uint8_t num=0,pos=0;

	   uint8_t hour_h,hour_l,minute_h,minute_l,seconds_h,seconds_l,year_q,year_b,year_s,year_g,month_h,month_l,day_h,day_l;
	
	  if (systick_1ms_status) // ÿ1ms
		 {
			systick_1ms_status	= 0;
			 
			if (++position_flash_cnt>=3)  // ÿ3ms
			{
				position_flash_cnt=0;
				position=(position+1)%8;
			}
			
			if (++flow_flash_cnt>=flow_flash_time)  // ÿ500ms
			{
				flow_flash_cnt=0;
				flow_cnt=(flow_cnt+1)%16;
			}
			
		 }
   
		 hour_h=seg7[second/3600/10];
		 hour_l=seg7[second/3600%10];
		 minute_h=seg7[second%3600/60/10];
		 minute_l=seg7[second%3600/60%10];
		 seconds_h=seg7[second%60/10];
		 seconds_l=seg7[second%60%10];
		 year_q=seg7[year/1000];
		 year_b=seg7[year%1000/100];
		 year_s=seg7[year%100/10];
		 year_g=seg7[year%10] | 0x80;
		 month_h=seg7[month/10];
		 month_l=seg7[month%10] | 0x80;
		 day_h=seg7[day/10];
		 day_l=seg7[day%10];
		 
		 flow16[0]=year_q;
		 flow16[1]=year_b;
		 flow16[2]=year_s;
		 flow16[3]=year_g;
		 flow16[4]=month_h;
		 flow16[5]=month_l;
		 flow16[6]=day_h;
		 flow16[7]=day_l;
		 flow16[8]=hour_h;
		 flow16[9]=hour_l;
		 flow16[10]=0x40;
		 flow16[11]=minute_h;
		 flow16[12]=minute_l;
		 flow16[13]=0x40;
		 flow16[14]=seconds_h;
		 flow16[15]=seconds_l;
		 
		 
		switch(position)            //��ʾ
		{
			case 0:num = flow16[flow_cnt];        pos= 0x01;break;
			case 1:num = flow16[(flow_cnt+1)%16];    pos= 0x02;break;
			case 2:num = flow16[(flow_cnt+2)%16];	                  pos= 0x04;break;
			case 3:num = flow16[(flow_cnt+3)%16];   pos= 0x08;break;
			case 4:num = flow16[(flow_cnt+4)%16];     pos= 0x10;break;
			case 5:num = flow16[(flow_cnt+5)%16];                    pos= 0x20;break;
			case 6:num = flow16[(flow_cnt+6)%16];		  pos= 0x40;break;
			case 7:num = flow16[(flow_cnt+7)%16];	    pos= 0x80;break;
		}
		

		
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);				//�����ȫ��
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,num);		     //write port 1
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,pos);				//write port 2
	
	
	
}



