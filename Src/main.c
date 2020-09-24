
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define DIR_FORWARD     1
#define DIR_BACKWARD   -1
#define TEST_DISTANCE  1200       // cm
#define SLOWDOWN_DISTANCE  (70)
#define FLASH_SAVE_ADDR 0x08008000///sector2 


volatile int gs32TestDistance = TEST_DISTANCE;	// unit(cm)
int gs32TestDistanceBkp;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t rx1Buffer[1];		//uart1 receive buffer
uint8_t rx2Buffer[1];		//uart2 receive buffer

enum {isNone=0, isReset, isTest, isCheck, isRecord, isLeft2Right, isRigth2Left, isCW, isCCW}rx1Order;
//static uint16_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数
int32_t calPulseCount1 = 0; //+ - 
int32_t calPulseCount2 = 0;

int32_t direction1;
int32_t direction2;
int32_t motordirection;//电机方向
uint32_t directionflag;//方向标志位
uint32_t speedplusfinish;//加速完成标志位
uint32_t speedminusfinish;//减速完成标志位
uint32_t pulsefinish;//每走1cm标志位
uint32_t startflag;//开始标志
uint32_t endflag;//终点标志
uint32_t fixflag=0;//定点测试标志
uint32_t sendflag=0;//定点发送接受到
uint32_t test_time_flag=0;
volatile uint32_t motorflag=0;//接收上位机指令 电机开始
uint32_t timeflag=0;//第一趟或者第二趟

uint32_t mininum=0;//tfmini
uint32_t tfnum=0;//tf02
int32_t steps1;
int32_t steps2;

//zero VW, HB, HG, HW(horizontal white)
int32_t positionArray[] = {0, 61, 411, 761, 1111};  //mm
//int32_t steps1Array[5] = {}; //mm *  200
int32_t angleArray[] = {0, 90, 150, 589}; //.1 degree
//int32_t steps2Array[4] = {}; //.1 degree * 50
uint8_t DA[] = {0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int32_t timFlag1 = -1;
int32_t timFlag2 = -1;
int32_t timFlag3 = -1;
int32_t temp = 0;
int32_t m = 0;
uint32_t iii = 0;
uint32_t ispeed=0;
uint32_t ccccc=0;
uint8_t stopHOrder = 0;
uint8_t stopVOrder = 0;
char nnn=0;
volatile uint32_t PWMcount=0;
volatile uint32_t Distance=0;
//定点测试
volatile uint32_t distance_interval=0;//test distance interval
volatile uint32_t test_time=0;
volatile uint32_t distance_dis=0;
volatile uint32_t distance_percent=0;//进度条
uint32_t sendtime=0;
uint32_t errortime=0;
uint32_t nummini=0;///////tfmini
uint32_t numtf=0;////////tf02
typedef struct {
  uint32_t position;  // mm
  uint32_t angle; //0~590     .1 degree
}Lidar;
Lidar Lidar1;

//ADC
uint8_t aa;
uint32_t adc13 = 0;
int32_t adcT = 0;
uint32_t ADC_Value[20];

char str1[50];//第一行 distance_dis
char str2[10];//第二行
char str3[10];//第三行
char str4[10];//进度条
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void motorInit(void); //motor 1,2 find zero and stop at (0 mm, 0 degree) 
//direction: 1, from left to right; -1: from right to left; 0: no change
//frequency: 2000~14000 Hz
void setMotor1Speed(int direction, int frequency); 
//direction: 1: CW, -1:CCW, 0: no change
//frequency: 2000~14000 Hz
void setMotor2Speed(int8_t direction, uint16_t frequency);
//complete return 1, else return 0.
void setDisplacementAndAngles(int32_t displacement, int32_t angles);
void printHex(void);

void speedminus(void);
void speedplus(void);

void readflashtest();
void writeflashtest();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
void delay_ms(uint32_t time)
{	
	uint32_t i=8000*time;
	while(i--);
}

//2037 PUL 1cm	150cm 50 50    

int main(void)
{		
	static int step = 0;
	int i=0;
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	reset_gm8125();
	MX_TIM3_Init();
	MX_TIM5_Init();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);/////
	MX_USART2_UART_Init();
	MX_USART5_UART_Init();
	HAL_UART_Receive_IT(&huart2, (uint8_t *)rx2Buffer, 1);
	HMIsendb();//串口屏初始化
	HAL_Delay(200);	
	readflashtest();
	while(1) 
	{		
		if(motorflag == 0) 
		{  
			if (HAL_GPIO_ReadPin(FORWARD_GPIO_Port, FORWARD_Pin) == GPIO_PIN_RESET) {
				motordirection = DIR_FORWARD;
				setMotor1Speed(motordirection, 8110);
			}
			else if (HAL_GPIO_ReadPin(BACKWARD_GPIO_Port, BACKWARD_Pin) == GPIO_PIN_RESET
				&& HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) != GPIO_PIN_RESET) {
				motordirection = DIR_BACKWARD;
				setMotor1Speed(motordirection, 8110);
			}
			else {
				setMotor1Speed(motordirection, 0);
			}
			continue;
		}		
		switch (step) {
			case 0:  //启动测试
				motordirection = DIR_FORWARD;
				break;
			case 1:  //返回的测试
				if (motorflag == 0x0c) {  // 等上位机发送继续的命令  0x04
					motorflag = 0;  // 清0等上位机发 0x04
					continue;
				}
				motordirection = DIR_BACKWARD;	// 其它测试直接返回
				break;
			default :  //测试结束
				motorflag = 0;
				PWMcount = 0;
				ispeed = 0;
				Distance = 0;
				gs32TestDistance = 0;
				step = 0;
				continue;
		}
		if(fixflag==0)//非定点测试
		{
			if (Distance < (gs32TestDistance-SLOWDOWN_DISTANCE)) 
			{  // 加速区段
				speedplus();
			} 
			else if ((Distance>=(gs32TestDistance-SLOWDOWN_DISTANCE)) && (Distance<gs32TestDistance)&&(step==0)) 
			{  //减速区段
				speedminus();
			} 
			else if ((Distance>=(gs32TestDistance-SLOWDOWN_DISTANCE)) && (Distance<gs32TestDistance-2)&&(step)) 
			{ 
				speedminus();
			} 
			else if ((Distance>=gs32TestDistance) && (motordirection==DIR_FORWARD))
			{  // 终点位置
				setMotor1Speed(motordirection, 0);								
				PWMcount = 0;
				ispeed = 0;
				Distance = 0;
				gs32TestDistanceBkp = gs32TestDistance;
				send_atend();
				HAL_Delay(2000);
				step++;
				transmit_update();
			}
			else
			{
				setMotor1Speed(motordirection,4000);
				if (step==1) {  //如果是返回的测试，要判断是否到起点了
					nnn=HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin);
					if (nnn == 0) {
						setMotor1Speed(motordirection, 0);
						distance_dis+=1;
						writeflashtest();
	//					HAL_Delay(5000);
	//					PWMcount = 0;
	//					ispeed = 0;
	//					Distance = 0;
						step = 200;  // 为了进default, 结束测试
					}
				}
			}
		}
		else  //定点测试
		{
			if(Distance<=gs32TestDistance)
			{								
				//setMotor1Speed(motordirection,5000);
				//HAL_Delay(150);//100
				if((Distance%distance_interval==0)&&(step==0)&&(Distance>1))
				{
					setMotor1Speed(0,0);
					HAL_Delay(2000);//1000
					if(test_time_flag<1)
					{
						for(i=0;i<1;i++)//test_time
						{
								send_fix();
								HAL_Delay(10);//20
						}
						test_time_flag++;
					}
					if(sendflag==1)
					{
					sendflag=0;
					test_time_flag=0;
					setMotor1Speed(motordirection,5000);
					HAL_Delay(200);
					}
				}
				else
				{
					setMotor1Speed(motordirection,5000);//speed5000
				}
			}
			if(Distance>gs32TestDistance)
			{
				setMotor1Speed(motordirection,0);
				fixflag=0;//
				step++;
				PWMcount = 0;
				ispeed = 0;
				Distance = 0;
				gs32TestDistanceBkp = gs32TestDistance;
			}
		}
	} 				
}	


//  /* USER CODE END 3 */



void readflashtest()
{
		HAL_FLASH_Unlock();
		distance_dis=STMFLASH_ReadWord((uint32_t)FLASH_SAVE_ADDR+16);
		if(distance_dis==0xFFFFFFFF){distance_dis=0;}
		sprintf(str1,"n0.val=%d",distance_dis);
		HMIsends((uint8_t *)str1);
		HMIsendb();	
		nummini=STMFLASH_ReadWord((uint32_t)FLASH_SAVE_ADDR+4);
		if(nummini==0xFFFFFFFF){nummini=0;}
		sprintf(str2,"n1.val=%d",nummini);
		HMIsends((uint8_t *)str2);
		HMIsendb();	
		numtf=STMFLASH_ReadWord((uint32_t)FLASH_SAVE_ADDR+8);
		if(numtf==0xFFFFFFFF){numtf=0;}
		sprintf(str3,"n2.val=%d",numtf);
		HMIsends((uint8_t *)str3);
		HMIsendb();	
		distance_percent=STMFLASH_ReadWord((uint32_t)FLASH_SAVE_ADDR+12);
		if(distance_percent==0xFFFFFFFF){distance_percent=0;}
		sprintf(str4,"j0.val=%d",distance_percent);
		HMIsends((uint8_t *)str4);
		HMIsendb();
		HAL_FLASH_Lock();
}
void writeflashtest()
{
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(FLASH_SECTOR_2,FLASH_VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_SAVE_ADDR+16,distance_dis);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_SAVE_ADDR+4,nummini);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_SAVE_ADDR+8,numtf);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_SAVE_ADDR+12,distance_percent);
		HAL_FLASH_Lock();
}
/**

  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
    f(PLL general clock output) = f(VCO clock) / PLLP
    f(USB OTG FS, SDIO, RNG clock output) = f(VCO clock) / PLLQ
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10; // 1<= x <= 2 MHz
  RCC_OscInitStruct.PLL.PLLN = 200;  //  64<= x <=432 MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  //  <= 120MHz
  RCC_OscInitStruct.PLL.PLLQ = 5;// 4   <=48MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void motor1SpeedControl(int32_t series, int32_t steps) {
//	static int32_t temp = 0;
//  static int32_t m = 0;
	
	if(calPulseCount1 == 0) {
		calPulseCount1 = 0;
		setMotor1Speed(0, 0);
		temp = 0;
		m = 0;
		//printf("calPulseCount1 = 0;\r\n");
	} else if(calPulseCount1 < 0) {
		temp = 0;
		m = 0;
		calPulseCount1 = 0;
		setMotor1Speed(0, 0);
		printf("\r\nMotor1 Maybe Lose One Step!\r\n");
	} else {
		temp = temp>calPulseCount1?temp:calPulseCount1;
		--calPulseCount1;
		
		if(temp>2*series*steps) {
			
			if(calPulseCount1%steps == 0) {
				if(calPulseCount1 > temp-series*steps) {  //+
					m+=6000/series;
					setMotor1Speed(0, 2000+m);
				} else if(calPulseCount1 < series*steps/2) {
					m-=6000/series;
					if(m > -2000) {
						setMotor1Speed(0, 2000+m);
					}  
				} else {
					;
				}
			}
		}
	}
}
#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == htim5.Instance) {
		motor1SpeedControl(100, 41);	
    if(direction1 == 1) {
      steps1++;
    } else if(direction1 == -1) {
      steps1--;
    } else {
    }
	}
	
  if (htim->Instance == htim13.Instance) {
    --calPulseCount2;

    if(direction2 == 1) {
      steps2++;
    } else if(direction2 == -1) {
      steps2--;
    } else {
      //printf("!!!Maybe steps2 error!!!\r\n");
    }
    
    if(calPulseCount2 == 0) {
      calPulseCount2 = 0;
//      setMotor2Speed(0, 0);
			//printf("calPulseCount2 = 0;\r\n");
    } else if(calPulseCount2 < 0) {
      calPulseCount2 = 0;
//      setMotor2Speed(0, 0);
      //printf("Motor2 Maybe Lose One Step!\r\n");
    } else {

    }

  }

  if(htim->Instance == htim6.Instance) {  //1ms
    if(timFlag1!=-1) {
      timFlag1++;
    }
    if(timFlag2!=-1) {
      timFlag2++;
    }
		
		if(timFlag3!=-1) {
			timFlag3++;      
		}
  }
	
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //接受上位机数据//gai
{
//  static uint16_t checksum1 = 0;
	static uint16_t count1 = 0;
//  int8_t i = 0;
	static uint8_t recieveData[21];
	if(huart == &huart2)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart2);
		HAL_UART_Receive_IT(&huart2, rx2Buffer, 1);
		recieveData[count1] = rx2Buffer[0];
		if(recieveData[0]!=0x50)
		{
			count1 = 0;
		} 
		else if(count1==1 && recieveData[1]!=0x4F) 
		{
			count1 = 0;
		} 
		else if(count1==20) 
		{
			if(recieveData[4]==0x01)
			{
				motorflag=0x01;
				gs32TestDistance =  (recieveData[6]<<8) | recieveData[5]; //(recieveData[8]<<24) | (recieveData[7]<<16)
					//| (recieveData[6]<<8) | recieveData[5];  // 暂时只取低2字节
				distance_interval=(recieveData[12]<<24)|(recieveData[11]<<16)|(recieveData[10]<<8)|recieveData[9];
				if(gs32TestDistance==1200){nummini+=1;}//1200  //266
				if(gs32TestDistance==2200){numtf+=1;}//2200    //522
				sprintf(str2,"n1.val=%d",nummini);
				sprintf(str3,"n2.val=%d",numtf);
				HMIsends((uint8_t *)str2);
				HMIsendb();
				HMIsends((uint8_t *)str3);
				HMIsendb();
				send_atstart();
			}
			else if (recieveData[4]==0x0c)
			{
				motorflag=0x0c;
				gs32TestDistance =  (recieveData[6]<<8) | recieveData[5]; // (recieveData[8]<<24) | (recieveData[7]<<16)
					//| (recieveData[6]<<8) | recieveData[5];
				send_atstart();
			}
			else if (recieveData[4]==0x04)//这里改
			{
				if(fixflag==0)//
				{
					motorflag=0x04;
					gs32TestDistance = gs32TestDistanceBkp;
				}
				else
				{
					sendflag=1;
				}
			}
			if(recieveData[4]==0x03)//如果接受第四位0x02//定点测试
			{
				fixflag=1;
				motorflag=0x02;
				gs32TestDistance =  (recieveData[6]<<8) | recieveData[5];
				distance_interval=(recieveData[12]<<24)|(recieveData[11]<<16)|(recieveData[10]<<8)|recieveData[9];
				test_time=(recieveData[16]<<24)|(recieveData[15]<<16)|(recieveData[14]<<8)|recieveData[13];
			}
			
			
//			if (gs32TestDistance <= SLOWDOWN_DISTANCE)  // 如果距离太短就不开始
//			{
//				motorflag=0;
//			}
			count1 = 0;
		} 
		else 
		{
			count1++;
		} 
	}
}
//
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  if(huart == &huart2) 
		{
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)rx2Buffer, 1);		
		sendtime++;
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if(GPIO_Pin==GPIO_PIN_0 && rx1Order!=isTest && rx1Order!=isLeft2Right && rx1Order!=isRigth2Left && rx1Order!=isCW && rx1Order!=isCCW) { //left limit
    //calPulseCount1 = 0;
    //Lidar1.position = 0;  //if no disturbance
		//if(Lidar1.position < 10) {
		//if(Lidar1.position < 10 && rx1Order!=isTest) {
			steps1 = 0;
			temp = 0;
			m = 0;
			calPulseCount1 = 0;
			setMotor1Speed(0, 0); 
		//}
    //printf("left limit exti!\r\n");
  }

  if(GPIO_Pin == GPIO_PIN_1 && rx1Order != isTest && rx1Order!=isLeft2Right && rx1Order!=isRigth2Left && rx1Order!=isCW && rx1Order!=isCCW) {  //indexPlate zero limit
    //calPulseCount1 = 0;
    //Lidar1.angle = 0; //if no disturbance
		//if(Lidar1.angle < 10 && (Lidar1.position<10 || Lidar1.position >= positionArray[4]-1)) {
//		if(timFlag3 > 500) {
//			timFlag3 = -1;
//		}
		//if(Lidar1.angle < 9 && timFlag3 == -1) {
//		if(Lidar1.angle < 50 && timFlag3 == -1 && rx1Order!=isTest) {
//			timFlag3 = 0;
//			steps2 = 0;
//			setMotor2Speed(0, 0); //CW
//		}

			//timFlag3 = 0;
			steps2 = 0;
			calPulseCount2 = 0;
//			setMotor2Speed(0, 0); //CW

//		if(timFlag3 == -1) {
//			timFlag3 = 0;
//		}
		//setMotor2Speed(0, 0); 
//		if(Lidar1.angle < 9) {
//			timFlag3 = 0;
//			steps2 = 0;
//			setMotor2Speed(0, 0); //CW
//		}
    //printf("indexPlate zero limit exti!\r\n");
  }
}
void speedplus(void)
{
	if(ispeed<40000)  // 38400 (12800微步细分), 19200(6400微步细分)
	{   
		ispeed=ispeed+1000;
		setMotor1Speed(motordirection,ispeed);
//		delay_ms(200);
		HAL_Delay(80);
	}
}
void speedminus(void)
{
	if(ispeed>4000)
	{
		ispeed=ispeed-800;//800
		setMotor1Speed(motordirection,ispeed);
		//delay_ms(200);
		HAL_Delay(50);
	}
}


void setMotor1Speed(int direction, int frequency) 
{
//	TIM_MasterConfigTypeDef sMasterConfig;
//    TIM_OC_InitTypeDef sConfigOC;

	static int s32Direction=0;
	static int s32Frequency=0, s32TimStartFlag=0;

	if ((s32Direction==direction) && (s32Frequency==frequency))  //如果参数都没有改变就不重新设置了
		return;
	s32Direction = direction;
	s32Frequency = frequency;

	if(direction == DIR_FORWARD) {
		HAL_GPIO_WritePin(DIR0_GPIO_Port, DIR0_Pin, GPIO_PIN_SET); 
		HAL_Delay(1);  // 方向比脉冲至少提前5us 建立
	} else if(direction == DIR_BACKWARD) {
		HAL_GPIO_WritePin(DIR0_GPIO_Port, DIR0_Pin, GPIO_PIN_RESET);  
		HAL_Delay(1);
	} else {  
	
	}

	if(frequency >= 1) {
    htim5.Instance->ARR = 10000000L/frequency-1;  // period = prescaler / freq - 1
    htim5.Instance->CCR1 = 10000000L/frequency/2;  // 50% 占空比    pulse=Period/2
		if (s32TimStartFlag == 0) {
			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim3);
			HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//			HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);
			s32TimStartFlag = 1;
		}
	} else {
		HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
		s32TimStartFlag = 0;
	}
}


void motorInit(void) {
	
	//setMotor1Speed(-1, 2800);  //stop at EXTI Callback
  //first motor1 right 2s, motor2 CW 2s
  //second stop 0.5s
  //third motor1 left to zero, motor2 left to zero 
  //to ensure stop at zero's right 
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  setMotor1Speed(1, 2800);  //from left to right
 // setMotor2Speed(1, 2000);  //CW
  HAL_Delay(2000); 
  setMotor1Speed(0, 0);
 // setMotor2Speed(0, 0);
  HAL_Delay(300);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_Delay(500);
  //if(HAL_GPIO_ReadPin(LS0_GPIO_Port, LS0_Pin)) {
    setMotor1Speed(-1, 2500);  //stop at EXTI Callback
  //}
  //if(HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin)) {
//    setMotor2Speed(-1, 2000);  //stop at EXTI Callback
  //} 
//	while(HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin) || HAL_GPIO_ReadPin(LS0_GPIO_Port, LS0_Pin)) {
//		if(HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin) == GPIO_PIN_RESET) {
//			setMotor2Speed(0, 0);
//		}
//		if(HAL_GPIO_ReadPin(LS0_GPIO_Port, LS0_Pin) == GPIO_PIN_RESET) {
//			setMotor1Speed(0, 0);
//		}
//	}
  while(HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin)){;}  //wait indexPlate to zero
  while(HAL_GPIO_ReadPin(LS0_GPIO_Port, LS0_Pin)){;}  //wait movingTable to zero
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
