/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
//#include "stm32f4xx_hal.h"
//#include "arm_neon.h"  // no neon for cortex-m4
#include "stddef.h"
//#include <stm32f4xx_hal_dma_ex.h>  // dma extended functions
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
// those are for ETHERNET
#include "loopback.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500.h"
// those are for LCD + Touch
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
//#include "snow_tiger.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MODE

#define DATA_SOCK	0
//#define CONTROL_SOCK	1  // not used
#define CAM_BUF_SIZE 13176
#define CAM_FLOAT_SIZE 3200  // 3200 floats, header 94 floats
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef DEBUG_MODE:
	#define NETWORK_MSG  		 "Network configuration:\r\n"
	#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
	#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
	#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
	#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"

	#define PRINT_NETINFO(netInfo) do { 																					\
	  HAL_UART_Transmit(&huart2, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
	  sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	  sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	  sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	  sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	} while(0)
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



uint8_t usart__request_command = 0;
uint8_t usart__respond_command = 0;

char cam_request_cartesian = 'Z';
char cam_request_radial = 'D';
char cam_request_close = 'q';


volatile uint8_t binarydata_flag = 0;

uint8_t LCDmode_flag = 0;
uint8_t binarydata_buffer0[CAM_BUF_SIZE];

volatile float floatframe_buffer[CAM_FLOAT_SIZE];




static uint8_t msg[20]={0,};  // ethernet config printing


uint8_t cam_destip[4] = {192, 168, 178, 69};	//




uint16_t cam_destport = 50002;	//
uint16_t local_port = 49152;;	//



// global flags

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void usb_transmit_byte(uint8_t transmit_int);
void usb_transmit_int(int32_t transmit_int);
void usb_transmit_uint(uint32_t transmit_uint);
void usb_transmit_float(float transmit_float);
void usb_transmit_char(char transmit_char);  // test this
void usb_transmit_string(char *transmit_string);  // test this
void set_LED(bool status);

void cs_sel();

void cs_desel();

uint8_t spi_rb(void);

void spi_wb(uint8_t buf);

void spi_rb_burst(uint8_t *buf, uint16_t len);

void spi_wb_burst(uint8_t *buf, uint16_t len);


void checkpushbutton(void);

/**
  * @brief  Read Data from camera vie W5500 on SPI.
  * @retval float
  */
int32_t get_camdata(void);
float canny_edge_mod(const float *floatbuf);
float canny_bad_solution(void);
// experimental stuff here:

uint32_t depth_yield(float input);
uint32_t edge_yield(float input);

/**
  * @brief  Low level check if float is negative value.
  * @retval 1 if x is negative, otherwise 0
  */
bool is_negative(float x) {  // fast check for negative float // only big endian?
//	usb_transmit_byte(signbit(x));
    return signbit(x);
}



// reordering from big endian to little endian
/**
  * @brief  Swapping uint32_t from big endian to little endian.
  * @retval uint32_t
  */
inline uint32_t SwapByteOrder_32(uint32_t a)  // ok
{
  return __builtin_bswap32(a);
}

/**
  * @brief  Swapping 4 bytes of uint8_t array from big endian to little endian.
  * @retval uint32_t
  */
//inline uint32_t byteswap2little(uint8_t *value)  // test it
inline uint32_t byteswap2little(uint8_t *value)  // test it
{
	uint32_t tmpint;
	memcpy(&tmpint, value, 4);
	return __builtin_bswap32(tmpint);
}

/**
  * @brief  Showing uint32_t bit representation of float value
  * @retval uint32_t
  */
inline uint32_t checkfloatbytes(float C) {
  union {
    uint32_t i;
    float f;
  } in;
  in.f = C;
//  out.i = SwapByteOrder_32(in.i);
  return in.i;
}

/**
  * @brief  Type punning access of 4 bytes as a 32 bit little endian float.
  * @retval float
  */
float pun2float(const uint8_t *buf) {
    // from https://blog.regehr.org/archives/959
  float num;
  memcpy(&num, buf, 4);
//    return __builtin_bswap32(num);
  return num;
}
/**
  * @brief  Swapping 4 bytes of uint8_t array and type punning access of 4 bytes as a 32 bit little endian float.
  * @retval float
  */
float swap2float(uint8_t *buf) {
// from https://blog.regehr.org/archives/959
	uint32_t tmpswap = byteswap2little(buf);
	float num;
	memcpy(&num, &tmpswap, 4);
	return num;
}


/**
  * @brief  Type punning access of 4 bytes as a 32 bit little endian uint32_t.
  * @retval uint32_t
  */
uint32_t pun2int(const uint8_t *buf) {
    // from https://blog.regehr.org/archives/959

  uint32_t num;
  memcpy(&num, buf, 4);
//  return __builtin_bswap32(num);  // swap endianness
  return num;
}
/**
  * @brief  Type punning access of 4 bytes as a 32 bit swapped bytes float.
  * @retval uint32_t
  */






void drawsquare(float inputvar, uint32_t position)
{
	if(LCDmode_flag == 0)
	{
		uint16_t zoom = 4;
		uint16_t row ,column;
		column = (position / 64)*zoom;  // y
		row = (position % 64)*zoom;  // x

		ILI9341_Draw_Rectangle(column,row , zoom, zoom, depth_yield(inputvar));
	}
}

void drawedge(float inputvar, uint32_t position)
{
	if(LCDmode_flag == 1)
	{
		uint16_t zoom = 4;
		uint16_t row ,column;
		column = (position / 63)*zoom;  // y
		row = (position % 63)*zoom;  // x
		ILI9341_Draw_Rectangle(column,row , zoom, zoom, edge_yield(inputvar));
	}
}


float accessfloatarray(uint8_t *buf, uint8_t floatposition)
{
	return swap2float((buf+4*floatposition));
}

void testfloatarray(void)
{
	float outfloat;
	//	int j = (3199-95);
		int j = 0;
		for(int i = j; i <j+10; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}

		for(int i = 50; i <60; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}

		for(int i = 3120; i <3136; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}

		for(int i = 3189; i <3200; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}
}


void arrayreshaping(const uint8_t *arrptr)
{
	for(size_t i = 0; i < 3200; i++)
	{
		floatframe_buffer[i] = swap2float(arrptr+376+4*i);  // ok
		drawsquare(floatframe_buffer[i],i);
	}
}


// Ethernet init procedure

void IO_LIBRARY_Init(void) {
	uint8_t RXbufSize[] =	{ 16, 0, 0, 0, 0, 0, 0, 0, }; // socket buffer size is 0...16
	uint8_t TXbufSize[] =	{ 16, 0, 0, 0, 0, 0, 0, 0, }; // socket buffer size

	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
//	reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);


	wizchip_init(TXbufSize, RXbufSize);
	wiz_NetInfo netInfo = { .mac =	{ 0x00, 0x08, 0xdc, 0xab, 0xaf, 0xfe },	// Mac address
							.ip =	{ 192, 168, 178, 1},		// IP address
							.sn =	{ 255, 255, 255, 0 },	// Subnet mask
							.gw =	{ 192,  168, 178, 69 } };	// Gateway address

	wizchip_setnetinfo(&netInfo);

	wizchip_getnetinfo(&netInfo);
	PRINT_NETINFO(netInfo);

	wiz_NetTimeout gWIZNETTIME = {.retry_cnt = 3,         //RCR = 3
	                               .time_100us = 2000};     //RTR = 2000
	wizchip_setinterruptmask(IK_SOCK_0);  // Enable interrupt on socket0
	ctlnetwork(CN_SET_TIMEOUT,(void*)&gWIZNETTIME);
	ctlnetwork(CN_GET_TIMEOUT, (void*)&gWIZNETTIME);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // start hardware stuff




  IO_LIBRARY_Init();
  HAL_UART_Receive_IT(&huart1, &usart__request_command, 1);
  ILI9341_Init();


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



	  uint32_t starttime;
	  uint32_t stoptime;
	  uint32_t difftime;

  set_LED(true);  // enable led
  while (1)
  {

	  starttime = HAL_GetTick();  // milliseconds precision
	  checkpushbutton();
// 	  Exec time measurement template:
//	  uint32_t starttime;
//	  uint32_t stoptime;
//	  uint32_t difftime;


	  // do something here:
//	  stoptime = HAL_GetTick();
//	  difftime = stoptime - starttime;
//	  usb_transmit_string("Time: ");
//	  usb_transmit_int(difftime);
//	  usb_transmit_string("\n\r");
//	volatile uint8_t binarydata_buffer0[CAM_BUF_SIZE];
//	volatile uint8_t binarydata_buffer1[CAM_BUF_SIZE];
//	volatile float floatframe_buffer[CAM_FLOAT_SIZE];
//      HAL_Delay(300);
//
//
//	starttime = HAL_GetTick();  // milliseconds precision


	  get_camdata();
	  arrayreshaping(binarydata_buffer0);
	  canny_bad_solution();




  // Transmit refresh rate
	char stringbuf[10];
	stoptime = HAL_GetTick();
	difftime = stoptime - starttime;
	sprintf(stringbuf,  "%i", difftime);
	usb_transmit_int(difftime);
	usb_transmit_string("ms\r\n");
	ILI9341_Draw_Text( stringbuf, 100,255, BLACK, 2, WHITE);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void usb_transmit_byte(uint8_t transmit_byte)  // ok but sprintf is dangerous
{
	HAL_UART_Transmit(&huart2, (uint8_t*) transmit_byte, 1, 100);  // alternativ strlen
}
void usb_transmit_int(int32_t transmit_int)  // ok but sprintf is dangerous
{
	uint8_t stringbuf[10];
	sprintf(stringbuf,  "%i", transmit_int);
	HAL_UART_Transmit(&huart2, (uint8_t*) stringbuf, strlen(stringbuf) , 100);
}
void usb_transmit_uint(uint32_t transmit_uint)  // ok but sprintf is dangerous
{
	uint8_t stringbuf[10];
	sprintf(stringbuf,  "%i", transmit_uint);

	HAL_UART_Transmit(&huart2, (uint8_t*) stringbuf, strlen(stringbuf) , 100);
}
void usb_transmit_char(char transmit_char)  // ok
{
	HAL_UART_Transmit(&huart2, &transmit_char, 1, 100);
}
void usb_transmit_string( char *transmit_string)  //  ok
{
	HAL_UART_Transmit(&huart2, (uint8_t*) transmit_string, strlen(transmit_string) , 100); // small bug.
}
void usb_transmit_float(float transmit_float)
{
	char fullstring[20];
	char *tmpSign = (transmit_float < 0) ? "-" : " ";
	float tmpVal = (transmit_float < 0) ? -transmit_float : transmit_float;
	int tmpInt1 = tmpVal;                  // Get the integer (9876543210).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.012).
	int tmpInt2 = (tmpFrac * 100);  // Turn into integer (12).
	sprintf (fullstring, "%s%d.%2d", tmpSign, tmpInt1, tmpInt2);
	HAL_UART_Transmit(&huart2, (uint8_t*) fullstring, strlen(fullstring), 100);
}

void cs_sel()
{
	HAL_GPIO_WritePin(ETHERNET_CS_GPIO_Port, ETHERNET_CS_Pin, GPIO_PIN_RESET); //CS LOW
}

void cs_desel()
{
	HAL_GPIO_WritePin(ETHERNET_CS_GPIO_Port, ETHERNET_CS_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void)
{
	uint8_t buf;
	HAL_SPI_Receive(&hspi3, &buf, 1, 0xFFFFFFFF);
	return buf;
}

void spi_wb(uint8_t buf)
{
	HAL_SPI_Transmit(&hspi3, &buf, 1, 0xFFFFFFFF);
}

void spi_rb_burst(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Receive(&hspi3, &buf, len, 0xFFFFFFFF);
}

void spi_wb_burst(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Transmit(&hspi3, &buf, len, 0xFFFFFFFF);
}

void set_LED(bool status)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, status);
}

void checkpushbutton(void)
{
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_RESET)
	{
		LCDmode_flag ^= 0x01;
	}
}

uint32_t depth_yield(float input)
{
	uint32_t colorarray[22] = {
			0xF800,	 // red
			0xFB20,
			0xFDE0,
			0xFDE0,  // yellow
			0xFEE0,
			0xFF80,
			0xDFE0,
			0xB7E0,  // green
			0x87E0,
			0x3FE0,
			0x07C7,
			0x07F3,
			0x07FB,  // green-blue
			0x079F,
			0x063F,
			0x041F,  // blue
			0x027F,
			0x017F,
			0x181F,
			0x401F,
			0x701F,  // magenta
			0x981F};

	// depth thredshold. Non-adaptive implementation
	const float t1 = 0.05;
	const float t2 = 0.1;
	const float t3 = 0.15;
	const float t4 = 0.2;
	const float t5 = 0.25;
	const float t6 = 0.3;
	const float t7 = 0.35;
	const float t8 = 0.4;
	const float t9 = 0.45;
	const float t10 = 0.5;
	const float t11 = 0.55;
	const float t12 = 0.6;
	const float t13 = 0.65;
	const float t14 = 0.7;
	const float t15 = 0.75;
	const float t16 = 0.8;
	const float t17 = 0.85;
	const float t18 = 0.9;
	const float t19 = 0.95;
	const float t20 = 1.0;

	if(is_negative(input)==1) {
		return BLACK;
	} else if (input<t1) {
		return colorarray[0];
	} else if (input<t2) {
		return colorarray[1];
	} else if (input<t3) {
		return colorarray[2];
	} else if (input<t4) {
		return colorarray[3];
	} else if (input<t5) {
		return colorarray[4];
	} else if (input<t6) {
		return colorarray[5];
	} else if (input<t7) {
		return colorarray[6];
	} else if (input<t8) {
		return colorarray[7];
	} else if (input<t9) {
		return colorarray[8];
	} else if (input<t10) {
		return colorarray[9];
	} else if (input<t11) {
		return colorarray[10];
	} else if (input<t12) {
		return colorarray[11];
	} else if (input<t13) {
		return colorarray[12];
	} else if (input<t14) {
		return colorarray[13];
	} else if (input<t15) {
		return colorarray[14];
	} else if (input<t16) {
		return colorarray[15];
	} else if (input<t17) {
		return colorarray[16];
	} else if (input<t18) {
		return colorarray[17];
	} else if (input<t19) {
		return colorarray[18];
	} else if (input<t20) {
		return colorarray[19];
	}else if (input>t20) {
		return colorarray[20];;
	}
}

uint32_t edge_yield(float input)
{
	// edge thredshold hysteresis
	const float t0 = -0.01;
	const float t1 = 0.15;

	if(input<t0) {
		return RED;
	} else if (input<t1 && input>t0) {
		return BLACK;
	} else {
		return WHITE;
	}
}

int32_t get_camdata(void)
{

	int32_t ret_send = 0;
	int32_t ret_sock = 0;
	int32_t ret_connect = 0;

	int32_t ret_rcv = 0;
	uint16_t size = 0;
	uint16_t sn = 0; // using only socket 0
	char testch = 'D';
	//   uint8_t testch = 0x44;
//   uint8_t testch = 9;

	while(!getPHYCFGR());  // phy link check

   switch(getSn_SR(DATA_SOCK))
   {
    	case SOCK_ESTABLISHED :
//    		usb_transmit_string("SocketEstablished.\r\n");


     // send data here:
    		ret_send = send(sn, &testch, 1);
    		// wait here between response
    	    HAL_Delay(5);  // delay for network reply needed




//    		do{
//    			usb_transmit_string("\n\r Waiting for data - ");
//    			size = getSn_RX_RSR(DATA_SOCK);
//    			usb_transmit_int(size);
//    			usb_transmit_string("\n\r");
//    		} while (size == 0);
//    		usb_transmit_string("\n\r received - ");
//    		usb_transmit_int(size);
//    		usb_transmit_string("\n\r");
    	     // receive data here:

// bug. wait for something in register.  // received size is not granted

    		if((size = getSn_RX_RSR(DATA_SOCK)) > 0){ // If data in rx buffer. maybe bug. wait for data in loop?
    			ret_rcv = recv(DATA_SOCK,binarydata_buffer0, CAM_BUF_SIZE);

				if(ret_rcv == CAM_BUF_SIZE)
				{

//					usb_transmit_string("RX OK.\n\r");  // set busy state, wait for RX buffer



				} else {
					usb_transmit_string("RX Err:\n\r");
					usb_transmit_int(ret_rcv);  //
					usb_transmit_string("\n\r");
					// hier timeout einbauen, da infinite loop in SOCK_ESTABLISHED
				}
    		} else {
    			usb_transmit_string("/r/n Transmission error \r\n");
				usb_transmit_int(size);
    		}
    		break;
   		case SOCK_CLOSE_WAIT :
			if((ret_connect=disconnect(DATA_SOCK)) != SOCK_OK) return ret_connect;
			#ifdef DEBUG_MODE:
			usb_transmit_string("CloseOK\r\n");
			#endif
   			break;
   		case SOCK_CLOSED :
			#ifdef DEBUG_MODE:
   			usb_transmit_string("SOCK_CLOSED\r\n");
			#endif
			if((ret_sock=socket(DATA_SOCK, Sn_MR_TCP, local_port, 0x0)) != DATA_SOCK)
			{
				usb_transmit_string("SOCK_CLOSED_ERROR\r\n");
				usb_transmit_int(ret_connect);
				close(DATA_SOCK);
				return ret_sock;
			}
			local_port++;  // avoid reusing local port
			if(local_port > 65000){
				local_port = 49152;
			}

   			break;

   		case SOCK_INIT :
			#ifdef DEBUG_MODE:
   			usb_transmit_string("SOCK_INIT\r\n");
			#endif

			while((ret_connect = connect(DATA_SOCK, cam_destip, cam_destport)) != SOCK_OK)  // -4 ,-13 timeout zu kurz?
			{
				#ifdef DEBUG_MODE
				HAL_Delay(10);
				usb_transmit_string("Connect error return:");
				usb_transmit_int(ret_connect);
				usb_transmit_string("r\n");
				usb_transmit_string("Reconnecting.\r\n");
				#endif
				return ret_connect;
			}
   			break;
   		default :
   			break;
   }
   return 0;  // finished
}


// original returning edge array. mod version shall return edge


void convolution(const float *in, float *out, const float *kernel,
                 const int nx, const int ny, const int kn,
                 const bool normalize)
{
//	uint8_t MAX_BRIGHTNESS = 255;  // not used
    const int khalf = kn / 2;  // half of kernel
    float min = FLT_MAX, max = -FLT_MAX;  // min and max float values

    for (int m = khalf; m < nx - khalf; m++)
    {
        for (int n = khalf; n < ny - khalf; n++)
        {
            float pixel = 0.0;
            size_t c = 0;
            for (int j = -khalf; j <= khalf; j++)
            {
                for (int i = -khalf; i <= khalf; i++)
                {
                    pixel += in[(n - j) * nx + m - i] * kernel[c];
                    c++;
                }
               out[n * nx + m] = (float)pixel;
            }
        }
	}
}

/*
 * gaussianFilter:
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 * determine size of kernel (odd #)
 * 0.0 <= sigma < 0.5 : 3
 * 0.5 <= sigma < 1.0 : 5
 * 1.0 <= sigma < 1.5 : 7
 * 1.5 <= sigma < 2.0 : 9
 * 2.0 <= sigma < 2.5 : 11
 * 2.5 <= sigma < 3.0 : 13 ...
 * kernelSize = 2 * int(2*sigma) + 3;
 */
void gaussian_filter(const float *in, float *out,
                     const int nx, const int ny, const float sigma)
{
    const int n = 2 * (int)(2 * sigma) + 3;
    const float mean = floorf(n / 2.0f);
    float kernel[n * n]; // variable length array
    size_t c = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            kernel[c] = exp(-0.5 * (pow((i - mean) / sigma, 2.0) +
                                    pow((j - mean) / sigma, 2.0)))
                        / (2 * M_PI * sigma * sigma);
            c++;
        }

    convolution(in, out, kernel, nx, ny, n, true);
}





float canny_bad_solution(void)
{
	// detection hysteresis min, max
	const float tmin = 0.5;
	const float tmax = 1.0;

	// matrix shape
    const int nx = 64;
    const int ny = 50;

    // memory allocation in heap
//	float *in = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *G = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *after_Gx = calloc(nx * ny * sizeof(float), sizeof(float));
//   float *after_Gy = calloc(3200, sizeof(float));
//    float *nms = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *out = malloc(nx * ny * sizeof(float));

	const float *in = &floatframe_buffer;

// memory allocation in stack
//    float G[nx * ny];
//    float after_Gx[nx * ny];
    float after_Gy[nx * ny];
//    float after_Gy[3200];
//    float nms[nx * ny];
//    float out[nx * ny];




//    size_t isize = sizeof(out);
//    usb_transmit_uint(isize);
//    HAL_Delay(10000);
//    	in = &floatbuf;

    // sobel operator for convolution
//    gaussian_filter(in, out, nx, ny, sigma);


// conventional intensity gradient calculation
	const int32_t khalf = 1;
	const int32_t kn = 3;

	// bug? memory leak? wrong values for array? no dynamic memory alloc on stm32?
	const float Gx[] = {-1.0, 0.0, 1.0,
						-1.0, 0.0, 1.0,
						-1.0, 0.0, 1.0};

    const float Gy[] = { 1.0, 1.0, 1.0,
                         0.0, 0.0, 0.0,
                        -1.0,-1.0,-1.0};

//    float testfloat = 0.0;
//    for(int i = 0; i <3200; i++)  // ok
//	{
//		after_Gy[i] = testfloat;
//	}
	int outint;


	size_t row;
	size_t column;
	size_t pixel_position = row * nx + column;


	// gradient calculation in y direction
        for (int m = khalf; m < nx - khalf; m++)  // m from 1 to 63
        {
            for (int n = khalf; n < ny - khalf; n++)  // n from 1 to 49
            {
                float pixel = 0.0;
                float tmpflaot = 0.0;
                size_t c = 0;
                for (int j = -khalf; j <= khalf; j++)  // j from -1 to 1
                {
                    for (int i = -khalf; i <= khalf; i++)  // i from -1 to 1
                    {
                    	// multiply input_array with kernel matrix

                        pixel += in[(n - j) * nx + m - i] * Gy[c];
                        c++;
                    }
                    //
                    // result of convolution is a gradient matrix
                    after_Gy[n * nx + m] = pixel;
                    // draw pixel on LCD
                    drawedge(pixel, n * nx + m);
                }
            }
    	}
//			// gradient calculation in x direction (not in use)
//        for (int m = khalf; m < nx - khalf; m++)
//        {
//        	{
//                for (int n = khalf; n < ny - khalf; n++)
//                {
//                    float pixel = 0.0;
//                    size_t c = 0;
//                    for (int j = -khalf; j <= khalf; j++)
//                    {
//                        for (int i = -khalf; i <= khalf; i++)
//                        {
//                            pixel += in[(n - j) * nx + m - i] * Gx[c];
//                            c++;
//                        }
//                        after_Gx[n * nx + m] = pixel;
//                    }
//                }
//        	}
//        }
//		for (int i = 1; i < nx - 1; i++)
//		{
//			for (int j = 1; j < ny - 1; j++) {
//			 size_t c = i + nx * j; // i = column spalte, nx*j = row zeile
////			 G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
//			 G[c] = hypot(after_Gx[c], after_Gy[c]);
//			}
//		}



//	usb_transmit_string("\n\roverwritten\n\r");

//    	float outfloat = 0.0;
////    	int outint;
////    	//	int j = (3199-95);
//    		int j = 300;
//    		for(int i = j; i <j+90; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = after_Gy[i];
//    			usb_transmit_float(outfloat);  //
////    			usb_transmit_int(outint);  //
//
//    			usb_transmit_string("\n\r");
//    		}
//    		HAL_Delay(300);
//    		testfloatarray();
//    		for(int i = 50; i <60; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = floatframe_buffer[i];
//    			usb_transmit_float(outfloat);  //
//    			usb_transmit_string("\n\r");
//    		}
//
//    		for(int i = 3120; i <3136; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = floatframe_buffer[i];
//    			usb_transmit_float(outfloat);  //
//    			usb_transmit_string("\n\r");
//    		}
//
//    		for(int i = 3189; i <3200; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = floatframe_buffer[i];
//    			usb_transmit_float(outfloat);  //
//    			usb_transmit_string("\n\r");
//    		}
//        		set_LED(false);

//		  free reserved memory
//        free(after_Gx);
//        free(after_Gy);
//        free(G);
//        free(nms);

//        return out;
    		return 0;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	#ifdef DEBUG
	  asm("BKPT #0");
	#else
	  while(1)
	  {
		HAL_Delay(250);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }
	#endif

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
