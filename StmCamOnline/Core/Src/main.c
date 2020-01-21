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
//#include "ILI9341_GFX.h"
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


//typedef float pixel_t;
volatile uint8_t binarydata_flag = 0;

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




/**
  * @brief  Read Data from camera vie W5500 on SPI.
  * @retval float
  */
int32_t get_camdata(void);
float canny_edge_mod(const float *floatbuf);
float canny_bad_solution(void);
// experimental stuff here:

uint32_t depth_yield(float input);


/**
  * @brief  Low level check if float is negative value.
  * @retval 1 if x is negative, otherwise 0
  */
bool is_negative(float x) {  // fast check for negative float // only big endian?
//	usb_transmit_byte(signbit(x));
    return signbit(x);
}

void checkruntime(void (*fptr)())
{
	uint32_t starttime;
	uint32_t stoptime;
	uint32_t difftime;
	usb_transmit_string("Time: ");
	starttime = HAL_GetTick();  // milliseconds precision
	fptr();   // function pointer
	stoptime = HAL_GetTick();
	difftime = stoptime - starttime;
	usb_transmit_int(difftime);
	usb_transmit_string("\n\r");
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


void lcdtest(void)
{
//	ILI9341_Fill_Screen(WHITE);
//	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	//  text, x, y, color, size, background
//	ILI9341_Draw_Text("Slow draw by selecting", 120, 10, BLACK, 2, GREEN);
//	ILI9341_Draw_Text("and adressing pixels", 100, 100, BLACK, 2, WHITE);
//	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);
//	HAL_Delay(10);
	uint32_t start_x, start_y, stop_x, stop_y, scale;  // y = high, x = width
	scale = 1;
//	start_x = 48/scale+1;
//	start_y = 60/scale+1;
//	stop_x = 432/scale;
//	stop_y = 310/scale;
	start_x = 0;
	start_y = 0;
	stop_x = 320;
	stop_y = 200;

	while (start_y < stop_y)
	{
//		while ((y < 432) && (x < 310))
		while ((start_y < stop_y) && (start_x < stop_x))
		{
			ILI9341_Draw_Pixel(start_y, start_x, BLACK);
			start_x++;
		}
		start_y++;
		start_x = 0;
	}


//	HAL_Delay(2000);
//	usb_transmit_string("\r\nRepeating\r\n");
}

void drawsquare(float inputvar, uint32_t position)
{
	uint16_t zoom = 4;
	uint16_t row ,column;
	column = (position / 64)*zoom;  // y
	row = (position % 64)*zoom;  // x

	ILI9341_Draw_Rectangle(column,row , zoom, zoom, depth_yield(inputvar));
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



// 	  Exec time measurement template:
//	  uint32_t starttime;
//	  uint32_t stoptime;
//	  uint32_t difftime;

//	  starttime = HAL_GetTick();  // milliseconds precision
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
//
//
//
//
	  get_camdata();
	  arrayreshaping(binarydata_buffer0);


//	  switch(binarydata_flag)
//	{
//		case 0:
//			arrayreshaping(binarydata_buffer1);
//			usb_transmit_string("\r\n reshape0 ok\r\n");
//			break;
//		 case 1:
//			 arrayreshaping(binarydata_buffer0);
//			 usb_transmit_string("\r\n reshape0 ok\r\n");
//			break;
//		default:
//			usb_transmit_string("\r\n Canny detector error\r\n");
//			break;
//	}
//	  canny_bad_solution();
//	  lcdtest();


//
//	  stoptime = HAL_GetTick();
//	  difftime = stoptime - starttime;
//	  usb_transmit_string("\n\rTime: ");
//	  usb_transmit_int(difftime);
//	  usb_transmit_string("\n\r");


//	  testmyarray(binarydata_flag);

		//test here:

		//test finish:


// print here:
//	  usb_transmit_string("\n\routput:\n\r");
//	  someint = checkfloatbytes(somefloat1);
//	  checkruntime(&get_camdata);
//	  usb_transmit_uint(someint);
//	  for (int i=0; i<8; i++)  // strlen() at runtime, sizeof() at compile
//	  {
//
//		  someint = pun2int(&inputarr32[(i*4)]);
//
//		  usb_transmit_string("\n\r");
//		  usb_transmit_string("i=");
//		  usb_transmit_uint(i);
//		  usb_transmit_string(" - ");
//
//
//
//
//		  usb_transmit_uint(someint);
//		  usb_transmit_string("\n\r");
//		  usb_transmit_string(" - ");
//		  outputint = byteswap2little(inputint);
//		  usb_transmit_uint(outputint);
//		  usb_transmit_string("\n\r");
//	  }

//	  usb_transmit_int((HAL_DMA_GetState(&hspi1)));  // is 0
//	  usb_transmit_string("endloop");


//	  SPI_DMATransmitCplt  // dont use
//	  usb_transmit_string("\n\loop end.\n\r");

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
    	    HAL_Delay(10);  // delay for network reply needed




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

    			// this should be a double buffer for DMA. it is not used since dma is disabled
//    			switch(binarydata_flag)
//				{
//					case 0:
//
//						ret_rcv = recv(DATA_SOCK,binarydata_buffer0, CAM_BUF_SIZE);
//
//						usb_transmit_string("binarydata_buffer0:");
//						usb_transmit_int(ret_rcv);
//						usb_transmit_string("\n\r");
//						binarydata_flag = 1;
//						break;
//					 case 1:
//
//						ret_rcv = recv(DATA_SOCK,binarydata_buffer1, CAM_BUF_SIZE);
//
//						usb_transmit_string("binarydata_buffer1:");
//						usb_transmit_int(ret_rcv);
//						usb_transmit_string("\n\r");
//						binarydata_flag = 0;
//						break;
//					default:
//						usb_transmit_string("/r/n Transmission error\r\n");
//						break;
//				}

				if(ret_rcv == CAM_BUF_SIZE)
				{

					usb_transmit_string("RX OK.\n\r");  // set busy state, wait for RX buffer
//					if(ret==SOCK_BUSY){
//						usb_transmit_string("SOCK_BUSY\n\r");
//						return 0;  // still waiting
//					}

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
				usb_transmit_string("bug?\r\n");
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
//			if((ret_connect = connect(DATA_SOCK, cam_destip, cam_destport)) != SOCK_OK)  // -4 ,-13 timeout zu kurz?
//			{
//				usb_transmit_string("Connect error return:\r\n");
//				usb_transmit_int(ret_connect);
//				usb_transmit_string("\r\n");
//				return ret_connect;
//			}
   			break;
   		default :
   			break;
   }
   return 0;  // finished
}





//raw stuff here:
//volatile uint8_t binarydata_buffer0[CAM_BUF_SIZE];
//volatile uint8_t binarydata_buffer1[CAM_BUF_SIZE];
//volatile float floatframe_buffer[CAM_FLOAT_SIZE];
//volatile float diff1_buffer[CAM_FLOAT_SIZE];  // 3200
//volatile float diff2_buffer[CAM_FLOAT_SIZE];  // 3200

// original returning edge array. mod version shall return edge yes/no?
float canny_edge_mod(const float *floatbuf)
{

	// detection hysteresis min, max
	const float tmin = 0.5;
	const float tmax = 1.0;
	//gauss filter sigma parameter
//	const float sigma = 1;  // not used. only needed for gauss filter

	// matrix shape

    const int nx = 64;
    const int ny = 50;

//    int MAX_BRIGHTNESS = 255;  // was
//    uint MAX_BRIGHTNESS = 255;  // max float brightness

    // some useful error checking


    // memory allocation
	float *in = calloc(nx * ny * sizeof(float), sizeof(float));
    float *G = calloc(nx * ny * sizeof(float), sizeof(float));
    float *after_Gx = calloc(nx * ny * sizeof(float), sizeof(float));
    float *after_Gy = calloc(nx * ny * sizeof(float), sizeof(float));
    float *nms = calloc(nx * ny * sizeof(float), sizeof(float));
    float *out = malloc(nx * ny * sizeof(float));


    	in = floatbuf;

//	some useless error checking
//    if (G == NULL || after_Gx == NULL || after_Gy == NULL ||
//        nms == NULL || out == NULL) {
//        fprintf(stderr, "canny_edge_detection:"
//                " Failed memory allocation(s).\n");
//        exit(1);  // check if error occured
//    }



//    gaussian_filter(in, out, nx, ny, sigma);
	const float Gx[] = {-1, 0, 1,
						-2, 0, 2,
						-1, 0, 1};

    // sobel operator for convolution
//    const float Gy[] = { 1, 2, 1,
//                         0, 0, 0,
//                        -1,-2,-1};
    const float Gy[] = { 1, 1, 1,
                         0, 0, 0,
                        -1,-1,-1};

//    convolution(in, after_Gy, Gy, nx, ny, 3, false);
// this is convolution() copypasted:
    	const int kn = 3;  // kernel size
        const int khalf = kn / 2;  // half of kernel. "center" of Gy matrix



//        float min = FLT_MAX, max = -FLT_MAX;  // min and max float values for normalization

//        if (normalize)  // if float value is overfloating
//            for (int m = khalf; m < nx - khalf; m++)
//                for (int n = khalf; n < ny - khalf; n++) {
//                    float pixel = 0.0;
//                    size_t c = 0;
//                    for (int j = -khalf; j <= khalf; j++)
//                        for (int i = -khalf; i <= khalf; i++) {
//                            pixel += in[(n - j) * nx + m - i] * kernel[c];
//                            c++;
//                        }
//                    if (pixel < min)
//                        min = pixel;
//                    if (pixel > max)
//                        max = pixel;
//                    }
        // iterating over input_array from "center"
        // convolution of input matrix and sobel operator
        // !bug - probably wrong array shape

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
                    	// multiply input_array with kernel matrix
                        pixel += in[(n - j) * nx + m - i] * Gy[c];
                        c++;
                    }

                    //
                    // result of convolution is a sobel operator matrix
                    after_Gy[n * nx + m] = pixel;
                }
            }
    	}


//    calculate intensity gradient
        // not used since gradient calculation is only made in y dir
//    for (int i = 1; i < nx - 1; i++)
//        for (int j = 1; j < ny - 1; j++) {
//            const int c = i + nx * j;
//            // G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
//            G[c] = (pixel_t)hypot(after_Gx[c], after_Gy[c]);
//        }

    // magnitude calculation disabled
    // Non-maximum suppression, straightforward implementation.

    // calculate surrounding
    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            const int nn = c - nx;
            const int ss = c + nx;
            const int ww = c + 1;
            const int ee = c - 1;
            const int nw = nn + 1;
            const int ne = nn - 1;
            const int sw = ss + 1;
            const int se = ss - 1;

//            const float dir = (float)(fmod(atan2(after_Gy[c], after_Gx[c]) + M_PI, M_PI) / M_PI) * 8;
            // calculate intensity vector direction. we use only Gy direction, so this one is not needed
            // if values on same axis differ too much, then nms[c] = G[c], else nms[c] = 0,
            //
            G = after_Gy;
            const float dir = (fmodf(after_Gy[c] + M_PI, M_PI) / M_PI) * 8.0f ;
            if (((dir <= 1 || dir > 7) && G[c] > G[ee] &&
                 G[c] > G[ww]) || // 0 deg
                ((dir > 1 && dir <= 3) && G[c] > G[nw] &&
                 G[c] > G[se]) || // 45 deg
                ((dir > 3 && dir <= 5) && G[c] > G[nn] &&
                 G[c] > G[ss]) || // 90 deg
                ((dir > 5 && dir <= 7) && G[c] > G[ne] &&
                 G[c] > G[sw]))   // 135 deg
                nms[c] = G[c];
            else
                nms[c] = 0;
        }

    // Reuse array
        // used as a stack. nx*ny/2 elements should be enough.
//        int *edges = (int*) after_Gy;
//
//        // filling *out and *edges arrays with zeros for reuse
//        memset(out, 0, sizeof(float) * nx * ny);
//        memset(edges, 0, sizeof(float) * nx * ny);
//
//        // Tracing edges with hysteresis . Non-recursive implementation.
//        size_t c = 1;
//        for (int j = 1; j < ny - 1; j++)
//        {
//        	for (int i = 1; i < nx - 1; i++)
//            {
//            	// if nms is over thresold and out is zero(why is this checked?) -> edge detected
//                if (nms[c] >= tmax && out[c] == 0)
//                {
//                    out[c] = FLT_MAX;
//
//                    int nedges = 1;
//                    edges[0] = c;  //c=1
//
//                    do {
//                        nedges--;  // nedges = 0
//                        const int t = edges[nedges];  // t =
//
//                        int nbs[8]; // neighbours
//                        nbs[0] = t - nx;     // nn
//                        nbs[1] = t + nx;     // ss
//                        nbs[2] = t + 1;      // ww
//                        nbs[3] = t - 1;      // ee
//                        nbs[4] = nbs[0] + 1; // nw
//                        nbs[5] = nbs[0] - 1; // ne
//                        nbs[6] = nbs[1] + 1; // sw
//                        nbs[7] = nbs[1] - 1; // se
//
//                        // iterating neighbors.
//                        for (int k = 0; k < 8; k++)
//                        {
//                        	//if neighbors are above tmin -> edge detected
//                        	if (nms[nbs[k]] >= tmin && out[nbs[k]] == 0)
//                            {
//                                out[nbs[k]] = FLT_MAX;
//                                edges[nedges] = nbs[k];
//                                nedges++;
//                                set_LED(true);  // enable led
//                                usb_transmit_string("\r\nEDGE!\r\n");
//                            } else {
////                            	set_LED(false);  // disable led
//                            }
//                        }
//
//                        //do iterate until end of
//                    } while (nedges > 0);
//                } else {
////                	set_LED(false);  // disable led
//                }
//                c++;
//            }
//        }

        for(size_t i = 1; i < 1000; i++)
        {


//        	if((after_Gy[i] -after_Gy[i-1]) >= tmin &&
//        			(after_Gy[i] -after_Gy[i-1]) <= tmax)
        	if((after_Gy[i] -after_Gy[i-1]) >= tmin )
        	{
//        		usb_transmit_string("\r\i - ");
//        		usb_transmit_uint(i);
//        		usb_transmit_string(" - ");
//
//				usb_transmit_float(after_Gy[i]);
//				usb_transmit_string("\r\n");
//        		usb_transmit_string("\r\nEDGE!\r\n");

        		set_LED(true);
//        		HAL_Delay(100);
        	}else {
        		set_LED(false);
        	}

        }
        // free reserved memory
        free(after_Gx);
        free(after_Gy);
        free(G);
        free(nms);

//        return out;
}



void convolution(const float *in, float *out, const float *kernel,
                 const int nx, const int ny, const int kn,
                 const bool normalize)
{
//	uint8_t MAX_BRIGHTNESS = 255;  // not used
    const int khalf = kn / 2;  // half of kernel
    float min = FLT_MAX, max = -FLT_MAX;  // min and max float values
    /*
    if (normalize)  // if float value is overfloating
    {


        for (int m = khalf; m < nx - khalf; m++)
        {
            for (int n = khalf; n < ny - khalf; n++) {
                float pixel = 0.0;
                size_t c = 0;
                for (int j = -khalf; j <= khalf; j++)
                {

                    for (int i = -khalf; i <= khalf; i++) {
                        pixel += in[(n - j) * nx + m - i] * kernel[c];
                        c++;
                    }
                if (pixel < min)
                    min = pixel;
                if (pixel > max)
                    max = pixel;
                }
            }
		}
	}
	*/
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

//    fprintf(stderr, "gaussian_filter: kernel size %d, sigma=%g\n",
//            n, sigma);
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
	//gauss filter sigma parameter
//	const float sigma = 1;  // not used. only needed for gauss filter

	// matrix shape

    const int nx = 64;
    const int ny = 50;

//    int MAX_BRIGHTNESS = 255;  // was
//    uint MAX_BRIGHTNESS = 255;  // max float brightness

    // some useful error checking
//    float after_Gy;

    // memory allocation in heap
//	float *in = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *G = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *after_Gx = calloc(nx * ny * sizeof(float), sizeof(float));
//      float *after_Gy = calloc(3200, sizeof(float));
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

//                    	outint = ((n - j) * nx + m - i);
//            			usb_transmit_string("(n - j) * nx + m - i = ");
//            			usb_transmit_int(outint);  //
//            			usb_transmit_string(" - ");
//            			usb_transmit_float(pixel);  //
//            			usb_transmit_string("\n\r");
//            			HAL_Delay(300);

                        pixel += in[(n - j) * nx + m - i] * Gy[c];  // bug here> access in[i]
                        c++;
//                        if(pixel  )



                    }
                    //
                    // result of convolution is a gradient matrix
                    after_Gy[n * nx + m] = pixel;
                }
            }
    	}
//
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

    	float outfloat = 0.0;
//    	int outint;
//    	//	int j = (3199-95);
    		int j = 300;
    		for(int i = j; i <j+90; i++)  // ok
    		{
    			usb_transmit_string("i = ");
    			usb_transmit_int(i);  //
    			usb_transmit_string(" - ");
    			outfloat = after_Gy[i];
    			usb_transmit_float(outfloat);  //
//    			usb_transmit_int(outint);  //

    			usb_transmit_string("\n\r");
    		}
    		HAL_Delay(300);
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
        // free reserved memory
//        free(after_Gx);
//        free(after_Gy);
//        free(G);
//        free(nms);

//        return out;
    		return 0;
}



/*
 * Links:
 * http://en.wikipedia.org/wiki/Canny_edge_detector
 * http://www.tomgibara.com/computer-vision/CannyEdgeDetector.java
 * http://fourier.eng.hmc.edu/e161/lectures/canny/node1.html
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 * https://medium.com/@nikatsanka/comparing-edge-detection-methods-638a2919476e
 * Note: T1 and T2 are lower and upper thresholds.
 */

/* original canny detection
pixel_t *canny_edge_detection_original(const pixel_t *in,
                              const bitmap_info_header_t *bmp_ih,
                              const int tmin, const int tmax,
                              const float sigma)
{
    const int nx = 64;
    const int ny = 50;

    pixel_t *G = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *after_Gx = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *after_Gy = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *nms = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *out = malloc(bmp_ih->bmp_bytesz * sizeof(pixel_t));

    if (G == NULL || after_Gx == NULL || after_Gy == NULL ||
        nms == NULL || out == NULL) {
        fprintf(stderr, "canny_edge_detection:"
                " Failed memory allocation(s).\n");
        exit(1);  // check if error occured
    }

    gaussian_filter(in, out, nx, ny, sigma);

    const float Gx[] = {-1, 0, 1,
                        -2, 0, 2,
                        -1, 0, 1};

    convolution(out, after_Gx, Gx, nx, ny, 3, false);

    const float Gy[] = { 1, 2, 1,
                         0, 0, 0,
                        -1,-2,-1};

    convolution(out, after_Gy, Gy, nx, ny, 3, false);

    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            // G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
            G[c] = (pixel_t)hypot(after_Gx[c], after_Gy[c]);
        }

    // Non-maximum suppression, straightforward implementation.
    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            const int nn = c - nx;
            const int ss = c + nx;
            const int ww = c + 1;
            const int ee = c - 1;
            const int nw = nn + 1;
            const int ne = nn - 1;
            const int sw = ss + 1;
            const int se = ss - 1;

            // calculate intensity gradient vector
            const float dir = (float)(fmod(atan2(after_Gy[c],
                                                 after_Gx[c]) + M_PI,
                                           M_PI) / M_PI) * 8;
            if (((dir <= 1 || dir > 7) && G[c] > G[ee] &&
                 G[c] > G[ww]) || // 0 deg
                ((dir > 1 && dir <= 3) && G[c] > G[nw] &&
                 G[c] > G[se]) || // 45 deg
                ((dir > 3 && dir <= 5) && G[c] > G[nn] &&
                 G[c] > G[ss]) || // 90 deg
                ((dir > 5 && dir <= 7) && G[c] > G[ne] &&
                 G[c] > G[sw]))   // 135 deg
                nms[c] = G[c];
            else
                nms[c] = 0;
        }

    // Reuse array
    // used as a stack. nx*ny/2 elements should be enough.
    int *edges = (int*) after_Gy;
    memset(out, 0, sizeof(pixel_t) * nx * ny);
    memset(edges, 0, sizeof(pixel_t) * nx * ny);

    // Tracing edges with hysteresis . Non-recursive implementation.
    size_t c = 1;
    for (int j = 1; j < ny - 1; j++)
        for (int i = 1; i < nx - 1; i++) {
            if (nms[c] >= tmax && out[c] == 0) { // trace edges
                out[c] = MAX_BRIGHTNESS;


                int nedges = 1;
                edges[0] = c;  //c=1

                do {
                    nedges--;  // nedges = 0
                    const int t = edges[nedges];  // t =

                    int nbs[8]; // neighbours
                    nbs[0] = t - nx;     // nn
                    nbs[1] = t + nx;     // ss
                    nbs[2] = t + 1;      // ww
                    nbs[3] = t - 1;      // ee
                    nbs[4] = nbs[0] + 1; // nw
                    nbs[5] = nbs[0] - 1; // ne
                    nbs[6] = nbs[1] + 1; // sw
                    nbs[7] = nbs[1] - 1; // se

                    for (int k = 0; k < 8; k++)
                        if (nms[nbs[k]] >= tmin && out[nbs[k]] == 0) {
                            out[nbs[k]] = MAX_BRIGHTNESS;
                            edges[nedges] = nbs[k];
                            nedges++;
                        }
                } while (nedges > 0);
            }
            c++;
        }

    free(after_Gx);
    free(after_Gy);
    free(G);
    free(nms);

    return out;
}
*/

//	free(after_Gx);
//    free(after_Gy);
//    free(G);
//    free(nms);

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
