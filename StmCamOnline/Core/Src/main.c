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
enum pixelcolor_t {
    unreliable = 64,
    edge = 126,
	no_edge = 0
} pixelcolor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MODE  // debug mode enables printing to USB. Debug should be always enabled
#define LCD_USAGE	// uncomment to enable LCD

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
#define CAM_BINARY_BUFSIZE 13176
#define CAM_FLOAT_BUFFSIZE 3294


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//  values for data exchange on UART1
volatile uint8_t uart_request_flag = 0;
volatile char uart_request_command = 0;

// if edge is detected egde flag is set to 1, otherwise set to 0
uint8_t edge_detected_flag = 0;
// if edge is detected this variable becomes the number of detection pixel
uint32_t edge_position_variable = 0;

const uint8_t cam_request_cartesian = 'Z';	// request cartesian data from camera
const uint8_t cam_request_radial = 'D';	// request radial data from camera
const uint8_t cam_request_close = 'q';  // close connection to camera(not used)
static uint8_t msg[20]={0,};  // ethernet config debug printing



//************* control parameter ***************************************

// LCD draw mode to start with. 0 = raw image. 1 = filtered image
uint8_t LCDmode_flag = 0;
const uint8_t cam_destip[4] = {192, 168, 178, 69};	// camera has static ip
const uint16_t cam_destport = 50002;	//	camera ip port
uint16_t local_port = 49152;	// this must not be constant and is changed in runtime


// hystetesis values for edge detection
// values less tmin is falling edges (white)
// values between tmin and tmax is noise (black)
// values larger tmax are rising edges (white)
const float tmin = -0.15;
const float tmax = 0.15;
//***********************************************************************


uint8_t binarydata_buffer0[CAM_BINARY_BUFSIZE];  // 13176 bytes raw image data
float floatframe_buffer[CAM_FLOAT_BUFFSIZE];  // 3200 floats, header 94 floats

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// function prototype definitions
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
int32_t get_cam_data(uint8_t *inputvar);
int32_t close_cam_connection(void);
float canny_edge_mod(const float *floatbuf);
float canny_filter_mod(void);
uint32_t yield_depth_color(float input);
uint32_t yield_edge_color(int8_t edge_value);
bool is_negative(float x);
inline uint32_t SwapByteOrder_32(uint32_t a);
inline uint32_t byteswap2little(uint8_t *value);
inline uint32_t checkfloatbytes(float C);
float pun2float(const uint8_t *buf);
float swap2float(uint8_t *buf);
uint32_t pun2int(const uint8_t *buf);

void drawsquare(float inputvar, uint32_t position);

void drawedge(uint32_t position, uint32_t color);
void detect_edge_hysteresis( float raw_input, float filter_input, int32_t position);
float accessfloatarray(uint8_t *buf, uint8_t floatposition);
void arrayreshaping(uint8_t *arrptr);
void set_edge_flag(int32_t edgeposition);
void uart_response(int32_t edgeposition);
/**
  * @brief  This function is used to print parts of the float array for testing and debugging
  * @retval none
  */
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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  // RX callback
{
  if (huart->Instance == USART1)
  {
	  if(uart_request_command == 'b')
	{
	  usb_transmit_string("\r\nuart\r\n");
	  uart_request_flag = true;
	}

//	HAL_UART_Transmit(&huart2, &point1, 1, 100);  // debug point 1 reached
//    /* Transmit one byte with 100 ms timeout */
//    HAL_UART_Transmit(&huart2, &byte, 1, 100);

    /* Receive one byte in interrupt mode */
    HAL_UART_Receive_IT(&huart1, &uart_request_command, 1);  // have to enable interrupt again
  }
}

// Ethernet init procedure
void IO_LIBRARY_Init(void) {
	uint8_t RXbufSize[] =	{ 16, 0, 0, 0, 0, 0, 0, 0, }; // rx buffer size is 0...16, only one socket is usable!
	uint8_t TXbufSize[] =	{ 16, 0, 0, 0, 0, 0, 0, 0, }; // tx buffer size

	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);

//	bug. spi burst mode does not work. reason unknown.
//	reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);


	wizchip_init(TXbufSize, RXbufSize);
	wiz_NetInfo netInfo = { .mac =	{ 0x00, 0x08, 0xdc, 0xab, 0xaf, 0xfe },	// Mac address
							.ip =	{ 192, 168, 178, 1},		// IP address
							.sn =	{ 255, 255, 255, 0 },	// Subnet mask
							.gw =	{ 192,  168, 178, 69 } };	// Gateway address

	wizchip_setnetinfo(&netInfo);

	wizchip_getnetinfo(&netInfo);
	PRINT_NETINFO(netInfo);
	// network timeout configuration
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




  IO_LIBRARY_Init();  // Ethernet lib
  ILI9341_Init();	// LCD Lib
  HAL_UART_Receive_IT(&huart1, &uart_request_command, 1);  // Interrupt on UART1 enabled


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	#ifndef LCD_USAGE
	ILI9341_Draw_Text( "LCD DISABLED", 50,300, BLACK, 2, WHITE);
	#endif
  uint32_t starttime;
  uint32_t stoptime;
  uint32_t difftime;
  HAL_Delay(1000);
  set_LED(true);  // enable led

  while (1)
  {

// 	  Exec time measurement template:
//	  uint32_t starttime;
//	  uint32_t stoptime;
//	  uint32_t difftime;
//	  starttime = HAL_GetTick();  // milliseconds precision
/////////// do something here:
//	  stoptime = HAL_GetTick();
//	  difftime = stoptime - starttime;
//	  usb_transmit_string("Time: ");
//	  usb_transmit_int(difftime);
//	  usb_transmit_string("\n\r");

	starttime = HAL_GetTick();  // milliseconds precision
	checkpushbutton();  // check if blue button was pressed
	get_cam_data(cam_request_cartesian);	// get camera data
	arrayreshaping(binarydata_buffer0);  // reshape and print raw camera data
	canny_filter_mod();	 // filter and print filtered camera data

	uart_response(edge_position_variable);
	set_edge_flag(edge_position_variable);
  // Transmit refresh rate
	char stringbuf[10];
	stoptime = HAL_GetTick();
	difftime = stoptime - starttime;
	sprintf(stringbuf,  "%i", difftime);
	#ifdef DEBUG_MODE
	usb_transmit_int(difftime);
	usb_transmit_string("ms\r\n");
	#endif

	#ifdef LCD_USAGE
	ILI9341_Draw_Text( stringbuf, 100,300, BLACK, 2, WHITE);
	#endif

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

// debug functions for usb printing stuff.
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
	HAL_UART_Transmit(&huart2, (uint8_t*) transmit_string, strlen(transmit_string) , 100);
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

// w5500 control functions
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

/**
  * @brief This function detects a blue button press and swaps(XOR) LCD drawing mode flag
  */
void checkpushbutton(void)
{
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_RESET)
	{
		LCDmode_flag ^= 0x01;
	}
}


/**
  * @brief  Low level check if float is negative value.
  * @retval 1 if x is negative, otherwise 0
  */
bool is_negative(float x) {  // fast check for negative float // only big endian?
//	usb_transmit_byte(signbit(x));
    return signbit(x);
}


/**
  * @brief  Swapping uint32_t from big endian to little endian, compiler optimized version
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
inline uint32_t byteswap2little(uint8_t *value)
{
	uint32_t tmpint;
	memcpy(&tmpint, value, 4);
	return __builtin_bswap32(tmpint);
}

/**
  * @brief  Showing uint32_t bit representation of float value
  * @retval uint32_t type punned value. not byteswapped version
  */
inline uint32_t checkfloatbytes(float C)
{
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
  * @retval float form 4 bytes. not byte-swapping version
  */
float pun2float(const uint8_t *buf)
{
    // from https://blog.regehr.org/archives/959
  float num;
  memcpy(&num, buf, 4);
//    return __builtin_bswap32(num);
  return num;
}
/**
  * @brief  Swapping 4 bytes of uint8_t array and type punning access of 4 bytes as a 32 bit little endian float.
  * @retval float from swapped 4 bytes
  */
float swap2float(uint8_t *buf)
{
// from https://blog.regehr.org/archives/959
	uint32_t tmpswap = byteswap2little(buf);
	float num;
	memcpy(&num, &tmpswap, 4);
	return num;
}


/**
  * @brief  Type punning access of 4 bytes as a 32 bit little endian uint32_t.
  * @retval uint32_t type from 8 bytes.
  */
uint32_t pun2int(const uint8_t *buf)
{
    // from https://blog.regehr.org/archives/959

  uint32_t num;
  memcpy(&num, buf, 4);
//  return __builtin_bswap32(num);  // swap endianness
  return num;
}


/**
  * @brief  Type punning access of 4 bytes as a 32 bit swapped bytes float.
  * @retval none
  */
void drawsquare(float inputvar, uint32_t position)
{
	uint16_t zoom = 4;
	uint16_t row ,column;
	column = (position / 64)*zoom;  // y
	row = (position % 64)*zoom;  // x
	ILI9341_Draw_Rectangle(column,row , zoom, zoom, yield_depth_color(inputvar));
}


/**
  * @brief  Type punning access of 4 bytes as a 32 bit swapped bytes float.
  * @retval uint32_t
  */
float accessfloatarray(uint8_t *buf, uint8_t floatposition)
{
	return swap2float((buf+4*floatposition));
}

/**
  * @brief this function yields a RGB color code from input value. it is used to draw a deep map from raw image data
  * @param random float value
  * @retval RGB565 color code, if input is positive. BLAC color if input negative
  */
uint32_t yield_depth_color(float input)
{
	// 565-RGB linear colorbar
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
			0x701F,  // purple
			0x981F};

	// depth thredshold. Non-adaptive implementation
	const float addsome = 0.05;  // ajust this if image is blurry
	const float t1 = 0.05 + addsome;
	const float t2 = 0.1 + addsome;
	const float t3 = 0.15 + addsome;
	const float t4 = 0.2 + addsome;
	const float t5 = 0.25 + addsome;
	const float t6 = 0.3 + addsome;
	const float t7 = 0.35 + addsome;
	const float t8 = 0.4 + addsome;
	const float t9 = 0.45 + addsome;
	const float t10 = 0.5 + addsome;
	const float t11 = 0.55 + addsome;
	const float t12 = 0.6 + addsome;
	const float t13 = 0.65 + addsome;
	const float t14 = 0.7 + addsome;
	const float t15 = 0.75 + addsome;
	const float t16 = 0.8 + addsome;
	const float t17 = 0.85 + addsome;
	const float t18 = 0.9 + addsome;
	const float t19 = 0.95 + addsome;
	const float t20 = 1.0 + addsome;

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
	return 0;  // foo should not reach this point
}

/**
  * @brief  drawing a pixel depending on absolute input array position and a color value
  * 		color value is taken from yield_edge_color()
  * @param position - absolute array position
  * @param color  - value from from yield_edge_color()
  * @retval none
  */
void drawedge(uint32_t position, uint32_t color)
{

		uint16_t zoomx = 4;
		uint16_t zoomy = 4;
		uint16_t row ,column;
		column = ((position) / 64)*zoomy;  // y
		row = ((position) % 64)*zoomx;  // x
		ILI9341_Draw_Rectangle(column, row, zoomy, zoomx, color);
}

// changed
/**
  * @brief this function yields WHITE if input value is detected edge,
  * 		BLACK if input value is no edge detected
  * 		and RED if unreliable data is detected which happens when
  * 		camera returns -1 or -2 while detecting false illumination
  * @param random float value
  * @retval RGB565 color code
  */
uint32_t yield_edge_color(int8_t edge_value)
{
	switch(edge_value)
	{
		case unreliable: return RED; break;
		case edge: return WHITE; break;
		case no_edge: return BLACK; break;
		default: return RED; break;  // if something goes wrong
	}
}

/**
  * @brief this function yields WHITE if input value is over hysteresis value,
  * 		BLACK if input value is between hysteresis value which is noise filtering
  * 		and RED if input value is a big negative number, which is unreliable image data
  * 		(camera returns -1 or -2 while detecting false illumination )
  * @param raw data float value at raw array position
  * @param calculated fliter value from colvolution at raw data array position
  * @param center position from raw data input array
  * @retval none
  */
void detect_edge_hysteresis( float raw_input, float filter_input, int32_t position)
{
	//	edge thredshold hysteresis
	if(is_negative(raw_input)==true)
	{
		edge_position_variable = -position;  // unreliable data detected at this position
		#ifdef LCD_USAGE
		if(LCDmode_flag == 1)
		{
			drawedge(position, yield_edge_color(unreliable));
		}
		#endif
	}else if(filter_input<tmin || filter_input>tmax)
	{
		edge_position_variable = position;  // edge detected on this position
		#ifdef LCD_USAGE
		if(LCDmode_flag == 1)
		{
			drawedge(position, yield_edge_color(edge));
		}
		#endif

	} else {
		edge_position_variable = 0x7FFF;  //  is 32767 in int and uint to avoid confusion
		#ifdef LCD_USAGE
		if(LCDmode_flag == 1)
		{
			drawedge(position, yield_edge_color(no_edge));
		}
		#endif
	}
}

/**
  * @brief  This function reshapes the raw camera data to a float array
  * 		reshaped data is written to floatframe_buffer global array
  * @param	pointer to raw data array
  * @retval none
  */
void arrayreshaping(uint8_t *arrptr)
{
	uint8_t *tmpptr;
	for(size_t i = 0; i < 3200; i++)  // maybe 3200 - i
	{
		// input pointer is shifted by 376 bit to cut off the data header
		tmpptr = arrptr + 376 + 4*i;
		floatframe_buffer[i] = swap2float(tmpptr );
		#ifdef LCD_USAGE
		if(LCDmode_flag == 0)
		{
			drawsquare(floatframe_buffer[i],i);
		}
		#endif
	}
}

/**
  * @brief  This function reshapes the raw camera data to a float array
  * 		reshaped data is written to floatframe_buffer global array
  * @param	pointer to raw data array
  * @retval none
  */
void set_edge_flag(int32_t edgeposition)
{
	if (edgeposition == 0x7FFF)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(USART_DATAFLAG_GPIO_Port, USART_DATAFLAG_Pin, 0);
	} else {
		HAL_GPIO_WritePin(USART_DATAFLAG_GPIO_Port, USART_DATAFLAG_Pin, 1);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	}
}

void uart_response(int32_t edgeposition)
{
	if(uart_request_flag == true)	// if request flag is set
	{
		char stringbuf[10];
		sprintf(stringbuf,  "%i", edgeposition);
		HAL_UART_Transmit(&huart1,  stringbuf, strlen(stringbuf) , 100);
	}
	uart_request_flag = false;  // clear request flag
}

/**
  * @brief this function requests data by using the w5500 chip socket feature.
  * 		after 5ms delay receiving buffer is read to 'binarydata_buffer0' global array
  * 		the socket is kept alive all the time so reconnect is not needed.
  *
  * @param none
  * @retval 0 when function finishes successfully
  */
int32_t get_cam_data(uint8_t *inputvar)
{
	int32_t ret_sock = 0;
	int32_t ret_connect = 0;
	int32_t ret_rcv = 0;
	int32_t bytes_toread;
	int32_t bytes_received;
	int32_t bytes_burst;
	uint16_t size = 0;
	uint16_t sn = 0; // using only socket 0

	while(!getPHYCFGR());  // phy link check
    switch(getSn_SR(sn))
    {
    	case SOCK_ESTABLISHED :
     // send data here:
    		send(sn, &inputvar, 1);
    		// wait here between response

				#ifndef LCD_USAGE
				HAL_Delay(55);  // delay for network reply needed
				#endif

    		if((size = getSn_RX_RSR(sn)) > 0)
    		{ // If data in rx buffer
    			bytes_toread = CAM_BINARY_BUFSIZE;
    			bytes_received = 0;
    			while(bytes_toread>=1)
    			{
    				bytes_burst = recv(sn, &binarydata_buffer0[bytes_received], bytes_toread);
    				bytes_received = bytes_received + bytes_burst;
    				bytes_toread = CAM_BINARY_BUFSIZE - bytes_received;

				}
    		} else {
    			usb_transmit_string("\r\n Transmission error \r\n");
				usb_transmit_int(size);
//				send(sn, &cam_request_close, 1);

    		}
    		break;
   		case SOCK_CLOSE_WAIT :
			if((ret_connect=disconnect(sn)) != SOCK_OK) return ret_connect;
			#ifdef DEBUG_MODE:
			usb_transmit_string("CloseOK\r\n");
			#endif
   			break;
   		case SOCK_CLOSED :
			#ifdef DEBUG_MODE:
   			usb_transmit_string("SOCK_CLOSED\r\n");
			#endif
			if((ret_sock=socket(sn, Sn_MR_TCP, local_port, 0x0)) != sn)
			{
				usb_transmit_string("SOCK_CLOSED_ERROR\r\n");
				usb_transmit_int(ret_connect);
				close(sn);
				return ret_sock;
			}
			local_port++;  // avoid reusing static local port
			if(local_port > 65000){
				local_port = 49152;
			}

   			break;

   		case SOCK_INIT :
			#ifdef DEBUG_MODE:
   			usb_transmit_string("SOCK_INIT\r\n");
			#endif

			while((ret_connect = connect(sn, cam_destip, cam_destport)) != SOCK_OK)
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



int32_t close_cam_connection(void)
{
	int32_t ret_sock = 0;
	int32_t ret_connect = 0;
//	int32_t ret_rcv = 0;
//	uint16_t size = 0;
	uint16_t sn = 0; // using only socket 0

	while(!getPHYCFGR());  // phy link check
    switch(getSn_SR(sn))
    {
    	case SOCK_ESTABLISHED :
     // send data here:
    		send(sn, &cam_request_close, 1);
    		// wait here between response
    	    HAL_Delay(100);  // delay for network reply needed
    		break;
   		case SOCK_CLOSE_WAIT :
			if((ret_connect=disconnect(sn)) != SOCK_OK) return ret_connect;
			usb_transmit_string("CloseOK\r\n");
   			break;
   		case SOCK_CLOSED :
   			usb_transmit_string("SOCK_CLOSED\r\n");
			if((ret_sock=socket(sn, Sn_MR_TCP, local_port, 0x0)) != sn)
			{
				usb_transmit_string("SOCK_CLOSED_ERROR\r\n");
				usb_transmit_int(ret_connect);
				close(sn);
				return ret_sock;
			}
			local_port++;  // avoid reusing static local port
			if(local_port > 65000){
				local_port = 49152;
			}

   			break;

   		case SOCK_INIT :
   			usb_transmit_string("SOCK_INIT\r\n");

			while((ret_connect = connect(sn, cam_destip, cam_destport)) != SOCK_OK)
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

/**
  * @brief	this function calculates a convolution between two matrices
  * @param in		pointer to input matrix to convolute with
  * @param out		pointer to output matrix
  * @param kernel	pointer to convolution matrix kernel
  * @param nx		x number of input matrix elements
  * @param ny		y number of input matrix elements
  * @param kn		kernel size of convolution matrix
  * @retval none
  */
void convolution(const float *in, float *out, const float *kernel,
                 const int nx, const int ny, const int kn )
{
    const int khalf = kn / 2;  // half of kernel
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
// function not in use
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
    convolution(in, out, kernel, nx, ny, n);
}




/**
  * @brief	experimantal function for edge calculation with sobel operator.
  * 		original code from https://rosettacode.org/wiki/Canny_edge_detector
  *
  * @param none
  * @retval none
  */
float canny_filter_mod(void)
{

	// matrix shape
    const int nx = 64;
    const int ny = 50;

    // acces raw data in global array
	float *in = &floatframe_buffer;

    // memory allocation
//  TODO: Find out, how calloc on stm works and why zeros are not filled
    // memory allocation in heap	// bug? memory leak? wrong values for array? no dynamic memory alloc on stm32?
//	float *in = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *G = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *after_Gx = calloc(nx * ny * sizeof(float), sizeof(float));
//   float *after_Gy = calloc(3200, sizeof(float));
//    float *nms = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *out = malloc(nx * ny * sizeof(float));


// memory allocation in stack
//    float G[nx * ny];
//    float after_Gx[nx * ny];
    float after_Gy[nx * ny];
//    float after_Gy[3200];
//    float nms[nx * ny];
//    float out[nx * ny];


//	  experimental gaussian filter not in use
//    gaussian_filter(in, out, nx, ny, sigma);


// intensity gradient calculation
	const int32_t khalf = 1;
	const int32_t kn = 3;

	// sobel operato in x direction (not in use)
	const float Gx[] = {-1.0, 0.0, 1.0,
						-1.0, 0.0, 1.0,
						-1.0, 0.0, 1.0};
	// sobel operato in y direction
    const float Gy[] = { 1.0, 1.0, 1.0,
                         0.0, 0.0, 0.0,
                        -1.0,-1.0,-1.0};

	// copy of convolution() function for testing.
    // only gradient in y direction is calculated since
    // edged never appear perpendicular to camera.
    // this improves execution time a lot
	for (int m = khalf; m < nx - khalf; m++)  // m from 1 to 63
	{
		for (int n = khalf; n < ny - khalf; n++)  // n from 1 to 49
		{
			float pixel = 0.0;
			size_t c = 0;
			for (int j = -khalf; j <= khalf; j++)  // j from -1 to 1
			{
				for (int i = -khalf; i <= khalf; i++)  // i from -1 to 1
				{
					// multiply input_array with kernel matrix

					pixel += in[(n - j) * nx + m - i] * Gy[c];  //n*nx is zeilennummer , m is spaltennummer
					c++;
				}
				//
				// result of convolution is a gradient matrix

				after_Gy[n * nx + m] = pixel;
			    detect_edge_hysteresis( in[n * nx + m], after_Gy[n * nx + m], n * nx + m);
			}
		}
	}

		// gradient calculation in x direction (not in use)
	/*
	for (int m = khalf; m < nx - khalf; m++)
	{
		{
			for (int n = khalf; n < ny - khalf; n++)
			{
				float pixel = 0.0;
				size_t c = 0;
				for (int j = -khalf; j <= khalf; j++)
				{
					for (int i = -khalf; i <= khalf; i++)
					{
						pixel += in[(n - j) * nx + m - i] * Gx[c];
						c++;
					}
					after_Gx[n * nx + m] = pixel;
				}
			}
		}
	}
	for (int i = 1; i < nx - 1; i++)
	{
		for (int j = 1; j < ny - 1; j++) {
		 size_t c = i + nx * j; // i = column spalte, nx*j = row zeile
  	     //G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
		 G[c] = hypot(after_Gx[c], after_Gy[c]);
		}
	}

	*/



//		  free reserved memory. not in use because memory allocation in stack used
//        free(after_Gx);
//        free(after_Gy);
//        free(G);
//        free(nms);

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
