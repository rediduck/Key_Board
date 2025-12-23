/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



uint8_t a1=0;
uint8_t a2=0;
uint8_t a3=0;
uint8_t a4=0;
uint8_t a5=0;
uint8_t a6=0;
uint8_t a7=0;
uint8_t a8=0;
uint8_t a9=0;
uint8_t a10=0;
uint8_t a11=0;
uint8_t a12=0;
uint8_t a13=0;
uint8_t a14=0;
uint8_t a15=0;
uint8_t a16=0;
uint8_t a17=0;
uint8_t a18=0;
uint8_t a19=0;
uint8_t a20=0;





uint8_t KeyBoard[8] = {0,0,0,0,0,0,0,0};
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
const uint8_t space = 0x2c;
const uint8_t alt=0x04;
const uint8_t win =0x08;
const uint8_t shift =0x02;
const uint8_t ctrl =0x01;
const uint8_t tab= 0x2B;
const uint8_t enter = 0x28;
const uint8_t M= 0x10;
const uint8_t C= 6;
const uint8_t W= 0x1A;
const uint8_t Z =0x1D;
const uint8_t X = 0x1B;
const uint8_t D= 7;
const uint8_t S =0x16;
const uint8_t A =0x04;
const uint8_t R =0x15;
const uint8_t E =0x08;
const uint8_t Q =0x14;
const uint8_t num_4= 0x21;
const uint8_t B = 0x05 ;
const uint8_t Esc = 0x29;
uint8_t sendchar[8] = {0,0,0,0,0,0,0,0};
uint8_t location1;
uint8_t location3;
 int i = 0;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  a1=1;
	  }
	  }
	  
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  a2=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  a3=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  a4=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1)
	  {
			HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1)
	  {
		  a5=1;
	  }
	  }
	  //if(a1==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&num_4,sizeof(KeyBoard));a1=0;}
	  //if(a2==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&Q,sizeof(KeyBoard));a2=0;}
	  //if(a3==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&W,sizeof(KeyBoard));a3=0;}
	  //if(a4==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&E,sizeof(KeyBoard));a4=0;}
	  //if(a5==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&R,sizeof(KeyBoard));a5=0;}
	 
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  a19=1;
	  }
	  }
	  
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  a6=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  a7=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  a8=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1)
	  {
			HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1)
	  {
		  a9=1;
	  }
	  }
	  //if(a1==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&tab,sizeof(KeyBoard));a1=0;}
	  //if(a2==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&A,sizeof(KeyBoard));a2=0;}
	  //if(a3==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&S,sizeof(KeyBoard));a3=0;}
	  //if(a4==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&D,sizeof(KeyBoard));a4=0;}
	  //if(a5==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&enter,sizeof(KeyBoard));a5=0;}
	 
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  a10=1;
	  }
	  }
	  
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  a11=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  a12=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  a13=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1)
	  {
			HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==1)
	  {
		  a14=1;
	  }
	  }
	  //if(a1==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&ctrl,sizeof(KeyBoard));a1=0;}
	  //if(a2==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&Z,sizeof(KeyBoard));a2=0;}
	  //if(a3==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&X,sizeof(KeyBoard));a3=0;}
	  //if(a4==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&C,sizeof(KeyBoard));a4=0;}
	  //if(a5==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&M,sizeof(KeyBoard));a5=0;}
	  
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)
	  {
		  a15=1;
	  }
	  }
	  
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
	  {
		  a16=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)
	  {
		  a17=1;
	  }
	  }
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  HAL_Delay(10);
		  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==1)
	  {
		  a18=1;
	  }
	  }
	  //if(a1==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&shift,sizeof(KeyBoard));a1=0;}
	  //if(a2==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&win,sizeof(KeyBoard));a2=0;}
	  //if(a3==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&B,sizeof(KeyBoard));a3=0;}
	  //if(a4==1){		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&space,sizeof(KeyBoard));a4=0;}

	  for(int j =0;j<=7;j++){sendchar[j]=0;}
	  
	  {
		 i=0;
	  if(a15 == 1){	sendchar[0] = ctrl; a15 = 0;}
	  if(a10 == 1){	sendchar[0] = shift; a10 = 0;}
	  if(a16 == 1){	sendchar[2+i] = num_4; a16 = 0;i++;}
	  if(a1 == 1){	sendchar[2+i] = Esc; a1 = 0;i++;}    
	  if(a2 == 1){	sendchar[2+i] = Q; a2 = 0;i++;}
	  if(a3 == 1){	sendchar[2+i] = W; a3 = 0;i++;}
	  if(a4 == 1){	sendchar[2+i] = E; a4 = 0;i++;}
	  if(a5 == 1){	sendchar[2+i] = R; a5 = 0;i++;}
	  if(a19 == 1){	sendchar[2+i] = tab; a19 = 0;i++;if(i>=6){i=5;}}
	  if(a6 == 1){	sendchar[2+i] = A; a6 = 0;i++;if(i>=6){i=5;}}
	  if(a7 == 1){	sendchar[2+i] = S; a7 = 0;i++;if(i>=6){i=5;}}
	  if(a8 == 1){	sendchar[2+i] = D; a8 = 0;i++;if(i>=6){i=5;}}
	  if(a9 == 1){	sendchar[2+i] = enter; a9 = 0;i++;if(i>=6){i=5;}}
      if(a11 == 1){	sendchar[2+i] = Z; a11 = 0;i++;if(i>=6){i=5;}}
	  if(a12 == 1){	sendchar[2+i] = X; a12 = 0;i++;if(i>=6){i=5;}}
	  if(a13 == 1){	sendchar[2+i] = C; a13 = 0;i++;if(i>=6){i=5;}}
	  if(a14 == 1){	sendchar[2+i] = M; a14 = 0;i++;if(i>=6){i=5;}}
	  if(a17 == 1){	sendchar[2+i] = B; a17 = 0;i++;if(i>=6){i=5;}}
	  if(a18 == 1){	sendchar[2+i] = space; a18 = 0;}
	}
	  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&sendchar,sizeof(sendchar));
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#define ROWS 4
#define COLS 5

GPIO_TypeDef* rowPorts[ROWS] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t rowPins[ROWS]     = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};

GPIO_TypeDef* colPorts[COLS] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t colPins[COLS]       = {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};

// HID 键值映射（按你原来的布局）



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
