/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dcmi.h"
#include "dma2d.h"
#include "eth.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "quadspi.h"
#include "rtc.h"
#include "sai.h"
#include "sdmmc.h"
#include "spdifrx.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "shell.h"
#include "stdlib.h"
#include "ADXL345.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Q_GT_LENGTH 10
#define Q_GT_SIZE sizeof(big_struct_t)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TaskHandle_t handle_blink_led;
TaskHandle_t handle_uart_com;
TaskHandle_t handle_task_give;
TaskHandle_t handle_task_take;
TaskHandle_t handle_task_shell;
TaskHandle_t handle_spam;
TaskHandle_t handle_task_bidon;
TaskHandle_t handle_task_status;

uint8_t uart_rx_flag;
SemaphoreHandle_t sem_state;
SemaphoreHandle_t sem_shell;

int * buffer;
int delay;
int delay_msg;
char * spam_msg;
QueueHandle_t queue ;
BaseType_t res ;
char pcWriteBuffer[400];


int count_task = 0;

int __io_putchar(int chr){
	HAL_UART_Transmit(&huart1, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	return chr;
}

//void Task_blink_led( void * unused ){
//	for(;;){
//		HAL_GPIO_TogglePin(GPIOI, LED_Pin);
//		//printf("LED state has changed\r\n");
//		vTaskDelay(delay);
//	}
//}

void Task_spam( void * unused ){
	for(;;){
		if(delay_msg!=0){
			printf("%s\r\n", spam_msg);
			vTaskDelay(delay_msg);
			vTaskSuspend(handle_spam);
	 		}
		else {
			vTaskResume(handle_spam);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t pxHigherPriorityTaskWoken;
	vTaskNotifyGiveFromISR( handle_task_shell, &pxHigherPriorityTaskWoken );
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void configureTimerForRunTimeStats(void)
{
	// Start timer 2
	HAL_TIM_Base_Start(&htim2);
}

unsigned long getRunTimeCounterValue(void)
{
	return (unsigned long)__HAL_TIM_GET_COUNTER(&htim2);
}

//void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName){
//	for(;;){
//		HAL_GPIO_TogglePin(GPIOI, LED_Pin);
//		HAL_Delay(1000);
//	}
//
//
//}

/*
void Task_uart_com( void * unused ){
	for(;;){
		char chr;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)&chr, 1);

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		//Ici il faut blocker le programme jusqu'a la reception d'un caractère
		HAL_UART_Transmit(&huart1, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	}
}
*/



void Task_give(void * unused){
	for(;;){
		printf("Task give semaphore before\r\n");
		xSemaphoreGive(sem_state);					//Semaphore
		for(int i = 0; i<15;i++){
			printf("Increment de i, multiple du delay 100ms : %d\r\n", i);
			printf("Task give semaphore before\r\n");
			xSemaphoreGive(sem_state);												//Semaphore
			xTaskNotifyGive(handle_task_take);										//Notification
			BaseType_t xQueuestatus = xQueueSend(queue, &i, portMAX_DELAY);				//Queue
			if (xQueuestatus!= pdTRUE){
				printf("Error QueueSend");
			}
			vTaskDelay(100*i);
		}

		printf("Task give semaphore after\r\n");
	}

}


void Task_take(void * unused){
	for(;;){
		printf("Task take semaphore before\r\n");
		xSemaphoreTake(sem_state, portMAX_DELAY);										//Semaphore
		uint32_t TaskStatus = ulTaskNotifyTake(pdTRUE, 1000 );							//Notification
		if (TaskStatus != pdTRUE){
			printf("SYSTEM RESET\r\n");
			NVIC_SystemReset();
		}
		BaseType_t XQueuestatus = xQueueReceive(queue, &buffer, 1000);					//Queue
		if (XQueuestatus != pdTRUE){
			printf("SYSTEM RESET\r\n");
			NVIC_SystemReset();
		}
		printf("Receive number : %d \r\n", *buffer);
		printf("Task take semaphore after\r\n");
	}
}



//int led_setup(int argc, char ** argv)
//{
//	delay = atoi(argv[1]);
//	if (delay == 0){
//		HAL_GPIO_WritePin(GPIOI, LED_Pin, GPIO_PIN_RESET);
//		vTaskSuspend(handle_blink_led);
//	}
//	else{
//		vTaskResume(handle_blink_led);
//	}
//	return 0;
//
//}

int spam_setup(int argc, char ** argv)
{
	delay_msg = atoi(argv[1]);
	spam_msg = argv[2];
	return 0;

}

int cpu_status(int argc, char ** argv){
	printf("Etat d'utilisation du CPU :\r\n");
	vTaskGetRunTimeStats(pcWriteBuffer); //Cette fonction écrit dans le buffer passé en paramètre

	printf("%s\r\n", pcWriteBuffer); //On récupère les données dans le buffer pour les mettres dans le printf

	vTaskList(pcWriteBuffer); //Cette fonction écrit dans le buffer passé en paramètre

	printf("%s\r\n", pcWriteBuffer);
	return 0;
}

int fonction(int argc, char ** argv)
{
	printf("argc = %d\r\n", argc);

	for(int itr = 0 ; itr < argc; itr++){
		printf("argv[%d] : %s\r\n",itr,argv[itr]);
	}
	//printf("Je suis une fonction bidon\r\n");

	return 0;
}

int addition(int argc, char ** argv){
	if (argc!=3){
		printf("Error expeted two args\r\n");
		return -1;
	}
	else
	{
		int a,b;
		a = atoi(argv[1]);
		b = atoi(argv[2]);

		for(int i=0; i<argc;i++){
			int c = a+b;
			printf("%d +%d = %d", a,b,c);
		}
	}
	return 0;
}

//void Task_bidon(void * unused){
//	int tableau_enorme[2000];
//	int i = 0;
//
//	printf("Task bidon\r\n");
//
//	for(;;){
//		tableau_enorme[i] = i;
//		i++;
//		printf("%d\r\n", tableau_enorme[i]);
//	}
//}

int acc_status(int argc, char ** argv){
	uint8_t address = 0x00;
	uint8_t p_data;
	uint16_t size = 1;
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET); //ChipSelect a 0
	HAL_SPI_Transmit(&hspi2, &address, 1, HAL_MAX_DELAY); //On transmet un msg a l'adresse 0x00
	HAL_SPI_Receive(&hspi2, &p_data, size, HAL_MAX_DELAY); // On recoit en SPI de la data
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET); //On remet le chipSelect a 1
	printf("%X \r\n", p_data); // On printf la valeur renvoyé, dans notre cas 229 ce qui est la rst value
	return 0;
}



int init_IT(int argc, char ** argv){
	//Init du power_control
	uint8_t POWER_CTL = 0x2D; //POWER_CTL addr
	uint8_t POWER_CTL_data = 0;
	ACC_READ(POWER_CTL, &POWER_CTL_data);	// & représente l'addresse de POWER_CTL_data
											// &POWER_CTL_data est de type uint8_t*
	POWER_CTL_data |=  0x08; //On active la mesure avec ce masque
	ACC_TRANSMIT(POWER_CTL, POWER_CTL_data); //On transmet la donnée au acc
	//Init des interuptions
	uint8_t INT_ENABLE = 0x2E; //INT_ENABLE addr
	uint8_t INT_ENABLE_data = 0 ;
	ACC_READ(INT_ENABLE, &INT_ENABLE_data);
	INT_ENABLE_data |= 0x80;
	ACC_TRANSMIT(INT_ENABLE, INT_ENABLE_data);


	printf("POWER_CTL_data = %x \r\n", POWER_CTL_data);
	printf("INT_ENABLE_data = %x \r\n", INT_ENABLE_data);

	uint8_t address = 0x32;
	uint16_t size = 6;
	uint8_t p_data[size];	// p_data c'est un pointeur sur le premier element du tableau
							// donc c'est un uint8_t*
							// *p_data, c'est la valeur pointée (du premier élément du tableau)
							// il n'y a pas de sens à écrire &p_data

	while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == 0);
	ACC_READ_Mult(p_data, address, size);
	int16_t acc_x = (int16_t)p_data[0] | ((int16_t)p_data[1]) << 8;
	int16_t acc_y = (int16_t)p_data[2] | ((int16_t)p_data[3]) << 8;
	int16_t acc_z = (int16_t)p_data[4] | ((int16_t)p_data[5]) << 8;
	printf("Valeur d'acc : %d, %d, %d \r\n", acc_x, acc_y, acc_z);

	return 0;
}


void Task_shell(void * unused){
	shell_init();
	shell_add('f', fonction, "Une fonction inutile");
	shell_add('a', addition, "Addition de deux arguments");
	//shell_add('l', led_setup, "Fais clignoter la led");
	shell_add('s', spam_setup,"Spam de la phrase écrit en argument");
	shell_add('c', cpu_status, "Donne le status du cpu");
	shell_add('g', acc_status, "Donne le status de l'acc");
	shell_add('i', init_IT, "Initialise l'acc");
	shell_run();
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_DCMI_Init();
  MX_DMA2D_Init();
  MX_ETH_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_SPDIFRX_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP;

	printf("==============================\r\n");
	sem_state = xSemaphoreCreateBinary();

//	//////////////////////TASK LED///////////////////////////
//
//	res = xTaskCreate(
//			Task_blink_led,	//Nom de la fonction
//			"BLINK_LED", 		//Nom de la tache
//			256, 				//Taille de la pile en mote de 32bits
//			NULL, 			//Pas de paramètres utilisé
//			4, 				//Priorité
//			&handle_blink_led
//	);
//	if (res != pdPASS)
//	{
//		printf("Error creating task blink led\r\n");
//		Error_Handler();
//	}

	//////////////////////TASK SPAM///////////////////////////

	res = xTaskCreate(
			Task_spam,		//Nom de la fonction
			"SPAM", 		//Nom de la tache
			256, 			//Taille de la pile en mote de 32bits
			NULL, 			//Pas de paramètres utilisé
			4, 				//Priorité
			&handle_spam
	);
	if (res != pdPASS)
	{
		printf("Error creating task blink led\r\n");
		Error_Handler();
	}
/*
	//////////////////////TASK UART///////////////////////////
	res = xTaskCreate(
			Task_uart_com,	//Nom de la fonction
			"Com_uart", 	//Nom de la tache
			256, 			//Taille de la pile en mote de 32bits
			NULL, 			//Pas de paramètres utilisé
			5, 				//Priorité
			&handle_uart_com
	);
	if (res != pdPASS)
	{
		printf("Error creating task com uart\r\n");
		Error_Handler();
	}
*/

	//////////////////////TASK GIVE///////////////////////////
	res = xTaskCreate(
			Task_give,		//Nom de la fonction
			"Task Give", 	//Nom de la tache
			256, 			//Taille de la pile en mote de 32bits
			NULL, 			//Pas de paramètres utilisé
			2, 				//Priorité
			&handle_task_give
	);
	if (res != pdPASS)
	{
		printf("Error creating task Task Give\r\n");
		Error_Handler();
	}

	//////////////////////TASK TAKE///////////////////////////
	res = xTaskCreate(
			Task_take,		//Nom de la fonction
			"Task Take", 	//Nom de la tache
			256, 			//Taille de la pile en mote de 32bits
			NULL, 			//Pas de paramètres utilisé
			3, 				//Priorité
			&handle_task_take
	);
	if (res != pdPASS)
	{
		printf("Error creating task Task Take\r\n");
		Error_Handler();
	}

	//////////////////////TASK SHELL///////////////////////////
	res = xTaskCreate(
			Task_shell,		//Nom de la fonction
			"Task shell", 	//Nom de la tache
			256, 			//Taille de la pile en mote de 32bits
			NULL, 			//Pas de paramètres utilisé
			6, 				//Priorité
			&handle_task_shell
	);
	if (res != pdPASS)
	{
		printf("Error creating task Task shell\r\n");
		Error_Handler();
	}
	printf("Task shell creation successful\r\n");

	HAL_GPIO_EXTI_Callback(INT_Pin);{
		uint8_t address = 0x32;
		uint16_t size = 6;
		uint8_t p_data[size];
		ACC_READ_Mult(p_data, address, size);
		int16_t acc_x = (int16_t)p_data[0] | ((int16_t)p_data[1]) << 8;
		int16_t acc_y = (int16_t)p_data[2] | ((int16_t)p_data[3]) << 8;
		int16_t acc_z = (int16_t)p_data[4] | ((int16_t)p_data[5]) << 8;
		printf("Valeur d'acc : %d, %d, %d \r\n", acc_x, acc_y, acc_z);
	}

	queue = xQueueCreate(10,sizeof(int));
	vTaskStartScheduler();// BOUCLE infinie on execute rien après !!!


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
