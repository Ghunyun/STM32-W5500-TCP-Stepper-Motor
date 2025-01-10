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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "socket.h"
#include "dhcp.h"
#include "math.h"
#include "stdlib.h"
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile bool stepperExecuted = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define SOCK_TCPS       0
#define SOCK_UDPS       1
#define PORT_TCPS       80
#define PORT_UDPS       3000

uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7};
uint8_t RX_BUF[1024];
uint8_t TX_BUF[1024];
char tcp_rx_buffer[100]; // Buffer for received TCP data
char uart_tx_buffer[100]; // Buffer for UART transmission
char ok[100];
/*
wiz_NetInfo net_info = {
    .mac  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },
    .dhcp = NETINFO_DHCP
};
*/

int flag = 0;

wiz_NetInfo net_info = {
		.mac = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },//MSB - LSB
		.ip ={ 192, 168, 100, 153 },
		.sn = { 255, 255, 255, 0 },
		.gw ={ 192, 168, 100, 1 },
		.dns = { 1, 1, 1, 1 },
		.dhcp = NETINFO_STATIC };

void wizchipSelect(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void wizchipUnselect(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}

void wizchipReadBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
}

void wizchipWriteBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
}

uint8_t wizchipReadByte(void) {
    uint8_t byte;
    wizchipReadBurst(&byte, sizeof(byte));
    return byte;
}

void wizchipWriteByte(uint8_t byte) {
    wizchipWriteBurst(&byte, sizeof(byte));
}

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
    ip_assigned = true;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
    starter();

}

void Callback_IPConflict(void) {
    ip_assigned = false;

}

uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_9) {
	    flag = 1;
	  }

}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void W5500Init() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    // Register W5500 callbacks
    reg_wizchip_cs_cbfunc(wizchipSelect, wizchipUnselect);
    reg_wizchip_spi_cbfunc(wizchipReadByte, wizchipWriteByte);
    reg_wizchip_spiburst_cbfunc(wizchipReadBurst, wizchipWriteBurst);

    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    uint32_t ctr = 100;
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
    }
    if(ip_assigned == false) {
    	NVIC_SystemReset();
    }
/*
    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);
*/
//    char charData[200]; // Data holder
//    sprintf(charData,"IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\n",
//        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
//        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
//        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]
//    );
//    HAL_UART_Transmit(&huart1,(uint8_t *)charData,strlen(charData),1000);

    wizchip_setnetinfo(&net_info);
}

void forward_tcp_to_uart(int socket_fd) {
    static int current_x = 0; // Cumulative x value, starts at 0

    int16_t received_length = recv(socket_fd, (uint8_t *)tcp_rx_buffer, sizeof(tcp_rx_buffer));
    if (received_length > 0) {
        tcp_rx_buffer[received_length] = '\0'; // Null-terminate the string

        // Parse the received string (assumed format: x,y)
        int x, y;
        if (sscanf((char*)tcp_rx_buffer, "%d,%d", &x, &y) == 2) {
            // Convert x and y values
            x = x / 0.05625;
            y = y / 0.05625 * 3;

            // Adjust x to ensure it stays within 0-360
            int new_x = current_x + x;
            if (new_x > 6400) {
                x -= (new_x - 6400); // Reduce x to keep total at 360
            } else if (new_x < 0) {
                x -= new_x; // Adjust x to keep total at 0
            }

            // Update the cumulative x value
            current_x += x;
            int printedx = x * 0.05625;
            int printedy = y * 0.05625 / 3;

            // Print the adjusted x value
            sprintf(uart_tx_buffer, "x: %d\r\ny: %d\r\n", printedx, printedy);
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), HAL_MAX_DELAY);

            reset_stepper_flag();

            // Execute motor control for x
            if (x < 0) {
                Xsteppermotorccw((int)abs(x));
            } else if (x > 0) {
                Xsteppermotorcw((int)abs(x));
            }

            // Process y value
            if (y < 0) {
                Ysteppermotorccw((int)abs(y));
            } else if (y > 0) {
                Ysteppermotorcw((int)abs(y));
            }

            if (!stepperExecuted) {
                stepperExecuted = true; // Mark as executed
            }
        } else {
            // If parsing fails, send an error message
            sprintf(uart_tx_buffer, "Invalid data format\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), HAL_MAX_DELAY);
        }
        sprintf(ok, "Ready to excecute next command\n\r");
        HAL_UART_Transmit(&huart2, (uint8_t *)ok, strlen(ok), HAL_MAX_DELAY);
    }
}



// Main loop to process TCP and forward data
void process_tcp_server(int socket_fd) {
	 while (1) {
	        // Check for any new connections or data
	        if (getSn_SR(socket_fd) == SOCK_ESTABLISHED) {
	            forward_tcp_to_uart(socket_fd);
	        } else if (getSn_SR(socket_fd) == SOCK_CLOSE_WAIT) {
	            disconnect(socket_fd);
	            close(socket_fd);
	            break;
	        }
	    }
}
void reset_stepper_flag(void) {
    stepperExecuted = false;
}

void starter (void){

	for (int i=0; i<6400; i++)
	   {
		 HAL_GPIO_WritePin (DIR2_GPIO_Port, DIR2_Pin, RESET);
	     HAL_GPIO_WritePin (ENA2_GPIO_Port, ENA2_Pin, SET);
	     HAL_GPIO_WritePin (PUL2_GPIO_Port, PUL2_Pin, SET);
	     delay_us(150);
	     HAL_GPIO_WritePin (PUL2_GPIO_Port, PUL2_Pin, RESET);
	     HAL_GPIO_WritePin (ENA2_GPIO_Port, ENA2_Pin, RESET);
	     delay_us(150);
	}


	for (int i=0; i<6400; i++)
	   {
		 HAL_GPIO_WritePin (DIR1_GPIO_Port, DIR1_Pin, RESET);
	     HAL_GPIO_WritePin (ENA1_GPIO_Port, ENA1_Pin, SET);
	     HAL_GPIO_WritePin (PUL1_GPIO_Port, PUL1_Pin, SET);
	     delay_us(300);
	     HAL_GPIO_WritePin (PUL1_GPIO_Port, PUL1_Pin, RESET);
	     HAL_GPIO_WritePin (ENA1_GPIO_Port, ENA1_Pin, RESET);
	     delay_us(300);
			if (flag == 1){
				flag = 0;
				break;
			}
	}


}

void Xsteppermotorcw(x){

	for (int i=0; i<x; i++)
	   {
		 HAL_GPIO_WritePin (DIR1_GPIO_Port, DIR1_Pin, SET);
	     HAL_GPIO_WritePin (ENA1_GPIO_Port, ENA1_Pin, SET);
	     HAL_GPIO_WritePin (PUL1_GPIO_Port, PUL1_Pin, SET);
	     delay_us(300);
	     HAL_GPIO_WritePin (PUL1_GPIO_Port, PUL1_Pin, RESET);
	     HAL_GPIO_WritePin (ENA1_GPIO_Port, ENA1_Pin, RESET);
	     delay_us(300);
			}
	}


void Ysteppermotorcw(y){

	for (int i=0; i<y; i++)
	   {
		 HAL_GPIO_WritePin (DIR2_GPIO_Port, DIR2_Pin, SET);
	     HAL_GPIO_WritePin (ENA2_GPIO_Port, ENA2_Pin, SET);
	     HAL_GPIO_WritePin (PUL2_GPIO_Port, PUL2_Pin, SET);
	     delay_us(150);
	     HAL_GPIO_WritePin (PUL2_GPIO_Port, PUL2_Pin, RESET);
	     HAL_GPIO_WritePin (ENA2_GPIO_Port, ENA2_Pin, RESET);
	     delay_us(150);
	}

}

void Xsteppermotorccw(x){

	for (int i=0; i<x; i++)
	   {
		 HAL_GPIO_WritePin (DIR1_GPIO_Port, DIR1_Pin, RESET);
	     HAL_GPIO_WritePin (ENA1_GPIO_Port, ENA1_Pin, SET);
	     HAL_GPIO_WritePin (PUL1_GPIO_Port, PUL1_Pin, SET);
	     delay_us(300);
	     HAL_GPIO_WritePin (PUL1_GPIO_Port, PUL1_Pin, RESET);
	     HAL_GPIO_WritePin (ENA1_GPIO_Port, ENA1_Pin, RESET);
	     delay_us(300);
	}
}

void Ysteppermotorccw(y){
	for (int i=0; i<y; i++)
	   {
		 HAL_GPIO_WritePin (DIR2_GPIO_Port, DIR2_Pin, RESET);
	     HAL_GPIO_WritePin (ENA2_GPIO_Port, ENA2_Pin, SET);
	     HAL_GPIO_WritePin (PUL2_GPIO_Port, PUL2_Pin, SET);
	     delay_us(150);
	     HAL_GPIO_WritePin (PUL2_GPIO_Port, PUL2_Pin, RESET);
	     HAL_GPIO_WritePin (ENA2_GPIO_Port, ENA2_Pin, RESET);
	     delay_us(150);
	}

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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  W5500Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    int socket_fd = socket(SOCK_TCPS, Sn_MR_TCP, PORT_TCPS, 0);
	    if (socket_fd >= 0 && listen(socket_fd) == SOCK_OK) {
	        process_tcp_server(socket_fd);  // Process incoming TCP connections
	        close(socket_fd);              // Close socket when done
	    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|PUL2_Pin|ENA2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR1_Pin|ENA1_Pin|PUL1_Pin|CS_Pin
                          |DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PUL2_Pin ENA2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|PUL2_Pin|ENA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin ENA1_Pin PUL1_Pin CS_Pin
                           DIR2_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|ENA1_Pin|PUL1_Pin|CS_Pin
                          |DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Proximity_Pin */
  GPIO_InitStruct.Pin = Proximity_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Proximity_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
