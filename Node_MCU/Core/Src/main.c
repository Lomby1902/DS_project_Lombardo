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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLASH_START_ADDR  0x08020000
#define FIRMWARE_MAX_SIZE (256 * 1024) // 256KB
#define FIRMWARE_END_ADDR (FLASH_START_ADDR + FIRMWARE_MAX_SIZE)
#define FIRMWARE_DONE_ID  0x12
#define FIRMWARE_START_ID 0x10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PFP */
void Flash_Erase(uint32_t start_addr, uint32_t size);
void Flash_Write(uint32_t address, uint8_t* data, uint8_t len);
void Flash_Write_Buffered(uint32_t address, uint8_t* data, uint8_t len);
void Flash_Write_Flush(void);
uint8_t IsValidApplication(void);
void JumpToApplication(void);
/* USER CODE END PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef	RxHeader;
uint8_t	RxData[8];
uint32_t current_flash_addr = FLASH_START_ADDR;
uint8_t programming_mode = 0;
uint8_t write_buffer[4];
uint8_t buffer_index = 0;


typedef void (*pFunction)(void);


void Flash_Erase(uint32_t start_addr, uint32_t size)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    // Calculate sectors for STM32F4xx
    // Sectors 0-3: 16KB each (0x08000000-0x0800FFFF)
    // Sectors 4: 64KB (0x08010000-0x0801FFFF)
    // Sectors 5-11: 128KB each (0x08020000-0x080FFFFF)

    uint32_t start_sector;
    uint32_t num_sectors;

    if (start_addr >= 0x08020000) {
        // Application area - 128KB sectors
        start_sector = 5 + (start_addr - 0x08020000) / 0x20000;
        num_sectors = (size + 0x1FFFF) / 0x20000; // Round up
    } else if (start_addr >= 0x08010000) {
        // 64KB sector
        start_sector = 4;
        num_sectors = 1;
    } else {
        // 16KB sectors
        start_sector = (start_addr - 0x08000000) / 0x4000;
        num_sectors = (size + 0x3FFF) / 0x4000; // Round up
    }

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7-3.6V
    EraseInitStruct.Sector = start_sector;
    EraseInitStruct.NbSectors = num_sectors;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

    HAL_FLASH_Lock();

    if (status != HAL_OK || SectorError != 0xFFFFFFFF) {
        // Handle erase error
        Error_Handler();
    }
}

// Enhanced application validation
uint8_t IsValidApplication(void)
{
    uint32_t appStack = *(volatile uint32_t*)FLASH_START_ADDR;
    uint32_t appResetHandler = *(volatile uint32_t*)(FLASH_START_ADDR + 4);

    // Check if flash is erased (all 0xFF)
    if (appStack == 0xFFFFFFFF || appResetHandler == 0xFFFFFFFF)
        return 0;

    // Check if flash is blank (all 0x00)
    if (appStack == 0x00000000 || appResetHandler == 0x00000000)
        return 0;

    // Check stack pointer is in RAM range (adjust for your MCU)
    if (appStack < 0x20000000 || appStack > 0x20020000)
        return 0;

    // Check reset handler is in valid flash range
    if (appResetHandler < 0x08020000 || appResetHandler > 0x0803FFFF)
        return 0;

    // Check reset handler has Thumb bit set (bit 0 = 1 for Cortex-M)
    if ((appResetHandler & 0x01) == 0)
        return 0;

    return 1;
}

void JumpToApplication(void)
{
    if (!IsValidApplication()) {
        // Flash LED to indicate no valid app
        for(int i = 0; i < 10; i++) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(100);
        }
        return;
    }

    uint32_t appStack = *(volatile uint32_t*)FLASH_START_ADDR;
    uint32_t appResetHandler = *(volatile uint32_t*)(FLASH_START_ADDR + 4);

    // Proper system cleanup
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeInit(&hcan1);
    HAL_RCC_DeInit();
    HAL_DeInit();

    // Disable SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Disable all interrupts
    __disable_irq();

    // Clear pending interrupts
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Relocate vector table
    SCB->VTOR = FLASH_START_ADDR;

    // Set stack pointer and jump
    __set_MSP(appStack);
    ((pFunction)appResetHandler)();
}

// Improved flash write with proper buffering
void Flash_Write_Buffered(uint32_t address, uint8_t* data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        write_buffer[buffer_index++] = data[i];

        if (buffer_index == 4) {
            HAL_FLASH_Unlock();
            uint32_t word = *(uint32_t*)write_buffer;
            HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_flash_addr, word);
            HAL_FLASH_Lock();

            if (status != HAL_OK) {
                Error_Handler();
            }

            buffer_index = 0;
            current_flash_addr += 4;  // 🔁 aggiorna solo dopo il write
        }
    }
}

// Flush remaining bytes in buffer
void Flash_Write_Flush(void)
{
    if (buffer_index > 0) {
        // Pad buffer with 0xFF
        while (buffer_index < 4) {
            write_buffer[buffer_index++] = 0xFF;
        }

        HAL_FLASH_Unlock();
        uint32_t word = *(uint32_t*)write_buffer;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_flash_addr, word);
        HAL_FLASH_Lock();

        buffer_index = 0;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    switch (RxHeader.StdId) {
        case FIRMWARE_START_ID:
            // Start programming - erase flash first
            programming_mode = 1;
            current_flash_addr = FLASH_START_ADDR;
            buffer_index = 0;

            // Erase application flash area
            Flash_Erase(FLASH_START_ADDR, FIRMWARE_MAX_SIZE);

            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            break;

        case FIRMWARE_DONE_ID:
            if (programming_mode) {
                // Flush any remaining data
                Flash_Write_Flush();
                programming_mode = 0;

                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                JumpToApplication();
            }
            break;

        default:
            // Data packet
            if (programming_mode) {
                Flash_Write_Buffered(current_flash_addr, RxData, RxHeader.DLC);
            }
            break;
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
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */

	HAL_CAN_Start(&hcan1);

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE END 2 */

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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 9;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x0000;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x0000;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)
	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
	/* USER CODE END CAN1_Init 2 */

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
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
