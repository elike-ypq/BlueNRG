/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : BLE_Chat_main.c
* Author             : AMS - VMA RF Application Team
* Version            : V1.1.0
* Date               : 15-January-2016
* Description        : BlueNRG-1 main file for Chat demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 Chat demo \see BLE_Chat_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "app_state.h"
#include "chat.h"
#include "SDK_EVAL_Config.h"
#include "Chat_config.h"
#include "OTA_btl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_CHAT_VERSION_STRING "1.0.0" 


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
volatile uint32_t lSystickCounter=0;

/* Private function prototypes -----------------------------------------------*/
void SdkDelayMs(volatile uint32_t lTimeMs);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program code.
  * @param  None
  * @retval None
  */
int main(void) 
{
  uint8_t ret;

  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();

  /* Enable the GPIO Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Configure the LEDs */
  GPIO_InitType GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = Get_LedGpioPin(LED1) | Get_LedGpioPin(LED2) | Get_LedGpioPin(LED3);
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = ENABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);

  /* Put the LEDs off */
  GPIO_WriteBit(Get_LedGpioPin(LED1) | Get_LedGpioPin(LED2) | Get_LedGpioPin(LED3), LED_OFF);

  /* Configure SysTick to generate interrupt */
  SysTick_Config(SYST_CLOCK/1000 - 1);
  /* Init Clock */
  /*Clock_Init();*/
  /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
  SdkEvalComIOConfig(Process_InputData);

  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
#if SERVER
  printf("BlueNRG-1 BLE Chat Server Application (version: %s)\r\n", BLE_CHAT_VERSION_STRING);
#else
  printf("BlueNRG-1 BLE Chat Client Application (version: %s)\r\n", BLE_CHAT_VERSION_STRING); 
#endif

  /* Init Chat Device */
  ret = CHAT_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("CHAT_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BLE Stack Initialized \n");
  
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
  /* Initialize the button */
  SdkEvalPushButtonInit(BUTTON_1); 
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
  
  while(1) {
    
    /* Disable UART IRQ to avoid calling BLE stack functions while BTLE_StackTick() is running. */
    NVIC_DisableIRQ(UART_IRQn);
    
    /* BlueNRG-1 stack tick */
    BTLE_StackTick();
    
    NVIC_EnableIRQ(UART_IRQn);
    //printf("LED2 on\r\n");
    
    /* Application tick */
    APP_Tick();
    //printf("led3 on \r\n");
    /*
    SdkDelayMs(250);
    GPIO_ToggleBits(Get_LedGpioPin(LED2));
    SdkDelayMs(250);
    GPIO_ToggleBits(Get_LedGpioPin(LED3));
    SdkDelayMs(250);
    */

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    /* Check if the OTA firmware upgrade session has been completed */
    if (OTA_Tick() == 1)
    {
      /* Jump to the new application */
      OTA_Jump_To_New_Application();
    }
#endif  /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
    if (SdkEvalPushButtonGetState(BUTTON_1) == RESET)
    {
      OTA_Jump_To_Service_Manager_Application();
    }
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
  }
  
} /* end main() */


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
