/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_manager.h"
#include "powertrain_state.h"
#include "rectifier_task.h"
#include "supervisor_task.h"
#include "pdb_task.h"
#include "sensor_task.h"
#include "log_task.h"
#include "fc_link_task.h"
#include "bms_task.h"
#include "ecu_task.h"
#include "experiment_task.h"
#include "dyno_setup_task.h"
#include "throttle_ctrl_task.h"
#include "control_law_test.h"
#include "periph_wrappers.h"

/* FATFS_SD timeout counters — decremented in vApplicationTickHook below.
 * Requires configUSE_TICK_HOOK = 1 (set in CubeMX → FreeRTOS → Config). */
extern uint16_t Timer1, Timer2;
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* 1 Hz LED heartbeat — toggled here so it freezes when the system is so
    * overloaded that idle never runs. HAL_GetTick is ISR-safe and doesn't
    * touch FreeRTOS state. */
   static uint32_t last_toggle = 0;
   uint32_t now = HAL_GetTick();
   if ((uint32_t)(now - last_toggle) >= 500u) {
       led_hw_toggle();
       last_toggle = now;
   }
}

void vApplicationTickHook( void )
{
   /* Decrement the FATFS_SD millisecond counters used inside the SD driver
    * for SPI/SD timeouts. Called from xPortSysTickHandler each FreeRTOS tick
    * (1 ms) when configUSE_TICK_HOOK = 1. */
   if (Timer1 > 0) Timer1--;
   if (Timer2 > 0) Timer2--;
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
/* vApplicationTickHook implemented above (USER CODE 2 block) — Timer1/Timer2
 * decrement for FATFS_SD lives there. CubeMX regen put an empty stub here;
 * removed to avoid duplicate-definition. */
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   (void)xTask;
   (void)pcTaskName;
   /* Inspect pcTaskName in the debugger to see which task overflowed.
    * Halts in Error_Handler so the live state is preserved. Disable IRQs
    * first so we don't keep ticking IWDG and reset before inspection. */
   __disable_irq();
   Error_Handler();
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* pvPortMalloc returned NULL — likely libcanard pool exhaustion or task
    * creation failed. Treat as fatal during bring-up. */
   __disable_irq();
   Error_Handler();
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  //control_law_self_test();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Rectifier-link bring-up: only what rectifier_task needs.
   * Re-enable the others once the PCU↔VESC protocol is verified end-to-end. */
  pt_init();
  can_mgr_init();
  rectifier_task_start();
  supervisor_task_start();
  expt_task_start();
  dyno_setup_task_start();
  throttle_ctrl_task_start();
  // pdb_task_start();
  sensor_task_start();
  log_task_start();
  // fc_link_task_start();
  // bms_task_start();    /* Phase 1: no BMS connected — re-enable for Phase 3 */
  // ecu_task_start();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* supervisor_task is now the sole writer of pt_set_setpoint (running
   * the I_bat closed-loop — see USE_IBAT_CONTROL_LAW in supervisor_task.c)
   * and the sole watchdog kicker. defaultTask just stays alive. */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

