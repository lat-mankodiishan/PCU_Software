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
  // supervisor_task_start();
  // pdb_task_start();
  // sensor_task_start();
  // log_task_start();
  // fc_link_task_start();
  // bms_task_start();
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
  /* Flight-stand-in profile: with no FC attached, walk a phase table that
   * mimics a representative mission. supervisor_task is disabled, so we are
   * the sole writer of I_rect_cmd_cA / mode. Linear-ramp between phases so
   * the rectifier_task / VESC see a continuously varying setpoint instead
   * of jumps. Replace this with fc_link_task once an FC is wired in. */
 // pt_set_bms_inputs(7000);   /* 70 % SOC, placeholder for bench */

  typedef struct {
      int16_t      I_cA;       /* I_rect_cmd_cA target at end of this phase */
      vesc_mode_t  mode;       /* mode held during the phase */
      uint32_t     ms;         /* duration of this phase (linear ramp from prev) */
  } phase_t;

  /* IDLE→TAKEOFF→CLIMB→CRUISE→LAND→IDLE, then loops. Edit numbers freely. */
  static const phase_t profile[] = {
      {    0, VESC_MODE_IDLE,     5000 },   /*  0..5  s  hold idle  */
      { 5000, VESC_MODE_TAKEOFF,  3000 },   /*  5..8  s  ramp 0  -> 50.00 A */
      { 5000, VESC_MODE_TAKEOFF,  2000 },   /*  8..10 s  hold 50 A          */
      { 3500, VESC_MODE_CLIMB,    3000 },   /* 10..13 s  ramp 50 -> 35 A    */
      { 3500, VESC_MODE_CLIMB,   12000 },   /* 13..25 s  hold 35 A (climb)  */
      { 2000, VESC_MODE_CRUISE,   5000 },   /* 25..30 s  ramp 35 -> 20 A    */
      { 2000, VESC_MODE_CRUISE,  25000 },   /* 30..55 s  hold 20 A (cruise) */
      {  500, VESC_MODE_LAND,     5000 },   /* 55..60 s  ramp 20 -> 5 A     */
      {  500, VESC_MODE_LAND,     5000 },   /* 60..65 s  hold 5 A (descent) */
      {    0, VESC_MODE_IDLE,     3000 },   /* 65..68 s  ramp 5 -> 0  A     */
  };
  const uint32_t N_PHASES = sizeof(profile) / sizeof(profile[0]);
  const uint32_t TICK_MS  = 10;             /* 100 Hz update */

  int16_t  prev_I = 0;                      /* setpoint at the start of phase[0] */
  uint32_t phase  = 0;
  uint32_t t_in   = 0;                      /* ms elapsed in current phase */

  for(;;)
  {
    watchdog_refresh();

    const phase_t *p = &profile[phase];
    int16_t I_cmd;
    if (p->ms == 0u) {
      I_cmd = p->I_cA;
    } else {
      /* linear interpolation prev_I -> p->I_cA over p->ms */
      int32_t num = (int32_t)(p->I_cA - prev_I) * (int32_t)t_in;
      I_cmd = (int16_t)((int32_t)prev_I + num / (int32_t)p->ms);
    }
    pt_set_setpoint(I_cmd, p->mode);

    osDelay(TICK_MS);
    t_in += TICK_MS;
    if (t_in >= p->ms) {
      prev_I = p->I_cA;
      t_in   = 0;
      phase  = (phase + 1u) % N_PHASES;
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

