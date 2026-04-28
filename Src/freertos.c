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
#include "usart.h"
#include "tim.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    int error;          // 当前误差
    int last_error;     // 上一次误差
    int prev_error;     // 上上一次误差
    int pwm_output;     // 最终输出的 PWM 占空比
} PID_TypeDef;


PID_TypeDef left_pid = {0.3, 0.05, 0.0, 0, 0, 0, 0};
PID_TypeDef right_pid = {1.2, 0.8, 0.1, 0, 0, 0, 0};
/* USER CODE END PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart1;
uint8_t aRxBuffer;                // 我们的接收缓冲区（存1个字节）
int target_speed = 0;
int car_dir = 0;
int left_speed = 0;  // 把这里改成全局变量
int left_pwm = 0;    // 专门用来存 PID 计算
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId SensorTaskHandle;
osThreadId CommTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int PID_Compute(PID_TypeDef *pid, int target, int actual);
void Send_Robot_State(int16_t l_speed, int16_t r_speed);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartMotorTask(void const * argument);
void StartSensorTask(void const * argument);
void StartCommTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of CommTask */
  osThreadDef(CommTask, StartCommTask, osPriorityAboveNormal, 0, 512);
  CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
  }

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;)
  {
    // 1. 先判断方向，控制四个引脚的高低电�?
    if (car_dir == 1) {
        // 前进：左轮正�? (AIN1=1, AIN2=0)，右轮正�? (BIN1=1, BIN2=0)
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else if (car_dir == -1) {
        // 后�??：左轮反�? (AIN1=0, AIN2=1)，右轮反�? (BIN1=0, BIN2=1)
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else {
        // 停止：四个引脚全�? 0，电机自由滑行停�?
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }

    // 2. 踩下油门 (输出 PWM 动力)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm); // 控制左轮
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, target_speed); // 控制右轮

    osDelay(10);
  }
  /* USER CODE END StartMotorTask */
}
/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动左轮测速
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动右轮测速

  /* Infinite loop */
  for(;;)
  {
      // 1. 获取真实速度
      left_speed = - (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
      __HAL_TIM_SET_COUNTER(&htim3, 0);

      // 2. 大脑开始计算
      left_pwm = PID_Compute(&left_pid, target_speed, left_speed);

      osDelay(50); // 50ms 算一次
  }
  /* USER CODE END StartSensorTask */
}
/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */

  HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
  printf("\r\n=== start ===\r\n");
  int print_counter = 0; // 声明一个小计数器

  /* Infinite loop */
  for(;;)
  {
      if(aRxBuffer == 'G') {
          target_speed = 30;
          car_dir = 1;
          aRxBuffer = 0;
          printf("收到前进指令！目标速度：%d\r\n", target_speed); // 这句如果不看终端可以删掉
      }
      else if(aRxBuffer == 'S') {
          target_speed = 0;
          car_dir = 0;
          aRxBuffer = 0;
      }
      else if(aRxBuffer == 'B') {
          target_speed = 30;
          car_dir = -1;
          aRxBuffer = 0;
      }


      if (target_speed != 0 || left_speed != 0) {
          print_counter++;
          if (print_counter >= 2) {

              Send_Robot_State(left_speed, 0); // 发送数据帧给上位机

              print_counter = 0;
          }
      }


      osDelay(20);

  }
  /* USER CODE END StartCommTask */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// 1. PID 计算函数
int PID_Compute(PID_TypeDef *pid, int target, int actual) {
    pid->error = target - actual;
    int increment = pid->Kp * (pid->error - pid->last_error)
                  + pid->Ki * pid->error
                  + pid->Kd * (pid->error - 2 * pid->last_error + pid->prev_error);
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    pid->pwm_output += increment;
    if(pid->pwm_output > 5000) pid->pwm_output = 5000;
    if(pid->pwm_output < 0) pid->pwm_output = 0;
    return pid->pwm_output;
}

// 2. 向上位机发送十六进制帧函数
void Send_Robot_State(int16_t l_speed, int16_t r_speed) {
    uint8_t tx_buf[8];
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = (l_speed >> 8) & 0xFF;
    tx_buf[3] = l_speed & 0xFF;
    tx_buf[4] = (r_speed >> 8) & 0xFF;
    tx_buf[5] = r_speed & 0xFF;
    tx_buf[6] = tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5];
    tx_buf[7] = 0x0A;
    HAL_UART_Transmit(&huart1, tx_buf, 8, 10);
}

// 3. 串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
    }
}
/* USER CODE END Application */
