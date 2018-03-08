/**
  ******************************************************************************
  * @file    RCC/RCC_Example/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"
#include "lambda.h"
#include "list.h"
#include "motion.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup RCC_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern long frame;
extern volatile int jidx;
extern volatile int sv_prm;
extern volatile int sv_rtn;
extern volatile short sv_cmd_real;
extern int func_mess[50];
extern int func_mess_num;
extern int servo_enable;
extern long servo_mon_disp;
extern int servo_mon_req;
extern int zmp_walk_on;
extern int zmp_walk_data_ready;
//extern int zmp_walk_cont_on;
extern int get_zmp_busy;
extern int servo_cmd_err;
extern int servo_cmd_err_flag;

extern ListS walklog;

/* Private function prototypes -----------------------------------------------*/
void servo_anglelist_set(void);
int servo_cntl(void);
void servo_status_read(void);
int servo_parameter_cntl(void);
void tmcom_send_end(void);
void servo_ack_receive(rcv_size);
void servo_ack_receive_end(void);
void servo_ack_receive_timeout(void);
void svcom_receive(void);
void svcom_receive_reset(void);
void svcom_receive_abort(void);
void cntlspi_read(void);
void key_control(void);
void direct_command_control(void);
void batt_check(void);
void damage_check(void);
void led_blink(void);
void sensor_read(void);
void tmcom_watchdog(void);
void servo_data_set(void);
void walk(void);
void zmp_walk(void);
void zmp_walk_data_load(void);
void servo_disable(void);
void servo_setting_command_chech(void);
void store_log(void);
void exec_motion();
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
  /* This interrupt is generated when HSE clock fails */

  if (RCC_GetITStatus(RCC_IT_CSS) != RESET)
  {
    /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
       is selected as system clock source */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Enable HSE Ready and PLL Ready interrupts */
    RCC_ITConfig(RCC_IT_HSERDY | RCC_IT_PLLRDY, ENABLE);

    /* Clear Clock Security System interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_CSS);

    /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
       routine the system clock will be reconfigured to its previous state (before
       HSE clock failure) */
  }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles RCC interrupt request. 
  * @param  None
  * @retval None
  */
void RCC_IRQHandler(void)
{
  if(RCC_GetITStatus(RCC_IT_HSERDY) != RESET)
  { 
    /* Clear HSERDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_HSERDY);

    /* Check if the HSE clock is still available */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    { 
      /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */ 
      RCC_PLLCmd(ENABLE);     
    }
  }

  if(RCC_GetITStatus(RCC_IT_PLLRDY) != RESET)
  { 
    /* Clear PLLRDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_PLLRDY);

    /* Check if the PLL is still locked */
    if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
    { 
      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
  }
}


void DMA2_Stream4_IRQHandler(void){
  //ADC : Stream4
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
      DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    }
}

void DMA1_Stream1_IRQHandler(void)
{
  //USART3 RX DMA : Stream1
  // servo ack 受信完了割り込み
  if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) != RESET){
    DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
    servo_ack_receive_end();
  }
}

void DMA1_Stream2_IRQHandler(void)
{
  //UART4 RX DMA : Stream2
  // servo ack 受信完了割り込み
  if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) != RESET){
    DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
    servo_ack_receive_end();
  }
}

void DMA2_Stream7_IRQHandler(void)
{
  //USART1 TX DMA : Stream7
  if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET){
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
    //tmcom_send_end();
  }
}

void TIM2_IRQHandler(void)
{
  // 周期のタイマー割り込み
  static int batt_check_count = 0;
//  TIM2_IRQHANDELER;
  if( TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    frame++;
//    tmcom_watchdog();
    led_blink();
    if(servo_enable){
    	servo_status_read();
    }
    sensor_read();  //gyro acc read
    //walk();
	zmp_walk_data_load();
	exec_motion();
    store_log();
    GPIO_ResetBits(SPICS);  // SPI CS enable
    SPI_SSOutputCmd(SPI1, ENABLE);
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);  // SPI動作待ちタイマー
    if(batt_check_count > 500){
      //        batt_check();
      batt_check_count = 0;
    }
    batt_check_count++;
  }
}

void TIM3_IRQHandler(void)
{
  // SPI動作待ちタイマー
  // SPI読み取り後、サーボ通信開始
  //TIM3_IRQHANDELER;
  if( TIM_GetITStatus( TIM3, TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_Cmd(TIM3, DISABLE);
    cntlspi_read();
    GPIO_SetBits(SPICS);   // SPI CS disable
    SPI_SSOutputCmd(SPI1, DISABLE);
    //key_control();
    //direct_command_control();
    servo_data_set();  //足裏センサーによる足首制御など
    if(servo_enable){
    	///////////////////////////////////////////////////////////////////////////////////////////////////
    	servo_setting_command_chech();
    	///////////////////////////////////////////////////////////////////////////////////////////////////
    	jidx = 0;
		sv_prm = 1;
		servo_parameter_cntl();      // servoパラメータコマンド送信
		if(!sv_prm){
		  servo_cntl();              // servoコマンド送信(joint[jidx]の処理)
		}
    }else{
    	servo_disable();
    }
  }
}

void USART1_IRQHandler(void)
{
  // Terminal 送信終了割り込み
  if(USART_GetITStatus(USART1, USART_IT_TC) != RESET){
    USART_ClearITPendingBit(USART1, USART_IT_TC);
    tmcom_send_end();
  }
}

void USART3_IRQHandler(void)
{
  // SERVO COMMAND送信終了割り込み
  int rcv_size;
  if(USART_GetITStatus(USART3, USART_IT_TC) != RESET){
    USART_ClearITPendingBit(USART3, USART_IT_TC);
    if(jidx < SV_VOL){
      if(sv_cmd_real & 0x21 | SV_ROM ){
    	  if(joint[jidx].d[JOINT_ICS] == SV_ICS_3_0){
    		  rcv_size = 60;
    	  }else{
    		  rcv_size = 66;
    	  }
      }else{
    	  rcv_size = 3;
      }
      servo_ack_receive(rcv_size);
    }
  }
}

void UART4_IRQHandler(void)
{
  // SERVO COMMAND送信終了割り込み
  int rcv_size;
  if(USART_GetITStatus(UART4, USART_IT_TC) != RESET){
    USART_ClearITPendingBit(UART4, USART_IT_TC);
    if(jidx < SV_VOL){
      if(sv_cmd_real & 0x21 | SV_ROM ){
    	  if(joint[jidx].d[JOINT_ICS] == SV_ICS_3_0){
    		  rcv_size = 60;
    	  }else{
    		  rcv_size = 66;
    	  }
      }else{
    	  rcv_size = 3;
      }
      servo_ack_receive(rcv_size);
    }
  }
}

void TIM4_IRQHandler(void)
{
  // servo ack timeout監視用タイマー
  TIM4_IRQHANDELER;
  if( TIM_GetITStatus( TIM4, TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_Cmd(TIM4, DISABLE);
    svcom_receive_reset();  //UART受信停止
    servo_ack_receive_timeout();  //送信フラグが立ったままならtimeout
    jidx++;
    if(jidx < SV_VOL){
      if(sv_prm){
    	  servo_parameter_cntl();
      }
      if(!sv_prm){
        servo_cntl();           // 次のservoコマンド送信
      }
    }else{
    	if(servo_mon_req){
           	servo_mon_disp = frame;
    	}
    	if(servo_cmd_err){
    		servo_cmd_err_flag = 1;
    	}
    }
  }
}


/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
