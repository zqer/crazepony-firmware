/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * main.c - Containing the main function.
 */

/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "config.h"
#include "system.h"
#include "nvic.h"
#include "usec_time.h"

#include "led.h"

/* ST includes */
#include "stm32f10x.h"

/* Private functions */
static void prvClockInit(void);

#if 0
void vApplicationStackOverflowHook(){
}
void debugSendTraceInfo() {
}
#endif

int main()
{
  //Low level init: Clock and Interrupt controller
  prvClockInit();
  nvicInit();
  initUsecTimer();
  ledInit();
  motorsInit();

  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
  TIM2->CCR3 = 0;
  TIM2->CCR4 = 0;

  //Launch the system task that will initialize and start everything
  systemLaunch();

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  //Should never reach this point!
  while(1);

  return 0;
}

//Clock configuration
static void prvClockInit(void)
{
  ErrorStatus HSEStartUpStatus;

  RCC_DeInit();
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
        /*As boot mode is selected, flash memory is used as boot space*/
        FLASH->ACR |= FLASH_ACR_PRFTBE;  /*Enable prefetch buffer*/
        FLASH->ACR &= ((uint32_t)~FLASH_ACR_LATENCY); /*flash 2 latency cycles as working requency <=72MHz, larger than 48MHz*/
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
        /*PLLMULL is enabled, PLLXTPRE and PLLSRC select HSE/2 as the PLL input frequency, we have 16MHz OSC, in order to get 72MHz SYSCLK, we first
            divide HSE by 2 then PLLMUL is set to 9, 16/2*9=72MHz*/
        RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);

        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){
        }/*wait until PLL is ready*/

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);/*SW selects PLLCLK as SYSCLK*/
        while(RCC_GetSYSCLKSource() != 0x08){
        }/*wait until SYSCLK is ready*/

/*Configure AHB, APB1, APB2 output, APB1 is divided by 2*/
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* Set the Vector Table base address at 0x08000000 */
        NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
        /* NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); */
        /* Configure HCLK clock as SysTick clock source. */
        SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
  } else {
      GPIO_InitTypeDef GPIO_InitStructure;

    //Cannot start the main oscillator: red/green LED of death...
    GPIO_InitStructure.GPIO_Pin = LED_GPIO_RED | LED_GPIO_GREEN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, LED_RED);
    GPIO_ResetBits(GPIOB, LED_GREEN);

    //Cannot start xtal oscillator!
    while(1);
  }
}

void SystemInit(){

}
