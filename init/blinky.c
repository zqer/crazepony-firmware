#include "stm32f10x.h"
#include "led.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "ledseq.h"

#include "i2c_gpio.h"
#include "mpu6050_periodic.h"

//#include "stdio.h"

#define period 7999   /*Counting number in counter*/
#define prescale 8999 /*Prescale value of the clock*/

static bool isInit;

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;

void vApplicationStackOverflowHook(){
}
void debugSendTraceInfo() {
}

/**
BlinkInit is a function to initialize the port PB5. Port mode is push-pull output.
**/
void BlinkInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | LED_GPIO_PERIF, ENABLE);

  // Remap PB4
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);

  //Initialize the LED pins as an output
  GPIO_InitStructure.GPIO_Pin = LED_GPIO_GREEN | LED_GPIO_RED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOB,GPIO_Pin_4);
  GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

void Delay(uint32_t times){
    for(;times>0;times--);
}

/**
*TIM2_Configuration is to configure TIM2 as a counting clock to generate time interrrupts.
**/
void TIM2_Configuration(){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); /*Enable the clock of TIM2*/
    TIM_TimeBaseInitTypeDef TIM_Structure;  /*define a timer variable*/
    TIM_DeInit(TIM2);           /*reset TIM2*/
/**
    output frequency is sysclok/((period+1)*(prescale+1))
**/
    TIM_Structure.TIM_Period =period;
    TIM_Structure.TIM_Prescaler =prescale;
    TIM_Structure.TIM_ClockDivision = TIM_CKD_DIV1; /* source clock without division*/
    TIM_Structure.TIM_CounterMode = TIM_CounterMode_Up;/*count up*/
    TIM_Structure.TIM_RepetitionCounter = 0; /*update every time when counter overflows*/
    TIM_TimeBaseInit(TIM2,&TIM_Structure);  /*initialize interrupt*/
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);    /*clear TIM2 interrupt flag*/
    TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE); /*8enable TIM2 interrupt*/
    TIM_Cmd(TIM2,ENABLE);           /*TIM2 switch on*/
}
/**
*NVIC_Configuration is a function to configure interrupt vector control, so that interrupt function can match it.
**/
void NVIC_Configuration(){
    NVIC_InitTypeDef NVIC_InitStructure;            /*Define a NVIC configuration variable*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   /*there is only one interrupt, so choose group 0*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   /*channel TIM2 enable*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; /*Preemption priority*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  /*Sub priority*/
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**
* TIM2_IRQHandler is the interrupt function which will execute once when each internal time interrupt occurs.
**/
void TIM2_IRQHandler(void){
    static u8 state=1;      /*led state, 0 is off, 1 is on*/
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET){
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
            if(state==1){   /* the previous state is on*/
                GPIO_SetBits(GPIOB,GPIO_Pin_4); /*set PB5 high, led off*/
                state=0;
            }
            else{   /* the precvious state is off*/
                GPIO_ResetBits(GPIOB,GPIO_Pin_4);/*set PB5 low, led on*/
                state=1;
            }
    } /*interrupt occurs*/
}

void MotorInit2(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    uint16_t PrescalerValue = 0;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);



    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    TIM_DeInit(TIM2);


    PrescalerValue = (uint16_t) (72000000 / 24000000) - 1;

    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);


    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2,&TIM_OCInitStructure);
    TIM_OC2Init(TIM2,&TIM_OCInitStructure);
    TIM_OC3Init(TIM2,&TIM_OCInitStructure);
    TIM_OC4Init(TIM2,&TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_Cmd(TIM2,ENABLE);

    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
}

static void systemTask3(void *arg);

void MySystemInit(){
/*HSEStartUpStatus checks whether HSE is enabled*/
    ErrorStatus HSEStartUpStatus;
/*Reset all RCC registers to reset values, details in manual reference, in default HSI is on*/
    RCC_DeInit();
/*HSEON,PLLON, CSSON, HSEBYP PLLXTPRE=1, PLLXTPRE is written only when PLL is disabled*/
    RCC_HSEConfig(RCC_HSE_ON); /*Set HSE on*/
    HSEStartUpStatus=RCC_WaitForHSEStartUp();/*Wait until HSE is ready, HSERDY*/
    if(HSEStartUpStatus==SUCCESS){      /*HSE is ready*/

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
        while(RCC_GetSYSCLKSource()!=0x08){
        }/*wait until SYSCLK is ready*/

/*Configure AHB, APB1, APB2 output, APB1 is divided by 2*/
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* Set the Vector Table base address at 0x08000000 */
        NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
        NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
        /* Configure HCLK clock as SysTick clock source. */
        SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
    }

}


extern void motorsTestTask(void* params);
void main(){
    MySystemInit();
        initUsecTimer();
        nvicInit();
        //MotorInit2();
    //BlinkInit();
        ledInit();
        motorsInit();

        TIM2->CCR1 = 0;
        TIM2->CCR2 = 0;
        TIM2->CCR3 = 0;
        TIM2->CCR4 = 0;

    /* NVIC_Configuration();
    TIM2_Configuration(); */


        xTaskCreate(systemTask3, (const signed char * const)"SYSTEM",
              configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

        /* xTaskCreate(motorsTestTask, (const signed char * const)"MOTORS",
              configMINIMAL_STACK_SIZE, NULL, 2, NULL); */


/*  while(1){
        GPIO_SetBits(GPIOB,GPIO_Pin_5); //set PB5 high, led off
        Delay(7000000);
        GPIO_ResetBits(GPIOB,GPIO_Pin_5);//set PB5 low, led on
        Delay(7000000);
    }
*/
    vTaskStartScheduler();
    while(1);
}

void systemInit2(void)
{

  //canStartMutex = xSemaphoreCreateMutex();
  //xSemaphoreTake(canStartMutex, portMAX_DELAY);

  configblockInit();
  workerInit();
  /* adcInit(); ignore ADC by jannson */
  ledseqInit();
  /* pmInit(); ignore power management by jannson */

}

void systemStart()
{
  //xSemaphoreGive(canStartMutex);
}

void systemTask3(void *arg)
{
    //ledSet(LED_RED, 0);

    systemInit2();

    uartInit();

    crtpInit();
    crtpserviceInit();
    consoleInit();
    paramInit();

    DEBUG_PRINT("Crazyflie is up and running!\n");

    //ledseqRun(LED_RED, seq_alive);
    //ledseqRun(LED_GREEN, seq_testPassed);

#if 0
    while(!MPU6050_Check()) {
        ledSet(LED_RED, 1);
        Delay(7000000);
        ledSet(LED_RED, 0);
        Delay(1000000);
    }

    mpu6050Reset();
    vTaskDelay(M2T(50));
    mpu6050SetSleepEnabled(FALSE);
    mpu6050SetTempSensorEnabled(TRUE);
    mpu6050SetIntEnabled(FALSE);
#endif

    stabilizerInit();

    systemStart();

    workerLoop();

    while(1)
        vTaskDelay(portMAX_DELAY);
}

