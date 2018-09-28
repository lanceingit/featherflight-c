/**
 *
 * timer.c
 *
 * simple timer, delay and time block function 
 */

#include "board.h"

#include "timer.h"


static volatile time_t timer_cnt = 0;


void timer_init()
{
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_DeInit(TIM7);
        
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
    TIM_TimeBaseStructure.TIM_Period = 10-1;                //10us 
    TIM_TimeBaseStructure.TIM_Prescaler = 72;       
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
    
    TIM_ARRPreloadConfig(TIM7, DISABLE);
    
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    TIM_Cmd(TIM7, ENABLE);
}


void timer_disable(void)
{
    NVIC_DisableIRQ(TIM7_IRQn);    
}


static void timer_irs(void)
{    
    TIM_ClearFlag(TIM7, TIM_IT_Update);
    timer_cnt++;
}

time_t timer_create(uint32_t us)
{
    return (timer_cnt + (us/10) - 1);
}


bool timer_is_timeout(time_t t)
{
    if(t >= timer_cnt)
    {
        return false;
    }
    else
    {
        return true;
    }
}

time_t timer_now()
{
	return timer_cnt*10;
}

time_t timer_elapsed(time_t* t)
{
	return timer_cnt*10 - *t;
}

void delay(float s)
{
    volatile time_t wait;

    wait = timer_create((uint32_t)(s*1000*1000));
    while (!timer_is_timeout(wait));
}

void delay_ms(uint32_t ms)
{
    volatile time_t wait;

    wait = timer_create(ms*1000);
    while (!timer_is_timeout(wait));
}

void delay_us(uint32_t us)
{
    volatile time_t wait;

    wait = timer_create(us);
    while (!timer_is_timeout(wait));
}
    
void sleep(float s)
{
    volatile time_t wait;

    wait = timer_create((uint32_t)(s*1000*1000));
    while (!timer_is_timeout(wait));
}

void TIM7_IRQHandler(void)
{
    timer_irs();
}


