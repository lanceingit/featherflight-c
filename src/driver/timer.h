/**
 *
 * timer.h
 *
 * simple timer, delay and time block function 
 */


#pragma once

typedef uint64_t times_t;

void timer_init(void);
void timer_disable(void);

times_t timer_new(uint32_t us);
bool timer_is_timeout(times_t t);

times_t timer_now(void);
times_t timer_elapsed(times_t* t);

void delay(float s);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
#ifdef F3_EVO
void sleep(float s);
#endif





     

