/**
 *
 * timer.h
 *
 * simple timer, delay and time block function 
 */


#pragma once

typedef uint64_t time_t;

void timer_init(void);
void timer_disable(void);

time_t timer_create(uint32_t us);
bool timer_is_timeout(time_t t);

time_t timer_now(void);
time_t timer_elapsed(time_t* t);

void delay(float s);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void sleep(float s);





     

