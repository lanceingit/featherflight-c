#pragma once

#include "timer.h"

struct task_s;

typedef void(*task_callback_func)(void);

struct task_s 
{
    times_t rate;
    times_t time_use;
    times_t last_run;
    task_callback_func callback;
    bool run;
};

void task_create(struct task_s* t, times_t interval, task_callback_func cb);
void task_set_rate(struct task_s* t, times_t time);
void task_disable(struct task_s* t);

void sheduler_run(void);


