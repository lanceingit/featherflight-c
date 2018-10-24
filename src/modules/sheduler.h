#pragma once

#include "timer.h"

struct task_s;

typedef void(*task_callback_func)(struct task_s*);

struct task_s 
{
    times_t next_time;
    times_t time_use;
    task_callback_func callback;
    bool run;
};

void task_create(struct task_s* t, times_t interval, task_callback_func cb);
void task_set_next(struct task_s* t, times_t time);
void task_disable(struct task_s* t);

void sheduler_run(void);


