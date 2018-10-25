#include "board.h"
#include "scheduler.h"

#define TASK_MAX     20

static struct task_s* task_tab[TASK_MAX];
static uint8_t task_cnt=0;

void task_create(struct task_s* t, times_t interval, task_callback_func cb)
{
    if(task_cnt >= TASK_MAX) return;

    t->callback = cb;
    t->rate = interval;    
    t->last_run = 0;
    t->run = true;

    task_tab[task_cnt++] = t;
}

void task_set_rate(struct task_s* t, times_t time)
{
    t->rate = time;
}

void task_disable(struct task_s* t)
{
    t->run = false;
}

void scheduler_run(void)
{
    for(uint8_t i=0; i<task_cnt; i++) {
        if(task_tab[i]->run) {
//            printf("task:%d ",i);
            if(timer_check(&task_tab[i]->last_run, task_tab[i]->rate)) {
                task_tab[i]->callback();
                // printf("run \n");
            } else {
                // printf("wait \n");
            }        
        }
    }
}
