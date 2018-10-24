#include "board.h"
#include "sheduler.h"

void task_create(struct task_s* t, times_t interval, task_callback_func cb)
{
    t->callback = cb;
    t->next_time = interval;    
    t->run = true;
}

void task_set_next(struct task_s* t, times_t time)
{
    t->next_time = time;
}

void task_disable(struct task_s* t)
{
    t->run = false;
}

void sheduler_run(void)
{

}
