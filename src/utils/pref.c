#include "pref.h"

void pref_init(struct Pref* pref)
{
    pref->t0=0;
    pref->t1 = 0;
    pref->cnt=0;
    pref->avg=0;
    pref->sum=0;
    pref->t_max=0;
    pref->t_min=0xFFFFFFFF;
}

void pref_interval(struct Pref* pref)
{
    pref->t1 = Timer_getTime() - pref->t0;
    if(pref->t0 != 0)
    {
        pref->sum += pref->t1;
        pref->cnt++;
        pref->avg = pref->sum / pref->cnt;
        if(pref->t1>pref->t_max) pref->t_max=pref->t1;
        if(pref->t1<pref->t_min) pref->t_min=pref->t1;
    }
    pref->t0 = Timer_getTime();
}

void pref_begin(struct Pref* pref)
{
    pref->t0 = Timer_getTime();
}

void pref_end(struct Pref* pref)
{
    pref->t1 = Timer_getTime() - pref->t0;
    if(pref->t0 != 0)
    {
        pref->sum += pref->t1;
        pref->cnt++;
        pref->avg = pref->sum / pref->cnt;
        if(pref->t1>pref->t_max) pref->t_max=pref->t1;
        if(pref->t1<pref->t_min) pref->t_min=pref->t1;
    }
}
