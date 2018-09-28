#pragma once

#include "timer.h"

struct perf_s
{
    time_t t0;
    time_t t1;
    uint64_t cnt;
    time_t avg;
    time_t sum;
    time_t t_max;
    time_t t_min;
};

void perf_init(struct perf_s* perf);
void perf_interval(struct perf_s* perf);
void perf_begin(struct perf_s* perf);
void perf_end(struct perf_s* perf);

