#pragma once

#include "timer.h"

struct perf_s
{
    times_t t0;
    times_t t1;
    uint64_t cnt;
    times_t avg;
    times_t sum;
    times_t t_max;
    times_t t_min;
};

void perf_init(struct perf_s* perf);
void perf_interval(struct perf_s* perf);
void perf_begin(struct perf_s* perf);
void perf_end(struct perf_s* perf);
void perf_print(struct perf_s* perf, char* name);

