#pragma once

struct Pref
{
    uint64_t t0;
    uint64_t t1;
    uint64_t cnt;
    uint64_t avg;
    uint64_t sum;
    uint64_t t_max;
    uint64_t t_min;
};

void pref_init(struct Pref* pref);
void pref_interval(struct Pref* pref);
void pref_begin(struct Pref* pref);
void pref_end(struct Pref* pref);

