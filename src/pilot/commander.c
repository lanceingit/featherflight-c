#include "board.h"
#include "commander.h"

struct commander_s
{
    bool aremd;
};


struct commander_s commander = {
    .aremd = false,
};

struct commander_s* this = &commander;

bool system_armed(void)
{
    return this->aremd;
}

void commander_update(void)
{
    
}
