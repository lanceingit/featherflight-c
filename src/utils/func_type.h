#pragma once

#include <stdbool.h>
#include "rotation.h"

typedef bool (init_func)(void);
typedef bool (init_func_rocation)(enum Rotation r);
typedef void (update_func)(void);
typedef bool (read_status_func)(void);
typedef void (set_status_func)(void);

typedef void (run_func)(void);
