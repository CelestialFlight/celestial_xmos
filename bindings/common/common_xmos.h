#ifndef __COMMON_XMOS_H__
#define __COMMON_XMOS_H__

#include "common.h"
#include <stdint.h>
#include <timer.h>
#include <xs1.h>

// Task for tracking system time.
// Note: This can be combined, but will likely be inaccurate.
[[combinable]]
void SystemTimeTask(void);

#endif
