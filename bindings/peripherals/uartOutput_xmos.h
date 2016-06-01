#ifndef __UART_OUTPUT_H__XMOS__
#define __UART_OUTPUT_H__XMOS__

#include "uartOutput.h"

[[combinable]]
void UartOutputTask(struct UartOutput* output, port outputPort);

#endif
