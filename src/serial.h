#ifndef __SERIAL_H
#define __SERIAL_H

#include <string.h>

#include <libopencm3/stm32/usart.h>

#include "calibration.h"
#include "logging.h"

void execute_commands(void);

#endif /* __SERIAL_H */
