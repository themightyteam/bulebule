#ifndef __BUTTONS_H
#define __BUTTONS_H

#include <libopencm3/stm32/gpio.h>

bool button_read_user(void);
bool button_read_user_left(void);

#endif /* __BUTTONS_H */
