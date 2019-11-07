#include "buttons.h"

/**
 * @brief Function to read user button.
 */
bool button_read_user(void)
{
	return (bool)(gpio_get(GPIOA, GPIO12));
}


/**
 * @brief Function to read user button.
 */
bool button_read_user_left(void)
{
	return (bool)(gpio_get(GPIOA, GPIO11));
}
