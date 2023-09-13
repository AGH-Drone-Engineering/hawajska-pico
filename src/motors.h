#ifndef _APP_MOTORS_H_
#define _APP_MOTORS_H_

#include <stdint.h>

void motors_init(void);

void motors_set(uint8_t left, uint8_t right);

#endif
