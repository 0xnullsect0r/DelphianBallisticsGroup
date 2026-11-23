#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
typedef unsigned int uint;
typedef uint64_t absolute_time_t;
absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t t);
void gpio_put(uint pin, bool v); bool gpio_get(uint pin);
void gpio_init(uint p); void gpio_set_dir(uint p, bool o);
void sleep_ms(uint32_t ms);
#define GPIO_OUT 1
#define tight_loop_contents() ((void)0)
