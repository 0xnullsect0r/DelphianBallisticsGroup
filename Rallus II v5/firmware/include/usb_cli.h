#pragma once
#include <stdint.h>

// USB CDC command line, driven from the core 1 loop.
void usb_cli_init(void);
void usb_cli_poll(uint32_t now_ms);
