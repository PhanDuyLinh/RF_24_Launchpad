
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#include <stddef.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define _BV(x) (1<<(x))

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define IF_SERIAL_DEBUG(x) {x;}
#else
#define IF_SERIAL_DEBUG(x)
#endif

typedef uint16_t prog_uint16_t;
#define PSTR(x) (x)
#define printf_P UARTprintf
#define strlen_P strlen
#define PROGMEM
#define pgm_read_word(p) (*(p)) 
#define pgm_read_byte(p) (*(p))
#define PRIPSTR "%s"

#endif // __RF24_CONFIG_H__
// vim:ai:cin:sts=2 sw=2 ft=cpp
