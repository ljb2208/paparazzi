/*
 * $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*              lisa/L  lisa/M
 *  ACC_DRDY     PD2     PB2
 *  ACC_SS       PB12    PB12
 *
 */


#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "sys_time.h"
#include "downlink.h"

#include "modules/sensors/alt_srf08.h"
//#include "my_debug_servo.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

void exti2_irq_handler(void);

bool_t ins_update_on_agl;

int main(void) {
  main_init();

  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  main_init_hw();

}

static inline void main_periodic_task( void ) {

	RunOnceEvery(10,
	    {
	      DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
	      LED_PERIODIC();
	      srf08_periodic();

	    });
}


static inline void main_event_task( void ) {

}

static inline void main_init_hw( void ) {

	srf08_init();

}

