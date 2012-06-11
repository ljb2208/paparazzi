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

#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

#include "lisa/lisa_overo_link.h"


static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

static inline void main_on_overo_msg_received(void);
static inline void main_on_overo_link_lost(void);
static inline void main_on_overo_link_error(void);

static uint32_t spi_msg_cnt = 0;
//static uint16_t spi_errors = 0;

extern void exti2_irq_handler(void);
extern void exti3_irq_handler(void);
extern void exti4_irq_handler(void);

#define OVERO_LINK_LED_OK 1
#define OVERO_LINK_LED_KO 2

int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  main_init_hw();

  overo_link_init();

}

static inline void main_periodic_task( void ) {

  RunOnceEvery(10,
	       {
		 DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
		 LED_PERIODIC();
	       });

  OveroLinkPeriodic(main_on_overo_link_lost)

}


static inline void main_event_task( void ) {
	OveroLinkEvent(main_on_overo_msg_received,main_on_overo_link_error);

}

static inline void main_init_hw( void ) {



}

static inline void main_on_overo_msg_received(void) {

/*  overo_link.up.msg.bench_sensor.x = bench_sensors.angle_1;
  overo_link.up.msg.bench_sensor.y = bench_sensors.angle_2;
  overo_link.up.msg.bench_sensor.z = bench_sensors.angle_3;

  overo_link.up.msg.accel.x = imu.accel_unscaled.x;
  overo_link.up.msg.accel.y = imu.accel_unscaled.y;
  overo_link.up.msg.accel.z = imu.accel_unscaled.z;

  overo_link.up.msg.gyro.p = imu.gyro_unscaled.p;
  overo_link.up.msg.gyro.q = imu.gyro_unscaled.q;
  overo_link.up.msg.gyro.r = imu.gyro_unscaled.r;

  //can_err_flags (uint16) represents the board number that is not communicating regularly
  //spi_errors (uint16) reflects the number of crc errors on the spi link
  //TODO: if >10% of messages are coming in with crc errors, assume something is really wrong
  //and disable the motors.
  overo_link.up.msg.can_errs = can_err_flags;*/


  //spi_msg_cnt shows number of spi transfers since last link lost event
  overo_link.up.msg.stm_msg_cnt = spi_msg_cnt++;

}


static inline void main_on_overo_link_lost(void) {
  //actuators_set(FALSE);
  spi_msg_cnt = 0;
}

static inline void main_on_overo_link_error(void) {

}

void exti2_irq_handler(void) {

}

void exti3_irq_handler(void) {


}

void exti4_irq_handler(void) {


}

