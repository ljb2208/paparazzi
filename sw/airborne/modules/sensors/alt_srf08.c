/*
 * $Id$
 *
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2002  Chris efstathiou hendrix@otenet.gr
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
 *
 */
/** \file srf08.c
 *  \brief Basic library for SRF08 telemeter
 *
 */

#include "mcu_periph/i2c.h"
#include "alt_srf08.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "telemetry.h"
#include "downlink.h"
#include "led.h"
#include "subsystems/ins.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef SRF08_I2C_DEV
#define SRF08_I2C_DEV i2c2
#endif

/* Global Variables */
bool_t srf08_received, srf08_got, srf08_range_requested, srf08_begin_receive;

struct AltSrf08 altSrf08;
struct i2c_transaction srf_trans;
uint16_t srf08_range, srf08_delay_count;

uint16_t sonar_meas;

//bool_t ins_update_on_agl;


/*###########################################################################*/

void srf08_init(void)
{
	altSrf08.data_ready = 0;
	altSrf08.srf08_range = 0;
	altSrf08.status = srf08Uninit;
	srf08_range_requested = FALSE;
	srf08_begin_receive = FALSE;
  srf08_received = FALSE;
  srf08_got = FALSE;
  srf08_delay_count = 0;

  srf_trans.buf[0] = 0x00;
  srf_trans.buf[1] = 0x51;
  I2CTransmit(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 2);

  /* set gain to specified value */
  srf_trans.buf[0] = SRF08_SET_GAIN;
  srf_trans.buf[1] = SRF08_GAIN;
  I2CTransmit(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 2);

  /* set gain to specified value */
  srf_trans.buf[0] = SRF08_SET_RANGE;
  srf_trans.buf[1] = SRF08_RANGE;
  I2CTransmit(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 2);

  altSrf08.status = srf08Ready;
  return;
}
/*###########################################################################*/

void srf08_initiate_ranging(void) {
LED_ON(2);
  srf_trans.buf[0] = SRF08_COMMAND;
  srf_trans.buf[1] = SRF08_CENTIMETERS;
  I2CTransmit(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 2);
}

/** Ask the value to the device */
void srf08_receive(void) {
	LED_OFF(2);
  srf_trans.buf[0] = SRF08_ECHO_1;
  //srf08_received = TRUE;
  I2CTransmit(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 1);
}

/** Read values on the bus */
void srf08_read(void) {
  I2CReceive(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 2);
}

/** Copy the I2C buffer */
void srf08_copy(void) {
  altSrf08.srf08_range = srf_trans.buf[0] << 8 | srf_trans.buf[1];
  sonar_meas = altSrf08.srf08_range;
}

void srf08_ping()
{
  srf08_initiate_ranging();
  while (srf_trans.status != I2CTransSuccess){
	  if (srf_trans.status == I2CTransFailed) {
		  srf08_initiate_ranging();
	  }
  }
;  /* blocking */

  srf08_receive();
}
/*###########################################################################*/

uint32_t srf08_read_register(uint8_t srf08_register)
{
  uint8_t cnt;

  union i2c_union {
    uint32_t  rx_word;
    uint8_t   rx_byte[2];
  } i2c;


  srf_trans.buf[0] = srf08_register;

  /* get high byte msb first */
  if (srf08_register>=2)
    cnt = 2;
  else
    cnt = 1;

  I2CTransceive(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 1, cnt);

  /* get high byte msb first */
  if(srf08_register>=2) {
    i2c.rx_byte[1]=srf_trans.buf[1];
  }

  /* get low byte msb first  */
  i2c.rx_byte[0]=srf_trans.buf[0];

  return(i2c.rx_word);
}

void srf08_get_swrevision(void) {
	LED_OFF(2);
  srf_trans.buf[0] = SRF08_COMMAND;
  //srf08_received = TRUE;
  I2CTransmit(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 1);
}

void srf08_read_swrevision(void) {
	 I2CReceive(SRF08_I2C_DEV, srf_trans, SRF08_UNIT_0, 2);
}

void srf08_periodic(void) {

	if (altSrf08.status == srf08RangeRequested) {
		RunOnceEvery(25, srf08_request_range());
	} else	{
		srf08_request_range();
	}
}

void srf08_request_range(void) {

	switch(altSrf08.status)
	{
	case srf08Uninit:
		return;
	case srf08Ready:
		altSrf08.status = srf08RangeRequested;
		srf08_initiate_ranging();
		break;
	case srf08RangeRequested:
		// check to ensure range request completed succesfully
		if (srf_trans.status == I2CTransSuccess){
			altSrf08.status = srf08Ranging;
			srf08_receive();
			altSrf08.srf08_range_failure_count = 0;
		} else if (srf_trans.status == I2CTransFailed) {
			altSrf08.status = srf08Ready;
			altSrf08.srf08_i2c_error_count++;
			ins_update_on_agl = FALSE;
		}
		break;
	case srf08Ranging:
		if (srf_trans.status == I2CTransSuccess){
			altSrf08.status = srf08Reading;
			srf08_read();
		} else if (srf_trans.status == I2CTransFailed) {
			altSrf08.status = srf08Ready;
			altSrf08.srf08_i2c_error_count++;
			ins_update_on_agl = FALSE;
		}
		break;
	case srf08Reading:
		// check to ensure ranging completed succesfully
		if (srf_trans.status == I2CTransSuccess){

			if (srf_trans.buf[0] != 0xFF || srf_trans.buf[1] != 0xFF) {
				// range returned
				srf08_copy();
				altSrf08.status = srf08Ready;
				srf08_got = TRUE;
				ins_update_on_agl = TRUE;
				ins_update_sonar();
				//PERIODIC_SEND_SONAR(DefaultChannel, &altSrf08.srf08_range, &ins_update_on_agl);
			} else {
				// no range available so re-request
				srf08_read();
			}
		} else if (srf_trans.status == I2CTransFailed) {
			altSrf08.srf08_i2c_error_count++;

			if (altSrf08.srf08_range_failure_count > MAX_I2C_RANGE_FAILURE_COUNT) {
				ins_update_on_agl = FALSE; // do not update in INS as ranging has failed.
				altSrf08.status = srf08Ready;
				return;
			}

			altSrf08.srf08_range_failure_count++;
		}
		break;
	default:
		break;
	}
}
