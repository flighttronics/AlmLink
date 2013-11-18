/*
 * gpio.h
 *
 *  Created on: 1 dec 2012
 *      Author: Andreas Kingbäck
 */
/******************************************************************************
 spi.h for Allmogetracker 3

 Allmogetracker receives data from a GPS receiver and transmits it
 over amature radio using the APRS protocol version 1.0.

 Copyright (C)2012, Andreas Kingbäck  (andki234@gmail.com)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ******************************************************************************/

/**
 * @file spi.h
 * SPI driver headers
 * @brief This section contains headers for SPI driver
 *
 * @author Andreas Kingbäck
 * @version 0.00
 */

//-------------------------------------------------------------------------------------------------
//--------  I N C L U D E -------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <linux/spi/spidev.h>
#include "../drivers.h"

//-------------------------------------------------------------------------------------------------
//--------  D E F I N E S  ------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//--------  E N U M S  ----------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//--------  F U N C T I O N S  --------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
T_STATUS SPI__Open(void);
T_STATUS SPI__Close(void);
T_STATUS SPI__Transfer(uint8_t *tx, uint8_t *rx, uint16_t txrx_length);
T_STATUS SPI__SetParameters(void);

