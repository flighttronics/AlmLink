/******************************************************************************
 gpio.h for Almlink

 AlmLink uses the rfm23bp radio module to transmit and receive data

 Copyright (C)2013, Andreas Kingbaeck  (andki234@gmail.com)
 Copyright (C)2012, Derek Molloy, School of Electronic Engineering, DCU  (www.derekmolloy.ie)
 Copyright (C)2011, RidgeRun

 Modifications by Andreas Kingbäck
 Modifications by Derek Molloy, School of Electronic Engineering, DCU  (www.derekmolloy.ie)
 Base Software by RidgeRun

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
 * @file gpio.h
 * GPIO header file for GPIO driver
 * @brief This section contains GPIO headers
 *
 * @author Andreas Kingbäck
 * @version 0.00
 */

#ifndef SIMPLEGPIO__H_
#define SIMPLEGPIO__H_

/****************************************************************
 * Constants
 ****************************************************************/

#define SYSFS_GPIO__DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

typedef enum
{
    INPUT_PIN = 0, OUTPUT_PIN = 1
} T_PIN_DIRECTION;

typedef enum
{
    LOW = 0, HIGH = 1
} T_PIN_VALUE;

/****************************************************************
 * GPIO__export
 ****************************************************************/
int GPIO__Export(unsigned int gpio);
int GPIO__Unexport(unsigned int gpio);
int GPIO__SetDir(unsigned int gpio, T_PIN_DIRECTION out_flag);
int GPIO__SetValue(unsigned int gpio, T_PIN_VALUE value);
int GPIO__GetValue(unsigned int gpio, unsigned int *value);
int GPIO__SetEdge(unsigned int gpio, char *edge);
int GPIO__Open(unsigned int gpio);
int GPIO__Close(int fd);

#endif /* SIMPLEGPIO__H_ */
