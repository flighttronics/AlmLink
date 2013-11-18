/******************************************************************************
 gpio.c for Almlink

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
 * @file gpio.c
 * GPIO driver for RFM23BP
 * @brief This section contains GPIO code
 *
 * @author Andreas Kingbäck
 * @version 0.00
 */

/*
 * SimpleGPIO.cpp
 *
 * Modifications by Derek Molloy, School of Electronic Engineering, DCU
 * www.derekmolloy.ie
 * Almost entirely based on Software by RidgeRun:
 *
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include "gpio.h"

/****************************************************************
 * GPIO__export
 ****************************************************************/
int GPIO__Export(unsigned int gpio)
{
    int fd, len;
    char buf[MAX_BUF];

    fd = open(SYSFS_GPIO__DIR "/export", O_WRONLY);

    if (fd < 0)
    {
        perror("gpio/export");

        return fd;
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);

    write(fd, buf, len);

    close(fd);

    return 0;
}

/****************************************************************
 * GPIO__unexport
 ****************************************************************/
int GPIO__Unexport(unsigned int gpio)
{
    int fd, len;
    char buf[MAX_BUF];

    fd = open(SYSFS_GPIO__DIR "/unexport", O_WRONLY);

    if (fd < 0)
    {
        perror("gpio/export");
        return fd;
    }


    len = snprintf(buf, sizeof(buf), "%d", gpio);

    write(fd, buf, len);

    close(fd);

    return 0;
}

/****************************************************************
 * GPIO__set_dir
 ****************************************************************/
int GPIO__SetDir(unsigned int gpio, T_PIN_DIRECTION out_flag)
{
    int fd;
    char buf[MAX_BUF];

    snprintf(buf, sizeof(buf), SYSFS_GPIO__DIR "/gpio%d/direction", gpio);

    fd = open(buf, O_WRONLY);

    if (fd < 0)
    {
        perror("gpio/direction");
        return fd;
    }

    if (out_flag == OUTPUT_PIN)
    {
        write(fd, "out", 4);
    }
    else
    {
        write(fd, "in", 3);
    }

    close(fd);

    return 0;
}

/****************************************************************
 * GPIO__set_value
 ****************************************************************/
int GPIO__SetValue(unsigned int gpio, T_PIN_VALUE value)
{
    int fd;
    char buf[MAX_BUF];

    snprintf(buf, sizeof(buf), SYSFS_GPIO__DIR "/gpio%d/value", gpio);

    fd = open(buf, O_WRONLY);

    if (fd < 0)
    {
        perror("gpio/set-value");
        return fd;
    }

    if (value == LOW)
    {
        write(fd, "0", 2);
    }
    else
    {
        write(fd, "1", 2);
    }

    close(fd);

    return 0;
}

/****************************************************************
 * GPIO__get_value
 ****************************************************************/
int GPIO__GetValue(unsigned int gpio, unsigned int *value)
{
    int fd;
    char buf[MAX_BUF];
    char ch;

    snprintf(buf, sizeof(buf), SYSFS_GPIO__DIR "/gpio%d/value", gpio);

    fd = open(buf, O_RDONLY);

    if (fd < 0)
    {
        perror("gpio/get-value");

        return fd;
    }

    read(fd, &ch, 1);

    if (ch != '0')
    {
        *value = 1;
    }
    else
    {
        *value = 0;
    }

    close(fd);

    return 0;
}

/****************************************************************
 * GPIO__set_edge
 ****************************************************************/

int GPIO__SetEdge(unsigned int gpio, char *edge)
{
    int fd;
    int len;
    char buf[MAX_BUF];

    len = snprintf(buf, sizeof(buf), SYSFS_GPIO__DIR "/gpio%d/edge", gpio);

    fd = open(buf, O_WRONLY);

    if (fd < 0 || len < 0)
    {
        perror("gpio/set-edge");

        return fd;
    }

    write(fd, edge, strlen(edge) + 1);

    close(fd);

    return 0;
}

/****************************************************************
 * GPIO__fd_open
 ****************************************************************/

int GPIO__Open(unsigned int gpio)
{
    int fd;
    int len;
    char buf[MAX_BUF];

    len = snprintf(buf, sizeof(buf), SYSFS_GPIO__DIR "/gpio%d/value", gpio);

    fd = open(buf, O_RDONLY); // | O_NONBLOCK);

    if (fd < 0 || len <= 0)
    {
        perror("gpio/fd_open");
    }
    else
    {
        read(fd, buf, 2);
    }

    return fd;
}

/****************************************************************
 * GPIO__fd_close
 ****************************************************************/

int GPIO__Close(int fd)
{
    return close(fd);
}
