/******************************************************************************
 spi.h for Allmogetracker 3

 Allmogetracker receives data from a GPS receiver and transmits it
 over amature radio using the APRS protocol version 1.0.

 Copyright (C)2012, Andreas Kingb�ck  (andki234@gmail.com)

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
 * SPI driver
 * @brief This section contains code for SPI communication
 *
 * @author Andreas Kingb�ck
 * @version 0.00
 */
//-------------------------------------------------------------------------------------------------
//--------  I N C L U D E -------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <fcntl.h>
#include "spi.h"

//-------------------------------------------------------------------------------------------------
//--------  D E F I N E S  ------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#define TX_BUFFER_SIZE 1

//-------------------------------------------------------------------------------------------------
//--------  C O N S T A N T S  --------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//--------  P R I V A T E  V A R I A B L E S  -----------------------------------------------------
//-------------------------------------------------------------------------------------------------
static int fd;
static uint8_t mode = 0x00;
static uint8_t bits = 8;
static uint32_t speed = 1500000;
static uint16_t delay = 0x00;
//-------------------------------------------------------------------------------------------------
//--------  P R I V A T E  F U N C T I O N S ------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//--------  S H A R E D  F U N C T I O N S --------------------------------------------------------
//-------------------------------------------------------------------------------------------------

/**
 * This function open the SPI driver connection to the kernel driver
 *
 * Uses /dev/spidev2.0
 *
 * @return T_SPI_STATUS
 *   SUCCESS if the file is opened or FAIL if it isn't.
 *
 * -------------------------------------------------------------------------------------------------
 */
T_STATUS SPI__Open(void)
{
    int status;

    fd = open("/dev/spidev1.0", O_RDWR | O_SYNC | O_NONBLOCK);

    if (fd < 0)
    {
        printf("Can't open spi device file\n");

        status = FAIL;
    }
    else
    {
        status = SUCCESS;
    }

    return status;
}

/**
 * This function closes the GPIO driver connection to the kernel driver
 *
 * Uses /dev/spidev2.0
 *
 * @return T_SPI_STATUS
 *   SUCCESS if the file is closed or FAIL if it isn't.
 * -------------------------------------------------------------------------------------------------
 */
T_STATUS SPI__Close(void)
{
    close(fd);

    return SUCCESS;
}

T_STATUS SPI__Transfer(uint8_t *tx, uint8_t *rx, uint16_t txrx_length)
{
    ssize_t blen;

    struct spi_ioc_transfer tr =
    { .tx_buf = (unsigned long) tx, .rx_buf = (unsigned long) rx, .len =
            txrx_length, .delay_usecs = delay, .speed_hz = speed,
            .bits_per_word = bits, };

    int ret;

    blen = write(fd, tx, txrx_length);

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if (ret < 1)
    {
        printf("can't send spi message");
    }

    /*for (ret = 0; ret < txrx_length; ret++)
    {
        if (!(ret % 6))
        {
            puts("");
        }

        printf("%.2X ", rx[ret]);
    }

    puts("");
    */

    return SUCCESS;
}

T_STATUS SPI__SetParameters(void)
{
    int ret;

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);

    if (ret == -1)
        printf("can't set spi mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);

    if (ret == -1)
        printf("can't get spi mode");

    /*
     * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);

    if (ret == -1)
        printf("can't set bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);

    if (ret == -1)
        printf("can't get bits per word");

    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    if (ret == -1)
        printf("can't set max speed hz");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);

    if (ret == -1)
        printf("can't get max speed hz");

    printf("spi mode: %d\n", mode);
    printf("bits per word: %d\n", bits);
    printf("max speed: %d Hz (%d KHz)\n", speed, speed / 1000);

    return SUCCESS;
}
