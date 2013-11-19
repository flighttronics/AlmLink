/******************************************************************************
 rfm23bp.c for Almlink

 AlmLink uses the rfm23bp radio module to transmit and receive data

 Copyright (C)2013, Andreas Kingbaeck  (andki234@gmail.com)

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
 * @file rfm23bp.c
 * Driver for RFM23BP
 * @brief This section contains RFM23BP code
 *
 * @author Andreas Kingbäck
 * @version 0.00
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include "rfm23bp.h"
#include "spi.h"
#include "gpio.h"

static uint8_t rx_buffer[1024];
static uint8_t tx_buffer[1024];
static T_MODE _mode;
static uint8_t _idleMode;

// FIFO Buffers
volatile uint8_t _dataBufferLength;
uint8_t _dataBuffer[MAX_MESSAGE_LEN];

volatile bool _rxBufValid;

volatile bool _txPacketSent;
volatile uint8_t _txBufSentIndex;

volatile uint16_t _rxBad;
volatile uint16_t _rxGood;
volatile uint16_t _txGood;

volatile uint8_t _lastRssi;

static uint8_t _interrupts_enabled;

// GPIO
unsigned int INT_GPIO = 44;   // GPIO1_12 = (1x32) + 12 = 44
int int_fd;

bool RFM23BP__Init(void)
{
    uint8_t data;
    uint8_t syncwords[] =
    { 0x2d, 0xd4 };

    SPI__Open();

// Setup GPIO pins
    GPIO__Export(INT_GPIO);                // Register GPIO

    /* "direction" ... reads as either "in" or "out".  This value may
     normally be written.  Writing as "out" defaults to
     initializing the value as low.  To ensure glitch free
     operation, values "low" and "high" may be written to
     configure the GPIO as an output with that initial value.

     Note that this attribute *will not exist* if the kernel
     doesn't support changing the direction of a GPIO, or
     it was exported by kernel code that didn't explicitly
     allow userspace to reconfigure this GPIO's direction.
     */

    GPIO__SetDir(INT_GPIO, INPUT_PIN);

    /*
     If the pin can be configured as interrupt-generating interrupt
     and if it has been configured to generate interrupts (see the
     description of "edge"), you can poll(2) on that file and
     poll(2) will return whenever the interrupt was triggered. If
     you use poll(2), set the events POLLPRI and POLLERR. If you
     use select(2), set the file descriptor in exceptfds. After
     poll(2) returns, either lseek(2) to the beginning of the sysfs
     file and read the new value or close the file and re-open it
     to read the value.

     "edge" ... reads as either "none", "rising", "falling", or
     "both". Write these strings to select the signal edge(s)
     that will make poll(2) on the "value" file return.

     This file exists only if the pin can be configured as an
     interrupt generating input pin.
     */

    GPIO__SetEdge(INT_GPIO, "falling");   // Set interrupt to falling edge

    int_fd = GPIO__Open(INT_GPIO);

// Check if have a valid connection to the RFM23BP module
    data = RFM23BP__ReadRegister(DEVICE_TYPE);

    if (data != DEVICE_TYPE_RX_TRX && data != DEVICE_TYPE_TX)
    {
        return false;
    }

// Software reset the device
    RFM23BP__Reset();

// Most of these are the POR default
    RFM23BP__WriteRegister(TX_FIFO_CONTROL2, TXFFAEM_THRESHOLD);
    RFM23BP__WriteRegister(RX_FIFO_CONTROL, RXFFAFULL_THRESHOLD);
    RFM23BP__WriteRegister(DATA_ACCESS_CONTROL, ENPACRX | ENPACTX | ENCRC | CRC_CRC_16_IBM);
// Configure the message headers
// Here we set up the standard packet format for use by the RF22 library
// 8 nibbles preamble
// 2 SYNC words 2d, d4
// Header length 4 (to, from, id, flags)
// 1 octet of data length (0 to 255)
// 0 to 255 octets data
// 2 CRC octets as CRC16(IBM), computed on the header, length and data
// On reception the to address is check for validity against RF22_REG_3F_CHECK_HEADER3
// or the broadcast address of 0xff
// If no changes are made after this, the transmitted
// to address will be 0xff, the from address will be 0xff
// and all such messages will be accepted. This permits the out-of the box
// RF22 config to act as an unaddresed, unreliable datagram service
    RFM23BP__WriteRegister(HEADER_CONTROL1, BCEN_HEADER3 | HDCH_HEADER3);
    RFM23BP__WriteRegister(HEADER_CONTROL2, HDLEN_4 | SYNCLEN_2);

    RFM23BP__SetPreambleLength(8);

    RFM23BP__SetSyncWords(syncwords, sizeof(syncwords));

    RFM23BP__SetPromiscuous(false);

// Check the TO header against RF22_DEFAULT_NODE_ADDRESS
    RFM23BP__WriteRegister(CHECK_HEADER3, DEFAULT_NODE_ADDRESS);

// Set the default transmit header values
    RFM23BP__SetHeaderTo(DEFAULT_NODE_ADDRESS);
    RFM23BP__SetHeaderFrom(DEFAULT_NODE_ADDRESS);
    RFM23BP__SetHeaderId(0);
    RFM23BP__SetHeaderFlags(0);

// Ensure the antenna can be switched automatically according to transmit and receive
// This assumes GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
// This assumes GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive

    RFM23BP__WriteRegister(GPIO_CONFIGURATION0, 0xD2); // TX state 0x12
    RFM23BP__WriteRegister(GPIO_CONFIGURATION1, 0xD5); // RX state 0x15

// Disable interrupts
    RFM23BP__DisableInterrupts();
    // RFM23BP__WriteRegister(INTERRUPT_ENABLE1, ENTXFFAEM | ENRXFFAFULL | ENPKSENT | ENPKVALID | ENCRCERROR | ENFFERR);
    // RFM23BP__WriteRegister(INTERRUPT_ENABLE2, ENPREAVAL);

// Set some defaults. An innocuous ISM frequency, and reasonable pull-in
    RFM23BP__SetFrequency(434.0, 0.05);

// Some slow, reliable default speed and modulation
    RFM23BP__SetModemConfig(FSK_Rb2_4Fd36);

    _idleMode = XTON | PLLON; // Default idle state is READY mode
    _mode = MODE_IDLE; // We start up in idle mode

    printf("Device = 0x%02x\n", data);
    printf("Version code = 0x%02x\n", RFM23BP__ReadRegister(VERSION_CODE));
    printf("Device status = 0x%02x\n", RFM23BP__ReadRegister(DEVICE_STATUS));

    printf("Temp = %4.1f\n", RFM23BP__ReadTemperature(TSRANGE_M64_64C, ADCOFFS_NONE));

    return true;
}

void RFM23BP__Close(void)
{
    GPIO__Close(int_fd);
    GPIO__Unexport(INT_GPIO);
    SPI__Close();
}

void RFM23BP__ClearBuffers(void)
{
    memset(rx_buffer, 0x00, sizeof(rx_buffer));
    memset(tx_buffer, 0x00, sizeof(tx_buffer));
}

uint8_t RFM23BP__ReadRegister(T_RFM23BP taddress)
{
    RFM23BP__ClearBuffers();

    tx_buffer[0] = RFM23BPLOOKUP__GetAddress(taddress);

    SPI__Transfer(tx_buffer, rx_buffer, 2);

    return rx_buffer[1];
}

void RFM23BP__ReadBurst(uint8_t taddress, uint8_t* dest, uint8_t len)
{
    RFM23BP__ClearBuffers();

    tx_buffer[0] = (RFM23BPLOOKUP__GetAddress(taddress) & ~SPI_WRITE_MASK);
    tx_buffer[1] = 0x00;

    SPI__Transfer(tx_buffer, rx_buffer, len + 1);

    memcpy(dest, rx_buffer + 1, len);
}

void RFM23BP__WriteRegister(T_RFM23BP taddress, uint8_t tdata)
{
    tx_buffer[0] = SPI_WRITE_MASK | RFM23BPLOOKUP__GetAddress(taddress);
    tx_buffer[1] = tdata;

    SPI__Transfer(tx_buffer, rx_buffer, 2);
}

void RFM23BP__WriteBurst(uint8_t taddress, const uint8_t* src, uint8_t len)
{
    tx_buffer[0] = SPI_WRITE_MASK | RFM23BPLOOKUP__GetAddress(taddress);

    memcpy(&tx_buffer[1], src, len);

    SPI__Transfer(tx_buffer, rx_buffer, len + 1);
}

uint8_t RFM23BP__GetRXBuffer(uint8_t *abuffer)
{
    memcpy(abuffer, _dataBuffer, _dataBufferLength);

    return _dataBufferLength;
}

void RFM23BP__Reset()
{
    RFM23BP__WriteRegister(OPERATING_MODE1, SWRES);
// Wait for it to settle
    sleep(1); // SWReset time is nominally 100usec
}

// Returns true if centre + (fhch * fhs) is within limits
// Caution, different versions of the RF22 support different max freq
// so YMMV
bool RFM23BP__SetFrequency(float centre, float afcPullInRange)
{
    uint8_t fbsel = SBSEL;
    uint8_t afclimiter;
    uint8_t fb;
    uint16_t fc;
    float integerPart;
    float fractionalPart;

    if (centre < 240.0 || centre > 960.0) // 930.0 for early silicon
    {
        return false;
    }

    if (centre >= 480.0)
    {
        if (afcPullInRange < 0.0 || afcPullInRange > 0.318750)
        {
            return false;
        }

        centre /= 2;
        fbsel |= HBSEL;
        afclimiter = afcPullInRange * 1000000.0 / 1250.0;
    }
    else
    {
        if (afcPullInRange < 0.0 || afcPullInRange > 0.159375)
        {
            return false;
        }

        afclimiter = afcPullInRange * 1000000.0 / 625.0;
    }

    centre /= 10.0;

    integerPart = floor(centre);
    fractionalPart = centre - integerPart;

    fb = (uint8_t) integerPart - 24; // Range 0 to 23
    fbsel |= fb;
    fc = fractionalPart * 64000;

    RFM23BP__WriteRegister(FREQUENCY_OFFSET1, 0); // REVISIT
    RFM23BP__WriteRegister(FREQUENCY_OFFSET2, 0);
    RFM23BP__WriteRegister(FREQUENCY_BAND_SELECT, fbsel);
    RFM23BP__WriteRegister(NOMINAL_CARRIER_FREQUENCY1, fc >> 8);
    RFM23BP__WriteRegister(NOMINAL_CARRIER_FREQUENCY0, fc & 0xff);
    RFM23BP__WriteRegister(AFC_LIMITER, afclimiter);

    return !(RFM23BP__StatusRead() & FREQERR);
}

bool RFM23BP__SetModemConfig(T_MODEM_CONFIG_CHOICE index)
{
    T_MODEM_CONFIG cfg;

    bool status;

    if (index > (sizeof(MODEM_CONFIG_TABLE) / sizeof(T_MODEM_CONFIG)))
    {
        status = false;
    }
    else
    {
        memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(T_MODEM_CONFIG));

        RFM23BP__SetModemRegisters(&cfg);

        status = true;
    }

    return status;
}

// Sets registers from a canned modem configuration structure
void RFM23BP__SetModemRegisters(const T_MODEM_CONFIG* config)
{
    RFM23BP__WriteRegister(IF_FILTER_BANDWIDTH, config->reg_1c);
    RFM23BP__WriteRegister(CLOCK_RECOVERY_GEARSHIFT_OVERRIDE, config->reg_1f);
    RFM23BP__WriteBurst(CLOCK_RECOVERY_OVERSAMPLING_RATE, &config->reg_20, 6);
    RFM23BP__WriteBurst(OOK_COUNTER_VALUE_1, &config->reg_2c, 3);
    RFM23BP__WriteRegister(CHARGE_PUMP_CURRENT_TRIMMING, config->reg_58);
    RFM23BP__WriteRegister(AGC_OVERRIDE1, config->reg_69);
    RFM23BP__WriteBurst(TX_DATA_RATE1, &config->reg_6e, 5);
}

// REVISIT: top bit is in Header Control 2 0x33
void RFM23BP__SetPreambleLength(uint8_t nibbles)
{
    RFM23BP__WriteRegister(PREAMBLE_LENGTH, nibbles);
}

// Caution doesn't set sync word len in Header Control 2 0x33
void RFM23BP__SetSyncWords(const uint8_t* syncWords, uint8_t len)
{
    RFM23BP__WriteBurst(SYNC_WORD3, syncWords, len);
}

void RFM23BP__SetHeaderTo(uint8_t to)
{
    RFM23BP__WriteRegister(TRANSMIT_HEADER3, to);
}

void RFM23BP__SetHeaderFrom(uint8_t from)
{
    RFM23BP__WriteRegister(TRANSMIT_HEADER2, from);
}

void RFM23BP__SetHeaderId(uint8_t id)
{
    RFM23BP__WriteRegister(TRANSMIT_HEADER1, id);
}

void RFM23BP__SetHeaderFlags(uint8_t flags)
{
    RFM23BP__WriteRegister(TRANSMIT_HEADER0, flags);
}

uint8_t RFM23BP__HeaderTo(void)
{
    return RFM23BP__ReadRegister(RECEIVED_HEADER3);
}

uint8_t RFM23BP__HeaderFrom(void)
{
    return RFM23BP__ReadRegister(RECEIVED_HEADER2);
}

uint8_t RFM23BP__HeaderId(void)
{
    return RFM23BP__ReadRegister(RECEIVED_HEADER1);
}

uint8_t RFM23BP__HeaderFlags(void)
{
    return RFM23BP__ReadRegister(RECEIVED_HEADER0);
}

void RFM23BP__SetPromiscuous(bool promiscuous)
{
    RFM23BP__WriteRegister(HEADER_ENABLE3, promiscuous ? 0x00 : 0xff);
}

uint8_t RFM23BP__StatusRead(void)
{
    return RFM23BP__ReadRegister(DEVICE_STATUS);
}

uint8_t RFM23BP__RSSIRead(void)
{
    return RFM23BP__ReadRegister(RSSI);
}

void RFM23BP__SetMode(uint8_t mode)
{
    RFM23BP__WriteRegister(OPERATING_MODE1, mode);
}

void RFM23BP__SetModeIdle(void)
{
    if (_mode != MODE_IDLE)
    {
        RFM23BP__SetMode(_idleMode);
        _mode = MODE_IDLE;
    }
}

void RFM23BP__SetModeTx(void)
{
    if (_mode != MODE_TX)
    {
        RFM23BP__SetMode(_idleMode | TXON);
        _mode = MODE_TX;
        // Hmmm, if you dont clear the RX FIFO here, then it appears that going
        // to transmit mode in the middle of a receive can corrupt the
        // RX FIFO
        RFM23BP__ResetRxFifo();
//  RFM23BP__ClearRxBuf();
    }
}

void RFM23BP__SetModeRx(void)
{
    RFM23BP__SetModeIdle();

    if (_mode != MODE_RX)
    {
        RFM23BP__ResetFifos();
        RFM23BP__ClearRxBuf();

        RFM23BP__SetMode(_idleMode | RXON);

        _mode = MODE_RX;

        RFM23BP__EnableInterrupts();
    }
}

void RFM23BP__SetTxPower(uint8_t power)
{
    RFM23BP__WriteRegister(TX_POWER, power);
}

// Clear the FIFOs
void RFM23BP__ResetFifos()
{
    RFM23BP__WriteRegister(OPERATING_MODE2, FFCLRRX | FFCLRTX);
    RFM23BP__WriteRegister(OPERATING_MODE2, 0);
}

// Clear the Rx FIFO
void RFM23BP__ResetRxFifo()
{
    RFM23BP__WriteRegister(OPERATING_MODE2, FFCLRRX);
    RFM23BP__WriteRegister(OPERATING_MODE2, 0);
}

// CLear the TX FIFO
void RFM23BP__ResetTxFifo()
{
    RFM23BP__WriteRegister(OPERATING_MODE2, FFCLRTX);
    RFM23BP__WriteRegister(OPERATING_MODE2, 0);
}

void RFM23BP__StartTransmit(void)
{
    RFM23BP__SendNextFragment(); // Actually the first fragment
    RFM23BP__WriteRegister(PACKET_LENGTH, _dataBufferLength); // Total length that will be sent
    RFM23BP__SetModeTx(); // Start the transmitter, turns off the receiver
}

// Restart the transmission of a packet that had a problem
void RFM23BP__RestartTransmit(void)
{
    _mode = MODE_IDLE;
    _txBufSentIndex = 0;
//      printf("Restart");
    RFM23BP__StartTransmit();
}

void RFM23BP__WaitPacketSent(void)  // TODO will lock!!!
{

    struct pollfd fdset[2];
    uint8_t buf[10];
    int status;

    if (_mode == MODE_TX)
    {
        //read(int_fd, buf, 2);

        memset((void*) fdset, 0, sizeof(fdset));

        fdset[0].fd = int_fd;
        fdset[0].events = POLLIN;
        fdset[0].revents = 0;

        fdset[1].fd = int_fd;
        fdset[1].events = POLLPRI;
        fdset[1].revents = 0;

        status = poll(fdset, 2, 5000);

        if (status < 0)
        {
            printf("poll() failed!\n");
        }
        else
        {
            //_mode = MODE_IDLE;

            if (status == 0)
            {
                printf(".");
            }

            if (fdset[1].revents & POLLPRI)
            {
                printf("poll() GPIO interrupt occurred\n");
                RFM23BP__HandleInterrupt();
            }

            if (fdset[0].revents & POLLIN)
            {
                (void) read(fdset[0].fd, buf, 1);
                printf("poll() stdin read 0x%2.2X\n", (unsigned int) buf[0]);
            }
        }
    }
}

bool RFM23BP__Send(const uint8_t* data, uint8_t len)
{
//_mode = MODE_TX;
    RFM23BP__WaitPacketSent();

    if (!RFM23BP__FillTxBuf(data, len))
    {
        return false;
    }

    RFM23BP__StartTransmit();

    printf("send: %s, [%d]\n", data, len);

    return true;
}

void RFM23BP__ClearRxBuf(void)
{
    _dataBufferLength = 0;
    _rxBufValid = false;
}

void RFM23BP__ClearTxBuf(void)
{
    _dataBufferLength = 0;
    _txBufSentIndex = 0;
}

bool RFM23BP__FillTxBuf(const uint8_t* data, uint8_t len)
{
    RFM23BP__ClearTxBuf();

    if (!len)
    {
        return false;
    }

    return RFM23BP__AppendTxBuf(data, len);
}

bool RFM23BP__AppendTxBuf(const uint8_t* data, uint8_t len)
{
    if (((uint16_t) _dataBufferLength + len) > MAX_MESSAGE_LEN)
    {
        return false;
    }

    memcpy(_dataBuffer + _dataBufferLength, data, len);
    _dataBufferLength += len;

//    printBuffer("txbuf:", _dataBuffer, _dataBufferLength);
    return true;
}

// Assumption: there is currently <= RF22_TXFFAEM_THRESHOLD bytes in the Tx FIFO
void RFM23BP__SendNextFragment(void)
{
    uint8_t len = 0;

    if (_txBufSentIndex < _dataBufferLength)
    {
        // Some left to send?
        len = _dataBufferLength - _txBufSentIndex;
        // But dont send too much
        if (len > (FIFO_SIZE - TXFFAEM_THRESHOLD - 1))
        {
            len = (FIFO_SIZE - TXFFAEM_THRESHOLD - 1);
        }

        RFM23BP__WriteBurst(FIFO_ACCESS, _dataBuffer + _txBufSentIndex, len);

        _txBufSentIndex += len;
    }
}

// Assumption: there are at least RF22_RXFFAFULL_THRESHOLD in the RX FIFO
// That means it should only be called after a RXFFAFULL interrupt
void RFM23BP__ReadNextFragment(void)
{
    if (((uint16_t) _dataBufferLength + RXFFAFULL_THRESHOLD) > MAX_MESSAGE_LEN)
    {
        return; // Hmmm receiver overflow. Should never occur
    }
    else
    {
        // Read the RF22_RXFFAFULL_THRESHOLD octets that should be there
        RFM23BP__ReadBurst(FIFO_ACCESS, _dataBuffer + _dataBufferLength, RXFFAFULL_THRESHOLD);
        _dataBufferLength += RXFFAFULL_THRESHOLD;
    }
}

uint8_t RFM23BP__ReadADC(uint8_t adcsel, uint8_t adcref, uint8_t adcgain, uint8_t adcoffs)
{
    uint8_t configuration;

    configuration = adcsel | adcref | (adcgain & ADCGAIN);

    RFM23BP__WriteRegister(ADC_CONFIGURATION, configuration | ADCSTART);
    RFM23BP__WriteRegister(ADC_SENSOR_AMP_OFFSET, adcoffs);

// Conversion time is nominally 305usec
// Wait for the DONE bit
    while (!(RFM23BP__ReadRegister(ADC_CONFIGURATION) & ADCDONE))
        ;
// Return the value
    return RFM23BP__ReadRegister(ADC_VALUE);
}

float RFM23BP__ReadTemperature(uint8_t tsrange, uint8_t tvoffs)
{
    float temp;

    RFM23BP__WriteRegister(TEMPERATURE_SENSOR_CALIBRATION, tsrange | ENTSOFFS);
    RFM23BP__WriteRegister(TEMPERATURE_VALUE_OFFSET, tvoffs);

    temp = (float) RFM23BP__ReadADC(ADCSEL_INTERNAL_TEMPERATURE_SENSOR, ADCREF_BANDGAP_VOLTAGE, ADCGAIN_NONE, ADCOFFS_NONE);

    switch (tsrange)
    {
        case TSRANGE_M64_64C:
        {
            temp = temp * 0.5 - 64.0;

            break;
        }

        case TSRANGE_M64_192C:
        {
            temp = temp * 1.0 - 64.0;

            break;
        }

        case TSRANGE_0_128C:
        {
            temp = temp * 0.5 - 0.0;

            break;
        }

        case TSRANGE_M40_216F:
        {
            temp = temp * 1.0 - 40.0;

            break;
        }
    }

    return temp;
}

void RFM23BP__DisableInterrupts(void)
{
    uint8_t buffer[2] =
    { 0x00, 0x00 };

    RFM23BP__WriteBurst(INTERRUPT_ENABLE1, buffer, 2);
    RFM23BP__ReadBurst(INTERRUPT_STATUS1, buffer, 2);

    _interrupts_enabled = false;

    //RFM23BP__WriteRegister(INTERRUPT_ENABLE1, 0x00);
    //RFM23BP__WriteRegister(INTERRUPT_ENABLE2, 0x00);

    //(void) RFM23BP__ReadRegister(INTERRUPT_STATUS1);
    //(void) RFM23BP__ReadRegister(INTERRUPT_STATUS2);
}

void RFM23BP__EnableInterrupts(void)
{
    RFM23BP__DisableInterrupts();

    (void) RFM23BP__ReadGPIOValue(int_fd);  // Read to clear Linux interrupt signal

    switch (_mode)
    {
        case MODE_IDLE:
        {
            break;
        }

        case MODE_TX:
        {
            break;
        }

        case MODE_RX:
        {
            RFM23BP__WriteRegister(INTERRUPT_ENABLE1, ENPKVALID);

            //RFM23BP__WriteRegister(INTERRUPT_ENABLE1, ENRXFFAFULL | ENPKVALID | ENCRCERROR | ENFFERR);
            //RFM23BP__WriteRegister(INTERRUPT_ENABLE2, ENPREAINVAL | ENPREAVAL | ENSWDET);

            break;
        }

    }
}

uint8_t RFM23BP__ReadGPIOValue(int fd)
{
    int status;
    uint8_t buffer[2];

    status = lseek(fd, 0, SEEK_SET);

    if (status < 0)
    {
        printf("error lseek value\n");
    }

    status = read(fd, buffer, 2);

    if (status != 2)
    {
        printf("error read value\n");
    }

    return buffer[0];
}

void RFM23BP__PollInterrupt(int timeout_ms)
{
    struct pollfd fdset[2];
    uint8_t buffer[10];
    int status;

    memset((void*) fdset, 0, sizeof(fdset));

    fdset[0].fd = int_fd;
    fdset[0].events = POLLPRI;
    fdset[0].revents = 0;

    fdset[1].fd = int_fd;
    fdset[1].events = POLLERR;
    fdset[1].revents = 0;

    status = poll(fdset, 1, timeout_ms);

    if (status < 0)
    {
        printf("poll() failed!\n");
    }
    else
    {
        if (status == 0)
        {
            printf("Timeout RFM23BP__PollInterrupt\n");
        }
        else
        {
            if (fdset[0].revents & POLLPRI)
            {
                printf("poll() GPIO interrupt occurred\n");

                RFM23BP__HandleInterrupt();
            }

            if (fdset[1].revents & POLLERR)
            {
                printf("poll() stdin read 0x%2.2X\n", (unsigned int) buffer[0]);
            }

            (void) RFM23BP__ReadGPIOValue(int_fd);
        }
    }
}

void RFM23BP__PrintInterruptRegisters(void)
{
    uint8_t _lastInterruptFlags[2];
// Read the interrupt flags which clears the interrupt

    _lastInterruptFlags[0] = RFM23BP__ReadRegister(INTERRUPT_STATUS1);
    _lastInterruptFlags[1] = RFM23BP__ReadRegister(INTERRUPT_STATUS2);

    //RFM23BP__ReadBurst(INTERRUPT_STATUS1, _lastInterruptFlags, 2);

// Caution: Serial printing in this interrupt routine can cause mysterious crashes
    printf("interrupt ");
    printf("0x%02x", _lastInterruptFlags[0]);
    printf(" ");
    printf("0x%02x\n", _lastInterruptFlags[1]);

    if (_lastInterruptFlags[0] == 0 && _lastInterruptFlags[1] == 0)
    {
        printf("FUNNY: no interrupt!\n");
    }
}

// Blocks until a valid message is received
void RFM23BP__WaitAvailable()
{
    while (!_rxBufValid)
    {
        RFM23BP__PollInterrupt(5000);
    }
}

// C++ level interrupt handler for this instance
void RFM23BP__HandleInterrupt()
{
    uint8_t _lastInterruptFlags[2];
    uint8_t len;
// Read the interrupt flags which clears the interrupt

    //printf("INTREGS = 0x%02x, 0x%02x\n", RFM23BP__ReadRegister(INTERRUPT_ENABLE1), RFM23BP__ReadRegister(INTERRUPT_ENABLE2));

    _lastInterruptFlags[0] = RFM23BP__ReadRegister(INTERRUPT_STATUS1);
    _lastInterruptFlags[1] = RFM23BP__ReadRegister(INTERRUPT_STATUS2);

//    RFM23BP__ReadBurst(INTERRUPT_STATUS1, _lastInterruptFlags, 2);

#if 1
// Caution: Serial printing in this interrupt routine can cause mysterious crashes

    //printf("RFM23BP__HandleInterrupt()\n");

    if (_lastInterruptFlags[0] == 0 && _lastInterruptFlags[1] == 0)
    {
        printf("FUNNY: no interrupt!\n");
    }
#endif

#if 0
// TESTING: fake an RF22_IFFERROR
    static int counter = 0;
    if (_lastInterruptFlags[0] & RF22_IPKSENT && counter++ == 10)
    {
        _lastInterruptFlags[0] = RF22_IFFERROR;
        counter = 0;
    }
#endif

    if (_lastInterruptFlags[0] & IFFERROR)
    {
        RFM23BP__ResetFifos(); // Clears the interrupt

        if (_mode == MODE_TX)
        {
            RFM23BP__RestartTransmit();
        }
        else if (_mode == MODE_RX)
        {
            RFM23BP__ClearRxBuf();
        }

        printf("IFFERROR\n");
    }

// Caution, any delay here may cause a FF underflow or overflow
    if ((_lastInterruptFlags[0] & ITXFFAEM) && (_mode == MODE_TX))
    {
        // See if more data has to be loaded into the Tx FIFO
        RFM23BP__SendNextFragment();

        printf("ITXFFAEM\n");
    }

    if (_lastInterruptFlags[0] & IRXFFAFULL)
    {
        // Caution, any delay here may cause a FF overflow
        // Read some data from the Rx FIFO
        RFM23BP__ReadNextFragment();

        printf("IRXFFAFULL\n");
    }

    if (_lastInterruptFlags[0] & IEXT)
    {
        // This is not enabled by the base code, but users may want to enable it
        //handleExternalInterrupt();

        printf("IEXT\n");
    }

    if (_lastInterruptFlags[1] & IWUT)
    {
        // This is not enabled by the base code, but users may want to enable it
        //handleWakeupTimerInterrupt();

        printf("IWUT\n");
    }

    if (_lastInterruptFlags[0] & IPKSENT)
    {
        printf("IPKSENT\n");
        _txGood++;
        // Transmission does not automatically clear the tx buffer.
        // Could retransmit if we wanted
        // RF22 transitions automatically to Idle
        _mode = MODE_IDLE;
    }

    if ((_lastInterruptFlags[0] & IPKVALID) && (_mode = MODE_RX))
    {
        len = RFM23BP__ReadRegister(RECEIVED_PACKET_LENGTH);

        printf("IPKVALID\n");

        //printf("len=%d\n", len);
        //printf("bufLen=%d\n", _dataBufferLength);

        // May have already read one or more fragments
        // Get any remaining unread octets, based on the expected length
        // First make sure we don't overflow the buffer in the case of a stupid length
        // or partial bad receives

        if (len > MAX_MESSAGE_LEN || len < _dataBufferLength)
        {
            _rxBad++;
            _mode = MODE_IDLE;
            RFM23BP__ClearRxBuf();

            printf("Hmmm receiver buffer overflow.\n");

            return; // Hmmm receiver buffer overflow.
        }

        RFM23BP__ReadBurst(FIFO_ACCESS, _dataBuffer + _dataBufferLength, len - _dataBufferLength);

        _rxGood++;
        _dataBufferLength = len;
        _mode = MODE_IDLE;
        _rxBufValid = true;

    }

    if (_lastInterruptFlags[0] & ICRCERROR)
    {
        printf("ICRCERR\n");
        _rxBad++;
        RFM23BP__ClearRxBuf();
        RFM23BP__ResetRxFifo();
        _mode = MODE_IDLE;
        RFM23BP__SetModeRx(); // Keep trying
    }

    if (_lastInterruptFlags[1] & IPREAVAL)
    {
        printf("IPREAVAL\n");
        _lastRssi = RFM23BP__ReadRegister(RSSI);
        RFM23BP__ClearRxBuf();
    }
}
