/******************************************************************************
 rfm23bp.h for Almlink

 AlmLink uses the rfm23bp radio module to transmit and receive data

 Copyright (C) 2013, Andreas Kingbaeck  (andki234@gmail.com)
 Copyright (C) 2011 Mike McCauley (mikem@airspayce.com)

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
 * @file rfm23bp.h
 * Header file for RFM23BP
 * @brief This section contains RFM23BP definitions
 *
 * @author Andreas Kingbäck
 * @version 0.00
 */

#ifndef RFM23BP_H_
#define RFM23BP_H_

#include <stdbool.h>
#include "rfm23bp_table.h"
#include "rfm23bp_lookup.h"
#include "rfm23bp_config.h"

// This is the maximum message length that can be supported by this library. Limited by
// the single message length octet in the header.
// Yes, 255 is correct even though the FIFO size in the RF22 is only
// 64 octets. We use interrupts to refill the Tx FIFO during transmission and to empty the
// Rx FIFO during reception
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
#ifndef MAX_MESSAGE_LEN
//#define RF22_MAX_MESSAGE_LEN 255
#define MAX_MESSAGE_LEN 50
#endif

// Max number of octets the RF22 Rx and Tx FIFOs can hold
#define FIFO_SIZE 64

// These values we set for FIFO thresholds (4, 55) are actually the same as the POR values
#define TXFFAEM_THRESHOLD 4
#define RXFFAFULL_THRESHOLD 55

// This is the bit in the SPI address that marks it as a write
#define SPI_WRITE_MASK 0x80

// These values we set for FIFO thresholds (4, 55) are actually the same as the POR values
#define TXFFAEM_THRESHOLD 4
#define RXFFAFULL_THRESHOLD 55

// This is the default node address,
#define DEFAULT_NODE_ADDRESS 0

// Keep track of the mode the RF23 is in
#define MODE_IDLE         0
#define MODE_RX           1
#define MODE_TX           2

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RF-22 Manual
// REG_00_DEVICE_TYPE                      0x00
#define DEVICE_TYPE_RX_TRX                 0x08
#define DEVICE_TYPE_TX                     0x07

// REG_02_DEVICE_STATUS                    0x02
#define FFOVL                              0x80
#define FFUNFL                             0x40
#define RXFFEM                             0x20
#define HEADERR                            0x10
#define FREQERR                            0x08
#define LOCKDET                            0x04
#define CPS                                0x03
#define CPS_IDLE                           0x00
#define CPS_RX                             0x01
#define CPS_TX                             0x10

// REG_03_INTERRUPT_STATUS1                0x03
#define IFFERROR                           0x80
#define ITXFFAFULL                         0x40
#define ITXFFAEM                           0x20
#define IRXFFAFULL                         0x10
#define IEXT                               0x08
#define IPKSENT                            0x04
#define IPKVALID                           0x02
#define ICRCERROR                          0x01

// REG_04_INTERRUPT_STATUS2                0x04
#define ISWDET                             0x80
#define IPREAVAL                           0x40
#define IPREAINVAL                         0x20
#define IRSSI                              0x10
#define IWUT                               0x08
#define ILBD                               0x04
#define ICHIPRDY                           0x02
#define IPOR                               0x01

// REG_05_INTERRUPT_ENABLE1                0x05
#define ENFFERR                            0x80
#define ENTXFFAFULL                        0x40
#define ENTXFFAEM                          0x20
#define ENRXFFAFULL                        0x10
#define ENEXT                              0x08
#define ENPKSENT                           0x04
#define ENPKVALID                          0x02
#define ENCRCERROR                         0x01

// REG_06_INTERRUPT_ENABLE2                0x06
#define ENSWDET                            0x80
#define ENPREAVAL                          0x40
#define ENPREAINVAL                        0x20
#define ENRSSI                             0x10
#define ENWUT                              0x08
#define ENLBDI                             0x04
#define ENCHIPRDY                          0x02
#define ENPOR                              0x01

// REG_07_OPERATING_MODE                   0x07
#define SWRES                              0x80
#define ENLBD                              0x40
#define ENWT                               0x20
#define X32KSEL                            0x10
#define TXON                               0x08
#define RXON                               0x04
#define PLLON                              0x02
#define XTON                               0x01

// REG_08_OPERATING_MODE2                  0x08
#define ANTDIV                             0xc0
#define RXMPK                              0x10
#define AUTOTX                             0x08
#define ENLDM                              0x04
#define FFCLRRX                            0x02
#define FFCLRTX                            0x01

// REG_0F_ADC_CONFIGURATION                0x0f
#define ADCSTART                           0x80
#define ADCDONE                            0x80
#define ADCSEL                             0x70
#define ADCSEL_INTERNAL_TEMPERATURE_SENSOR 0x00
#define ADCSEL_GPIO0_SINGLE_ENDED          0x10
#define ADCSEL_GPIO1_SINGLE_ENDED          0x20
#define ADCSEL_GPIO2_SINGLE_ENDED          0x30
#define ADCSEL_GPIO0_GPIO1_DIFFERENTIAL    0x40
#define ADCSEL_GPIO1_GPIO2_DIFFERENTIAL    0x50
#define ADCSEL_GPIO0_GPIO2_DIFFERENTIAL    0x60
#define ADCSEL_GND                         0x70
#define ADCREF                             0x0c
#define ADCREF_BANDGAP_VOLTAGE             0x00
#define ADCREF_VDD_ON_3                    0x08
#define ADCREF_VDD_ON_2                    0x0c
#define ADCGAIN                            0x03
#define ADCGAIN_NONE                       0x00

// REG_10_ADC_SENSOR_AMP_OFFSET            0x10
#define ADCOFFS                            0x0f
#define ADCOFFS_NONE                       0x00

// REG_12_TEMPERATURE_SENSOR_CALIBRATION   0x12
#define TSRANGE                            0xc0
#define TSRANGE_M64_64C                    0x00
#define TSRANGE_M64_192C                   0x40
#define TSRANGE_0_128C                     0x80
#define TSRANGE_M40_216F                   0xc0
#define ENTSOFFS                           0x20
#define ENTSTRIM                           0x10
#define TSTRIM                             0x0f

// REG_14_WAKEUP_TIMER_PERIOD1             0x14
#define WTR                                0x3c
#define WTD                                0x03

// REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE      0x1d
#define AFBCD                              0x80
#define ENAFC                              0x40
#define AFCGEARH                           0x38
#define AFCGEARL                           0x07

// REG_1E_AFC_TIMING_CONTROL               0x1e
#define SWAIT_TIMER                        0xc0
#define SHWAIT                             0x38
#define ANWAIT                             0x07

// REG_30_DATA_ACCESS_CONTROL              0x30
#define ENPACRX                            0x80
#define MSBFRST                            0x00
#define LSBFRST                            0x40
#define CRCHDRS                            0x00
#define CRCDONLY                           0x20
#define ENPACTX                            0x08
#define ENCRC                              0x04
#define CRC                                0x03
#define CRC_CCITT                          0x00
#define CRC_CRC_16_IBM                     0x01
#define CRC_IEC_16                         0x02
#define CRC_BIACHEVA                       0x03

// REG_32_HEADER_CONTROL1                  0x32
#define BCEN                               0xf0
#define BCEN_NONE                          0x00
#define BCEN_HEADER0                       0x10
#define BCEN_HEADER1                       0x20
#define BCEN_HEADER2                       0x40
#define BCEN_HEADER3                       0x80
#define HDCH                               0x0f
#define HDCH_NONE                          0x00
#define HDCH_HEADER0                       0x01
#define HDCH_HEADER1                       0x02
#define HDCH_HEADER2                       0x04
#define HDCH_HEADER3                       0x08

// REG_33_HEADER_CONTROL2                  0x33
#define HDLEN                              0x70
#define HDLEN_0                            0x00
#define HDLEN_1                            0x10
#define HDLEN_2                            0x20
#define HDLEN_3                            0x30
#define HDLEN_4                            0x40
#define VARPKLEN                           0x00
#define FIXPKLEN                           0x08
#define SYNCLEN                            0x06
#define SYNCLEN_1                          0x00
#define SYNCLEN_2                          0x02
#define SYNCLEN_3                          0x04
#define SYNCLEN_4                          0x06
#define PREALEN8                           0x01

// REG_6D_TX_POWER                         0x6d
#define TXPOW                              0x07
//#define TXPOW_4X31                         0x08  // Not used in RFM23BP
//#define TXPOW_1DBM                         0x00  // Not used in RFM23BP
//#define TXPOW_2DBM                         0x01  // Not used in RFM23BP
//#define TXPOW_5DBM                         0x02  // Not used in RFM23BP
//#define TXPOW_8DBM                         0x03  // Not used in RFM23BP
//#define TXPOW_11DBM                        0x04  // Not used in RFM23BP
#define TXPOW_28DBM                        0x05
#define TXPOW_29DBM                        0x06
#define TXPOW_30DBM                        0x07
// IN RFM23B
#define TXPOW_LNA_SW                       0x08

// REG_71_MODULATION_CONTROL2              0x71
#define TRCLK                              0xc0
#define TRCLK_NONE                         0x00
#define TRCLK_GPIO                         0x40
#define TRCLK_SDO                          0x80
#define TRCLK_NIRQ                         0xc0
#define DTMOD                              0x30
#define DTMOD_DIRECT_GPIO                  0x00
#define DTMOD_DIRECT_SDI                   0x10
#define DTMOD_FIFO                         0x20
#define DTMOD_PN9                          0x30
#define ENINV                              0x08
#define FD8                                0x04
#define MODTYP                             0x30
#define MODTYP_UNMODULATED                 0x00
#define MODTYP_OOK                         0x01
#define MODTYP_FSK                         0x02
#define MODTYP_GFSK                        0x03

// REG_75_FREQUENCY_BAND_SELECT            0x75
#define SBSEL                              0x40
#define HBSEL                              0x20
#define FB                                 0x1f

/// Choices for setModemConfig() for a selected subset of common modulation types,
/// and data rates. If you need another configuration, use the register calculator.
/// and call setModemRegisters() with your desired settings
/// These are indexes into _modemConfig
typedef enum
{
    UnmodulatedCarrier = 0, ///< Unmodulated carrier for testing
    FSK_PN9_Rb2Fd5,      ///< FSK, No Manchester, Rb = 2kbs, Fd = 5kHz, PN9 random modulation for testing

    FSK_Rb2Fd5,          ///< FSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
    FSK_Rb2_4Fd36,       ///< FSK, No Manchester, Rb = 2.4kbs,  Fd = 36kHz
    FSK_Rb4_8Fd45,       ///< FSK, No Manchester, Rb = 4.8kbs,  Fd = 45kHz
    FSK_Rb9_6Fd45,       ///< FSK, No Manchester, Rb = 9.6kbs,  Fd = 45kHz
    FSK_Rb19_2Fd9_6,     ///< FSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
    FSK_Rb38_4Fd19_6,    ///< FSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
    FSK_Rb57_6Fd28_8,    ///< FSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
    FSK_Rb125Fd125,      ///< FSK, No Manchester, Rb = 125kbs,  Fd = 125kHz

    GFSK_Rb2Fd5,         ///< GFSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
    GFSK_Rb2_4Fd36,      ///< GFSK, No Manchester, Rb = 2.4kbs,  Fd = 36kHz
    GFSK_Rb4_8Fd45,      ///< GFSK, No Manchester, Rb = 4.8kbs,  Fd = 45kHz
    GFSK_Rb9_6Fd45,      ///< GFSK, No Manchester, Rb = 9.6kbs,  Fd = 45kHz
    GFSK_Rb19_2Fd9_6,    ///< GFSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
    GFSK_Rb38_4Fd19_6,   ///< GFSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
    GFSK_Rb57_6Fd28_8,   ///< GFSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
    GFSK_Rb125Fd125,     ///< GFSK, No Manchester, Rb = 125kbs,  Fd = 125kHz

    OOK_Rb1_2Bw75,       ///< OOK, No Manchester, Rb = 1.2kbs,  Rx Bandwidth = 75kHz
    OOK_Rb2_4Bw335,      ///< OOK, No Manchester, Rb = 2.4kbs,  Rx Bandwidth = 335kHz
    OOK_Rb4_8Bw335,      ///< OOK, No Manchester, Rb = 4.8kbs,  Rx Bandwidth = 335kHz
    OOK_Rb9_6Bw335,      ///< OOK, No Manchester, Rb = 9.6kbs,  Rx Bandwidth = 335kHz
    OOK_Rb19_2Bw335,     ///< OOK, No Manchester, Rb = 19.2kbs, Rx Bandwidth = 335kHz
    OOK_Rb38_4Bw335,     ///< OOK, No Manchester, Rb = 38.4kbs, Rx Bandwidth = 335kHz
    OOK_Rb40Bw335        ///< OOK, No Manchester, Rb = 40kbs,   Rx Bandwidth = 335kHz
} T_MODEM_CONFIG_CHOICE;

bool RFM23BP__Init(void);
bool RFM23BP__SetFrequency(float centre, float afcPullInRange);
bool RFM23BP__SetModemConfig(T_MODEM_CONFIG_CHOICE index);
bool RFM23BP__Send(const uint8_t* data, uint8_t len);
bool RFM23BP__AppendTxBuf(const uint8_t* data, uint8_t len);
bool RFM23BP__FillTxBuf(const uint8_t* data, uint8_t len);

void RFM23BP__WriteRegister(T_RFM23BP taddress, uint8_t tdata);
void RFM23BP__WriteBurst(uint8_t taddress, const uint8_t* src, uint8_t len);
void RFM23BP__ReadBurst(uint8_t reg, uint8_t* dest, uint8_t len);
void RFM23BP__Reset();
void RFM23BP__Close(void);
void RFM23BP__SetPreambleLength(uint8_t nibbles);
void RFM23BP__SetSyncWords(const uint8_t* syncWords, uint8_t len);
void RFM23BP__SetHeaderTo(uint8_t to);
void RFM23BP__SetHeaderFrom(uint8_t from);
void RFM23BP__SetHeaderId(uint8_t id);
void RFM23BP__SetHeaderFlags(uint8_t flags);
void RFM23BP__SetHeaderFlags(uint8_t flags);
void RFM23BP__SetPromiscuous(bool promiscuous);
void RFM23BP__SetModemRegisters(const T_MODEM_CONFIG* config);
void RFM23BP__SetMode(uint8_t mode);
void RFM23BP__SetModeRx(void);
void RFM23BP__SetModeTx(void);
void RFM23BP__SetModeIdle(void);
void RFM23BP__SetTxPower(uint8_t power);
void RFM23BP__ResetFifos();
void RFM23BP__ResetRxFifo();
void RFM23BP__ResetTxFifo();
void RFM23BP__StartTransmit(void);
void RFM23BP__RestartTransmit(void);
void RFM23BP__ReadNextFragment(void);
void RFM23BP__SendNextFragment(void);
void RFM23BP__ClearRxBuf(void);
void RFM23BP__ClearTxBuf(void);
void RFM23BP__WaitPacketSent(void);
void RFM23BP__ClearBuffers(void);


uint8_t RFM23BP__HeaderTo(void);
uint8_t RFM23BP__HeaderFrom(void);
uint8_t RFM23BP__HeaderId(void);
uint8_t RFM23BP__HeaderFlags(void);
uint8_t RFM23BP__StatusRead(void);
uint8_t RFM23BP__RSSIRead(void);
uint8_t RFM23BP__ReadRegister(T_RFM23BP taddress);
uint8_t RFM23BP__ReadADC(uint8_t adcsel, uint8_t adcref, uint8_t adcgain, uint8_t adcoffs);

float RFM23BP__ReadTemperature(uint8_t tsrange, uint8_t tvoffs);

#endif /* RFM23BP_H_ */

