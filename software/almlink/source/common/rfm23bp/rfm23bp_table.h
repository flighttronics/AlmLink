/******************************************************************************
 rfm23bp.h for Almlink

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
 * @file rfm23bp.h
 * Header file for RFM23BP
 * @brief This section contains RFM23BP definitions
 *
 * @author Andreas Kingbäck
 * @version 0.00
 */

#if !defined(RFM23BP_TABLE_H_) || defined(CREATE_REGISTER_TABLE)
#define RFM23BP_TABLE_H_

#include <inttypes.h>

//-------------------------------------------------------------------------------------------------
//--------  M A C R O S  ------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#ifdef CREATE_REGISTER_TABLE
#define SYMBOL_DEF_RFM23BP(REGISTER_NAME, REGISTER_DESCRIPTION, REGISTER_ADDRESS)  \
                {REGISTER_DESCRIPTION, REGISTER_ADDRESS}
#else
#define SYMBOL_DEF_RFM23BP(REGISTER_NAME, REGISTER_DESCRIPTION, REGISTER_ADDRESS) REGISTER_NAME
#endif

//-------------------------------------------------------------------------------------------------
//--------  D E F I N E S  ------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
typedef struct
{
    unsigned char description[60];
    uint8_t address;
} T_RFM23BP_REGISTERS;


#ifdef CREATE_REGISTER_TABLE
static const const T_RFM23BP_REGISTERS rfm23bp_address_table[] =
{
#else
typedef enum
{
#endif
	SYMBOL_DEF_RFM23BP(DEVICE_TYPE,                        "Device Type",                                 0x00),
	SYMBOL_DEF_RFM23BP(VERSION_CODE,                       "Device Version",                              0x01),
	SYMBOL_DEF_RFM23BP(DEVICE_STATUS,                      "Device Status",                               0x02),
	SYMBOL_DEF_RFM23BP(INTERRUPT_STATUS1,                  "Interrupt Status 1",                          0x03),
	SYMBOL_DEF_RFM23BP(INTERRUPT_STATUS2,                  "Interrupt Status 2",                          0x04),
	SYMBOL_DEF_RFM23BP(INTERRUPT_ENABLE1,                  "Interrupt Enable 1",                          0x05),
	SYMBOL_DEF_RFM23BP(INTERRUPT_ENABLE2,                  "Interrupt Enable 2",                          0x06),
	SYMBOL_DEF_RFM23BP(OPERATING_MODE1 ,                   "Operating & Function Control 1",              0x07),
	SYMBOL_DEF_RFM23BP(OPERATING_MODE2,                    "Operating & Function Control 2",              0x08),
	SYMBOL_DEF_RFM23BP(OSCILLATOR_LOAD_CAPACITANCE,        "Crystal Oscillator Load Capacitance",         0x09),
	SYMBOL_DEF_RFM23BP(UC_OUTPUT_CLOCK,                    "Microcontroller Output Clock",                0x0A),
	SYMBOL_DEF_RFM23BP(GPIO_CONFIGURATION0, 	           "GPIO0 Configuration",                         0x0B),
	SYMBOL_DEF_RFM23BP(GPIO_CONFIGURATION1,                "GPIO1 Configuration",                         0x0C),
	SYMBOL_DEF_RFM23BP(GPIO_CONFIGURATION2,                "GPIO2 Configuration",                         0x0D),
	SYMBOL_DEF_RFM23BP(IO_PORT_CONFIGURATION,              "I/O Port Configuration",                      0x0E),
	SYMBOL_DEF_RFM23BP(ADC_CONFIGURATION,                  "ADC Configuration",                           0x0F),
	SYMBOL_DEF_RFM23BP(ADC_SENSOR_AMP_OFFSET,              "ADC Sensor Amplifier Offset",                 0x10),
	SYMBOL_DEF_RFM23BP(ADC_VALUE,                          "ADC Value",                                   0x11),
	SYMBOL_DEF_RFM23BP(TEMPERATURE_SENSOR_CALIBRATION,     "Temperature Sensor Control",                  0x12),
	SYMBOL_DEF_RFM23BP(TEMPERATURE_VALUE_OFFSET,           "Temperature Sensor Control",                  0x13),
	SYMBOL_DEF_RFM23BP(WAKEUP_TIMER_PERIOD1,               "Wake-Up Timer Period 1",                      0x14),
	SYMBOL_DEF_RFM23BP(WAKEUP_TIMER_PERIOD2,               "Wake-Up Timer Period 2",                      0x15),
	SYMBOL_DEF_RFM23BP(WAKEUP_TIMER_PERIOD3,               "Wake-Up Timer Period 3",                      0x16),
	SYMBOL_DEF_RFM23BP(WAKEUP_TIMER_VALUE1,                "Wake-Up Timer Value 1",                       0x17),
	SYMBOL_DEF_RFM23BP(WAKEUP_TIMER_VALUE2,                "Wake-Up Timer Value 2",                       0x18),
	SYMBOL_DEF_RFM23BP(LDC_MODE_DURATION,                  "Low-Duty Cycle Mode Duration",                0x19),
	SYMBOL_DEF_RFM23BP(LOW_BATTERY_DETECTOR_THRESHOLD,     "Low Battery Detector Threshold",              0x1A),
	SYMBOL_DEF_RFM23BP(BATTERY_VOLTAGE_LEVEL,              "Battery Voltage Level",                       0x1B),
	SYMBOL_DEF_RFM23BP(IF_FILTER_BANDWIDTH,                "IF Filter Bandwidth",                         0x1C),
	SYMBOL_DEF_RFM23BP(AFC_LOOP_GEARSHIFT_OVERRIDE,        "AFC Loop Gearshift Override",                 0x1D),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_GEARSHIFT_OVERRIDE,  "Clock Recovery Gearshift Override",           0x1F),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_OVERSAMPLING_RATE,   "Clock Recovery Oversampling Ratio",           0x20),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_OFFSET2,             "Clock Recovery Offset 2",                     0x21),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_OFFSET1,             "Clock Recovery Offset 1",                     0x22),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_OFFSET0,             "Clock Recovery Offset 0",                     0x23),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_TIMING_LOOP_GAIN1,   "AClock Recovery Timing Loop Gain 1",          0x24),
	SYMBOL_DEF_RFM23BP(CLOCK_RECOVERY_TIMING_LOOP_GAIN0,   "AClock Recovery Timing Loop Gain 0",          0x25),
	SYMBOL_DEF_RFM23BP(RSSI,                               "Received Signal Strength Indicator",          0x26),
	SYMBOL_DEF_RFM23BP(RSSI_THRESHOLD,                     "RSSI Threshold for Clear Channel Indicator",  0x27),

	// Not all after this point
	SYMBOL_DEF_RFM23BP(AFC_LIMITER,                        "AFC Limiter",                                 0x2A),
	SYMBOL_DEF_RFM23BP(OOK_COUNTER_VALUE_1,                "OOK Counter Value 1",                         0x2C),
    SYMBOL_DEF_RFM23BP(DATA_ACCESS_CONTROL,                "Data Access Control",                         0x30),
    SYMBOL_DEF_RFM23BP(HEADER_CONTROL1,                    "Header Control 1",                            0x32),
    SYMBOL_DEF_RFM23BP(HEADER_CONTROL2,                    "Header Control 2",                            0x33),
    SYMBOL_DEF_RFM23BP(PREAMBLE_LENGTH,                    "Preamble Length",                             0x34),
    SYMBOL_DEF_RFM23BP(SYNC_WORD3,                         "Sync Word 3",                                 0x36),
    SYMBOL_DEF_RFM23BP(SYNC_WORD2,                         "Sync Word 2",                                 0x37),
    SYMBOL_DEF_RFM23BP(SYNC_WORD1,                         "Sync Word 1",                                 0x38),
    SYMBOL_DEF_RFM23BP(SYNC_WORD0,                         "Sync Word 0",                                 0x39),
	SYMBOL_DEF_RFM23BP(TRANSMIT_HEADER3,                   "Transmit Header 3",                           0x3A),
	SYMBOL_DEF_RFM23BP(TRANSMIT_HEADER2,                   "Transmit Header 2",                           0x3B),
	SYMBOL_DEF_RFM23BP(TRANSMIT_HEADER1,                   "Transmit Header 1",                           0x3C),
	SYMBOL_DEF_RFM23BP(TRANSMIT_HEADER0,                   "Transmit Header 0",                           0x3D),
	SYMBOL_DEF_RFM23BP(PACKET_LENGTH,                      "Transmit Packet Length",                      0x3E),
    SYMBOL_DEF_RFM23BP(CHECK_HEADER3,                      "Check Header 3",                              0x3F),
	SYMBOL_DEF_RFM23BP(HEADER_ENABLE3,                     "Header Enable 3",                             0x43),
	SYMBOL_DEF_RFM23BP(RECEIVED_HEADER3,                   "Received Header 3",                           0x47),
	SYMBOL_DEF_RFM23BP(RECEIVED_HEADER2,                   "Received  Header2",                           0x48),
	SYMBOL_DEF_RFM23BP(RECEIVED_HEADER1,                   "Received Header 1",                           0x49),
	SYMBOL_DEF_RFM23BP(RECEIVED_HEADER0,                   "Received Header 0",                           0x4A),
	SYMBOL_DEF_RFM23BP(RECEIVED_PACKET_LENGTH,             "Received Packet Length",                      0x4B),
	SYMBOL_DEF_RFM23BP(CHARGE_PUMP_CURRENT_TRIMMING,       "?",                                           0x58),
	SYMBOL_DEF_RFM23BP(AGC_OVERRIDE1,                      "AGC Override 1",                              0x69),
	SYMBOL_DEF_RFM23BP(TX_POWER,                           "TX Power",                                    0x6D),
	SYMBOL_DEF_RFM23BP(TX_DATA_RATE1,                      "TX Data Rate 1",                              0x6E),
	SYMBOL_DEF_RFM23BP(TX_DATA_RATE0,                      "TX Data Rate 0",                              0x6F),
	SYMBOL_DEF_RFM23BP(FREQUENCY_OFFSET1,                  "Frequency Offset 1",                          0x73),
	SYMBOL_DEF_RFM23BP(FREQUENCY_OFFSET2,                  "Frequency Offset 1",                          0x74),
	SYMBOL_DEF_RFM23BP(FREQUENCY_BAND_SELECT,              "Frequency Band Select",                       0x75),
	SYMBOL_DEF_RFM23BP(NOMINAL_CARRIER_FREQUENCY1,         "Nominal Carrier Frequency 1",                 0x76),
	SYMBOL_DEF_RFM23BP(NOMINAL_CARRIER_FREQUENCY0,         "Nominal Carrier Frequency 0",                 0x77),
	SYMBOL_DEF_RFM23BP(TX_FIFO_CONTROL2,                   "TX FIFO Control 2 ",                          0x7D),
	SYMBOL_DEF_RFM23BP(RX_FIFO_CONTROL,                    "RX FIFO Control",                             0x7E),
	SYMBOL_DEF_RFM23BP(FIFO_ACCESS,                        "FIFO Access",                                 0x7F),
#ifdef CREATE_REGISTER_TABLE
};
#define RFM23BP_REGISTER_TABLE_SIZE         (sizeof(rfm23bp_address_table)/sizeof(rfm23bp_address_table[0]))
#define RFM23BP_REGISTER_TABLE_LAST_INDEX   (RFM23BP_TABLE_SIZE)
#else
} T_RFM23BP;
#endif

#endif /* RFM23BP_TABLE_H_ */

/*
#define RF22_REG_00_DEVICE_TYPE                         0x00
#define RF22_REG_01_VERSION_CODE                        0x01
#define RF22_REG_02_DEVICE_STATUS                       0x02
#define RF22_REG_03_INTERRUPT_STATUS1                   0x03
#define RF22_REG_04_INTERRUPT_STATUS2                   0x04
#define RF22_REG_05_INTERRUPT_ENABLE1                   0x05
#define RF22_REG_06_INTERRUPT_ENABLE2                   0x06
#define RF22_REG_07_OPERATING_MODE1                     0x07
#define RF22_REG_08_OPERATING_MODE2                     0x08
#define RF22_REG_09_OSCILLATOR_LOAD_CAPACITANCE         0x09
#define RF22_REG_0A_UC_OUTPUT_CLOCK                     0x0a
#define RF22_REG_0B_GPIO_CONFIGURATION0                 0x0b
#define RF22_REG_0C_GPIO_CONFIGURATION1                 0x0c
#define RF22_REG_0D_GPIO_CONFIGURATION2                 0x0d
#define RF22_REG_0E_IO_PORT_CONFIGURATION               0x0e
#define RF22_REG_0F_ADC_CONFIGURATION                   0x0f
#define RF22_REG_10_ADC_SENSOR_AMP_OFFSET               0x10
#define RF22_REG_11_ADC_VALUE                           0x11
#define RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION      0x12
#define RF22_REG_13_TEMPERATURE_VALUE_OFFSET            0x13
#define RF22_REG_14_WAKEUP_TIMER_PERIOD1                0x14
#define RF22_REG_15_WAKEUP_TIMER_PERIOD2                0x15
#define RF22_REG_16_WAKEUP_TIMER_PERIOD3                0x16
#define RF22_REG_17_WAKEUP_TIMER_VALUE1                 0x17
#define RF22_REG_18_WAKEUP_TIMER_VALUE2                 0x18
#define RF22_REG_19_LDC_MODE_DURATION                   0x19
#define RF22_REG_1A_LOW_BATTERY_DETECTOR_THRESHOLD      0x1a
#define RF22_REG_1B_BATTERY_VOLTAGE_LEVEL               0x1b
#define RF22_REG_1C_IF_FILTER_BANDWIDTH                 0x1c
#define RF22_REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE         0x1d
#define RF22_REG_1E_AFC_TIMING_CONTROL                  0x1e


#define RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE   0x1f
#define RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE    0x20
#define RF22_REG_21_CLOCK_RECOVERY_OFFSET2              0x21
#define RF22_REG_22_CLOCK_RECOVERY_OFFSET1              0x22
#define RF22_REG_23_CLOCK_RECOVERY_OFFSET0              0x23
#define RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1    0x24
#define RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0    0x25
#define RF22_REG_26_RSSI                                0x26
#define RF22_REG_27_RSSI_THRESHOLD                      0x27
#define RF22_REG_28_ANTENNA_DIVERSITY1                  0x28
#define RF22_REG_29_ANTENNA_DIVERSITY2                  0x29
#define RF22_REG_2A_AFC_LIMITER                         0x2a
#define RF22_REG_2B_AFC_CORRECTION_READ                 0x2b
#define RF22_REG_2C_OOK_COUNTER_VALUE_1                 0x2c
#define RF22_REG_2D_OOK_COUNTER_VALUE_2                 0x2d
#define RF22_REG_2E_SLICER_PEAK_HOLD                    0x2e
#define RF22_REG_30_DATA_ACCESS_CONTROL                 0x30
#define RF22_REG_31_EZMAC_STATUS                        0x31
#define RF22_REG_32_HEADER_CONTROL1                     0x32
#define RF22_REG_33_HEADER_CONTROL2                     0x33
#define RF22_REG_34_PREAMBLE_LENGTH                     0x34
#define RF22_REG_35_PREAMBLE_DETECTION_CONTROL1         0x35
#define RF22_REG_36_SYNC_WORD3                          0x36
#define RF22_REG_37_SYNC_WORD2                          0x37
#define RF22_REG_38_SYNC_WORD1                          0x38
#define RF22_REG_39_SYNC_WORD0                          0x39
#define RF22_REG_3A_TRANSMIT_HEADER3                    0x3a
#define RF22_REG_3B_TRANSMIT_HEADER2                    0x3b
#define RF22_REG_3C_TRANSMIT_HEADER1                    0x3c
#define RF22_REG_3D_TRANSMIT_HEADER0                    0x3d
#define RF22_REG_3E_PACKET_LENGTH                       0x3e
#define RF22_REG_3F_CHECK_HEADER3                       0x3f
#define RF22_REG_40_CHECK_HEADER2                       0x40
#define RF22_REG_41_CHECK_HEADER1                       0x41
#define RF22_REG_42_CHECK_HEADER0                       0x42
#define RF22_REG_43_HEADER_ENABLE3                      0x43
#define RF22_REG_44_HEADER_ENABLE2                      0x44
#define RF22_REG_45_HEADER_ENABLE1                      0x45
#define RF22_REG_46_HEADER_ENABLE0                      0x46
#define RF22_REG_47_RECEIVED_HEADER3                    0x47
#define RF22_REG_48_RECEIVED_HEADER2                    0x48
#define RF22_REG_49_RECEIVED_HEADER1                    0x49
#define RF22_REG_4A_RECEIVED_HEADER0                    0x4a
#define RF22_REG_4B_RECEIVED_PACKET_LENGTH              0x4b
#define RF22_REG_50_ANALOG_TEST_BUS_SELECT              0x50
#define RF22_REG_51_DIGITAL_TEST_BUS_SELECT             0x51
#define RF22_REG_52_TX_RAMP_CONTROL                     0x52
#define RF22_REG_53_PLL_TUNE_TIME                       0x53
#define RF22_REG_55_CALIBRATION_CONTROL                 0x55
#define RF22_REG_56_MODEM_TEST                          0x56
#define RF22_REG_57_CHARGE_PUMP_TEST                    0x57
#define RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING        0x58
#define RF22_REG_59_DIVIDER_CURRENT_TRIMMING            0x59
#define RF22_REG_5A_VCO_CURRENT_TRIMMING                0x5a
#define RF22_REG_5B_VCO_CALIBRATION                     0x5b
#define RF22_REG_5C_SYNTHESIZER_TEST                    0x5c
#define RF22_REG_5D_BLOCK_ENABLE_OVERRIDE1              0x5d
#define RF22_REG_5E_BLOCK_ENABLE_OVERRIDE2              0x5e
#define RF22_REG_5F_BLOCK_ENABLE_OVERRIDE3              0x5f
#define RF22_REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS  0x60
#define RF22_REG_61_CHANNEL_FILTER_COEFFICIENT_VALUE    0x61
#define RF22_REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL      0x62
#define RF22_REG_63_RC_OSCILLATOR_COARSE_CALIBRATION    0x63
#define RF22_REG_64_RC_OSCILLATOR_FINE_CALIBRATION      0x64
#define RF22_REG_65_LDO_CONTROL_OVERRIDE                0x65
#define RF22_REG_66_LDO_LEVEL_SETTINGS                  0x66
#define RF22_REG_67_DELTA_SIGMA_ADC_TUNING1             0x67
#define RF22_REG_68_DELTA_SIGMA_ADC_TUNING2             0x68
#define RF22_REG_69_AGC_OVERRIDE1                       0x69
#define RF22_REG_6A_AGC_OVERRIDE2                       0x6a
#define RF22_REG_6B_GFSK_FIR_FILTER_COEFFICIENT_ADDRESS 0x6b
#define RF22_REG_6C_GFSK_FIR_FILTER_COEFFICIENT_VALUE   0x6c
#define RF22_REG_6D_TX_POWER                            0x6d
#define RF22_REG_6E_TX_DATA_RATE1                       0x6e
#define RF22_REG_6F_TX_DATA_RATE0                       0x6f
#define RF22_REG_70_MODULATION_CONTROL1                 0x70
#define RF22_REG_71_MODULATION_CONTROL2                 0x71
#define RF22_REG_72_FREQUENCY_DEVIATION                 0x72
#define RF22_REG_73_FREQUENCY_OFFSET1                   0x73
#define RF22_REG_74_FREQUENCY_OFFSET2                   0x74
#define RF22_REG_75_FREQUENCY_BAND_SELECT               0x75
#define RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1          0x76
#define RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0          0x77
#define RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT    0x79
#define RF22_REG_7A_FREQUENCY_HOPPING_STEP_SIZE         0x7a
#define RF22_REG_7C_TX_FIFO_CONTROL1                    0x7c
#define RF22_REG_7D_TX_FIFO_CONTROL2                    0x7d
#define RF22_REG_7E_RX_FIFO_CONTROL                     0x7e
#define RF22_REG_7F_FIFO_ACCESS                         0x7f*/





