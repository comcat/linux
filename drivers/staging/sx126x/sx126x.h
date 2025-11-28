#ifndef __SX126X_H__
#define __SX126X_H__

#include <linux/types.h>

// SX126X physical layer properties

#define SX126X_PA_CONFIG_SX1261                       0x01
#define SX126X_PA_CONFIG_SX1262                       0x00

#define SF10										0xA
#define SF11										0xB
#define SF12										0xC

#define BW125										0x4
#define BW250										0x5
#define BW500										0x6
// LORA_BW062 = 3
// LORA_BW041 = 10
// LORA_BW031 = 2
// LORA_BW020 = 9
// LORA_BW015 = 1
// LORA_BW010 = 8
// LORA_BW007 = 0

#define CR45										0x1
#define CR46										0x2
#define CR47										0x3
#define CR48										0x4

#define SX126X_SYNCWORD_PUBLIC                       0x3444
#define SX126X_SYNCWORD_PRIVATE                      0x1424

#define SX126X_TXMODE_ASYNC                           0x01
#define SX126X_TXMODE_SYNC                            0x02
#define SX126X_TXMODE_BACK2RX                         0x04

#define SX126X_PKT_TYPE_GFSK						0x00
#define SX126X_PKT_TYPE_LORA						0x01
#define SX126X_PKT_TYPE_BPSK						0x02
#define SX126X_PKT_TYPE_LR_FHSS						0x03

#define	SX126X_MAX_LORA_SYMB_NUM_TIMEOUT			248

//SX126X_CMD_SET_SLEEP
#define SX126X_SLEEP_START_COLD                       0b00000000	//  2     2     sleep mode: cold start, configuration is lost (default)
#define SX126X_SLEEP_START_WARM                       0b00000100	//  2     2                 warm start, configuration is retained
#define SX126X_SLEEP_RTC_OFF                          0b00000000	//  0     0     wake on RTC timeout: disabled
#define SX126X_SLEEP_RTC_ON                           0b00000001	//  0     0                          enabled

//SX126X_CMD_SET_STANDBY
#define SX126X_STANDBY_RC                             0x00	//  7     0     standby mode: 13 MHz RC oscillator
#define SX126X_STANDBY_XOSC                           0x01	//  7     0                   32 MHz crystal oscillator

//Radio complete Wake-up Time with TCXO stabilisation time
#define RADIO_TCXO_SETUP_TIME                         5	// [ms]

//SX126X_CMD_SET_REGULATOR_MODE
#define SX126X_REGULATOR_LDO                          0x00	//  7     0     set regulator mode: LDO (default)
#define SX126X_REGULATOR_DC_DC                        0x01	//  7     0                         DC-DC

//SX126X_CMD_SET_DIO3_AS_TCXO_CTRL
#define SX126X_DIO3_OUTPUT_1_6                        0x00	//  7     0     DIO3 voltage output for TCXO: 1V6
#define SX126X_DIO3_OUTPUT_1_7                        0x01	//  7     0                                   1V7
#define SX126X_DIO3_OUTPUT_1_8                        0x02	//  7     0                                   1V8
#define SX126X_DIO3_OUTPUT_2_2                        0x03	//  7     0                                   2V2
#define SX126X_DIO3_OUTPUT_2_4                        0x04	//  7     0                                   2V4
#define SX126X_DIO3_OUTPUT_2_7                        0x05	//  7     0                                   2V7
#define SX126X_DIO3_OUTPUT_3_0                        0x06	//  7     0                                   3V0
#define SX126X_DIO3_OUTPUT_3_3                        0x07	//  7     0                                   3V3

//SX126X_CMD_SET_TX_PARAMS
#define SX126X_PA_RAMP_10U                            0x00	//  7     0     ramp time: 10 us
#define SX126X_PA_RAMP_20U                            0x01	//  7     0                20 us
#define SX126X_PA_RAMP_40U                            0x02	//  7     0                40 us
#define SX126X_PA_RAMP_80U                            0x03	//  7     0                80 us
#define SX126X_PA_RAMP_200U                           0x04	//  7     0                200 us
#define SX126X_PA_RAMP_800U                           0x05	//  7     0                800 us
#define SX126X_PA_RAMP_1700U                          0x06	//  7     0                1700 us
#define SX126X_PA_RAMP_3400U                          0x07	//  7     0                3400 us

//SX126X_CMD_CALIBRATE
#define SX126X_CALIBRATE_IMAGE_OFF                    0b00000000	//  6     6     image calibration: disabled
#define SX126X_CALIBRATE_IMAGE_ON                     0b01000000	//  6     6                        enabled
#define SX126X_CALIBRATE_ADC_BULK_P_OFF               0b00000000	//  5     5     ADC bulk P calibration: disabled
#define SX126X_CALIBRATE_ADC_BULK_P_ON                0b00100000	//  5     5                             enabled
#define SX126X_CALIBRATE_ADC_BULK_N_OFF               0b00000000	//  4     4     ADC bulk N calibration: disabled
#define SX126X_CALIBRATE_ADC_BULK_N_ON                0b00010000	//  4     4                             enabled
#define SX126X_CALIBRATE_ADC_PULSE_OFF                0b00000000	//  3     3     ADC pulse calibration: disabled
#define SX126X_CALIBRATE_ADC_PULSE_ON                 0b00001000	//  3     3                            enabled
#define SX126X_CALIBRATE_PLL_OFF                      0b00000000	//  2     2     PLL calibration: disabled
#define SX126X_CALIBRATE_PLL_ON                       0b00000100	//  2     2                      enabled
#define SX126X_CALIBRATE_RC13M_OFF                    0b00000000	//  1     1     13 MHz RC osc. calibration: disabled
#define SX126X_CALIBRATE_RC13M_ON                     0b00000010	//  1     1                                 enabled
#define SX126X_CALIBRATE_RC64K_OFF                    0b00000000	//  0     0     64 kHz RC osc. calibration: disabled
#define SX126X_CALIBRATE_RC64K_ON                     0b00000001	//  0     0                                 enabled

//SX126X_CMD_SET_CAD_PARAMS
#define SX126X_CAD_ON_1_SYMB                          0x00	//  7     0     number of symbols used for CAD: 1
#define SX126X_CAD_ON_2_SYMB                          0x01	//  7     0                                     2
#define SX126X_CAD_ON_4_SYMB                          0x02	//  7     0                                     4
#define SX126X_CAD_ON_8_SYMB                          0x03	//  7     0                                     8
#define SX126X_CAD_ON_16_SYMB                         0x04	//  7     0                                     16
#define SX126X_CAD_GOTO_STDBY                         0x00	//  7     0     after CAD is done, always go to STDBY_RC mode
#define SX126X_CAD_GOTO_RX                            0x01	//  7     0     after CAD is done, go to Rx mode if activity is detected

//SX126X_CMD_CALIBRATE_IMAGE
#define SX126X_CAL_IMG_430_MHZ_1                      0x6B
#define SX126X_CAL_IMG_430_MHZ_2                      0x6F
#define SX126X_CAL_IMG_470_MHZ_1                      0x75
#define SX126X_CAL_IMG_470_MHZ_2                      0x81
#define SX126X_CAL_IMG_779_MHZ_1                      0xC1
#define SX126X_CAL_IMG_779_MHZ_2                      0xC5
#define SX126X_CAL_IMG_863_MHZ_1                      0xD7
#define SX126X_CAL_IMG_863_MHZ_2                      0xDB
#define SX126X_CAL_IMG_902_MHZ_1                      0xE1
#define SX126X_CAL_IMG_902_MHZ_2                      0xE9

//SX126X_CMD_SET_PA_CONFIG
#define SX126X_PA_CONFIG_HP_MAX                       0x07
#define SX126X_PA_CONFIG_SX1268                       0x01
#define SX126X_PA_CONFIG_PA_LUT                       0x01

//SX126X_CMD_SET_RX_TX_FALLBACK_MODE
#define SX126X_RX_TX_FALLBACK_MODE_FS                 0x40	//  7     0     after Rx/Tx go to: FS mode
#define SX126X_RX_TX_FALLBACK_MODE_STDBY_XOSC         0x30	//  7     0                        standby with crystal oscillator
#define SX126X_RX_TX_FALLBACK_MODE_STDBY_RC           0x20	//  7     0                        standby with RC oscillator (default)
///////////////////////////////////////////////////////////
//SX126X_CMD_GET_STATUS
#define SX126X_STATUS_MODE_STDBY_RC                   0b00100000	//  6     4     current chip mode: STDBY_RC
#define SX126X_STATUS_MODE_STDBY_XOSC                 0b00110000	//  6     4                        STDBY_XOSC
#define SX126X_STATUS_MODE_FS                         0b01000000	//  6     4                        FS
#define SX126X_STATUS_MODE_RX                         0b01010000	//  6     4                        RX
#define SX126X_STATUS_MODE_TX                         0b01100000	//  6     4                        TX
#define SX126X_STATUS_DATA_AVAILABLE                  0b00000100	//  3     1     command status: packet received and data can be retrieved
#define SX126X_STATUS_CMD_TIMEOUT                     0b00000110	//  3     1                     SPI command timed out
#define SX126X_STATUS_CMD_INVALID                     0b00001000	//  3     1                     invalid SPI command
#define SX126X_STATUS_CMD_FAILED                      0b00001010	//  3     1                     SPI command failed to execute
#define SX126X_STATUS_TX_DONE                         0b00001100	//  3     1                     packet transmission done

//SX126X_CMD_GET_DEVICE_ERRORS
#define SX126X_PA_RAMP_ERR                           0b100000000	//  8     8     device errors: PA ramping failed
#define SX126X_PLL_LOCK_ERR                          0b001000000	//  6     6                    PLL failed to lock
#define SX126X_XOSC_START_ERR                        0b000100000	//  5     5                    crystal oscillator failed to start
#define SX126X_IMG_CALIB_ERR                         0b000010000	//  4     4                    image calibration failed
#define SX126X_ADC_CALIB_ERR                         0b000001000	//  3     3                    ADC calibration failed
#define SX126X_PLL_CALIB_ERR                         0b000000100	//  2     2                    PLL calibration failed
#define SX126X_RC13M_CALIB_ERR                       0b000000010	//  1     1                    RC13M calibration failed
#define SX126X_RC64K_CALIB_ERR                       0b000000001	//  0     0                    RC64K calibration failed

//SX126X_CMD_SET_TX_PARAMS
#define SX126X_PA_RAMP_10U                            0x00	//  7     0     ramp time: 10 us
#define SX126X_PA_RAMP_20U                            0x01	//  7     0                20 us
#define SX126X_PA_RAMP_40U                            0x02	//  7     0                40 us
#define SX126X_PA_RAMP_80U                            0x03	//  7     0                80 us
#define SX126X_PA_RAMP_200U                           0x04	//  7     0                200 us
#define SX126X_PA_RAMP_800U                           0x05	//  7     0                800 us
#define SX126X_PA_RAMP_1700U                          0x06	//  7     0                1700 us
#define SX126X_PA_RAMP_3400U                          0x07	//  7     0                3400 us

//SX126X_CMD_SET_DIO_IRQ_PARAMS
#define SX126X_IRQ_TIMEOUT                          0b1000000000	//  9     9     Rx or Tx timeout
#define SX126X_IRQ_CAD_DETECTED                     0b0100000000	//  8     8     channel activity detected
#define SX126X_IRQ_CAD_DONE                         0b0010000000	//  7     7     channel activity detection finished
#define SX126X_IRQ_CRC_ERR                          0b0001000000	//  6     6     wrong CRC received
#define SX126X_IRQ_HEADER_ERR                       0b0000100000	//  5     5     LoRa header CRC error
#define SX126X_IRQ_HEADER_VALID                     0b0000010000	//  4     4     valid LoRa header received
#define SX126X_IRQ_SYNCWORD_VALID                  	0b0000001000	//  3     3     valid sync word detected
#define SX126X_IRQ_PREAMBLE_DETECTED                0b0000000100	//  2     2     preamble detected
#define SX126X_IRQ_RX_DONE                          0b0000000010	//  1     1     packet received
#define SX126X_IRQ_TX_DONE                          0b0000000001	//  0     0     packet transmission completed
#define SX126X_IRQ_ALL                              0b1111111111	//  9     0     all interrupts
#define SX126X_IRQ_NONE                             0b0000000000	//  9     0     no interrupts

///////////////////////////////////////////////////////////

enum sx126x_ioctl_cmd {
	SX126X_IOCTL_CMD_SETUP_V0,
	SX126X_IOCTL_CMD_SETUP_V1,
	SX126X_IOCTL_CMD_GET_MODULATION,
	SX126X_IOCTL_CMD_SET_MODULATION,
	SX126X_IOCTL_CMD_GET_FREQ,
	SX126X_IOCTL_CMD_SET_FREQ,
	SX126X_IOCTL_CMD_GET_SF,
	SX126X_IOCTL_CMD_SET_SF,
	SX126X_IOCTL_CMD_GET_OPMODE,
	SX126X_IOCTL_CMD_SET_OPMODE,
	SX126X_IOCTL_CMD_GET_POWER,
	SX126X_IOCTL_CMD_SET_POWER,
	SX126X_IOCTL_CMD_GET_BW,
	SX126X_IOCTL_CMD_SET_BW,
	SX126X_IOCTL_CMD_GET_SYNCWORD,
	SX126X_IOCTL_CMD_SET_SYNCWORD,
	SX126X_IOCTL_CMD_GET_CRC,
	SX126X_IOCTL_CMD_SET_CRC,
	SX126X_IOCTL_CMD_GET_CR,
	SX126X_IOCTL_CMD_SET_CR,
};

enum sx126x_modulation {
	SX126X_MODULATION_FSK,
	SX126X_MODULATION_OOK,
	SX126X_MODULATION_LORA,
	SX126X_MODULATION_INVALID
};

/* the last 3 modes are only valid in lora mode */
enum sx126x_opmode {
	SX126X_OPMODE_SLEEP,
	SX126X_OPMODE_STANDBY,
	SX126X_OPMODE_FSTX,
	SX126X_OPMODE_TX,
	SX126X_OPMODE_FSRX,
	SX126X_OPMODE_RX,
	SX126X_OPMODE_RXCONTINUOS,
	SX126X_OPMODE_RXSINGLE,
	SX126X_OPMODE_CAD
};

enum sx126x_pa {
	SX126X_PA_RFO,
	SX126X_PA_PABOOST
};

struct sx126x_pkt {
	size_t len;
	size_t hdrlen;
	size_t payloadlen;

	int16_t rssi;
} __attribute__((packed));

#endif
