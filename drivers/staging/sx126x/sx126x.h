#ifndef __SX126X_H__
#define __SX126X_H__

#include <linux/types.h>

// SX126X physical layer properties
#define FREQ_STEP                       0.95367431640625

#define SX126X_PA_CONFIG_SX1261                       0x01
#define SX126X_PA_CONFIG_SX1262                       0x00

#define SF10										0xA
#define SF11										0xB
#define SF12										0xC

#define BW125										0x4
#define BW250										0x5
#define BW500										0x6

#define CR45										0x1
#define CR46										0x2
#define CR47										0x3

#define SX126X_SYNCWORD_PUBLIC                       0x3444
#define SX126X_SYNCWORD_PRIVATE                      0x1424

#define SX126x_TXMODE_ASYNC                           0x01
#define SX126x_TXMODE_SYNC                            0x02
#define SX126x_TXMODE_BACK2RX                         0x04

#define SX126X_PKT_TYPE_GFSK						0x00
#define SX126X_PKT_TYPE_LORA						0x01
#define SX126X_PKT_TYPE_BPSK						0x02
#define SX126X_PKT_TYPE_LR_FHSS						0x03

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
#define SX126X_IRQ_SYNCWORD_VALID                  0b0000001000	//  3     3     valid sync word detected
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

	__s16 snr;
	__s16 rssi;
	__u32 fei;
	__u8 crcfail;
} __attribute__((packed));

#endif
