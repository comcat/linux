/*
 *  Copyright (c) 2025 - 2035 MaiKe Labs
 *
 *  driver for sx126x/asr6500
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>

#include "sx126x_regs.h"
#include "sx126x.h"

#include "ccx.h"

//#define	SX126X_SHOW_IRQ_TIME		1
//#define SX126X_DEBUG_IRQ			1
/*
 * F1C:
 *  SPI  - SPI1 (PA0 ~ PA3)
 *  RST  - PE3
 *  DIO1 - PE4
 *  BUSY - PE5
*/
#define SX126X_DRIVERNAME	"sx126x"
#define SX126X_CLASSNAME	"sx126x"
#define SX126X_DEVICENAME	"radio%d"

#define MIN_PAYLOAD_LEN				18
#define MAX_PAYLOAD_LEN				128

#define DELAY_MS_BEFORE_CAD			500
#define	CAD_TIMEOUT_MS				1000
#define	TX_TIMEOUT_MS				800

#define	SINGLE_RX_WIN_MS			230		// max payload 50 bytes
#define	RX_DUTY_RX_PERIOD			230		// same as rx_single rx win
#define	RX_DUTY_SLEEP_PERIOD		230		// suggest pre_len 108 symbol

static int devmajor;
static struct class *devclass;

static unsigned bwmap[] = {7810, 15630, 31250, 62500, 125000, 250000, 500000};
static char *crmap[] = {NULL, "4/5", "4/6", "4/7", "4/8"};

/* Commands Interface */
typedef enum sx126x_commands_e
{
	SX126X_NOP						= 0x0,
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/*
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    // Operational Modes Functions
    SX126X_SIZE_SET_SLEEP                  = 2,
    SX126X_SIZE_SET_STANDBY                = 2,
    SX126X_SIZE_SET_FS                     = 1,
    SX126X_SIZE_SET_TX                     = 4,
    SX126X_SIZE_SET_RX                     = 4,
    SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_SIZE_SET_RX_DUTY_CYCLE          = 7,
    SX126X_SIZE_SET_CAD                    = 1,
    SX126X_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    SX126X_SIZE_SET_REGULATOR_MODE         = 2,
    SX126X_SIZE_CALIBRATE                  = 2,
    SX126X_SIZE_CALIBRATE_IMAGE            = 3,
    SX126X_SIZE_SET_PA_CFG                 = 5,
    SX126X_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    SX126X_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    SX126X_SIZE_GET_IRQ_STATUS             = 2,
    SX126X_SIZE_CLR_IRQ_STATUS             = 3,
    SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_SIZE_SET_RF_FREQUENCY           = 5,
    SX126X_SIZE_SET_PKT_TYPE               = 2,
    SX126X_SIZE_GET_PKT_TYPE               = 2,
    SX126X_SIZE_SET_TX_PARAMS              = 3,
    SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_SIZE_SET_MODULATION_PARAMS_BPSK = 5,
    SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    SX126X_SIZE_SET_PKT_PARAMS_BPSK        = 2,
    SX126X_SIZE_SET_PKT_PARAMS_LORA        = 7,
    SX126X_SIZE_SET_CAD_PARAMS             = 8,
    SX126X_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    SX126X_SIZE_GET_STATUS           = 1,
    SX126X_SIZE_GET_RX_BUFFER_STATUS = 2,
    SX126X_SIZE_GET_PKT_STATUS       = 2,
    SX126X_SIZE_GET_RSSI_INST        = 2,
    SX126X_SIZE_GET_STATS            = 2,
    SX126X_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    SX126X_SIZE_GET_DEVICE_ERRORS = 2,
    SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
    SX126X_SIZE_MAX_BUFFER        = 255,
    SX126X_SIZE_DUMMY_BYTE        = 1,
} sx126x_commands_size_t;

typedef enum sx126x_cad_symbs_e
{
	SX126X_CAD_01_SYMB = 0x00,
	SX126X_CAD_02_SYMB = 0x01,
	SX126X_CAD_04_SYMB = 0x02,
	SX126X_CAD_08_SYMB = 0x03,
	SX126X_CAD_16_SYMB = 0x04,
} sx126x_cad_symbs_t;

typedef enum sx126x_cad_exit_mode_e
{
	SX126X_CAD_ONLY = 0x00,
	SX126X_CAD_RX   = 0x01,
	SX126X_CAD_LBT  = 0x10,
} sx126x_cad_exit_mode_t;

typedef struct sx126x_cad_param_s
{
	sx126x_cad_symbs_t      sym_num;		// CAD number of symbols
	uint8_t                 det_pek;		// CAD peak detection
	uint8_t                 det_min;		// CAD minimum detection
	sx126x_cad_exit_mode_t	exit_mode;		// CAD exit mode
	uint32_t                timeout;		// CAD timeout value
} sx126x_cad_param_t;

typedef enum sx126x_rx_mode_e
{
	SX126X_RX_CON	= 0x00,
	SX126X_RX_SIN	= 0x01,
	SX126X_RX_DUTY	= 0x02,
	SX126X_RX_CAD	= 0x03,
} sx126x_rx_mode_t;

struct sx126x {
	struct device *chardevice;
	struct work_struct irq_work;
	struct spi_device *spi;
	struct gpio_desc *gpio_reset, *gpio_busy, *gpio_swctrl;
	u32 fosc;
	struct mutex mutex;
	int irq;

	struct list_head device_entry;
	dev_t devt;
	bool open;

	int dev_num;

	/* device state */
	//enum sx126x_opmode opmode;

	/* tx */
	wait_queue_head_t writewq;
	int transmitted;
	bool tx_active;

	/* rx */
	wait_queue_head_t readwq;
	struct kfifo out;

	/* rf param */
	bool _cad_on;
	size_t _preamble_len;
	u8 _sf;
	u8 _bw;
	u8 _cr;
	u8 _tx_power;
	u32 _tx_freq;
	bool _ldro;

	sx126x_cad_param_t cad_param;
	sx126x_rx_mode_t rx_mode;

	bool rx_boost;

	/* single rx window in ms */
	u32 rx_win;

	/* rx duty sleep win in ms */
	u32 rxduty_sleep;

	/* used by rx irq  */
	u8 irq_buf[MAX_PAYLOAD_LEN];
	size_t irq_plen;
	int16_t irq_rssi;
	struct sx126x_pkt irq_pkt;
	uint16_t irq_st;

	u8 tx_buf[MAX_PAYLOAD_LEN];

	/* rx cnt */
	u32 cnt_crc_err;
	u32 cnt_rx255;
	u32 cnt_rx;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* Internal frequency of the radio */
#define SX126X_XTAL_FREQ					32000000UL

/* Internal frequency of the radio */
#define SX126X_RTC_FREQ_IN_HZ				64000UL

/* Scaling factor used to perform fixed-point operations */
#define SX126X_PLL_STEP_SHIFT_AMOUNT		14

/* PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT */
#define SX126X_PLL_STEP_SCALED (SX126X_XTAL_FREQ >> (25 - SX126X_PLL_STEP_SHIFT_AMOUNT))

uint32_t sx126x_convert_freq_to_pll_step(uint32_t freq_in_hz)
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / SX126X_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - (steps_int * SX126X_PLL_STEP_SCALED);

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return (steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT) +
           ((( steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT) + (SX126X_PLL_STEP_SCALED >> 1)) /
             SX126X_PLL_STEP_SCALED);
}

uint32_t sx126x_convert_timeout_to_rtc_step(uint32_t timeout_in_ms)
{
    return (uint32_t)(timeout_in_ms * (SX126X_RTC_FREQ_IN_HZ / 1000));
}

int sx126x_set_standby(struct sx126x *dev, uint8_t cfg);

void sx126x_wait_on_busy(struct sx126x *dev)
{
	int val = -1;
	u32 cnt_10us = 0;
    do
    {
        val = gpiod_get_value(dev->gpio_busy);
		udelay(10);
		cnt_10us++;

    } while(1 == val && cnt_10us < 100000);

	if (cnt_10us >= 100000) {
		/* 1s */
		dev_info(&dev->spi->dev, "wait on busy timeout 1000ms! \n");

		sx126x_set_standby(dev, SX126X_STANDBY_RC);
	}
}

/* sx126x register and buffer api */
static int sx126x_read_reg(struct sx126x *dev, u16 reg, u8 *result, size_t len)
{
	int ret;
	u8 cmd[SX126X_SIZE_READ_REGISTER];

	cmd[0] = SX126X_READ_REGISTER;
	cmd[1] = (uint8_t) (reg >> 8);
	cmd[2] = (uint8_t) reg;
	cmd[3] = SX126X_NOP;

	sx126x_wait_on_busy(dev);
	ret = spi_write_then_read(dev->spi, cmd, SX126X_SIZE_READ_REGISTER, result, len);

	//dev_dbg(&dev->spi->dev, "read: @%02x %02x\n", reg, *result);
	return ret;
}

static int sx126x_write_reg(struct sx126x *dev, u16 reg, u8 *value, size_t len)
{
	u8 cmd[SX126X_SIZE_WRITE_REGISTER];
	int ret = 0;

	struct spi_transfer fifotransfers[] = {
		{.tx_buf = &cmd, .len = SX126X_SIZE_WRITE_REGISTER},
		{.tx_buf = value, .len = len},
	};

	cmd[0] = SX126X_WRITE_REGISTER;
	cmd[1] = (reg >> 8) & 0xff;
	cmd[2] = reg & 0xff;

	//dev_info(&dev->spi->dev, "reg write: %d\n", len);
	//print_hex_dump(KERN_DEBUG, NULL, DUMP_PREFIX_NONE, 16, 1, value, len, true);

	sx126x_wait_on_busy(dev);
	ret = spi_sync_transfer(dev->spi, fifotransfers, ARRAY_SIZE(fifotransfers));

	//dev_dbg(&dev->spi->dev, "write: @%02x %02x\n", reg, value[0]);

	return ret;
}

/* used by RX done */
int sx126x_stop_rtc(struct sx126x *dev)
{
	int ret = 0;
	u8 reg_val = 0;
	ret = sx126x_write_reg(dev, SX126X_REG_RTC_CTRL, &reg_val, 1);

	if (0 == ret) {
		ret = sx126x_read_reg(dev, SX126X_REG_EVT_CLR, &reg_val, 1);

		if (0 == ret) {
			reg_val |= SX126X_REG_EVT_CLR_TIMEOUT_MASK;
			ret = sx126x_write_reg(dev, SX126X_REG_EVT_CLR, &reg_val, 1);
		}
	}

	return ret;
}

static int sx126x_get_rxbuf_status(struct sx126x *dev, uint8_t *plen, uint8_t *rxbuf_start)
{
	uint8_t cmd[2] = {SX126X_GET_RX_BUFFER_STATUS, 0};
	uint8_t buf[2] = {0};
	int ret = 0;

	sx126x_wait_on_busy(dev);
	ret = spi_write_then_read(dev->spi, cmd, 2, buf, 2);

	*plen = buf[0];
	*rxbuf_start = buf[1];

	return ret;
}

static int sx126x_read_buf(struct sx126x *dev, void *buffer, size_t *len)
{
	u8 pktstart, rx_len, off, fifoaddr;
	u8 ptx[3];
	int ret = -1;
	unsigned readlen;

	size_t maxtransfer = spi_max_transfer_size(dev->spi);

	ret = sx126x_get_rxbuf_status(dev, &rx_len, &pktstart);

	if (rx_len == 255) {
		dev->cnt_rx255 += 1;
	}

	//dev_info(&(dev->spi->dev), "Rx: %d Bytes @ 0x%02x\n", rx_len, pktstart);

	if (rx_len >= MIN_PAYLOAD_LEN && rx_len <= MAX_PAYLOAD_LEN) {
		/* buffer is ok */
		for (off = 0; off < rx_len; off += maxtransfer) {

			readlen = min(maxtransfer, (size_t)(rx_len - off));
			fifoaddr = pktstart + off;

			ptx[0] = SX126X_READ_BUFFER;
			ptx[1] = fifoaddr;								/* offset */
			ptx[2] = SX126X_NOP;

			//dev_warn(&(dev->spi->dev), "FIFO read: %d from 0x%02x\n", readlen, fifoaddr);

			sx126x_wait_on_busy(dev);
			ret = spi_write_then_read(dev->spi, &ptx, 3, buffer + off, readlen);

			if (ret) {
				*len = 0;
				return ret;
			}
		}

		*len = rx_len;

	} else {

		/* do not read the buffer when rx_len is greater than MAX_PAYLOAD_LEN */
		*len = 0;
	}

	//print_hex_dump_bytes("", DUMP_PREFIX_NONE, buffer, rx_len);

	return ret;
}

static int sx126x_write_buf(struct sx126x *data, void *buffer, size_t len)
{
	int ret = 0;
	u8 cmd[SX126X_SIZE_WRITE_BUFFER] = {
		SX126X_WRITE_BUFFER,
		SX126X_NOP
	};

	struct spi_transfer fifotransfers[] = {
		{.tx_buf = &cmd, .len = SX126X_SIZE_WRITE_BUFFER},
		{.tx_buf = buffer, .len = len},
	};

	dev_info(&data->spi->dev, "FIFO write: %d\n", len);

	print_hex_dump(KERN_INFO, " | ", DUMP_PREFIX_NONE, 16, 1, buffer, len, true);

	ret = spi_sync_transfer(data->spi, fifotransfers, ARRAY_SIZE(fifotransfers));

	//if (memcmp(buffer, readbackbuff, len) != 0) {
	//	dev_err(&spi->dev, "FIFO readback doesn't match\n");
	//}
	return ret;
}

static int sx126x_get_rssi_inst(struct sx126x *dev, int16_t *rssi)
{
	int ret;
	u8 buf = 0;

	u8 cmd[SX126X_SIZE_GET_RSSI_INST] = {
        SX126X_GET_RSSI_INST,
        SX126X_NOP,
    };

	sx126x_wait_on_busy(dev);
	ret = spi_write_then_read(dev->spi, cmd, 2, &buf, 1);

	*rssi = -buf >> 1;

	return ret;
}

#if 0
static int sx126x_get_lora_stats(struct sx126x *dev, u16 *nb_pkt_rx, u16 *nb_pkt_crc_err
		u16 *nb_pkt_hdr_err)
{
    uint8_t cmd[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t buf[6] = { 0 };
	int ret = 0;

	sx126x_wait_on_busy(dev);
	ret = spi_write_then_read(spi, cmd, SX126X_SIZE_GET_STATS, &buf, 6);

	*nb_pkt_rx = (buf[0] << 8) | buf[1];
	*nb_pkt_crc_err = (buf[2] << 8) | buf[3];
	*nb_pkt_hdr_err = (buf[4] << 8) | buf[5];

	return ret;
}

static int sx126x_reset_stats(struct sx126x *dev)
{
    uint8_t cmd[SX126X_SIZE_RESET_STATS] = {
        SX126X_RESET_STATS,
		SX126X_NOP,
		SX126X_NOP,
		SX126X_NOP,
		SX126X_NOP,
		SX126X_NOP,
		SX126X_NOP
    };

	sx126x_wait_on_busy(dev);
	/* resets the value read by the command GetStats */
	return spi_write(dev->spi, cmd, SX126X_SIZE_RESET_STATS);
}
#endif

/*
 * pkt_type:
 *   gfsk: 0x0
 *   lora: 0x1
 *   bpsk: 0x2
 *   lr_fhss: 0x3
*/ 
uint8_t sx126x_get_pkt_type(struct sx126x *dev)
{
	uint8_t cmd[SX126X_SIZE_GET_PKT_TYPE];
	uint8_t rv = 9; 

	cmd[0] = SX126X_GET_PKT_TYPE;
	cmd[1] = SX126X_NOP;

	sx126x_wait_on_busy(dev);
	spi_write_then_read(dev->spi, cmd, SX126X_SIZE_GET_PKT_TYPE, &rv, 1);

	return rv;
}

int sx126x_set_pkt_type(struct sx126x *dev, uint8_t pkt_t)
{
	u8 cmd[SX126X_SIZE_SET_PKT_TYPE];

	cmd[0] = SX126X_SET_PKT_TYPE;
	cmd[1] = pkt_t;

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_PKT_TYPE);
}

int sx126x_set_lora_pkt_params(struct sx126x *dev, size_t pkt_len)
{
	uint8_t cmd[SX126X_SIZE_SET_PKT_PARAMS_LORA];
	int ret = 0;
	u8 reg_val = 0;

	cmd[0] = SX126X_SET_PKT_PARAMS;

	cmd[1] = (uint8_t)(dev->_preamble_len >> 8);
    cmd[2] = (uint8_t)(dev->_preamble_len);

    cmd[3] = 0x00;   /* Explicit Header */

    //cmd[4] = 0x30;   // 48 Bytes payload len
	cmd[4] = pkt_len;

    cmd[5] = 0x01;   /* crc on */
    cmd[6] = 0x00;   /* standard iq, no inverted iq */

	sx126x_wait_on_busy(dev);
	ret = spi_write(dev->spi, cmd, SX126X_SIZE_SET_PKT_PARAMS_LORA);

	// WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_SX1261-2_V1.2 15.4
	if (0 == ret) {
		ret = sx126x_read_reg(dev, SX126X_REG_IQ_POLARITY, &reg_val, 1);
		if (ret == 0) {
			reg_val |= (1 << 2);	/* bit 2 set to 1 when using standard IQ polarity */
			//reg_val &= ~( 1 << 2 );  // Bit 2 set to 0 when using inverted IQ polarity
			ret = sx126x_write_reg(dev, SX126X_REG_IQ_POLARITY, &reg_val, 1);
		}
	}

	return ret;
}

int sx126x_set_stop_rx_timer_on_preamble(struct sx126x *dev, bool enable)
{
	u8 cmd[SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE];

	cmd[0] = SX126X_SET_STOP_TIMER_ON_PREAMBLE;
	cmd[1] = enable;
	
	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE);
}

int sx126x_set_lora_symb_num_timeout(struct sx126x *dev, uint8_t symb_num)
{
	u8 cmd[SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT];
	u8 reg_val = 0;

    uint8_t exp = 0;
    uint8_t mant =
        (((symb_num > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : symb_num) + 1) >> 1;

	int ret = 0;

    while(mant > 31) {
        mant = (mant + 3) >> 2;
        exp++;
    }

	cmd[0] = SX126X_SET_LORA_SYMB_NUM_TIMEOUT;
	cmd[1] = mant << (2 * exp + 1);
	
	sx126x_wait_on_busy(dev);
	ret = spi_write(dev->spi, cmd, SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT);

	sx126x_read_reg(dev, SX126X_REG_LR_SYNCH_TIMEOUT, &reg_val, 1);
	printk("REG_LR_SYNCH_TIMEOUT: 0x%0X\n", reg_val);

	if (0 == ret && symb_num > 0) {

		reg_val = exp + (mant << 3);
		ret = sx126x_write_reg(dev, SX126X_REG_LR_SYNCH_TIMEOUT, &reg_val, 1);
	}

	sx126x_read_reg(dev, SX126X_REG_LR_SYNCH_TIMEOUT, &reg_val, 1);
	printk("REG_LR_SYNCH_TIMEOUT: 0x%0X\n", reg_val);

	return ret;
}

int sx126x_set_dio3_as_tcxo_ctrl(struct sx126x *dev, uint8_t volt, uint32_t timeout)
{
	uint8_t cmd[5];

	cmd[0] = SX126X_SET_DIO3_AS_TCXO_CTRL;
	cmd[1] = volt & 0x07;
	cmd[2] = (uint8_t) ((timeout >> 16) & 0xFF);
	cmd[3] = (uint8_t) ((timeout >> 8) & 0xFF);
	cmd[4] = (uint8_t) (timeout & 0xFF);

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL);
}

int sx126x_set_dio2_as_rfswitch_ctrl(struct sx126x *dev, uint8_t enable)
{
	u8 cmd[2];

	cmd[0] = SX126X_SET_DIO2_AS_RF_SWITCH_CTRL;
	cmd[1] = enable;
	
	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL);
}

static int sx126x_lora_tx_modulation_workaround(struct sx126x *dev, u8 bw)
{
    uint8_t reg_value = 0;

    int status = sx126x_read_reg(dev, SX126X_REG_TX_MODULATION, &reg_value, 1);

    if(status == 0) {

		if( bw == BW500 ) {
			reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
		} else {
			reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
		}

        status = sx126x_write_reg(dev, SX126X_REG_TX_MODULATION, &reg_value, 1);
    }
    return status;
}

static int sx126x_set_lora_modulation_params(struct sx126x *dev, int8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t cmd[SX126X_SIZE_SET_MODULATION_PARAMS_LORA];
	int ret;

	cmd[0] = SX126X_SET_MODULATION_PARAMS;
	cmd[1] = sf;
	cmd[2] = bw;
	cmd[3] = cr;
	cmd[4] = ldro & 0x01;

	sx126x_wait_on_busy(dev);
	ret = spi_write(dev->spi, cmd, 5);

    if(ret == 0) {
		// WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 15.1
        ret = sx126x_lora_tx_modulation_workaround(dev, bw);
    }

    return ret;
}

static int sx126x_set_rx_boost(struct sx126x *dev, bool st)
{
	uint8_t reg_val = 0;
	if (true == st) {
		reg_val = 0x96;
	} else {
		reg_val = 0x94;
	}

	return sx126x_write_reg(dev, SX126X_REG_RXGAIN, &reg_val, 1);
}

/* 
 * rv[6:4]: chip modes
 *   0x2: STBY_RC
 *   0x3: STBY_XOSC
 *   0x4: FS
 *   0x5: RX
 *   0x6: TX
 *
 * rv[3:1]: cmd status
 *   0x2: pkt rx ok and data can be check
 *   0x3: cmd timeout
 *   0x4: cmd err
 *   0x5: cmd failure
 *   0x6: cmd tx done
 *
 * ASR6500: 0x22
 * SX126x: 0x2A
 */ 
uint8_t sx126x_get_status(struct sx126x *dev)
{
    u8 cmd[SX126X_SIZE_GET_STATUS] = {
        SX126X_GET_STATUS,
    };
	u8 rv = 0xff;
    int ret = 0;

	sx126x_wait_on_busy(dev);
    ret = spi_write_then_read(dev->spi, cmd, SX126X_SIZE_GET_STATUS, &rv, 1);

	if (ret == 0) {
		return rv;
    } else {
		return -1;
	}
}

/*
 * cfg: STDBY_RC or STDBY_XOSC
 *   SX126X_STANDBY_XOSC
 *   SX126X_STANDBY_RC
 */
int sx126x_set_standby(struct sx126x *dev, uint8_t cfg)
{
	u8 cmd[SX126X_SIZE_SET_STANDBY] = {
		SX126X_SET_STANDBY,
		cfg
	};

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_STANDBY);
}

int sx126x_set_sleep(struct sx126x *dev, uint8_t cfg)
{
	u8 cmd[SX126X_SIZE_SET_SLEEP] = {
		SX126X_SET_SLEEP,
		cfg
	};

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_SLEEP);
}

uint16_t sx126x_get_irq_status(struct sx126x *dev)
{
	uint8_t cmd[2];
	uint8_t data[2] = {0};

	cmd[0] = SX126X_GET_IRQ_STATUS;
	cmd[1] = SX126X_NOP;

	sx126x_wait_on_busy(dev);
	spi_write_then_read(dev->spi, cmd, 2, data, 2);

	return (data[0] << 8) | data[1];
}

int sx126x_clear_irq_status(struct sx126x *dev, uint16_t irq)
{
	uint8_t cmd[SX126X_SIZE_CLR_IRQ_STATUS] = {
		SX126X_CLR_IRQ_STATUS,
		(uint8_t) (((uint16_t) irq >> 8) & 0x00FF),
		(uint8_t) ((uint16_t) irq & 0x00FF)
	};

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_CLR_IRQ_STATUS);
}

/*
 * timeout:
 *   0x0000: no timeout, RX Single mode and then return to STBY_RC mode
 *   0xFFFF: RX continuous mode.
 *   others: Timeout active. maximum timout is 262s. It's a rx window.
 *       LoRa: disable timer when preamble + Header are detected
 *       GFSK: disable timer when preamble + syncword are detected
 *       StopTimerOnPreamble(): disable timer only when preamble is detected
*/
int sx126x_set_rx(struct sx126x *dev, uint32_t timeout)
{
	uint8_t cmd[SX126X_SIZE_SET_RX];
	int ret;

	cmd[0] = SX126X_SET_RX;
	cmd[1] = (uint8_t) ((timeout >> 16) & 0xFF);
	cmd[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	cmd[3] = (uint8_t) (timeout & 0xFF);

	/* rx boosted gain, consumer more power and improve rx sensitivity */
	ret = sx126x_set_rx_boost(dev, dev->rx_boost);

	if (0 == ret) {
		sx126x_wait_on_busy(dev);
		ret = spi_write(dev->spi, cmd, SX126X_SIZE_SET_RX);
	}

	return ret;
}

int sx126x_set_rx_ms(struct sx126x *dev, uint32_t timeout_ms)
{
	uint8_t cmd[SX126X_SIZE_SET_RX];
	int ret;

	uint32_t timeout = sx126x_convert_timeout_to_rtc_step(timeout_ms);

	cmd[0] = SX126X_SET_RX;
	cmd[1] = (uint8_t) ((timeout >> 16) & 0xFF);
	cmd[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	cmd[3] = (uint8_t) (timeout & 0xFF);

	/* rx boosted gain, consumer more power and improve rx sensitivity */
	ret = sx126x_set_rx_boost(dev, dev->rx_boost);

	if (0 == ret) {
		sx126x_wait_on_busy(dev);
		ret = spi_write(dev->spi, cmd, SX126X_SIZE_SET_RX);
	}

	return ret;
}

int sx126x_set_tx(struct sx126x *dev, uint32_t timeout_ms)
{
	uint8_t cmd[SX126X_SIZE_SET_TX];
	//uint32_t tout = (uint32_t) (timeout_ms / 0.015625);
	uint32_t tout = sx126x_convert_timeout_to_rtc_step(timeout_ms);

	cmd[0] = SX126X_SET_TX;
	cmd[1] = (uint8_t) ((tout >> 16) & 0xFF);
	cmd[2] = (uint8_t) ((tout >> 8) & 0xFF);
	cmd[3] = (uint8_t) (tout & 0xFF);

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_TX);
}

/*
 * Need to reset when wakeup from sleep:
 *  SX126X_REG_IQ_POLARITY		0x0736
 *  SX126X_REG_TX_MODULATION	0x0889
 *  SX126X_REG_RXGAIN			0x08AC
 *
*/
int sx126x_set_rx_duty_cycle(struct sx126x *dev, uint32_t rx_win, uint32_t sleep_ms)
{
	uint8_t cmd[SX126X_SIZE_SET_RX_DUTY_CYCLE];

	u32 rx_win_step = sx126x_convert_timeout_to_rtc_step(rx_win);
	u32 sleep_ms_step = sx126x_convert_timeout_to_rtc_step(sleep_ms);

	cmd[0] = SX126X_SET_RX_DUTY_CYCLE;
	cmd[1] = (uint8_t) (rx_win_step >> 16);
	cmd[2] = (uint8_t) (rx_win_step >> 8);
	cmd[3] = (uint8_t) (rx_win_step >> 0);
	cmd[4] = (uint8_t) (sleep_ms_step >> 16);
	cmd[5] = (uint8_t) (sleep_ms_step >> 8);
	cmd[6] = (uint8_t) (sleep_ms_step >> 0);

	if (dev->rx_boost) {
		sx126x_write_reg(dev, 0x029F, (uint8_t []){0x01}, 1);
		sx126x_write_reg(dev, 0x02A0, (uint8_t []){0x08}, 1);
		sx126x_write_reg(dev, 0x02A1, (uint8_t []){0xAC}, 1);
	}

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_RX_DUTY_CYCLE);
}

int sx126x_set_cad(struct sx126x *dev)
{
    uint8_t cmd[SX126X_SIZE_SET_CAD] = {
        SX126X_SET_CAD,
    };

	sx126x_wait_on_busy(dev);
    return spi_write(dev->spi, cmd, SX126X_SIZE_SET_CAD);
}

/*
 * BW500:
 *  SF7: sym_num = 4, det_pek = 21, det_min = 10
 *  SF8: 4(0x2), 22, 10
 *  SF9: 4, 22, 10
 *  SF10: 4, 23, 10
 *  SF11: 4, 25, 10
 *  SF12: 8(0x3), 29, 10
 *
 * set_cad_params(4, 23, 10, SX126X_CAD_GOTO_STDBY, 0);
 *
 * BW125:
 *  SF7: 2(0x1),22,10
 *  SF8: 2,22,10
 *  SF9: 4,23,10
 *  SF10: 4,24,10
 *  SF11: 4,25,10
 *  SF12: 4,28,10
 */
int sx126x_set_cad_params(struct sx126x *dev, uint8_t sym_num, uint8_t det_pek,
							uint8_t det_min, uint8_t exit_mode, uint32_t timeout)
{
    uint8_t cmd[SX126X_SIZE_SET_CAD_PARAMS] = {
        SX126X_SET_CAD_PARAMS,
		sym_num,
		det_pek,
		det_min,
		exit_mode,
		(uint8_t)(timeout >> 16),
		(uint8_t)(timeout >> 8),
		(uint8_t)(timeout & 0xFF)
    };

	sx126x_wait_on_busy(dev);
    return spi_write(dev->spi, cmd, SX126X_SIZE_SET_CAD_PARAMS);
}

int sx126x_set_pa_config(struct sx126x *dev, u8 duty_cycle, u8 hp_max,
							u8 dev_sel, u8 lut)
{
	uint8_t cmd[5];

	cmd[0] = SX126X_SET_PA_CFG;
	cmd[1] = duty_cycle;
	cmd[2] = hp_max;
	cmd[3] = dev_sel;
	cmd[4] = lut;

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_PA_CFG);
}

int sx126x_set_over_current_protect(struct sx126x *dev, uint8_t value)
{
	return sx126x_write_reg(dev, SX126X_REG_OCP, &value, 1);
}

int sx126x_calibrate(struct sx126x *dev, uint8_t calib_param)
{
	u8 cmd[2] = {0};

	cmd[0] = SX126X_CALIBRATE;
	cmd[1] = calib_param;

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_CALIBRATE);
}

int sx126x_calibrate_image(struct sx126x *dev, uint32_t freq)
{
	u8 cmd[3] = {0};

	cmd[0] = SX126X_CALIBRATE_IMAGE;

	#if 0
	if (freq > 900000000) {
		cmd[1] = 0xE1;
		cmd[2] = 0xE9;
	} else if (freq > 850000000) {
		cmd[1] = 0xD7;
		cmd[2] = 0xD8;
	} else if (freq > 770000000) {
		cmd[1] = 0xC1;
		cmd[2] = 0xC5;
	#endif
	if (freq >= 470000000 && freq <= 510000000) {
		cmd[1] = 0x75;
		cmd[2] = 0x81;
	} else if (freq >= 430000000 && freq <= 440000000) {
		cmd[1] = 0x6B;
		cmd[2] = 0x6F;
	}

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_CALIBRATE_IMAGE);
}

int sx126x_set_regulator_mode(struct sx126x *dev, uint8_t mode)
{
	u8 cmd[2] = {0};

	cmd[0] = SX126X_SET_REGULATOR_MODE;
	cmd[1] = mode;

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_REGULATOR_MODE);
}

int sx126x_set_buffer_base_addr(struct sx126x *dev, uint8_t tx_addr, uint8_t rx_addr)
{
	uint8_t cmd[3];

	cmd[0] = SX126X_SET_BUFFER_BASE_ADDRESS;
	cmd[1] = tx_addr;
	cmd[2] = rx_addr;

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS);
}

int sx126x_set_tx_power(struct sx126x *dev, int8_t dbm)
{
    uint8_t cmd[3] = {0};
	int ret = -1;

	// sx1262 or sx1268
	if (dbm > 22) {
		dbm = 22;
	} else if (dbm < -3) {
		dbm = -3;
	}

	if (dbm <= 14) {
		ret = sx126x_set_pa_config(dev, 0x02, 0x02, 0x00, 0x01);
	} else {
		ret = sx126x_set_pa_config(dev, 0x04, 0x07, 0x00, 0x01);
	}

	if (0 == ret) {

		ret = sx126x_set_over_current_protect(dev, 0x38);		// set max current to 140mA
		//write_reg(SX126X_REG_OCP, 0x38);				// current max 160mA for the whole device

		cmd[0] = SX126X_SET_TX_PARAMS;
		cmd[1] = dbm;
		cmd[2] = SX126X_PA_RAMP_200U;				// TCXO
		// cmd[2] = RADIO_RAMP_20_US;				// XTAL

		if (0 == ret) {
			sx126x_wait_on_busy(dev);
			ret = spi_write(dev->spi, cmd, SX126X_SIZE_SET_TX_PARAMS);
		}
	}

	return ret;
}

int sx126x_set_syncword(struct sx126x *dev, u8 syncword)
{
    uint8_t buffer[2] = {0x00};
    uint8_t cmd[2] = {0x00};

	int ret = sx126x_read_reg(dev, SX126X_REG_LR_SYNCWORD, buffer, 2);

	/* reset val: 0x14 0x24 */
	dev_warn(&(dev->spi->dev), "syncword: 0x%0X 0x%0X\n", buffer[0], buffer[1]);

	if (ret >= 0) {
	#if 0
        buffer[0] = (buffer[0] & ~0xF0) + (syncword & 0xF0);
        buffer[1] = (buffer[1] & ~0xF0) + ((syncword & 0x0F) << 4);
	#else
		buffer[0] = syncword;
		buffer[1] = syncword;
	#endif

		ret = sx126x_write_reg(dev, SX126X_REG_LR_SYNCWORD, buffer, 2);

		dev_warn(&(dev->spi->dev), "Setting syncword to 0x%0X 0x%0X\n", buffer[0], buffer[1]);
	}

	sx126x_read_reg(dev, SX126X_REG_LR_SYNCWORD, cmd, 2);
	dev_warn(&dev->spi->dev, "Read syncword: 0x%0X 0x%0X\n", cmd[0], cmd[1]);

    return ret;
}

static int sx126x_set_freq(struct sx126x *dev, u32 freq)
{
	uint8_t cmd[SX126X_SIZE_SET_RF_FREQUENCY];
	int ret;

	ret = sx126x_calibrate_image(dev, freq);

	dev->_tx_freq = freq;

	//freq *= 33554432;;
	//do_div(freq, dev->fosc);
	freq = sx126x_convert_freq_to_pll_step(freq);

	cmd[0] = SX126X_SET_RF_FREQUENCY;
	cmd[1] = (uint8_t) ((freq >> 24) & 0xFF);
	cmd[2] = (uint8_t) ((freq >> 16) & 0xFF);
	cmd[3] = (uint8_t) ((freq >> 8) & 0xFF);
	cmd[4] = (uint8_t) (freq & 0xFF);

	if (0 == ret) {
		sx126x_wait_on_busy(dev);
		ret = spi_write(dev->spi, cmd, SX126X_SIZE_SET_RF_FREQUENCY);
	}

	return ret;
}

int sx126x_set_dio_irq_params(struct sx126x *dev, u16 irq_mask, u16 dio1_mask,
							 u16 dio2_mask, u16 dio3_mask)
{
	uint8_t cmd[SX126X_SIZE_SET_DIO_IRQ_PARAMS] = {
		SX126X_SET_DIO_IRQ_PARAMS,
		(uint8_t)(irq_mask >> 8),
		(uint8_t)(irq_mask >> 0),
		(uint8_t)(dio1_mask >> 8),
		(uint8_t)(dio1_mask >> 0),
		(uint8_t)(dio2_mask >> 8),
		(uint8_t)(dio2_mask >> 0),
		(uint8_t)(dio3_mask >> 8),
		(uint8_t)(dio3_mask >> 0)
	};

	sx126x_wait_on_busy(dev);
	return spi_write(dev->spi, cmd, SX126X_SIZE_SET_DIO_IRQ_PARAMS);
}

int sx126x_setup_v0(struct sx126x *data, uint32_t freq)
{
	int ret = 0;

	data->_sf = SF10;
	data->_bw = BW500;
	data->_cr = CR46;
	data->_ldro = true;

	data->_preamble_len = 8;

	data->tx_active = false;
	data->cnt_crc_err = 0;
	data->cnt_rx255 = 0;
	data->cnt_rx = 0;

	data->_tx_freq = freq;
	data->_tx_power = 22;

	sx126x_set_standby(data, SX126X_STANDBY_RC);
	printk("status = 0x%x\n", sx126x_get_status(data));

	ret = sx126x_set_pkt_type(data, SX126X_PKT_TYPE_LORA);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set pkt type failed %d\n", ret);
	else
		dev_info(&(data->spi->dev), "pkt type = 0x%X\n", sx126x_get_pkt_type(data));

	ret = sx126x_set_stop_rx_timer_on_preamble(data, true);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set top rx timer failed %d\n", ret);

	/* set to 1 ~ 8 can not rx go & t8 data */
	ret = sx126x_set_lora_symb_num_timeout(data, 0);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set lora symb failed %d\n", ret);

	ret = sx126x_set_lora_modulation_params(data, data->_sf, data->_bw, data->_cr, data->_ldro);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set modem param failed %d\n", ret);

	ret = sx126x_set_freq(data, data->_tx_freq);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set freq failed %d\n", ret);

	sx126x_set_tx_power(data, data->_tx_power);

	// set pkt param
	ret = sx126x_set_lora_pkt_params(data, 0xFF);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set pkt param failed %d\n", ret);

	data->_cad_on = true;
	data->cad_param.sym_num = SX126X_CAD_04_SYMB;
	data->cad_param.det_pek = 23;
	data->cad_param.det_min = 10;
	data->cad_param.exit_mode = SX126X_CAD_ONLY;
	data->cad_param.timeout = sx126x_convert_timeout_to_rtc_step(CAD_TIMEOUT_MS);

	data->rx_mode = SX126X_RX_SIN;
	data->rx_boost = true;
	data->rx_win = SINGLE_RX_WIN_MS;
	data->rxduty_sleep = RX_DUTY_SLEEP_PERIOD;

	/*
	 * BW500:
	 *  SF7: sym_num = 4, det_pek = 21, det_min = 10
	 *  SF8: 4(0x2), 22, 10
	 *  SF9: 4, 22, 10
	 *  SF10: 4, 23, 10
	 *  SF11: 4, 25, 10
	 *  SF12: 8(0x3), 29, 10
	 *
	 *
	 * BW125:
	 *  SF7: 2(0x1),22,10
	 *  SF8: 2,22,10
	 *  SF9: 4,23,10
	 *  SF10: 4,24,10
	 *  SF11: 4,25,10
	 *  SF12: 4,28,10
	 */
	ret = sx126x_set_cad_params(data, data->cad_param.sym_num, data->cad_param.det_pek,
								data->cad_param.det_min, data->cad_param.exit_mode,
								data->cad_param.timeout);
	if (ret != 0)
		dev_warn(&(data->spi->dev), "set cad param failed %d\n", ret);

	return ret;
}

bool sx126x_enter_rx(struct sx126x *dev)
{
	bool rv = false;
	u16 irq_on = 0;

	if (dev->tx_active == false) {

		switch(dev->rx_mode) {
			case SX126X_RX_SIN:
				sx126x_set_rx_ms(dev, dev->rx_win);
				irq_on = SX126X_IRQ_ALL;
				break;
			case SX126X_RX_DUTY:
				sx126x_set_standby(dev, SX126X_STANDBY_RC);
				sx126x_set_rx_duty_cycle(dev, dev->rx_win, dev->rxduty_sleep);
				sx126x_set_stop_rx_timer_on_preamble(dev, false);
				//irq_on = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR;
				irq_on = SX126X_IRQ_ALL;
				break;
			case SX126X_RX_CON:
			default:
				 /* set Rx Continuous mode, but also generate the irq timeout */
				sx126x_set_rx(dev, 0xFFFFFF);
				//irq_on = SX126X_IRQ_ALL;
				#if 0
				irq_on = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR |
							SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_VALID |
							SX126X_IRQ_HEADER_ERR;
				#endif
				irq_on = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR |
							SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_ERR;
				break;
		}

		sx126x_set_dio_irq_params(dev,
						irq_on,
						irq_on,
						SX126X_IRQ_NONE,
						SX126X_IRQ_NONE);

		sx126x_clear_irq_status(dev, SX126X_IRQ_ALL);

		rv = true;

	} else {

		rv = false;
	}

	return rv;
}

static void sx126x_start_cad_after_delay(struct sx126x *dev, uint16_t ms)
{	
	mdelay(ms);
	sx126x_set_cad(dev);
}

static int sx126x_send_tx_buf(struct sx126x *dev)
{
	int ret = -1;
	size_t p_len = dev->tx_buf[3] + 6;

	/* reset the devid */
	dev->tx_buf[3] = 0;

	update_crc(dev->tx_buf, p_len);

	/* computer the crc and mic */

	if (false == dev->tx_active) {
		dev->tx_active = true;

		ret = sx126x_set_lora_pkt_params(dev, p_len);

		ret = sx126x_write_buf(dev, dev->tx_buf, p_len);

        ret = sx126x_lora_tx_modulation_workaround(dev, dev->_bw);

		if (dev->_cad_on) {
			sx126x_set_dio_irq_params(dev,
				//SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE,
				//SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE,
				SX126X_IRQ_ALL,
				SX126X_IRQ_ALL,
				SX126X_IRQ_NONE,
				SX126X_IRQ_NONE);

			sx126x_clear_irq_status(dev, SX126X_IRQ_ALL);

			dev->cad_param.exit_mode = SX126X_CAD_LBT;
			ret = sx126x_set_cad_params(dev, dev->cad_param.sym_num, dev->cad_param.det_pek,
										dev->cad_param.det_min, dev->cad_param.exit_mode,
										dev->cad_param.timeout);

			sx126x_start_cad_after_delay(dev, DELAY_MS_BEFORE_CAD);

		} else {
			sx126x_set_dio_irq_params(dev,
									SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
									SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
									SX126X_IRQ_NONE,
									SX126X_IRQ_NONE);

			sx126x_clear_irq_status(dev, SX126X_IRQ_ALL);

			ret = sx126x_set_tx(dev, TX_TIMEOUT_MS);
		}

	} else {

		dev_warn(dev->chardevice, "TX is active\n");
		ret = -1;
	}

	return ret;
}

static int sx126x_send(struct sx126x *dev, uint8_t *buf, size_t len, uint8_t mode)
{
	int ret = -1;
	u32 cnt_100us = 0;

	if (false == dev->tx_active) {
		dev->tx_active = true;

		ret = sx126x_set_lora_pkt_params(dev, len);

		ret = sx126x_write_buf(dev, buf, len);

        ret = sx126x_lora_tx_modulation_workaround(dev, dev->_bw);

		if (dev->_cad_on) {
			//carrier_sense();
			#if 0
			sx126x_set_dio_irq_params(dev,
									SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE | SX126X_IRQ_TX_DONE |
									SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR,
									SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE | SX126X_IRQ_TX_DONE |
									SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR,
									SX126X_IRQ_NONE,
									SX126X_IRQ_NONE);
			#else
			sx126x_set_dio_irq_params(dev,
									SX126X_IRQ_ALL,
									SX126X_IRQ_ALL,
									SX126X_IRQ_NONE,
									SX126X_IRQ_NONE);
			#endif
			sx126x_clear_irq_status(dev, SX126X_IRQ_ALL);


			dev->cad_param.exit_mode = SX126X_CAD_LBT;
			ret = sx126x_set_cad_params(dev, dev->cad_param.sym_num, dev->cad_param.det_pek,
										dev->cad_param.det_min, dev->cad_param.exit_mode,
										dev->cad_param.timeout);

			sx126x_start_cad_after_delay(dev, DELAY_MS_BEFORE_CAD);
		} else {
			sx126x_set_dio_irq_params(dev,
									SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
									SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
									SX126X_IRQ_NONE,
									SX126X_IRQ_NONE);

			sx126x_clear_irq_status(dev, SX126X_IRQ_ALL);

			ret = sx126x_set_tx(dev, TX_TIMEOUT_MS);

			if (mode & SX126X_TXMODE_SYNC) {

				/* waitting the flag to false */
				do
				{
					udelay(100);
					cnt_100us++;

				} while(true == dev->tx_active && cnt_100us < 5000);

				if (cnt_100us >= 5000) {
					dev_err(dev->chardevice, "TX extended 500ms!\n");
					dev->tx_active = false;
					ret = -2;
				}
			}

		}

	} else {

		dev_warn(dev->chardevice, "TX is active\n");
		ret = -1;
	}

	return ret;
}

void sx126x_reset(struct sx126x *data)
{
	/*
	 * reset the sx126x
	 * reset pin is set to ACTIVE_LOW, so:
	 *  gpio_set(1) is LOW
	 *  gpio_set(0) is HIGH
	*/
	gpiod_set_value(data->gpio_reset, 1);
	mdelay(100);
	gpiod_set_value(data->gpio_reset, 0);
	mdelay(100);
}

int sx126x_cfg_tx_clamp(struct sx126x *data)
{
	u8 reg_val = 0;
	int ret = sx126x_read_reg(data, SX126X_REG_TX_CLAMP_CFG, &reg_val, 1);
	if (ret >= 0) {
		//reg_val |= SX126X_REG_TX_CLAMP_CFG_MASK;
		reg_val |= 0x1E;
		ret = sx126x_write_reg(data, SX126X_REG_TX_CLAMP_CFG, &reg_val, 1);
	}

	return ret;
}

void sx126x_workaround_ant_mismatch(struct sx126x *data)
{
    /*
     * Better Resistance of the SX1262 Tx to Antenna Mismatch
     * see DS_SX1261-2_V1.2 datasheet chapter 15.2
     * RegTxClampConfig = @address 0x08D8
     *
     * The register modification must be done
     * after a Power On Reset, or a wake-up
     * from cold Start
    */

    //spi_write_reg(data->spi, 0x08D8, read_reg(0x08D8) | 0x1E);
	sx126x_cfg_tx_clamp(data);
}

/*
 * api file located at: /sys/class/sx126x/sx126x0
 *
*/


/*
 * rv[6:4]: chip modes
 *   0x2: STBY_RC
 *   0x3: STBY_XOSC
 *   0x4: FS
 *   0x5: RX
 *   0x6: TX
 *
 * rv[3:1]: cmd status
 *   0x2: pkt rx ok and data can be check
 *   0x3: cmd timeout
 *   0x4: cmd err
 *   0x5: cmd failure
 *   0x6: cmd tx done
 */
static char *cmmap[] = {NULL, "RFU", "STBY_RC", "STBY_XOSC", "FS", "RX", "TX"};

static ssize_t sx126x_status_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	uint8_t rv = sx126x_get_status(data);

	int st_i = (rv >> 4) & 0x7;

	dev_info(data->chardevice, "status: 0x%0X (%d, %d)\n", rv, st_i, (rv >> 1) & 0x7);

	return sprintf(buf, "%s\n", cmmap[st_i]);
}

static ssize_t sx126x_status_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);

	if (strcmp(buf, "RX\n") == 0) {

		dev_info(data->chardevice, "Enter RX\n");
		sx126x_enter_rx(data);

	} else if (strcmp(buf, "TX\n") == 0) {

		dev_info(data->chardevice, "Enter TX\n");
		sx126x_send_tx_buf(data);

	} else if (strcmp(buf, "FS\n") == 0) {

		dev_info(data->chardevice, "Enter FS\n");

	} else if (strcmp(buf, "STBY_RC\n") == 0) {

		dev_info(data->chardevice, "Enter STBY_RC\n");
		sx126x_set_standby(data, SX126X_STANDBY_RC);

	} else if (strcmp(buf, "STBY_XOSC\n") == 0) {

		dev_info(data->chardevice, "Enter STBY_XOSC\n");
		sx126x_set_standby(data, SX126X_STANDBY_XOSC);
	}

	mutex_unlock(&data->mutex);

	return count;
}

static DEVICE_ATTR(status, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_status_show, sx126x_status_store);

static char *rmmap[] = {"RX_CON", "RX_SIN", "RX_DUTY", "RX_CAD"};

static ssize_t sx126x_rx_mode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	//dev_info(data->chardevice, "rx_mode: %d\n", data->rx_mode);

	return sprintf(buf, "%s\n", rmmap[data->rx_mode]);
}

static ssize_t sx126x_rx_mode_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);

	if (strcmp(buf, "RX_CON\n") == 0) {

		dev_info(data->chardevice, "Enter rx continous\n");

		data->rx_mode = SX126X_RX_CON;

	} else if (strcmp(buf, "RX_SIN\n") == 0) {

		dev_info(data->chardevice, "Enter rx single\n");

		data->rx_mode = SX126X_RX_SIN;

	} else if (strcmp(buf, "RX_DUTY\n") == 0) {

		dev_info(data->chardevice, "Enter rx duty\n");

		data->rx_mode = SX126X_RX_DUTY;

	} else if (strcmp(buf, "RX_CAD\n") == 0) {

		dev_info(data->chardevice, "Enter rx cad\n");

		data->rx_mode = SX126X_RX_CAD;

	} else {

		dev_info(data->chardevice, "Unknown rx mode, enter rx single\n");

		data->rx_mode = SX126X_RX_SIN;
	}

	sx126x_enter_rx(data);

	mutex_unlock(&data->mutex);

	return count;
}

static DEVICE_ATTR(rx_mode, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_rx_mode_show, sx126x_rx_mode_store);

static ssize_t sx126x_rx_win_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d ms\n", data->rx_win);
}

static ssize_t sx126x_rx_win_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	u32 rx_win;

	if (kstrtou32(buf, 10, &rx_win)) {
		goto out;
	}
	mutex_lock(&data->mutex);

	data->rx_win = rx_win;

	sx126x_enter_rx(data);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(rx_win, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_rx_win_show, sx126x_rx_win_store);

static ssize_t sx126x_rxduty_sleep_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d ms\n", data->rxduty_sleep);
}

static ssize_t sx126x_rxduty_sleep_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	u32 rxduty_sleep;

	if (kstrtou32(buf, 10, &rxduty_sleep)) {
		goto out;
	}
	mutex_lock(&data->mutex);

	data->rxduty_sleep = rxduty_sleep;

	sx126x_enter_rx(data);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(rxduty_sleep, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_rxduty_sleep_show, sx126x_rxduty_sleep_store);

static ssize_t sx126x_rx_boost_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->rx_boost);
}

static ssize_t sx126x_rx_boost_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	u32 rx_boost;

	if (kstrtou32(buf, 10, &rx_boost)) {
		goto out;
	}
	mutex_lock(&data->mutex);

	data->rx_boost = rx_boost;

	sx126x_enter_rx(data);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(rx_boost, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_rx_boost_show, sx126x_rx_boost_store);

static ssize_t sx126x_tx_buf_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	int i, offset = 0;
	size_t len = data->tx_buf[3];

	//printk("%d\n", len);
	//printk("len = %d\n", strlen(data->tx_buf));

	for (i = 0; i < len; i++) {

		//printk("%02x", data->tx_buf[i]);

        offset += sprintf(buf + offset, "%02x", data->tx_buf[i]);

		//printk("%s\n", buf);
	}

	buf[len*2] = '\n';

	return len*2+1;
}

static ssize_t sx126x_tx_buf_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);

	uint8_t rv = sx126x_get_status(data);
	int st_i = (rv >> 4) & 0x7;

	char ci[3] = {0};

	uint8_t tx_b;

	int i = 0, ret;

	size_t d_len = strlen(buf) - 1;

	uint8_t *p = data->tx_buf;

	memset(data->tx_buf, 0, MAX_PAYLOAD_LEN);

	//printk("%s", buf);

	if (0 != d_len % 2) {
		//printk("len = %d\n", strlen(buf));
		dev_err(&data->spi->dev, "tx_len is incorrect: %d\n", d_len);
		goto out;
	}

	while (buf[i] != 0 && buf[i] != '\n') {

		ci[0] = buf[i];
		ci[1] = buf[i+1];

		//printk("%s", ci);
		ret = kstrtou8(ci, 16, &tx_b);

		*p++ = tx_b;

		i += 2;
		//printk("%02X", tx_b);
	}

	/* save the pkt_len to dev_id */
	data->tx_buf[3] = i/2;

	if (st_i != SX126X_CMODE_STBY_RC) {

		sx126x_set_standby(data, SX126X_STANDBY_RC);
	}

	mutex_lock(&data->mutex);

	//sx126x_send(data, data->tx_buf, d_len+6, SX126X_TXMODE_SYNC);

	sx126x_send_tx_buf(data);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(tx_buf, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_tx_buf_show, sx126x_tx_buf_store);

static ssize_t sx126x_freq_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", data->_tx_freq);
}

static ssize_t sx126x_freq_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	u64 freq;

	if (kstrtou64(buf, 10, &freq)) {
		goto out;
	}
	mutex_lock(&data->mutex);
	sx126x_set_freq(data, freq);
	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(freq, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_freq_show, sx126x_freq_store);

static ssize_t sx126x_rssi_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int16_t rssi = 0;

	sx126x_get_rssi_inst(data, &rssi);

	return sprintf(buf, "%d\n", rssi);
}

static DEVICE_ATTR(rssi, S_IRUSR | S_IRGRP | S_IROTH, sx126x_rssi_show, NULL);

static ssize_t sx126x_crc_err_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->cnt_crc_err);
}

static DEVICE_ATTR(cnt_crc_err, S_IRUSR | S_IRGRP | S_IROTH, sx126x_crc_err_show, NULL);

static ssize_t sx126x_rx255_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->cnt_rx255);
}

static DEVICE_ATTR(cnt_rx255, S_IRUSR | S_IRGRP | S_IROTH, sx126x_rx255_show, NULL);

static ssize_t sx126x_rx_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->cnt_rx);
}

static DEVICE_ATTR(cnt_rx, S_IRUSR | S_IRGRP | S_IROTH, sx126x_rx_show, NULL);

static ssize_t sx126x_pre_len_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->_preamble_len);
}

static ssize_t sx126x_pre_len_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int pre_len;
	if (kstrtoint(buf, 10, &pre_len)) {
		goto out;
	}

	dev_info(data->chardevice, "setting preamble_len to %u\n", pre_len);

	mutex_lock(&data->mutex);
	data->_preamble_len = pre_len;

	sx126x_set_lora_pkt_params(data, 0xFF);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(pre_len, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_pre_len_show,
		   sx126x_pre_len_store);

static ssize_t sx126x_irq_st_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	u16 irq = sx126x_get_irq_status(data);

	return sprintf(buf, "0x%03X\n", irq);
}

static ssize_t sx126x_irq_st_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int irq_st;
	if (kstrtoint(buf, 10, &irq_st)) {
		goto out;
	}

	dev_info(data->chardevice, "clear irq\n");

	mutex_lock(&data->mutex);

	sx126x_clear_irq_status(data, SX126X_IRQ_ALL);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(irq_st, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_irq_st_show,
		   sx126x_irq_st_store);

static ssize_t sx126x_cad_on_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->_cad_on);
}

static ssize_t sx126x_cad_on_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int cad_on;
	if (kstrtoint(buf, 10, &cad_on)) {
		goto out;
	}

	dev_info(data->chardevice, "setting cad_on to %u\n", cad_on);

	mutex_lock(&data->mutex);
	data->_cad_on = cad_on;
	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(cad_on, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_cad_on_show,
		   sx126x_cad_on_store);

static ssize_t sx126x_dbm_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->_tx_power);
}

static ssize_t sx126x_dbm_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int dbm = 0;
	int ret = 0;

	if (kstrtoint(buf, 10, &dbm)) {
		goto out;
	}

	dev_info(data->chardevice, "setting dbm to %u\n", dbm);

	mutex_lock(&data->mutex);

	ret = sx126x_set_tx_power(data, dbm);

	if (0 == ret) {
		data->_tx_power = dbm;
	} else {
		dev_info(data->chardevice, "setting dbm failed\n");
	}

	mutex_unlock(&data->mutex);
 out:
	return count;
}

static DEVICE_ATTR(dbm, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_dbm_show,
		   sx126x_dbm_store);

static ssize_t sx126x_sf_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->_sf);
}

static int sx126x_set_sf(struct sx126x *data, unsigned sf)
{
	dev_info(data->chardevice, "setting spreading factor to %u\n", sf);

	data->_sf = sf;
	sx126x_set_lora_modulation_params(data, data->_sf, data->_bw, data->_cr, data->_ldro);

	return 0;
}

static ssize_t sx126x_sf_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int sf;
	if (kstrtoint(buf, 10, &sf)) {
		goto out;
	}
	mutex_lock(&data->mutex);
	sx126x_set_sf(data, sf);
	mutex_unlock(&data->mutex);
 out:
	return count;
}

static DEVICE_ATTR(sf, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_sf_show,
		   sx126x_sf_store);

static ssize_t sx126x_bw_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&data->mutex);
	ret = sprintf(buf, "%d\n", bwmap[data->_bw]);
	mutex_unlock(&data->mutex);

	return ret;
}

static int sx126x_set_bw(struct sx126x *data, unsigned bw){

	dev_info(data->chardevice, "setting BW to %u\n", bw);

	data->_bw = bw;
	sx126x_set_lora_modulation_params(data, data->_sf, data->_bw, data->_cr, data->_ldro);

	return 0;
}

static ssize_t sx126x_bw_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int bw = BW500;

	if (kstrtoint(buf, 10, &bw)) {
		goto out;
	}

	mutex_lock(&data->mutex);
	sx126x_set_bw(data, bw);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(bw, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_bw_show,
		   sx126x_bw_store);

static ssize_t sx126x_cr_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&data->mutex);
	ret = sprintf(buf, "%s\n", crmap[data->_cr]);
	mutex_unlock(&data->mutex);

	return ret;
}

static int sx126x_set_cr(struct sx126x *data, unsigned cr)
{
	dev_info(data->chardevice, "setting CR to %u\n", cr);

	data->_cr = cr;

	sx126x_set_lora_modulation_params(data, data->_sf, data->_bw, data->_cr, data->_ldro);

	return 0;
}

static ssize_t sx126x_cr_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int cr = CR46;

	if (kstrtoint(buf, 10, &cr)) {
		goto out;
	}

	mutex_lock(&data->mutex);
	sx126x_set_cr(data, cr);
	mutex_unlock(&data->mutex);

out:
	return count;
}

static DEVICE_ATTR(cr, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
				   sx126x_cr_show, sx126x_cr_store);

/* linux driver api */
static int sx126x_dev_open(struct inode *inode, struct file *file)
{
	struct sx126x *data;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(data, &device_list, device_entry) {
		if (data->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("sx126x: nothing for minor %d\n", iminor(inode));
		goto err_notfound;
	}

	mutex_lock(&data->mutex);
	if (data->open) {
		pr_debug("sx126x: already open\n");
		status = -EBUSY;
		goto err_open;
	}
	data->open = 1;
	mutex_unlock(&data->mutex);

	mutex_unlock(&device_list_lock);

	file->private_data = data;
	return 0;

 err_open:
	mutex_unlock(&data->mutex);
 err_notfound:
	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t sx126x_dev_read(struct file *filp, char __user * buf,
			       size_t count, loff_t * f_pos)
{
	struct sx126x *data = filp->private_data;
	unsigned copied;
	ssize_t ret = 0;
	wait_event_interruptible(data->readwq, kfifo_len(&data->out));
	ret = kfifo_to_user(&data->out, buf, count, &copied);
	if (!ret && copied > 0) {
		ret = copied;
	}
	return ret;
}

static ssize_t sx126x_dev_write(struct file *filp, const char __user * buf,
				size_t count, loff_t * f_pos)
{
	struct sx126x *data = filp->private_data;
	size_t packetsz, offset, maxpkt = MAX_PAYLOAD_LEN;
	int ret = -1;

	u8 kbuf[MAX_PAYLOAD_LEN];
	dev_info(&data->spi->dev, "char device write; %d\n", count);

	for (offset = 0; offset < count; offset += maxpkt) {

		packetsz = min((count - offset), maxpkt);

		mutex_lock(&data->mutex);

		ret = copy_from_user(kbuf, buf + offset, packetsz);

		sx126x_set_standby(data, SX126X_STANDBY_RC);

		data->transmitted = 0;

		sx126x_send(data, kbuf, packetsz, SX126X_TXMODE_SYNC);

		mutex_unlock(&data->mutex);

		wait_event_interruptible_timeout(data->writewq,
										 data->transmitted, 60 * HZ);
	}
	return count;
}

static int sx126x_dev_release(struct inode *inode, struct file *filp)
{
	struct sx126x *data = filp->private_data;

	mutex_lock(&data->mutex);

	sx126x_set_standby(data, SX126X_STANDBY_RC);

	data->open = 0;

	kfifo_reset(&data->out);

	mutex_unlock(&data->mutex);

	return 0;
}

static long sx126x_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct sx126x *data = filp->private_data;

	uint8_t buffer[2] = {0};
	int ret = -1;

	enum sx126x_ioctl_cmd ioctlcmd = cmd;

	mutex_lock(&data->mutex);

	switch (ioctlcmd) {
		case SX126X_IO_SETUP_V0:
			ret = sx126x_setup_v0(data, arg);
			break;
		case SX126X_IO_SET_FREQ:
			ret = sx126x_set_freq(data, arg);
			break;
		case SX126X_IO_GET_FREQ:
			ret = data->_tx_freq;
			break;
		case SX126X_IO_SET_SF:
			ret = sx126x_set_sf(data, arg);
			break;
		case SX126X_IO_GET_SF:
			ret = data->_sf;
			break;
		case SX126X_IO_SET_BW:
			ret = sx126x_set_bw(data, arg);
			break;
		case SX126X_IO_SET_CR:
			ret = sx126x_set_cr(data, arg);
			break;
		case SX126X_IO_SET_SYNCWORD:
			ret = sx126x_set_syncword(data, arg & 0xff);
			break;
		case SX126X_IO_GET_SYNCWORD:
			sx126x_read_reg(data, SX126X_REG_LR_SYNCWORD, buffer, 2);
			//printk("Syncword: 0x%0X 0x%0X\n", buffer[0], buffer[1]);
			ret = (buffer[0] << 8) | buffer[1];
			break;
		default:
			ret = -EINVAL;
			break;
	}
	mutex_unlock(&data->mutex);

	return ret;
}

static struct file_operations fops = {
	.open = sx126x_dev_open,
	.read = sx126x_dev_read,
	.write = sx126x_dev_write,
	.release = sx126x_dev_release,
	.unlocked_ioctl = sx126x_dev_ioctl
};

static irqreturn_t sx126x_irq(int irq, void *dev_id)
{
	struct sx126x *data = dev_id;
	schedule_work(&data->irq_work);
	return IRQ_HANDLED;
}

static irqreturn_t sx126x_key_irq(int irq, void *dev_id)
{
	struct sx126x *d = dev_id;

	dev_info(&d->spi->dev, "TX the cached buffer\n");

	return IRQ_HANDLED;
}

/*
 * 0x004: IRQ_PREAMBLE_DETECTED, irq consumer: 4.5 ms
 * 0x002: IRQ_RX_DONE, irq consumer: 45 ms
 *
*/
static void sx126x_irq_handler(struct work_struct *work)
{
	struct sx126x *d = container_of(work, struct sx126x, irq_work);

	d->irq_st = sx126x_get_irq_status(d);

	dev_info(&d->spi->dev, "irq_st = 0x%03X\n", (unsigned)d->irq_st);

#ifdef SX126X_SHOW_IRQ_TIME
	u32 s_ns, e_ns;

	mutex_lock(&d->mutex);
	s_ns = ktime_get_real_ns();
	mutex_unlock(&d->mutex);
#endif

	/* irq: 0x076 or 0x066 are incorrect state */

	/* irq: 0x302 maybe read 3 Bytes pkt */

	if (SX126X_IRQ_PREAMBLE_DETECTED == d->irq_st) {
		//printk(KERN_DEBUG"spi%d.0: goto out\n", d->dev_num);
		goto clr_out;
	}

	if (d->irq_st & SX126X_IRQ_TIMEOUT) {

		if(!(d->irq_st & SX126X_IRQ_RX_DONE) && false == d->tx_active) {

			if (SX126X_RX_SIN == d->rx_mode || SX126X_RX_DUTY == d->rx_mode) {
				sx126x_set_rx_ms(d, d->rx_win);
			}
		}

		if (!(d->irq_st & SX126X_IRQ_TX_DONE) && d->tx_active) {
			/* cad timeout before tx or tx timeout, need to re-tx */

			if (d->_cad_on) {
				sx126x_clear_irq_status(d, SX126X_IRQ_ALL);
				sx126x_set_cad(d);
				dev_warn(d->chardevice, "timeout re-cad\n");
				goto cad_out;
			} else {
				sx126x_set_tx(d, TX_TIMEOUT_MS);
				dev_warn(d->chardevice, "re-tx\n");
			}
		}
		//dev_info(d->chardevice, "Tx or Rx timeout\n");
		//goto irq_out;
	}

	if (d->irq_st & SX126X_IRQ_RX_DONE) {

		if (d->irq_st & SX126X_IRQ_CRC_ERR) {
			//printk(KERN_DEBUG"spi%d.0: crc err\n", d->dev_num);
			goto irq_out;
		}

		mutex_lock(&d->mutex);

		memset(d->irq_buf, 0, MAX_PAYLOAD_LEN);
		sx126x_read_buf(d, d->irq_buf, &d->irq_plen);

		mutex_unlock(&d->mutex);

		if (d->irq_plen > 0) {

			mutex_lock(&d->mutex);

			/* [min_payload_len, max_payload_len] */

			memset(&d->irq_pkt, 0, sizeof(d->irq_pkt));

			sx126x_get_rssi_inst(d, &(d->irq_pkt.rssi));

			d->irq_pkt.hdrlen = sizeof(d->irq_pkt);
			d->irq_pkt.payloadlen = d->irq_plen;
			d->irq_pkt.len = d->irq_pkt.hdrlen + d->irq_pkt.payloadlen;

			kfifo_in(&d->out, &d->irq_pkt, sizeof(d->irq_pkt));
			kfifo_in(&d->out, d->irq_buf, d->irq_plen);

			/* rx pkt number */
			d->cnt_rx += 1;

			mutex_unlock(&d->mutex);

			wake_up(&d->readwq);

			//print_hex_dump(KERN_DEBUG, " | ", DUMP_PREFIX_NONE, 16, 1, d->irq_buf, d->irq_plen, true);

		} else {
		#ifdef SX126X_DEBUG_IRQ
			dev_warn(d->chardevice, "payload len is 0\n");
		#endif
			if (SX126X_RX_SIN == d->rx_mode || SX126X_RX_DUTY == d->rx_mode) {
				/* incorrect pkt, re-rx */
				sx126x_set_rx_ms(d, d->rx_win);
			}
		}
	}

	if (d->irq_st & SX126X_IRQ_TX_DONE) {

		dev_warn(d->chardevice, "TX OK.\n");

		mutex_lock(&d->mutex);
		d->transmitted = 1;
		d->tx_active = false;
		mutex_unlock(&d->mutex);

		wake_up(&d->writewq);
	}

	if (d->irq_st & SX126X_IRQ_CAD_DONE) {

		if (d->irq_st & SX126X_IRQ_CAD_DETECTED) {
			dev_warn(d->chardevice, "CAD done, detected activity\n");

			switch(d->cad_param.exit_mode) {
				case SX126X_CAD_ONLY:
					dev_warn(d->chardevice, "Switch to STBY_RC mode\n");
					sx126x_clear_irq_status(d, SX126X_IRQ_ALL);
					sx126x_set_cad(d);
					goto cad_out;
				case SX126X_CAD_RX:
					dev_warn(d->chardevice, "Switch to RX mode\n");
					sx126x_enter_rx(d);
					break;
				case SX126X_CAD_LBT:
					dev_warn(d->chardevice, "Seek next win to tx\n");
					sx126x_clear_irq_status(d, SX126X_IRQ_ALL);
					sx126x_set_cad(d);
					goto cad_out;
				default:
					dev_warn(d->chardevice, "unknown cad exit mode\n");
					break;
			}


		} else {
			dev_info(d->chardevice, "CAD done, nothing detected\n");

			switch(d->cad_param.exit_mode) {
				case SX126X_CAD_ONLY:
					dev_warn(d->chardevice, "Switch to STBY_RC mode\n");
					sx126x_clear_irq_status(d, SX126X_IRQ_ALL);
					sx126x_set_cad(d);
					goto cad_out;
				case SX126X_CAD_RX:
					dev_warn(d->chardevice, "seek next win to rx\n");
					sx126x_clear_irq_status(d, SX126X_IRQ_ALL);
					sx126x_set_cad(d);
					goto cad_out;
				case SX126X_CAD_LBT:
					dev_warn(d->chardevice, "ch is ok, tx...\n");
					/* radio is waitting for tx */
					sx126x_set_tx(d, 200);
					break;
				default:
					dev_warn(d->chardevice, "unknown cad exit mode\n");
					break;
			}
		}
	}

#ifdef SX126X_DEBUG_IRQ
	if (d->irq_st & SX126X_IRQ_HEADER_VALID) {
		dev_warn(d->chardevice, "Header Valid\n");
	}
#endif

irq_out:

	if (d->irq_st & SX126X_IRQ_CRC_ERR) {

		mutex_lock(&d->mutex);
		d->cnt_crc_err += 1;
		mutex_unlock(&d->mutex);

		if (SX126X_RX_SIN == d->rx_mode) {
			sx126x_set_rx_ms(d, d->rx_win);
		}
	}

	if (d->irq_st & SX126X_IRQ_HEADER_ERR) {

		/* re-enter rx */
		sx126x_enter_rx(d);
	}

clr_out:
	sx126x_clear_irq_status(d, SX126X_IRQ_ALL);

cad_out:
#ifdef SX126X_SHOW_IRQ_TIME
	mutex_lock(&d->mutex);
	e_ns = ktime_get_real_ns();
	mutex_unlock(&d->mutex);
	dev_info(d->chardevice, "irq consumer %dns\n", e_ns - s_ns);
#endif
	return ;
}

static int sx126x_probe(struct spi_device *spi)
{
	int ret = 0;
	struct sx126x *data;
	int minor;
	int irq;

	uint8_t buffer[2] = {0x00};

	// allocate all of the crap we need
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		printk("Failed to allocate driver data\n");
		ret = -ENOMEM;
		goto err_allocdevdata;
	}

	data->open = 0;

	INIT_WORK(&data->irq_work, sx126x_irq_handler);
	INIT_LIST_HEAD(&data->device_entry);

	init_waitqueue_head(&data->readwq);
	init_waitqueue_head(&data->writewq);

	mutex_init(&data->mutex);

	data->fosc = 32000000;
	data->spi = spi;

	/* kfifo is about 4KB */
	ret = kfifo_alloc(&data->out, PAGE_SIZE, GFP_KERNEL);
	if (ret) {
		printk("<0>Failed to allocate out fifo\n");
		goto err_allocoutfifo;
	}

	// get the swctrl gpios
	data->gpio_swctrl =
	    devm_gpiod_get(&spi->dev, "swctrl", GPIOD_OUT_LOW);

	if (IS_ERR(data->gpio_swctrl)) {
		dev_warn(&spi->dev, "NO SWCTRL enable\n");
		data->gpio_swctrl = NULL;
	} else {
		/* enable the swctrl */
		gpiod_direction_output(data->gpio_swctrl, 1);
		//gpiod_set_value(data->gpio_swctrl, 1);
	}

	// get the busy gpios
	data->gpio_busy =
	    devm_gpiod_get(&spi->dev, "busy", GPIOD_OUT_LOW);

	if (IS_ERR(data->gpio_busy)) {
		dev_warn(&spi->dev, "NO BUSY enable\n");
		data->gpio_busy = NULL;
	} else {
		gpiod_direction_input(data->gpio_busy);
		dev_info(&spi->dev, "Set busy pin as input\n");
	}

	// get the reset gpio and reset the chip
	data->gpio_reset = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(data->gpio_reset)) {
		dev_err(&spi->dev, "reset gpio is required");
		ret = -ENOMEM;
		goto err_resetgpio;
	} else {
		/*
		 * reset the sx126x
		 * reset pin is set to ACTIVE_LOW, so:
		 *  gpio_set(1) is LOW
		 *  gpio_set(0) is HIGH
		*/
		//gpiod_direction_output(data->gpio_reset, 1);

		gpiod_set_value(data->gpio_reset, 1);
		mdelay(100);
		gpiod_set_value(data->gpio_reset, 0);
		mdelay(100);
	}

	//printk("<0>line %d @ %s\n", __LINE__, __FUNCTION__);

	if (0x2a != sx126x_get_status(data)) {
		dev_err(&spi->dev, "sx126x status error, maybe no spi connection");
	} else {

		sx126x_set_standby(data, SX126X_STANDBY_RC);
		printk("status = 0x%x\n", sx126x_get_status(data));
	}

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));
	///////////////////////////////////////////////////////////
	/* setup the basic lora cfg */
	sx126x_workaround_ant_mismatch(data);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));

	sx126x_set_regulator_mode(data, SX126X_REGULATOR_DC_DC);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));

	sx126x_set_dio3_as_tcxo_ctrl(data, SX126X_DIO3_OUTPUT_1_8, RADIO_TCXO_SETUP_TIME << 6);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));

	sx126x_calibrate(data, SX126X_CALIBRATE_IMAGE_ON
		| SX126X_CALIBRATE_ADC_BULK_P_ON
		| SX126X_CALIBRATE_ADC_BULK_N_ON
		| SX126X_CALIBRATE_ADC_PULSE_ON
		| SX126X_CALIBRATE_PLL_ON
		| SX126X_CALIBRATE_RC13M_ON | SX126X_CALIBRATE_RC64K_ON);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));

    sx126x_set_dio2_as_rfswitch_ctrl(data, true);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));

    sx126x_set_buffer_base_addr(data, 0, 0);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));

    sx126x_set_syncword(data, 0x12);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));
	///////////////////////////////////////////////////////////

	// get the irq
	data->irq = irq_of_parse_and_map(spi->dev.of_node, 0);
	if (!data->irq) {
		dev_err(&spi->dev, "NO irq in platform data\n");
		ret = -EINVAL;
		goto err_resetgpio;
	}
	ret = devm_request_irq(&spi->dev, data->irq, sx126x_irq, 0, SX126X_DRIVERNAME, data);

	if (ret) {
		dev_err(&spi->dev, "Error request dev irq: %d\n", ret);
		goto err_irq;
	}

	// get the irq of key
    // ret = of_property_read_string_index(node, "interrupt-names", i, &name);
	irq = irq_of_parse_and_map(spi->dev.of_node, 1);
	if (!irq) {
		dev_warn(&spi->dev, "NO key irq in dts\n");

	} else {
		ret = devm_request_irq(&spi->dev, irq, sx126x_key_irq, 0, "sx126x-key", data);
		if (ret) {
			dev_err(&spi->dev, "Error request key irq: %d\n", ret);
			goto err_irq;
		}
	}

	// create the frontend device and stash it in the spi device
	mutex_lock(&device_list_lock);

	minor = of_alias_get_id(spi->master->dev.of_node, "spi");
	if (minor < 0) {
		printk("Invalide dev num %d\n", minor);
		minor = 0;
	}

	data->dev_num = minor;

	data->devt = MKDEV(devmajor, minor);
	data->chardevice = device_create(devclass, &spi->dev, data->devt, data,
									  SX126X_DEVICENAME, minor);

	if (IS_ERR(data->chardevice)) {
		dev_err(&spi->dev, "Failed to create char device\n");
		ret = -ENOMEM;
		goto err_createdevice;
	}

	list_add(&data->device_entry, &device_list);

	mutex_unlock(&device_list_lock);

	spi_set_drvdata(spi, data);

	// setup sysfs nodes
	//ret = device_create_file(data->chardevice, &dev_attr_modulation);
	ret = device_create_file(data->chardevice, &dev_attr_freq);
	ret = device_create_file(data->chardevice, &dev_attr_rssi);
	ret = device_create_file(data->chardevice, &dev_attr_cnt_crc_err);
	ret = device_create_file(data->chardevice, &dev_attr_cnt_rx255);
	ret = device_create_file(data->chardevice, &dev_attr_cnt_rx);
	ret = device_create_file(data->chardevice, &dev_attr_cad_on);
	ret = device_create_file(data->chardevice, &dev_attr_dbm);
	ret = device_create_file(data->chardevice, &dev_attr_pre_len);
	ret = device_create_file(data->chardevice, &dev_attr_irq_st);

	// these are LoRa specifc
	ret = device_create_file(data->chardevice, &dev_attr_sf);
	ret = device_create_file(data->chardevice, &dev_attr_bw);
	ret = device_create_file(data->chardevice, &dev_attr_cr);
	ret = device_create_file(data->chardevice, &dev_attr_status);
	ret = device_create_file(data->chardevice, &dev_attr_rx_mode);
	ret = device_create_file(data->chardevice, &dev_attr_rx_win);
	ret = device_create_file(data->chardevice, &dev_attr_rxduty_sleep);
	ret = device_create_file(data->chardevice, &dev_attr_rx_boost);
	ret = device_create_file(data->chardevice, &dev_attr_tx_buf);

	/////////////////////////
	//for test
	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));
	ret = sx126x_setup_v0(data, 472500000);
	printk("setup_v0() = %d\n", ret);

	/* syncword: 0x1412 after setup_v0() */
	sx126x_read_reg(data, SX126X_REG_LR_SYNCWORD, buffer, 2);
	printk("Syncword: 0x%0X 0x%0X\n", buffer[0], buffer[1]);

	printk("%d: status = 0x%x\n", __LINE__, sx126x_get_status(data));
	sx126x_enter_rx(data);

	ret = sx126x_get_status(data);
	printk("%d: status = 0x%x\n", __LINE__, ret);

	if (ret == 0x2a) {
		sx126x_enter_rx(data);
	}

	ret = sx126x_get_status(data);
	printk("%d: status = 0x%x\n", __LINE__, ret);

    //printk("swctrl = %d\n", gpiod_get_value(data->gpio_swctrl));

	return 0;

 //err_sysfs:
	device_destroy(devclass, data->devt);

 err_createdevice:
	mutex_unlock(&device_list_lock);

 err_irq:
	devm_free_irq(&spi->dev, data->irq, data);
 err_resetgpio:
	kfifo_free(&data->out);

 err_allocoutfifo:
	kfree(data);

 err_allocdevdata:
	return ret;
}

static int sx126x_remove(struct spi_device *spi)
{
	struct sx126x *data = spi_get_drvdata(spi);

	device_remove_file(data->chardevice, &dev_attr_freq);
	device_remove_file(data->chardevice, &dev_attr_rssi);
	device_remove_file(data->chardevice, &dev_attr_cnt_crc_err);
	device_remove_file(data->chardevice, &dev_attr_cnt_rx255);
	device_remove_file(data->chardevice, &dev_attr_cnt_rx);
	device_remove_file(data->chardevice, &dev_attr_cad_on);
	device_remove_file(data->chardevice, &dev_attr_dbm);
	device_remove_file(data->chardevice, &dev_attr_pre_len);
	device_remove_file(data->chardevice, &dev_attr_irq_st);

	device_remove_file(data->chardevice, &dev_attr_sf);
	device_remove_file(data->chardevice, &dev_attr_bw);
	device_remove_file(data->chardevice, &dev_attr_cr);
	device_remove_file(data->chardevice, &dev_attr_status);
	device_remove_file(data->chardevice, &dev_attr_rx_mode);
	device_remove_file(data->chardevice, &dev_attr_rx_win);
	device_remove_file(data->chardevice, &dev_attr_rxduty_sleep);
	device_remove_file(data->chardevice, &dev_attr_rx_boost);
	device_remove_file(data->chardevice, &dev_attr_tx_buf);

	device_destroy(devclass, data->devt);

	kfifo_free(&data->out);
	kfree(data);

	devm_free_irq(&spi->dev, data->irq, data);

	return 0;
}

static const struct of_device_id sx126x_of_match[] = {
	{ .compatible = "idodo,cch" },
	{ .compatible = "idodo,dch" },
	{},
};
MODULE_DEVICE_TABLE(of, sx126x_of_match);

static struct spi_driver sx126x_driver = {
	.probe = sx126x_probe,
	.remove = sx126x_remove,

	.driver = {
	   .name = SX126X_DRIVERNAME,
	   .of_match_table = of_match_ptr(sx126x_of_match),
	   .owner = THIS_MODULE,
   },
};

/* char & spi dev api */
static int __init sx126x_init(void)
{
	int ret;

	ret = register_chrdev(0, SX126X_DRIVERNAME, &fops);

	if (ret < 0) {
		printk(KERN_ERR "Failed to register char device\n");
		goto out;
	}

	devmajor = ret;

	printk(KERN_INFO "dev_major = %d\n", devmajor);

	devclass = class_create(THIS_MODULE, SX126X_CLASSNAME);

	if (!devclass) {
		printk(KERN_ERR "Failed to register class\n");
		ret = -ENOMEM;
		goto out1;
	}

	ret = spi_register_driver(&sx126x_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register spi driver\n");
		goto out2;
	}

	goto out;

 out2:
 out1:
	class_destroy(devclass);
	devclass = NULL;

 out:
	printk(KERN_INFO "SX126x init OK");

	return ret;
}

module_init(sx126x_init);

static void __exit sx126x_exit(void)
{
	spi_unregister_driver(&sx126x_driver);
	unregister_chrdev(devmajor, SX126X_DRIVERNAME);
	class_destroy(devclass);
	devclass = NULL;
}

module_exit(sx126x_exit);
MODULE_LICENSE("GPL");

//MODULE_INFO(intree, "Y");
