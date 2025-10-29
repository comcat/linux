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

#include "sx126x_regs.h"
#include "sx126x.h"

/*
 * F1C:
 *  SPI  - SPI1 (PA0 ~ PA3)
 *  RST  - PE3
 *  DIO1 - PE4
 *  BUSY - PE5
*/
#define SX126X_DRIVERNAME	"sx126x"
#define SX126X_CLASSNAME	"sx126x"
#define SX126X_DEVICENAME	"sx126x%d"

static int devmajor;
static struct class *devclass;

static unsigned bwmap[] = {20800, 31250, 41700, 62500, 125000, 250000, 500000};
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

struct sx126x {
	struct device *chardevice;
	struct work_struct irq_work;
	struct spi_device *spi;
	struct gpio_desc *gpio_reset, *gpio_busy, *gpio_swctrl;
	u32 fosc;
	struct mutex mutex;

	struct list_head device_entry;
	dev_t devt;
	bool open;

	/* device state */
	//enum sx126x_opmode opmode;

	/* tx */
	wait_queue_head_t writewq;
	int transmitted;

	/* rx */
	wait_queue_head_t readwq;
	struct kfifo out;

	/* rf param */
	bool _cad_on;
	size_t _preamble_len;
	int8_t _dbm;
	u8 _sf;
	u8 _bw;
	u8 _cr;
	u32 _tx_freq;
	bool _ldro;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* sx126x register and buffer api */
static int sx126x_read_reg(struct spi_device *spi, u16 reg, u8 *result, size_t len)
{
	u8 cmd[3];
	int ret;

	cmd[0] = SX126X_READ_REGISTER;
	cmd[1] = (reg >> 8) & 0xff;
	cmd[2] = reg & 0xff;

	ret = spi_write_then_read(spi, cmd, 3, result, len);

	dev_dbg(&spi->dev, "read: @%02x %02x\n", reg, *result);
	return ret;
}

static int sx126x_write_reg(struct spi_device *spi, u16 reg, u8 *value, size_t len)
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

	//dev_info(&spi->dev, "reg write: %d\n", len);
	//print_hex_dump(KERN_DEBUG, NULL, DUMP_PREFIX_NONE, 16, 1, value, len, true);

	spi_sync_transfer(spi, fifotransfers, ARRAY_SIZE(fifotransfers));

	//ret = spi_write(spi, ptx, 4);
	dev_dbg(&spi->dev, "write: @%02x %02x\n", reg, value[0]);

	return ret;
}

static void sx126x_get_rxbuf_status(struct spi_device *spi, uint8_t *plen, uint8_t *rxbuf_start);

static int sx126x_read_buf(struct spi_device *spi, void *buffer, u8 *len)
{
	u8 pktstart, rxbytes, off, fifoaddr;
	u8 ptx[3];
	int ret;
	unsigned readlen;

	size_t maxtransfer = spi_max_transfer_size(spi);

	sx126x_get_rxbuf_status(spi, &rxbytes, &pktstart);

	for (off = 0; off < rxbytes; off += maxtransfer) {
		readlen = min(maxtransfer, (size_t)(rxbytes - off));
		fifoaddr = pktstart + off;

		ptx[0] = SX126X_READ_BUFFER;
		ptx[1] = fifoaddr;								/* offset */
		ptx[2] = SX126X_NOP;

		dev_warn(&spi->dev, "FIFO read: %02x from %02x\n", readlen, fifoaddr);

		ret = spi_write_then_read(spi, &ptx, 3, buffer + off, readlen);

		if (ret) {
			break;
		}

	}

	print_hex_dump_bytes("", DUMP_PREFIX_NONE, buffer, rxbytes);
	*len = rxbytes;

	return ret;
}

static int sx126x_write_buf(struct spi_device *spi, void *buffer, u8 len)
{
	int ret = 0;
	u8 cmd[2] = {SX126X_WRITE_BUFFER, 0};

	struct spi_transfer fifotransfers[] = {
		{.tx_buf = &cmd, .len = 2},
		{.tx_buf = buffer, .len = len},
	};

	dev_info(&spi->dev, "FIFO write: %d\n", len);
	print_hex_dump(KERN_DEBUG, NULL, DUMP_PREFIX_NONE, 16, 1, buffer, len, true);

	spi_sync_transfer(spi, fifotransfers, ARRAY_SIZE(fifotransfers));

	//if (memcmp(buffer, readbackbuff, len) != 0) {
	//	dev_err(&spi->dev, "FIFO readback doesn't match\n");
	//}
	return ret;
}

static void sx126x_get_rxbuf_status(struct spi_device *spi, uint8_t *plen, uint8_t *rxbuf_start)
{
	uint8_t cmd[2] = {SX126X_GET_RX_BUFFER_STATUS, 0};
	uint8_t buf[2] = {0};

	spi_write_then_read(spi, cmd, 2, buf, 2);

	*plen = buf[0];
	*rxbuf_start = buf[1];
}

static int sx126x_indexofstring(const char *str, const char **options,
				unsigned noptions)
{
	int i;
	for (i = 0; i < noptions; i++) {
		if (sysfs_streq(str, options[i])) {
			return i;
		}
	}
	return -1;
}

static int sx126x_get_pkt_rssi(struct spi_device *spi)
{
	int ret;
	int rssi = 0;
	u8 buf = 0;

	u8 cmd[SX126X_SIZE_GET_RSSI_INST] = {
        SX126X_GET_RSSI_INST,
        SX126X_NOP,
    };

	//read_op_cmd(SX126X_GET_RSSI_INST, buf, 3);
	ret = spi_write_then_read(spi, cmd, 2, &buf, 1);

	rssi = -buf >> 1;

	return rssi;
}

/*
 * pkt_type:
 *   gfsk: 0x0
 *   lora: 0x1
 *   bpsk: 0x2
 *   lr_fhss: 0x3
*/ 
uint8_t sx126x_get_pkt_type(struct spi_device *spi)
{
	uint8_t cmd[2];
	uint8_t rv = 0; 

	cmd[0] = SX126X_GET_PKT_TYPE;
	cmd[1] = 0;

	spi_write_then_read(spi, cmd, 2, &rv, 1);

	return rv;
}

void sx126x_set_pkt_type(struct spi_device *spi, uint8_t pkt_t)
{
	u8 cmd[SX126X_SIZE_SET_PKT_TYPE];

	cmd[0] = SX126X_SET_PKT_TYPE;
	cmd[1] = pkt_t;

	spi_write(spi, cmd, SX126X_SIZE_SET_PKT_TYPE);
}

void sx126x_set_stop_rx_timer_on_preamble(struct spi_device *spi, bool enable)
{
	u8 cmd[2];

	cmd[0] = SX126X_SET_STOP_TIMER_ON_PREAMBLE;
	cmd[1] = enable;
	
	spi_write(spi, cmd, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE);
}

void sx126x_set_lora_symb_num_timeout(struct spi_device *spi, uint8_t symb_num)
{
	u8 cmd[2];

	cmd[0] = SX126X_SET_LORA_SYMB_NUM_TIMEOUT;
	cmd[1] = symb_num;
	
	spi_write(spi, cmd, SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT);
}

void sx126x_config_dio_irq(struct spi_device *spi, uint16_t irq_mask, uint16_t dio1_mask,
							 uint16_t dio2_mask, uint16_t dio3_mask)
{
	uint8_t cmd[9];

	cmd[0] = SX126X_SET_DIO_IRQ_PARAMS;
	cmd[1] = (uint8_t) ((irq_mask >> 8) & 0x00FF);
	cmd[2] = (uint8_t) (irq_mask & 0x00FF);
	cmd[3] = (uint8_t) ((dio1_mask >> 8) & 0x00FF);
	cmd[4] = (uint8_t) (dio1_mask & 0x00FF);
	cmd[5] = (uint8_t) ((dio2_mask >> 8) & 0x00FF);
	cmd[6] = (uint8_t) (dio2_mask & 0x00FF);
	cmd[7] = (uint8_t) ((dio3_mask >> 8) & 0x00FF);
	cmd[8] = (uint8_t) (dio3_mask & 0x00FF);

	spi_write(spi, cmd, SX126X_SIZE_SET_DIO_IRQ_PARAMS);
}

void sx126x_set_dio3_as_tcxo_ctrl(struct spi_device *spi, uint8_t volt, uint32_t timeout)
{
	uint8_t cmd[5];

	cmd[0] = SX126X_SET_DIO3_AS_TCXO_CTRL;
	cmd[1] = volt & 0x07;
	cmd[2] = (uint8_t) ((timeout >> 16) & 0xFF);
	cmd[3] = (uint8_t) ((timeout >> 8) & 0xFF);
	cmd[4] = (uint8_t) (timeout & 0xFF);

	spi_write(spi, cmd, SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL);
}

void sx126x_set_dio2_as_rfswitch_ctrl(struct spi_device *spi, uint8_t enable)
{
	u8 cmd[2];

	cmd[0] = SX126X_SET_DIO2_AS_RF_SWITCH_CTRL;
	cmd[1] = enable;
	
	spi_write(spi, cmd, SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL);
}

static int sx126x_lora_tx_modulation_workaround(struct spi_device *spi, u8 bw )
{
    uint8_t reg_value = 0;

    int status = sx126x_read_reg(spi, SX126X_REG_TX_MODULATION, &reg_value, 1);

    if(status == 0) {

		if( bw == BW500 ) {
			reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
		} else {
			reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
		}

        status = sx126x_write_reg(spi, SX126X_REG_TX_MODULATION, &reg_value, 1);
    }
    return status;
}

static int sx126x_set_lora_modulation_params(struct spi_device *spi, int8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t cmd[SX126X_SIZE_SET_MODULATION_PARAMS_LORA];
	int ret;

	cmd[0] = SX126X_SET_MODULATION_PARAMS;
	cmd[1] = sf;
	cmd[2] = bw;
	cmd[3] = cr;
	cmd[4] = ldro & 0x01;

	ret = spi_write(spi, cmd, 5);

    if(ret == 0) {
	// WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 §15.1
        ret = sx126x_lora_tx_modulation_workaround(spi, bw);
    }

    return ret;
}

/* 
 * rv[6:4]: chip modes
 * rv[3:1]: cmd status
 *
 * ASR6500: 0x22
 * SX126x: 0x2A
 */ 
uint8_t sx126x_get_status(struct spi_device *spi)
{
    u8 cmd[SX126X_SIZE_GET_STATUS] = {
        SX126X_GET_STATUS,
    };
	u8 rv = 0xff;
    int ret = 0;

    ret = spi_write_then_read(spi, cmd, SX126X_SIZE_GET_STATUS, &rv, 1);

	if (ret == 0) {
		return rv;
    } else {
		return -1;
	}
}

uint16_t sx126x_get_irq_status(struct spi_device *spi)
{
	uint8_t cmd[2];
	uint8_t data[2];

	cmd[0] = SX126X_GET_IRQ_STATUS;
	cmd[1] = 0;

	spi_write_then_read(spi, cmd, 2, data, 2);

	return (data[0] << 8) | data[1];
}

void sx126x_clear_irq_status(struct spi_device *spi, uint16_t irq)
{
	uint8_t cmd[3];

	cmd[0] = SX126X_CLR_IRQ_STATUS;
	cmd[1] = (uint8_t) (((uint16_t) irq >> 8) & 0x00FF);
	cmd[2] = (uint8_t) ((uint16_t) irq & 0x00FF);

	spi_write(spi, cmd, 3);
}

void sx126x_set_rx(struct spi_device *spi, uint32_t timeout)
{
	uint8_t cmd[4];

	cmd[0] = SX126X_SET_RX;
	cmd[1] = (uint8_t) ((timeout >> 16) & 0xFF);
	cmd[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	cmd[3] = (uint8_t) (timeout & 0xFF);

	spi_write(spi, cmd, 4);
}

void sx126x_set_tx(struct spi_device *spi, uint32_t timeout_ms)
{
	uint8_t cmd[4];
	uint32_t tout = (uint32_t) (timeout_ms / 0.015625);

	cmd[0] = SX126X_SET_TX;
	cmd[1] = (uint8_t) ((tout >> 16) & 0xFF);
	cmd[2] = (uint8_t) ((tout >> 8) & 0xFF);
	cmd[3] = (uint8_t) (tout & 0xFF);

	spi_write(spi, cmd, SX126X_SIZE_SET_TX);
}

void sx126x_set_pa_config(struct spi_device *spi, u8 duty_cycle, u8 hp_max, u8 dev_sel, u8 lut)
{
	uint8_t cmd[5];

	cmd[0] = SX126X_SET_PA_CFG;
	cmd[1] = duty_cycle;
	cmd[2] = hp_max;
	cmd[3] = dev_sel;
	cmd[4] = lut;

	spi_write(spi, cmd, SX126X_SIZE_SET_PA_CFG);
}

void sx126x_set_over_current_protect(struct spi_device *spi, uint8_t value)
{
	sx126x_write_reg(spi, SX126X_REG_OCP, &value, 1);
}

void sx126x_calibrate(struct spi_device *spi, uint8_t calibParam)
{
	u8 cmd[2] = {0};

	cmd[0] = SX126X_CALIBRATE;
	cmd[1] = calibParam;

	spi_write(spi, cmd, SX126X_SIZE_CALIBRATE);
}

void sx126x_calibrate_image(struct spi_device *spi, uint32_t frequency)
{
	u8 cmd[3] = {0};

	cmd[0] = SX126X_CALIBRATE_IMAGE;

	//if (frequency > 900000000) {
	//	cal_freq[0] = 0xE1;
	//	cal_freq[1] = 0xE9;
	//} else if (frequency > 850000000) {
	//	cal_freq[0] = 0xD7;
	//	cal_freq[1] = 0xD8;
	//} else if (frequency > 770000000) {
	//	cal_freq[0] = 0xC1;
	//	cal_freq[1] = 0xC5;
	if (frequency > 460000000) {
		cmd[1] = 0x75;
		cmd[2] = 0x81;
	} else if (frequency > 425000000) {
		cmd[1] = 0x6B;
		cmd[2] = 0x6F;
	}

	spi_write(spi, cmd, SX126X_SIZE_CALIBRATE_IMAGE);
}

void sx126x_set_regulator_mode(struct spi_device *spi, uint8_t mode)
{
	u8 cmd[2] = {0};

	cmd[0] = SX126X_SET_REGULATOR_MODE;
	cmd[1] = mode;

	spi_write(spi, cmd, SX126X_SIZE_SET_REGULATOR_MODE);
}

void sx126x_set_buffer_base_addr(struct spi_device *spi, uint8_t tx_addr, uint8_t rx_addr)
{
	uint8_t cmd[3];

	cmd[0] = SX126X_SET_BUFFER_BASE_ADDRESS;
	cmd[1] = tx_addr;
	cmd[2] = rx_addr;

	spi_write(spi, cmd, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS);
}

void sx126x_set_tx_power(struct spi_device *spi, int8_t dbm)
{
    uint8_t cmd[3] = {0};

	cmd[0] = SX126X_SET_TX_PARAMS;

	// sx1262 or sx1268
	if (dbm > 22) {
		dbm = 22;
	} else if (dbm < -3) {
		dbm = -3;
	}

	if (dbm <= 14) {
		sx126x_set_pa_config(spi, 0x02, 0x02, 0x00, 0x01);
	} else {
		sx126x_set_pa_config(spi, 0x04, 0x07, 0x00, 0x01);
	}

	sx126x_set_over_current_protect(spi, 0x38);		// set max current to 140mA
	//write_reg(SX126X_REG_OCP, 0x38);				// current max 160mA for the whole device

    cmd[1] = dbm;

    //if ( _crystal_select == 0) {
    // TCXO
	cmd[2] = SX126X_PA_RAMP_200U;
    //} else {
    // XTAL
    //    cmd[2] = RADIO_RAMP_20_US;
    //}

    spi_write(spi, cmd, SX126X_SIZE_SET_TX_PARAMS);
}

/////////////////////////////////////////////////////////////////////////////////

static int sx126x_set_syncword(struct sx126x *dev, u16 syncword)
{
	int status;
    uint8_t buffer[2] = {0x00};

	dev_warn(dev->chardevice, "Setting syncword to 0x%0X\n", syncword);

	buffer[0] = (syncword & 0xFF00) >> 8;	/* MSB */
	buffer[1] = (syncword & 0xFF);			/* LSB */

    //int status = sx126x_read_reg(dev->spi, SX126X_REG_LR_SYNCWORD, buffer, 2);

	#if 0
    if(status == 0) {
        buffer[0] = (buffer[0] & ~0xF0) + (sync_word & 0xF0);
        buffer[1] = (buffer[1] & ~0xF0) + ((sync_word & 0x0F) << 4);
    }
	#endif

	status = sx126x_write_reg(dev->spi, SX126X_REG_LR_SYNCWORD, buffer, 2);

    return status;
}

static int sx126x_set_crc(struct sx126x *data, bool crc)
{
	dev_warn(data->chardevice, "Setting crc to %d\n", crc);

	//sx126x_read_reg(data->spi, SX126X_REG_LORA_MODEMCONFIG2, &reg);

	//sx126x_write_reg(data->spi, SX126X_REG_LORA_MODEMCONFIG2, reg);

	return 0;
}

static ssize_t sx126x_crc_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	u8 config2;
	int crc;

	mutex_lock(&data->mutex);

	//sx126x_read_reg(data->spi, SX126X_REG_LORA_MODEMCONFIG2, &config2);
	//crc = config2 >> SX126X_REG_LORA_MODEMCONFIG2_CRCON_SHIFT;

	dev_warn(dev, "cfg2: 0x%02X\n", config2);

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", crc & 0x1);
}

static ssize_t sx126x_crc_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int crc = 1;

	if (kstrtoint(buf, 10, &crc)) {
		goto out;
	}

	mutex_lock(&data->mutex);

	//sx126x_set_crc(data, crc);

	mutex_unlock(&data->mutex);

 out:
	return count;
}

static DEVICE_ATTR(crc, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   sx126x_crc_show, sx126x_crc_store);

static ssize_t sx126x_freq_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);

	//u32 frf;
	//u32 freq;
	//freq = ((u64) data->fosc * frf) / 524288;

	return sprintf(buf, "%u\n", data->_tx_freq);
}

static int sx126x_set_freq(struct sx126x *dev, u32 freq)
{
	uint8_t cmd[5];

	sx126x_calibrate_image(dev->spi, freq);

	freq = (uint32_t) ((double)freq / (double)FREQ_STEP);
	//do_div(freq, (double)FREQ_STEP);

	cmd[0] = SX126X_SET_RF_FREQUENCY;
	cmd[1] = (uint8_t) ((freq >> 24) & 0xFF);
	cmd[2] = (uint8_t) ((freq >> 16) & 0xFF);
	cmd[3] = (uint8_t) ((freq >> 8) & 0xFF);
	cmd[4] = (uint8_t) (freq & 0xFF);

	spi_write(dev->spi, cmd, SX126X_SIZE_SET_RF_FREQUENCY);

	dev->_tx_freq = freq;

	return 0;
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

static ssize_t sx126x_rssi_show(struct device *child,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static DEVICE_ATTR(rssi, S_IRUSR | S_IRGRP | S_IROTH, sx126x_rssi_show, NULL);

static ssize_t sx126x_sf_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	//uint8_t config2;
	int sf;

	mutex_lock(&data->mutex);
	//sx126x_read_reg(data->spi, SX126X_REG_LORA_MODEMCONFIG2, &config2);
	//sf = config2 >> SX126X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;

	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", sf);
}

static int sx126x_set_sf(struct sx126x *data, unsigned sf)
{
	dev_info(data->chardevice, "setting spreading factor to %u\n", sf);


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
	//uint8_t config1;
	int bw, ret = 0;

	mutex_lock(&data->mutex);

	//sx126x_read_reg(data->spi, SX126X_REG_LORA_MODEMCONFIG1, &config1);
	//bw = config1 >> SX126X_REG_LORA_MODEMCONFIG1_BW_SHIFT;

	sprintf(buf, "%d\n", bwmap[bw]);

	mutex_unlock(&data->mutex);
	return ret;
}

static int sx126x_set_bw(struct sx126x *data, unsigned bw){

	dev_info(data->chardevice, "setting BW to %u\n", bw);

	// set the BW

	return 0;
}

static ssize_t sx126x_bw_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	//struct sx126x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(bw, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx126x_bw_show,
		   sx126x_bw_store);

static ssize_t sx126x_cr_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct sx126x *data = dev_get_drvdata(dev);
	int cr, ret = 0;

	mutex_lock(&data->mutex);

	//sprintf(buf, "%s\n", crmap[cr]);

	mutex_unlock(&data->mutex);

	return ret;
}

static int sx126x_set_cr(struct sx126x *data, unsigned cr){
	//u8 r;
	dev_info(data->chardevice, "setting CR to %u\n", cr);


	return 0;
}

static ssize_t sx126x_cr_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	//struct sx126x *data = dev_get_drvdata(dev);
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
	size_t packetsz, offset, maxpkt = 256;

	u8 kbuf[256];
	dev_info(&data->spi->dev, "char device write; %d\n", count);

	for (offset = 0; offset < count; offset += maxpkt) {

		packetsz = min((count - offset), maxpkt);

		mutex_lock(&data->mutex);
		copy_from_user(kbuf, buf + offset, packetsz);

		//sx126x_set_opmode(data, SX126X_OPMODE_STANDBY, false);

		sx126x_write_buf(data->spi, kbuf, packetsz);

		data->transmitted = 0;

		//sx126x_set_opmode(data, SX126X_OPMODE_TX, false);

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

	//sx126x_set_opmode(data, SX126X_OPMODE_STANDBY, true);

	data->open = 0;
	kfifo_reset(&data->out);
	mutex_unlock(&data->mutex);

	return 0;
}

static long sx126x_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct sx126x *data = filp->private_data;
	int ret;
	enum sx126x_ioctl_cmd ioctlcmd = cmd;

	mutex_lock(&data->mutex);

	switch (ioctlcmd) {
		case SX126X_IOCTL_CMD_SET_FREQ:
			ret = sx126x_set_freq(data, arg);
			break;
		case SX126X_IOCTL_CMD_GET_FREQ:
			ret = 0;
			break;
		case SX126X_IOCTL_CMD_SET_SF:
			ret = sx126x_set_sf(data, arg);
			break;
		case SX126X_IOCTL_CMD_GET_SF:
			ret = 0;
			break;
		case SX126X_IOCTL_CMD_SET_BW:
			ret = sx126x_set_bw(data, arg);
			break;
		case SX126X_IOCTL_CMD_SET_CR:
			ret = sx126x_set_cr(data, arg);
			break;
		case SX126X_IOCTL_CMD_SET_SYNCWORD:
			ret = sx126x_set_syncword(data, arg & 0xff);
			break;
		case SX126X_IOCTL_CMD_GET_SYNCWORD:
			ret = 0;
			break;
		case SX126X_IOCTL_CMD_SET_CRC:
			ret = sx126x_set_crc(data, arg & 0x1);
			break;
		case SX126X_IOCTL_CMD_GET_CRC:
			ret = 0;
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

static void sx126x_irq_work_handler(struct work_struct *work)
{
	struct sx126x *data = container_of(work, struct sx126x, irq_work);

	u8 buf[128], len, snr, rssi;
	struct sx126x_pkt pkt;

	u16 irqflags;

	mutex_lock(&data->mutex);

	irqflags = sx126x_get_irq_status(data->spi);
	

	if (false ==(irqflags & SX126X_IRQ_CRC_ERR)) {

		dev_warn(data->chardevice, "no crc err, reading pkt\n");
		memset(&pkt, 0, sizeof(pkt));

		//sx126x_read_buf(data->spi, buf, &len);

		//sx126x_read_reg(data->spi, SX126X_REG_LORA_PKTSNRVALUE, &snr);
		//sx126x_read_reg(data->spi, SX126X_REG_LORA_PKTRSSIVALUE, &rssi);
		//sx126x_read_reg24(data->spi, SX126X_REG_LORA_FEIMSB, &fei);

		pkt.hdrlen = sizeof(pkt);
		pkt.payloadlen = len;
		pkt.len = pkt.hdrlen + pkt.payloadlen;
		pkt.snr = (__s16) (snr << 2) / 4;
		pkt.rssi = -157 + rssi;	//TODO fix this for the LF port

		kfifo_in(&data->out, &pkt, sizeof(pkt));
		kfifo_in(&data->out, buf, len);
		wake_up(&data->readwq);

	} else if (irqflags & SX126X_IRQ_CRC_ERR) {
			dev_warn(data->chardevice,
				 "CRC Error for received payload\n");
			pkt.crcfail = 1;

	} else if (irqflags & SX126X_IRQ_HEADER_ERR) {
			dev_warn(data->chardevice,
				 "Header Error for received payload\n");

	} else if (irqflags & SX126X_IRQ_TX_DONE) {

		//if (data->gpio_txen) {
		//	gpiod_set_value(data->gpio_txen, 0);
		//}
		dev_warn(data->chardevice, "transmitted packet\n");


		data->transmitted = 1;
		wake_up(&data->writewq);

	} else if (irqflags & SX126X_IRQ_CAD_DONE) {

		if (irqflags & SX126X_IRQ_CAD_DETECTED) {
			dev_info(data->chardevice,
				 "CAD done, detected activity\n");
		} else {
			dev_info(data->chardevice,
				 "CAD done, nothing detected\n");
		}

	} else {
		dev_err(&data->spi->dev,
			"unhandled interrupt state %02x\n", (unsigned)irqflags);
	}

	//sx126x_write_reg(data->spi, SX126X_REG_LORA_IRQFLAGS, 0xff);

	mutex_unlock(&data->mutex);
}

static int sx126x_probe(struct spi_device *spi)
{
	int ret = 0;
	struct sx126x *data;
	int irq;
	unsigned minor;

	// allocate all of the crap we need
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		printk("Failed to allocate driver data\n");
		ret = -ENOMEM;
		goto err_allocdevdata;
	}

	data->open = 0;

	INIT_WORK(&data->irq_work, sx126x_irq_work_handler);
	INIT_LIST_HEAD(&data->device_entry);

	init_waitqueue_head(&data->readwq);
	init_waitqueue_head(&data->writewq);

	mutex_init(&data->mutex);

	data->fosc = 32000000;
	data->spi = spi;
	//data->opmode = SX126X_OPMODE_STANDBY;

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
		gpiod_set_value(data->gpio_swctrl, 1);
	}

	// get the busy gpios
	data->gpio_busy =
	    devm_gpiod_get(&spi->dev, "busy", GPIOD_OUT_LOW);

	if (IS_ERR(data->gpio_busy)) {
		dev_warn(&spi->dev, "NO BUSY enable\n");
		data->gpio_busy = NULL;
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
		gpiod_set_value(data->gpio_reset, 1);
		mdelay(100);
		gpiod_set_value(data->gpio_reset, 0);
		mdelay(100);
	}

	//printk("<0>line %d @ %s\n", __LINE__, __FUNCTION__);

	if (0x2a != sx126x_get_status(spi)) {
		dev_err(&spi->dev, "sx126x status error, maybe no spi connection");
	} else {
		printk("<0>status = 0x%x\n", sx126x_get_status(spi));
	}

	// get the irq
	irq = irq_of_parse_and_map(spi->dev.of_node, 0);
	if (!irq) {
		dev_err(&spi->dev, "NO irq in platform data\n");
		ret = -EINVAL;
		goto err_irq;
	}
	devm_request_irq(&spi->dev, irq, sx126x_irq, 0, SX126X_DRIVERNAME, data);

	// create the frontend device and stash it in the spi device
	mutex_lock(&device_list_lock);

	minor = 0;
	data->devt = MKDEV(devmajor, minor);
	data->chardevice = device_create(devclass, &spi->dev, data->devt, data,
									  SX126X_DEVICENAME, minor);

	if (IS_ERR(data->chardevice)) {
		printk("<0>Failed to create char device\n");
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
	//ret = device_create_file(data->chardevice, &dev_attr_dbm);

	// these are LoRa specifc
	ret = device_create_file(data->chardevice, &dev_attr_sf);
	ret = device_create_file(data->chardevice, &dev_attr_bw);
	ret = device_create_file(data->chardevice, &dev_attr_cr);
	ret = device_create_file(data->chardevice, &dev_attr_crc);

	return 0;

 //err_sysfs:
	device_destroy(devclass, data->devt);

 err_createdevice:
	mutex_unlock(&device_list_lock);

 err_irq:
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

	//device_remove_file(data->chardevice, &dev_attr_modulation);
	device_remove_file(data->chardevice, &dev_attr_freq);
	device_remove_file(data->chardevice, &dev_attr_rssi);
	//device_remove_file(data->chardevice, &dev_attr_dbm);

	device_remove_file(data->chardevice, &dev_attr_sf);
	device_remove_file(data->chardevice, &dev_attr_bw);
	device_remove_file(data->chardevice, &dev_attr_cr);
	device_remove_file(data->chardevice, &dev_attr_crc);

	device_destroy(devclass, data->devt);

	kfifo_free(&data->out);
	kfree(data);

	return 0;
}

static const struct of_device_id sx126x_of_match[] = {
	{
		.compatible = "semtech,sx126x",
	},
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
		printk("<0>Failed to register char device\n");
		goto out;
	}

	devmajor = ret;

	printk("<0>dev_major = %d\n", devmajor);

	devclass = class_create(THIS_MODULE, SX126X_CLASSNAME);

	if (!devclass) {
		printk("<0>Failed to register class\n");
		ret = -ENOMEM;
		goto out1;
	}

	ret = spi_register_driver(&sx126x_driver);
	if (ret) {
		printk("Failed to register spi driver\n");
		goto out2;
	}

	goto out;

 out2:
 out1:
	class_destroy(devclass);
	devclass = NULL;

 out:
	printk("<0>SX126x init OK.");

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
