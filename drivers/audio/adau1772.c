/*
 * Copyright (c) 2017 comsuisse AG
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief ADAU1772 Audio Processor.
 */

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/i2c.h>
#include <drivers/i2s.h>
#include <audio/audio.h>
#include "adau1772.h"

#define LOG_LEVEL CONFIG_AUDIO_CODEC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adau1772);

#define PLL_OUTPUT_FREQ_HZ   24576000

/* Device constant configuration parameters */
struct adau1772_dev_cfg {
	char *i2s_dev_name;
	char *i2c_dev_name;
	u8_t *i2c_addr_list;
	u8_t num_chips;
};

/* Device run time data */
struct adau1772_dev_data {
	struct k_sem rx_sem;
	struct k_sem tx_sem;
	struct device *dev_i2s;
	struct device *dev_i2c;
};

#define DEV_CFG(dev) \
	((const struct adau1772_dev_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct adau1772_dev_data *const)(dev)->driver_data)

struct reg_data {
	u8_t addr;
	u8_t val;
};

static struct reg_data adau1772_reg_pll_init[] = {
	/* PLL Denominator MSB register */
	{ADAU1772_PLL_CTRL0, CONFIG_ADAU1772_0_PLL_DENOMINATOR >> 8},
	/* PLL Denominator LSB register */
	{ADAU1772_PLL_CTRL1, CONFIG_ADAU1772_0_PLL_DENOMINATOR & 0xff},
	/* PLL Numerator MSB register */
	{ADAU1772_PLL_CTRL2, CONFIG_ADAU1772_0_PLL_NUMERATOR >> 8},
	/* PLL Numerator LSB register */
	{ADAU1772_PLL_CTRL3, CONFIG_ADAU1772_0_PLL_NUMERATOR & 0xff},
	/* PLL Integer Setting register */
	{ADAU1772_PLL_CTRL4, CONFIG_ADAU1772_0_PLL_INTEGER << 3 |
		(CONFIG_ADAU1772_0_PLL_INPUT_DIVIDER - 1) << 1 | 1},
};

static struct reg_data adau1772_reg_format_i2s[] = {
	/* Set serial port data format and sampling rate */
	{ADAU1772_SAI_0, 1<<0},
	/* Set serial: 16-bit data, 16 BCLK per channel */
	{ADAU1772_SAI_1, 1<<6 | 1<<2},
};

static struct reg_data adau1772_reg_format_pcm_short_tdm2[] = {
	/* data synchronized to the edge of the frame clock, sampling rate
	 * 8000 Hz, TDM2 mode
	 */
	{ADAU1772_SAI_0, 1<<6 | 1<<4 | 1<<0},
	/* Set serial: 16-bit data, frame clock 1 BCLK long, positive frame
	 * pulse, 16 BCLK per channel, tristate unused outputs
	 */
	{ADAU1772_SAI_1, 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<2 | 1<<1},
};

static struct reg_data adau1772_reg_format_pcm_short_tdm4[] = {
	/* data synchronized to the edge of the frame clock, sampling rate
	 * 8000 Hz, TDM4 mode
	 */
	{ADAU1772_SAI_0, 1<<6 | 2<<4 | 1<<0},
	/* Set serial: 16-bit data, frame clock 50% duty cycle, positive frame
	 * pulse, 16 BCLK per channel, tristate unused outputs
	 */
	{ADAU1772_SAI_1, 1<<7 | 1<<6 | 0<<5 | 1<<4 | 1<<2 | 1<<1},
};

static struct reg_data adau1772_reg_format_pcm_short_tdm8[] = {
	/* data synchronized to the edge of the frame clock, sampling rate
	 * 8000 Hz, TDM8 mode
	 */
	{ADAU1772_SAI_0, 1<<6 | 3<<4 | 1<<0},
	/* Set serial: 16-bit data, frame clock 1 BCLK long, positive frame
	 * pulse, 16 BCLK per channel, tristate unused outputs
	 */
	{ADAU1772_SAI_1, 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<2 | 1<<1},
};

static struct reg_data adau1772_reg_init_playback[] = {
	/* Disable DSP core, not used */
	{ADAU1772_CORE_ENABLE, 0x00},
	/* Power up ASRC & DAC */
	{ADAU1772_INTERP_PWR_MODES, 0x0F},
	/* Route ASRC output to DAC input */
	{ADAU1772_DAC_SOURCE_0_1, 0xDC},
	/* Enable DAC */
	{ADAU1772_DAC_CONTROL1, 0x3},
	/* Turn volume up to 0 dB / maximum level */
	{ADAU1772_DAC0_VOLUME, 30}, /* TODO: verify default value */
	{ADAU1772_DAC1_VOLUME, 30},
	/* Enable analog outputs */
	{ADAU1772_OP_STAGE_CTRL, 0},
	/* Unmute */
	{ADAU1772_OP_STAGE_MUTES, 0},
};

static struct reg_data adau1772_reg_init_capture[] = {
	/* Enable serial output */
	{ADAU1772_MODE_MP1, 0x00},
	{ADAU1772_ASRCO_SOURCE_0_1, 0x44},
	/* Route ASRC output 0 to slot 0, serial input 0 to slot 1 */
	{ADAU1772_SOUT_SOURCE_0_1, 0x84},
	/* Route ASRC output 0 to slot 2, serial input 2 to slot 3 */
	{ADAU1772_SOUT_SOURCE_2_3, 0xa4},
	/* Route ASRC output 0 to slot 4, serial input 4 to slot 5 */
	{ADAU1772_SOUT_SOURCE_4_5, 0xc4},
	/* Enable power to ASRC0, ASRC1, ADC0 */
	{ADAU1772_DECIM_PWR_MODES, 0x31},
	/* Enable Programmable Gain Amplifier */
	{ADAU1772_PGA_CONTROL_0, 0xBF},
	/* Enable Microphone Bias */
	{ADAU1772_MIC_BIAS, 0x11},
	/* Enable ADC0 */
	{ADAU1772_ADC_CONTROL2, 0x61},
	/* Set analog inputs sensitivity */
	{ADAU1772_ADC0_VOLUME, 00}, /* TODO: verify default value */
	/* Unmute ADC0, set sample rate at 96 kHz */
	{ADAU1772_ADC_CONTROL0, 0x10},
};

static int read_reg(struct device *dev, u8_t i2c_addr, u8_t reg, u8_t *data)
{
	u8_t iaddr[2] = {0, reg};
	int ret;

	ret = i2c_write_read(dev, i2c_addr, iaddr, sizeof(iaddr), data, 1);
	if (ret != 0) {
		LOG_ERR("I2C_addr=0x%x, reg_addr=0x%x, result=%d",
			    i2c_addr, reg, ret);
	}

	return ret;
}

static int write_reg(struct device *dev, u8_t i2c_addr, u8_t reg, u8_t val)
{
	u8_t buf[3] = {0, reg, val};
	int ret;

	ret = i2c_write(dev, buf, sizeof(buf), i2c_addr);
	if (ret != 0) {
		LOG_ERR("I2C_addr=0x%x, reg_addr=0x%x, result=%d",
			    i2c_addr, reg, ret);
	}

	return ret;
}

static void write_reg_list(struct device *i2c_dev, u8_t i2c_addr,
			   struct reg_data *reg_data, u32_t len)
{
	for (int i = 0; i < len; i++) {
		write_reg(i2c_dev, i2c_addr, reg_data[i].addr, reg_data[i].val);
	}
}

static int initialize_pll(struct device *i2c_dev, u8_t i2c_addr)
{
	u8_t reg_val;
	u32_t retries = 20;  /* will wait at least 20 ms */

	/* Ensure PLL is disabled, disable XTAL */
	write_reg(i2c_dev, i2c_addr, ADAU1772_CLK_CONTROL, 1 << 4);

	/* Configure parameters */
	write_reg_list(i2c_dev, i2c_addr, adau1772_reg_pll_init,
		       ARRAY_SIZE(adau1772_reg_pll_init));

	/* Enable PLL */
	write_reg(i2c_dev, i2c_addr, ADAU1772_CLK_CONTROL, 1 << 7 | 1 << 4);

	/* Wait until PLL is locked */
	do {
		if (retries-- == 0) {
			LOG_ERR("timeout");
			return -ETIMEDOUT;
		}

		k_sleep(1);
		read_reg(i2c_dev, i2c_addr, ADAU1772_PLL_CTRL5, &reg_val);
	} while (!(reg_val & 1));

	/* Enable the main clock and connect it to the PLL */
	write_reg(i2c_dev, i2c_addr, ADAU1772_CLK_CONTROL,
		  1 << 7 | 1 << 4 | 1 << 3 | 1 << 0);

	LOG_DBG("PLL @0x%x initialized", i2c_addr);

	return 0;
}

static int adau1772_configure(struct device *dev,
			      struct audio_config *audio_cfg)
{
	const struct adau1772_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);
	struct i2s_config i2s_cfg;
	u32_t data_rate;
	u32_t word_size_bits;
	u32_t channels = 2 * dev_cfg->num_chips;
	struct reg_data *reg_format;
	u32_t reg_format_len;

	if (dev_cfg->num_chips == 0) {
		LOG_ERR("No ADAU1772 chip is enabled");
		return -EINVAL;
	}

	/* Configure data rate */
	switch (audio_cfg->format & AUDIO_PCM_RATE_MASK) {
	case AUDIO_PCM_RATE_8000:
		data_rate = 8000;
		break;
	default:
		LOG_ERR("Unsupported data rate");
		return -EIO;
	}

	/* Configure data format */
	switch (audio_cfg->format & AUDIO_PCM_FORMAT_MASK) {
	case AUDIO_PCM_FORMAT_S16_BE:
		word_size_bits = 16;
		break;
	default:
		LOG_ERR("Unsupported audio format");
		return -EIO;
	}

	i2s_cfg.word_size = word_size_bits;
	i2s_cfg.channels = channels;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_PCM_LONG;
	i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	i2s_cfg.frame_clk_freq = data_rate;
	i2s_cfg.mem_slab = audio_cfg->rx_mem_slab;
	i2s_cfg.block_size = audio_cfg->block_size;
	i2s_cfg.timeout = audio_cfg->rx_timeout;

	i2s_configure(dev_data->dev_i2s, I2S_DIR_RX, &i2s_cfg);

	i2s_cfg.mem_slab = audio_cfg->tx_mem_slab;
	i2s_cfg.timeout = audio_cfg->tx_timeout;

	i2s_configure(dev_data->dev_i2s, I2S_DIR_TX, &i2s_cfg);

	if (dev_cfg->num_chips == 1) {
		reg_format = adau1772_reg_format_pcm_short_tdm2;
		reg_format_len = ARRAY_SIZE(adau1772_reg_format_pcm_short_tdm2);
	} else if (dev_cfg->num_chips == 2) {
		reg_format = adau1772_reg_format_pcm_short_tdm4;
		reg_format_len = ARRAY_SIZE(adau1772_reg_format_pcm_short_tdm4);
	} else {
		reg_format = adau1772_reg_format_pcm_short_tdm8;
		reg_format_len = ARRAY_SIZE(adau1772_reg_format_pcm_short_tdm8);
	}

	/* Program ADAU registers */
	for (int i = 0; i < dev_cfg->num_chips; i++) {
		write_reg_list(dev_data->dev_i2c, dev_cfg->i2c_addr_list[i],
			       reg_format, reg_format_len);
		/* Enable input ASRC, output ASRC, configure TDM slot */
		write_reg(dev_data->dev_i2c, dev_cfg->i2c_addr_list[i],
			  ADAU1772_ASRC_MODE, i<<2 | 1<<1 | 1<<0);
		/* Disable unused TDM slots */
		write_reg(dev_data->dev_i2c, dev_cfg->i2c_addr_list[i],
			  ADAU1772_SOUT_CONTROL0, ~(3<<(2*i)));
		write_reg_list(dev_data->dev_i2c, dev_cfg->i2c_addr_list[i],
			       adau1772_reg_init_playback,
			       ARRAY_SIZE(adau1772_reg_init_playback));
		write_reg_list(dev_data->dev_i2c, dev_cfg->i2c_addr_list[i],
			       adau1772_reg_init_capture,
			       ARRAY_SIZE(adau1772_reg_init_capture));
	}
#if 0
	u8_t out_mask = (1 << (dev_cfg->num_chips - 1)) - 1;
	write_reg(dev_data->dev_i2c, dev_cfg->i2c_addr_list[dev_cfg->num_chips - 1],
		  ADAU1772_SOUT_CONTROL0, out_mask);
#endif
	return 0;
}

static int adau1772_read(struct device *dev, void **mem_block, size_t *size)
{
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);

	return i2s_read(dev_data->dev_i2s, mem_block, size);
}

static int adau1772_write(struct device *dev, void *mem_block, size_t size)
{
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);

	return i2s_write(dev_data->dev_i2s, mem_block, size);
}

static int adau1772_trigger(struct device *dev, u32_t dir,
			    enum audio_trigger_cmd cmd)
{
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);

	return i2s_trigger(dev_data->dev_i2s, dir, cmd);
}

static int adau1772_register_callback(struct device *dev, enum i2s_dir dir,
				      i2s_callback_t cb, void *arg)
{
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);

	return i2s_register_callback(dev_data->dev_i2s, dir, cb, arg);
}

static int adau1772_volume(struct device *dev, u32_t channel, s32_t volume)
{
	const struct adau1772_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);
	int num_chip = channel / 2;
	int num_dac = channel % 2;
	int ret;

	ret = write_reg(dev_data->dev_i2c, dev_cfg->i2c_addr_list[num_chip],
			ADAU1772_DAC0_VOLUME + num_dac, volume);

	return ret;
}

static int adau1772_initialize(struct device *dev)
{
	const struct adau1772_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);
	int ret;

	/* Initialize semaphores */
	k_sem_init(&dev_data->rx_sem, 0, 1);
	k_sem_init(&dev_data->tx_sem, 0, 1);

	dev_data->dev_i2c = device_get_binding(dev_cfg->i2c_dev_name);
	if (!dev_data->dev_i2c) {
		LOG_ERR("I2C: Device not found");
		return -EIO;
	}

	dev_data->dev_i2s = device_get_binding(dev_cfg->i2s_dev_name);
	if (!dev_data->dev_i2s) {
		LOG_ERR("I2S: Device not found");
		return -EIO;
	}

	/* Wait 12ms for an ADAU1772 to come out of an internal reset */
	k_sleep(12);

	/*
	 * Initialize PLL
	 */
	for (int i = 0; i < dev_cfg->num_chips; i++) {
		ret = initialize_pll(dev_data->dev_i2c,
					dev_cfg->i2c_addr_list[i]);
		if (ret < 0) {
			return ret;
		}
	}

	LOG_INF("Device %s initialized"
#ifdef CONFIG_ADAU1772_0_MULTICHIP
		    " in multi-chip mode"
#endif
		    , dev->config->name);

	return 0;
}

static const struct audio_driver_api adau1772_driver_api = {
	.pcm_configure = adau1772_configure,
	.read = adau1772_read,
	.write = adau1772_write,
	.trigger = adau1772_trigger,
	.register_callback = adau1772_register_callback,
	.volume = adau1772_volume,
};

/* ADAU1772_0 */

#ifdef CONFIG_AUDIO_ADAU1772_0
static struct device DEVICE_NAME_GET(adau1772_0);

static u8_t adau1772_0_i2c_addr_list[] = {
#if defined(CONFIG_ADAU1772_0_I2C_SLAVE_0) || defined(CONFIG_ADAU1772_0_I2C_SLAVE_MULTICHIP_0)
	0x3D,
#endif
#if defined(CONFIG_ADAU1772_0_I2C_SLAVE_1) || defined(CONFIG_ADAU1772_0_I2C_SLAVE_MULTICHIP_1)
	0x3C,
#endif
#if defined(CONFIG_ADAU1772_0_I2C_SLAVE_2) || defined(CONFIG_ADAU1772_0_I2C_SLAVE_MULTICHIP_2)
	0x3E,
#endif
#if defined(CONFIG_ADAU1772_0_I2C_SLAVE_3) || defined(CONFIG_ADAU1772_0_I2C_SLAVE_MULTICHIP_3)
	0x3F,
#endif
};

static const struct adau1772_dev_cfg adau1772_0_config = {
	.i2s_dev_name = CONFIG_ADAU1772_0_I2S_DEV_NAME,
	.i2c_dev_name = CONFIG_ADAU1772_0_I2C_DEV_NAME,
	.i2c_addr_list = adau1772_0_i2c_addr_list,
	.num_chips = ARRAY_SIZE(adau1772_0_i2c_addr_list),
};

static struct adau1772_dev_data adau1772_0_dev_data;

DEVICE_AND_API_INIT(adau1772_0, CONFIG_ADAU1772_0_NAME, &adau1772_initialize,
		    &adau1772_0_dev_data, &adau1772_0_config, POST_KERNEL,
		    CONFIG_AUDIO_CODEC_INIT_PRIORITY, &adau1772_driver_api);
#endif /* CONFIG_AUDIO_ADAU1772_0 */
