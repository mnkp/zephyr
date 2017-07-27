/*
 * Copyright (c) 2021 Nearsoft PL
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief ADAU1772 Audio Processor.
 */

#define DT_DRV_COMPAT adi_adau1772

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <audio/codec.h>
#include "adau1772.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(adau1772, CONFIG_AUDIO_CODEC_LOG_LEVEL);

#define PLL_OUTPUT_FREQ_HZ   24576000

/* Device constant configuration parameters */
struct adau1772_dev_cfg {
	struct gpio_dt_spec pin_pd;
	const struct device *dev_i2c;
	uint8_t i2c_addr;
	int8_t instance;
};

/* Device run time data */
struct adau1772_dev_data {
	struct k_sem rx_sem;
	struct k_sem tx_sem;
};

#define DEV_CFG(dev) \
	((const struct adau1772_dev_cfg *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct adau1772_dev_data *const)(dev)->data)

struct reg_data {
	uint8_t addr;
	uint8_t val;
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
	{ADAU1772_SAI_1, 1<<7 | 1<<6 | 0<<5 | 1<<4 | 1<<2 | 1<<1},
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
	{ADAU1772_SAI_1, 1<<7 | 1<<6 | 0<<5 | 1<<4 | 1<<2 | 1<<1},
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
	/* Enable PGA (Programmable Gain Amplifier) */
	{ADAU1772_PGA_CONTROL_0, 0xBF},
	/* Boost PGA by 10 dB */
	{ADAU1772_PGA_10DB_BOOST, 0x01},
	/* Enable Microphone Bias */
	{ADAU1772_MIC_BIAS, 0x11},
	/* Enable ADC0 */
	{ADAU1772_ADC_CONTROL2, 0x61},
	/* Set analog inputs sensitivity */
	{ADAU1772_ADC0_VOLUME, 00}, /* TODO: verify default value */
	/* Unmute ADC0, set sample rate at 96 kHz */
	{ADAU1772_ADC_CONTROL0, 0x10},
};

static int write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct adau1772_dev_cfg *const dev_cfg = DEV_CFG(dev);
	const struct device *dev_i2c = dev_cfg->dev_i2c;
	uint8_t i2c_addr = dev_cfg->i2c_addr;
	uint8_t buf[3] = {0, reg, val};
	int ret;

	LOG_DBG("I2C_addr=0x%x, reg_addr=0x%x, val=%d", i2c_addr, reg, val);

	ret = i2c_write(dev_i2c, buf, sizeof(buf), i2c_addr);
	if (ret != 0) {
		LOG_ERR("I2C_addr=0x%x, reg_addr=0x%x, result=%d", i2c_addr, reg, ret);
	}

	return ret;
}

static void write_reg_list(const struct device *dev, struct reg_data *reg_data,
		uint32_t len)
{
	for (int i = 0; i < len; i++) {
		write_reg(dev, reg_data[i].addr, reg_data[i].val);
	}
}

static int adau1772_configure(const struct device *dev,
		struct audio_codec_cfg *audio_cfg)
{
	const struct adau1772_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct i2s_config *i2s = &audio_cfg->dai_cfg.i2s;
	struct reg_data *reg_format;
	uint32_t reg_format_len;

	if (audio_cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("dai_type must be AUDIO_DAI_TYPE_I2S");
		return -EINVAL;
	}

	if (i2s->channels == 2) {
		reg_format = adau1772_reg_format_pcm_short_tdm2;
		reg_format_len = ARRAY_SIZE(adau1772_reg_format_pcm_short_tdm2);
	} else if (i2s->channels == 4) {
		reg_format = adau1772_reg_format_pcm_short_tdm4;
		reg_format_len = ARRAY_SIZE(adau1772_reg_format_pcm_short_tdm4);
	} else if (i2s->channels == 8) {
		reg_format = adau1772_reg_format_pcm_short_tdm8;
		reg_format_len = ARRAY_SIZE(adau1772_reg_format_pcm_short_tdm8);
	} else {
		return -EINVAL;
	}

	/* Program ADAU registers */
	write_reg_list(dev, reg_format, reg_format_len);
	/* Enable input ASRC, output ASRC, configure TDM slot */
	write_reg(dev, ADAU1772_ASRC_MODE, dev_cfg->instance<<2 | 1<<1 | 1<<0);
	/* Disable unused TDM slots */
	write_reg(dev, ADAU1772_SOUT_CONTROL0, ~(3<<(2*dev_cfg->instance)));
	write_reg_list(dev,
			adau1772_reg_init_playback,
			ARRAY_SIZE(adau1772_reg_init_playback));
	write_reg_list(dev,
			adau1772_reg_init_capture,
			ARRAY_SIZE(adau1772_reg_init_capture));
#if 0
	uint8_t out_mask = (1 << (CONFIG_ADAU1772_0_MULTICHIP - 1)) - 1;
	write_reg(dev_cfg->dev_i2c, dev_cfg->i2c_addr_list[CONFIG_ADAU1772_0_MULTICHIP - 1],
		  ADAU1772_SOUT_CONTROL0, out_mask);
#endif
	return 0;
}

static int adau1772_set_output_volume(const struct device *dev,
		audio_channel_t channel, int vol)
{
	int ret = -EINVAL;

	if (vol > 0) {
		return ret;
	}

	/* Remove sign */
	vol = -vol;

	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
		ret = write_reg(dev, ADAU1772_DAC0_VOLUME, vol);
		break;

	case AUDIO_CHANNEL_FRONT_RIGHT:
		ret = write_reg(dev, ADAU1772_DAC1_VOLUME, vol);
		break;

	case AUDIO_CHANNEL_ALL:
		ret = write_reg(dev, ADAU1772_DAC0_VOLUME, vol);
		if (ret != 0) {
			break;
		}
		ret = write_reg(dev, ADAU1772_DAC1_VOLUME, vol);
		break;

	default:
		break;
	}

	return ret;
}

static int adau1772_mute_output(const struct device *dev,
		audio_channel_t channel, bool mute)
{
	int ret = -EINVAL;

	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
	case AUDIO_CHANNEL_FRONT_RIGHT:
	case AUDIO_CHANNEL_ALL:
	default:
		break;
	}

	return ret;
}

static int adau1772_set_property(const struct device *dev,
		audio_property_t property,
		audio_channel_t channel,
		audio_property_value_t val)
{
	int ret = -EINVAL;

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		ret = adau1772_set_output_volume(dev, channel, val.vol);
		break;

	case AUDIO_PROPERTY_OUTPUT_MUTE:
		ret = adau1772_mute_output(dev, channel, val.mute);
		break;

	default:
		break;
	}

	return ret;
}

static void adau1772_start_output(const struct device *dev)
{
	/* Enable analog outputs */
	write_reg(dev, ADAU1772_OP_STAGE_CTRL, 0);

	/* Unmute analog outputs */
	write_reg(dev, ADAU1772_OP_STAGE_MUTES, 0);
}

static void adau1772_stop_output(const struct device *dev)
{
	/* Mute analog outputs */
	write_reg(dev, ADAU1772_OP_STAGE_MUTES, 0xF);

	/* Disable analog outputs */
	write_reg(dev, ADAU1772_OP_STAGE_CTRL, 0xF);
}

static int adau1772_apply_properties(const struct device *dev)
{
	/* Nothing to do, there is nothing cached */
	return 0;
}

static int adau1772_initialize(const struct device *dev)
{
	const struct adau1772_dev_cfg *const dev_cfg = DEV_CFG(dev);
	struct adau1772_dev_data *const dev_data = DEV_DATA(dev);

	/* Initialize semaphores */
	k_sem_init(&dev_data->rx_sem, 0, 1);
	k_sem_init(&dev_data->tx_sem, 0, 1);

	if (dev_cfg->pin_pd.port != NULL) {
		gpio_pin_configure_dt(&dev_cfg->pin_pd, GPIO_OUTPUT_INACTIVE);
	}

	if (!device_is_ready(dev_cfg->dev_i2c)) {
		LOG_ERR("I2C: Device is not ready");
		return -EIO;
	}

	/* Wait 12ms for an ADAU1772 to come out of an internal reset */
	k_sleep(K_MSEC(12));

	/*
	 * Initialize the clock
	 */
	write_reg(dev, ADAU1772_CLK_CONTROL, 1 << 4 | 1 << 2 | 1 << 1 | 1 << 0);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static const struct audio_codec_api adau1772_driver_api = {
	.configure = adau1772_configure,
	.start_output = adau1772_start_output,
	.stop_output = adau1772_stop_output,
	.set_property = adau1772_set_property,
	.apply_properties = adau1772_apply_properties,
};

#define ADAU1772_INIT(idx)	\
	static const struct adau1772_dev_cfg adau1772_##idx##_dev_cfg = { \
		.pin_pd = GPIO_DT_SPEC_INST_GET_OR(idx, pd_gpios, {0}),	\
		.dev_i2c = DEVICE_DT_GET(DT_BUS(DT_INST(idx, adi_adau1772))), \
		.i2c_addr = DT_INST_REG_ADDR(idx),	\
		.instance = DT_INST_PROP(idx, input_channels##_ENUM_IDX), \
	};	\
	\
	static struct adau1772_dev_data adau1772_##idx##_dev_data;	\
	\
	DEVICE_DT_INST_DEFINE(idx, adau1772_initialize, NULL,	\
			      &adau1772_##idx##_dev_data, &adau1772_##idx##_dev_cfg, \
			      POST_KERNEL, CONFIG_AUDIO_CODEC_INIT_PRIORITY, \
			      &adau1772_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADAU1772_INIT)
