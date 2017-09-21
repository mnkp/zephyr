/*
 * Copyright (c) 2017 comsuisse AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for the Audio interface.
 */

#ifndef __AUDIO_H__
#define __AUDIO_H__

/**
 * @brief Audio Interface
 * @defgroup audio_interface Audio Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <device.h>
#include <drivers/i2s.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*audio_callback_t)(struct device *dev, void *arg);

/** PCM Rate bit field position */
#define AUDIO_PCM_RATE_SHIFT         0
/** PCM Rate bit field mask */
#define AUDIO_PCM_RATE_MASK         (0xf << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_5512         (0 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_8000         (1 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_11025        (2 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_16000        (3 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_22050        (4 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_32000        (5 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_44100        (6 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_48000        (7 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_64000        (8 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_88200        (9 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_96000       (10 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_176400      (11 << AUDIO_PCM_RATE_SHIFT)
#define AUDIO_PCM_RATE_192000      (12 << AUDIO_PCM_RATE_SHIFT)

/** PCM Format bit field position */
#define AUDIO_PCM_FORMAT_SHIFT       4
/** PCM Format bit field mask */
#define AUDIO_PCM_FORMAT_MASK       (0xf << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 8-bit */
#define AUDIO_PCM_FORMAT_S8         (0 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 8-bit */
#define AUDIO_PCM_FORMAT_U8         (1 << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 16-bit, little endian */
#define AUDIO_PCM_FORMAT_S16_LE     (2 << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 16-bit, big endian */
#define AUDIO_PCM_FORMAT_S16_BE     (3 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 16-bit, little endian */
#define AUDIO_PCM_FORMAT_U16_LE     (4 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 16-bit, big endian */
#define AUDIO_PCM_FORMAT_U16_BE     (5 << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 24-bit, little endian */
#define AUDIO_PCM_FORMAT_S24_LE     (6 << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 24-bit, big endian */
#define AUDIO_PCM_FORMAT_S24_BE     (7 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 24-bit, little endian */
#define AUDIO_PCM_FORMAT_U24_LE     (8 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 24-bit, big endian */
#define AUDIO_PCM_FORMAT_U24_BE     (9 << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 32-bit, little endian */
#define AUDIO_PCM_FORMAT_S32_LE    (10 << AUDIO_PCM_FORMAT_SHIFT)
/** Signed, 32-bit, big endian */
#define AUDIO_PCM_FORMAT_S32_BE    (11 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 32-bit, little endian */
#define AUDIO_PCM_FORMAT_U32_LE    (12 << AUDIO_PCM_FORMAT_SHIFT)
/** Unsigned, 32-bit, big endian */
#define AUDIO_PCM_FORMAT_U32_BE    (13 << AUDIO_PCM_FORMAT_SHIFT)

#ifdef AUDIO_BIG_ENDIAN
#define AUDIO_PCM_FORMAT_S16       AUDIO_PCM_FORMAT_S16_BE
#define AUDIO_PCM_FORMAT_U16       AUDIO_PCM_FORMAT_U16_BE
#define AUDIO_PCM_FORMAT_S24       AUDIO_PCM_FORMAT_S24_BE
#define AUDIO_PCM_FORMAT_U24       AUDIO_PCM_FORMAT_U24_BE
#define AUDIO_PCM_FORMAT_S32       AUDIO_PCM_FORMAT_S32_BE
#define AUDIO_PCM_FORMAT_U32       AUDIO_PCM_FORMAT_U32_BE
#else
#define AUDIO_PCM_FORMAT_S16       AUDIO_PCM_FORMAT_S16_LE
#define AUDIO_PCM_FORMAT_U16       AUDIO_PCM_FORMAT_U16_LE
#define AUDIO_PCM_FORMAT_S24       AUDIO_PCM_FORMAT_S24_LE
#define AUDIO_PCM_FORMAT_U24       AUDIO_PCM_FORMAT_U24_LE
#define AUDIO_PCM_FORMAT_S32       AUDIO_PCM_FORMAT_S32_LE
#define AUDIO_PCM_FORMAT_U32       AUDIO_PCM_FORMAT_U32_LE
#endif

#define AUDIO_STREAM_PLAYBACK      I2S_DIR_TX
#define AUDIO_STREAM_CAPTURE       I2S_DIR_RX

enum audio_trigger_cmd {
	AUDIO_TRIGGER_START,
	AUDIO_TRIGGER_STOP,
	AUDIO_TRIGGER_DRAIN,
	AUDIO_TRIGGER_DROP,
	AUDIO_TRIGGER_PREPARE,
};

struct audio_config {
	u32_t format;
	struct k_mem_slab *rx_mem_slab;
	struct k_mem_slab *tx_mem_slab;
	u16_t block_size;
	s32_t rx_timeout;
	s32_t tx_timeout;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal use only, skip these in public documentation.
 */
typedef int (*audio_api_pcm_configure_t)(struct device *dev,
					 struct audio_config *cfg);
typedef int (*audio_api_read_t)(struct device *dev, void **mem_block,
				size_t *size);
typedef int (*audio_api_write_t)(struct device *dev, void *mem_block,
				 size_t size);
typedef int (*audio_api_trigger_t)(struct device *dev, u32_t dir,
				   enum audio_trigger_cmd cmd);
typedef int (*audio_api_audio_volume_t)(struct device *dev, u32_t channel,
					s32_t volume);
typedef int (*audio_register_callback_t)(struct device *dev, enum i2s_dir dir,
					 audio_callback_t cb, void *arg);

struct audio_driver_api {
	audio_api_pcm_configure_t pcm_configure;
	audio_api_read_t read;
	audio_api_write_t write;
	audio_api_trigger_t trigger;
	audio_register_callback_t register_callback;
	audio_api_audio_volume_t volume;
};
/**
 * @endcond
 */

/**
 * @brief Configure operation of a host controller.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param audio_config Bit-packed 32-bit value to the device runtime
 *        configuration for the audio controller.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int audio_pcm_configure(struct device *dev,
				      struct audio_config *cfg)
{
	const struct audio_driver_api *api = dev->driver_api;

	return api->pcm_configure(dev, cfg);
}

/**
 * @brief Read data from an Audio device.
 *
 * This routine reads a set amount of data synchronously.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mem_block Pointer to block address area.
 * @param size Number of bytes to read.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int audio_read(struct device *dev, void **mem_block, size_t *size)
{
	const struct audio_driver_api *api = dev->driver_api;

	return api->read(dev, mem_block, size);
}

/**
 * @brief Write data to an Audio device.
 *
 * This routine writes a set amount of data asynchronously.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mem_block Pointer to block address area.
 * @param size Number of bytes to write.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int audio_write(struct device *dev, void *mem_block, size_t size)
{
	const struct audio_driver_api *api = dev->driver_api;

	return api->write(dev, mem_block, size);
}

/**
 * @brief Start/stop audio operation.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param cmd Trigger command
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ENOMEM Buffer memory not available.
 */
static inline int audio_trigger(struct device *dev, u32_t dir,
				enum audio_trigger_cmd cmd)
{
	const struct audio_driver_api *api = dev->driver_api;

	return api->trigger(dev, dir, cmd);
}

/**
 * @brief Install transfer callback.
 *
 * The callback function will be executed at the end of each transferred or
 * received memory block. It's primary goal is to help synchronize timing.
 *
 * @remark The callback function will be executed in the interrupt context.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dir Stream direction: RX or TX.
 * @param cb  Pointer to the callback function.
 * @param arg argument which will be passed to the callback function.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid argument.
 */
static inline int audio_register_callback(struct device *dev, enum i2s_dir dir,
					  audio_callback_t cb, void *arg)
{
	const struct audio_driver_api *api = dev->driver_api;

	return api->register_callback(dev, dir, cb, arg);
}

/**
 * @brief Control audio volume.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param channel channel number
 * @param volume audio volume value
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ENOMEM Buffer memory not available.
 */
static inline int audio_volume(struct device *dev, u32_t channel,
				s32_t volume)
{
	const struct audio_driver_api *api = dev->driver_api;

	return api->volume(dev, channel, volume);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __AUDIO_H__ */
