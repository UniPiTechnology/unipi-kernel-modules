/*
 * Unipi spi device driver - Copyright (C) 2021 Unipi Technology
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *    functions to update firware in Unipi boards over spi
 */

/************
 * Includes *
 ************/
#include <linux/version.h>

#include "unipi_common.h"
#include "unipi_spi.h"
#include "unipi_spi_crc.h"


#define UNIPI_FW_DETAILED_DEBUG 0
#if UNIPI_FW_DETAILED_DEBUG > 1
# define unipi_fw_trace_1(spi, f, args...)	dev_info(&spi->dev, f, ##args)
#else
# define unipi_fw_trace_1(spi, f, args...) {}
#endif

#if UNIPI_FW_DETAILED_DEBUG > 0
# define unipi_fw_trace(spi, f, args...)	dev_info(&spi->dev, f, ##args)
#else
# define unipi_fw_trace(spi, f, args...) {}
#endif

#define unipi_fw_error(spi, f, args...)	dev_err(&spi->dev, f, ##args)


struct spi_message * unipi_spi_setup_message(struct spi_device* spi_dev,
                                             u8* send_buf, u8* recv_buf, int len)
{
	struct spi_transfer * s_trans;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	int trans_count, remain, i;
	int freq;
	struct spi_message *message;

	len += 2;
	trans_count = ((len-1) / NEURONSPI_MAX_TX) + 2; // number of transmissions

	message = kzalloc(sizeof(struct spi_message) + trans_count * sizeof(struct spi_transfer), GFP_ATOMIC);
	if (! message) {
		return NULL;
	}
	freq = n_spi->frequency;
	if (freq > 6000000) freq = 6000000;

	s_trans = (struct spi_transfer *)(message + 1);
	spi_message_init_with_transfers(message, s_trans, trans_count);

	s_trans[0].delay.value = UNIPI_SPI_EDGE_DELAY;
	s_trans[0].delay.unit = SPI_DELAY_UNIT_USECS;
	s_trans[0].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[0].speed_hz = freq;

	unipi_spi_crc_set(send_buf, len-2, 0);
	unipi_fw_trace_1(spi_dev, "Fw write(%3d) %32ph\n", len, send_buf);
	remain = len;
	for (i = 1; i < trans_count; i++) {
			s_trans[i].delay.value = 0;
			s_trans[i].delay.unit = SPI_DELAY_UNIT_USECS;
			s_trans[i].bits_per_word = UNIPI_SPI_B_PER_WORD;
			s_trans[i].speed_hz = freq;
			s_trans[i].tx_buf = send_buf + (NEURONSPI_MAX_TX * (i - 1));
			s_trans[i].rx_buf = recv_buf + (NEURONSPI_MAX_TX * (i - 1));
			s_trans[i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
			remain -= NEURONSPI_MAX_TX;
	}
	// len is size of second message WITHOUT crc
	len -= 2;
    message->spi = spi_dev;
    return message;
}

/* - len is without crc -- must be corrected in char driver */
int unipi_spi_firmware_op(void *proto_self, u8* send_buf, u8* recv_buf, s32 len, int firmware_cookie)
{
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	struct spi_message *message;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	unsigned long flags;
	int ret_code;

	spin_lock_irqsave(&n_spi->firmware_lock, flags);
	if (n_spi->firmware_in_progress) {
		if (n_spi->firmware_in_progress != firmware_cookie) {
			spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
			unipi_fw_error(spi_dev,"Firmare operation already in-progress");
			return -EACCES;
		}
	} else {
		n_spi->firmware_in_progress = firmware_cookie;
	}
	spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
	message = unipi_spi_setup_message(spi_dev, send_buf, recv_buf, len);
	if (message == NULL) 
		return -ENOMEM;
	ret_code = spi_sync(spi_dev, message);
	kfree(message);
	return ret_code;
}
EXPORT_SYMBOL_GPL(unipi_spi_firmware_op);

int unipi_spi_lock(void *proto_self)
{
	unsigned long flags;
	int cookie;
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);

	n_spi->enable_v2 = 0;
	unipi_spi_set_v1(spi_dev);

	spin_lock_irqsave(&n_spi->firmware_lock, flags);
	if (n_spi->firmware_in_progress) {
		spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
		return 0;
	}
	cookie = hash_64(get_cycles(),32);
	n_spi->firmware_in_progress = cookie;
	spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
	return cookie;
}
EXPORT_SYMBOL_GPL(unipi_spi_lock);

void unipi_spi_unlock(void *proto_self)
{
	unsigned long flags;
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);

	spin_lock_irqsave(&n_spi->firmware_lock, flags);
	if (n_spi->firmware_in_progress) n_spi->enable_v2 = n_spi->allow_v2;
	n_spi->firmware_in_progress = 0;
	spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
}
EXPORT_SYMBOL_GPL(unipi_spi_unlock);
