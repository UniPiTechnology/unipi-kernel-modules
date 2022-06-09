/*
 * Unipi PLC spi device driver - Copyright (C) 2018 Unipi Technology
 * 
 * Author: Tomas Knot <tomasknot@gmail.com>
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

/************
 * Includes *
 ************/
#include <linux/completion.h>
#include <linux/cpufreq.h>
#include <linux/version.h>
#include <uapi/linux/sched/types.h>

#include "unipi_common.h"
//#include "unipi_modbus.h"
#include "unipi_spi.h"
#include "unipi_spi_crc.h"

extern struct unipi_protocol_op spi_op_v1;
extern struct unipi_protocol_op spi_op_v2;
#define UNIPI_MFD_COIL_PROTOCOL_MODE 1007

static ssize_t messages_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.messages);
	return len;
}
static ssize_t bytes_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%llu", n_spi->stat.bytes);
	return len;
}
static ssize_t errors_tx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_tx);
	return len;
}
static ssize_t messages_prio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	//unsigned long flags;
	//spin_lock_irqsave(&n_spi->lock, flags);
	len = sprintf(buf, "%lu", n_spi->stat.messages_prio);
	//spin_unlock_irqrestore(&n_spi->lock, flags);
	return len;
}
static ssize_t errors_crc1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_crc1);
	return len;
}
static ssize_t errors_crc2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_crc2);
	return len;
}
static ssize_t errors_opcode1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_opcode1);
	return len;
}

static ssize_t firmware_in_progress_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%d", n_spi->firmware_in_progress);
	return len;
}

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%u Hz", n_spi->frequency);
	return len;
}
static ssize_t frequency_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		n_spi->frequency = val;
	}
	return count;
}

static ssize_t hmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%d", n_spi->hmode);
	return len;
}
static ssize_t hmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		if (val == 0 || val == 1) {
			n_spi->hmode = val;
			n_spi->channel.op = (val == 1)? &spi_op_v2 : &spi_op_v1;
		}
	}
	return count;
}

DEVICE_ATTR_RO(messages);
DEVICE_ATTR_RO(errors_tx);
DEVICE_ATTR_RO(bytes);
DEVICE_ATTR_RO(messages_prio);
DEVICE_ATTR_RO(errors_crc1);
DEVICE_ATTR_RO(errors_crc2);
DEVICE_ATTR_RO(errors_opcode1);

DEVICE_ATTR_RW(frequency);
DEVICE_ATTR_RW(hmode);
DEVICE_ATTR_RO(firmware_in_progress);

static struct attribute *unipi_spi2_attrs[] = {
    &dev_attr_messages.attr,
    &dev_attr_errors_tx.attr,
    &dev_attr_bytes.attr,
    &dev_attr_messages_prio.attr,
    &dev_attr_errors_crc1.attr,
    &dev_attr_errors_crc2.attr,
    &dev_attr_errors_opcode1.attr,
	NULL,
};
static const struct attribute_group unipi_spi2_group = {
	.name  = "statistics2",
	.attrs  = unipi_spi2_attrs,
};

static struct attribute *unipi_spi1_attrs[] = {
    &dev_attr_frequency.attr,
    &dev_attr_firmware_in_progress.attr,
    &dev_attr_hmode.attr,
	NULL,
};
static const struct attribute_group unipi_spi1_group = {
	.attrs  = unipi_spi1_attrs,
};

static const struct attribute_group *unipi_spi_groups[] = {
	&unipi_spi1_group,
	&unipi_spi2_group,
	NULL,
};


u16 unipi_spi_crc_set(u8* data, int len, u16 start)
{
	start = unipi_spi_crc(data, len, start);
	memcpy(data+len, &start, sizeof(start));
	return start;
}

int unipi_spi_set_v1(struct spi_device* spi_dev)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	u16 data = 0x0;

	if (n_spi->channel.op->write_bits_async(spi_dev, 1007, 1, (u8*)&data, NULL, NULL) == 0) {
		n_spi->hmode = 0;
		n_spi->channel.op = &spi_op_v1;
	}
	return 0;
}


static void try_v2_callback(void* cb_data, int result, u8* recv)
{
	struct spi_device* spi = (struct spi_device*) cb_data;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	struct spi_message *msg;
	struct unipi_spi_context* context;
	unsigned long flags;

	n_spi->enable_v2 = 1;
	if (result==2) {
		n_spi->hmode = 1;
		n_spi->channel.op = &spi_op_v2;
		spin_lock_irqsave(&n_spi->busy_lock, flags);
		while (!list_empty(&n_spi->queue)) {
			msg = list_first_entry(&n_spi->queue, struct spi_message, queue);
			list_del_init(&msg->queue);
			context = ((container_of((msg), struct unipi_spi_context, message)));
			if (context->operation_callback)
				context->operation_callback(context->operation_callback_data, -EAGAIN, context->data);
			kfree(context);
		}
		spin_unlock_irqrestore(&n_spi->busy_lock, flags);
	}
}

int unipi_spi_try_v2(struct spi_device* spi_dev)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	u16 data = 0x2;

	if (n_spi->enable_v2) {
		n_spi->enable_v2 = 0;
		/* send multicoil operation to ensure validity of coil */
		if (spi_op_v1.write_bits_async(spi_dev, UNIPI_MFD_COIL_PROTOCOL_MODE-1, 2,
		                         (u8*)&data, (void*)spi_dev, try_v2_callback) != 0) {
			n_spi->enable_v2 = 1;
		}
	}
	return 0;
}

void unipi_spi_check_busy_queue(struct unipi_spi_device* n_spi)
{
	struct spi_message *msg;
	struct unipi_spi_context* context;
	//unsigned long flags;
	int ret;

	//spin_lock_irqsave(&n_spi->busy_lock, flags);
	while (!list_empty(&n_spi->queue)) {
		unipi_spi_trace(n_spi->spi_dev, "Remove from busyqueue\n");
		/* Extract head of queue and try to insert into spi_async queue*/
		msg = list_first_entry(&n_spi->queue, struct spi_message, queue);
		list_del_init(&msg->queue);
		context = ((container_of((msg), struct unipi_spi_context, message)));
		ret = spi_async(n_spi->spi_dev, msg);
		if (ret != 0) {
			/* if error -> inform caller by callback and try next */
			if (context->operation_callback) 
				context->operation_callback(context->operation_callback_data, ret, context->data);
			kfree(context);
		} else {
			if (context->interframe_nsec > 0)
				return;
		}
	}
	n_spi->busy = 0;
	//spin_unlock_irqrestore(&n_spi->busy_lock, flags);
}

/* callback of inter-frame timer. Try to insert waiting messages from n_spi->queue */

enum hrtimer_restart unipi_spi_timer_func(struct hrtimer *timer)
{
	struct unipi_spi_device* n_spi = ((container_of((timer), struct unipi_spi_device, frame_timer)));
	unsigned long flags;

	spin_lock_irqsave(&n_spi->busy_lock, flags);
	unipi_spi_check_busy_queue(n_spi);
	spin_unlock_irqrestore(&n_spi->busy_lock, flags);
	return HRTIMER_NORESTART;
}

/* bottom part of all async op */
static void unipi_spi_op_complete(void *arg)
{
	struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
	struct spi_device* spi = context->message.spi;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	struct unipi_spi_devstatus devstatus;
	struct unipi_spi_reply reply;
	int len = context->len;

	// schedule timer to unblock busy flag
	if (context->interframe_nsec) {
		hrtimer_start_range_ns(&n_spi->frame_timer, context->interframe_nsec, 5000, HRTIMER_MODE_REL);
	}
	//unipi_spi_trace_1(spi, "Read (op1) st=%d %16ph\n", context->message.status, context->rx_header);
	reply.ret = -EIO;
	reply.data = context->data;

	if (context->message.status != 0) {
		// sending unsuccessfull - reason unknown
		n_spi->stat.errors_tx++;
		unipi_spi_trace(spi, "Unsuccessful spi transaction: opcode:%d\n", context->tx_header[0]);

	} else if (context->parse_frame(context, &devstatus, &reply) != 0) {
		n_spi->stat.errors_crc1++;
		if (!n_spi->probe_mode)
			unipi_spi_error(context->message.spi, "Incorrect CRC: opcode:%02x %8ph\n", context->tx_header[0], context->rx_header);
		if (reply.ret == -ENOEXEC) {
			// return to protocol version 1
			n_spi->hmode = 0;
			n_spi->channel.op = &spi_op_v1;
		}
	} else {
		// process valid rx_header with out-of-order data
		n_spi->stat.bytes += len + UNIPI_SPI_MESSAGE_HEADER_LENGTH;
		//unipi_spi_trace(spi, "devstatus: opcode:%02x\n", devstatus.opcode);
		switch (devstatus.opcode) {
			case UNIPI_SPI_OP_WRITECHAR:
				n_spi->stat.messages_prio++;
				// Signal the UART about received char
				unipi_mfd_rx_char(&n_spi->channel, devstatus.port, devstatus.ch, devstatus.remain);
				fallthrough;
			case UNIPI_SPI_OP2_IDLE:
			case UNIPI_SPI_OP_IDLE:
				if (devstatus.interrupt != 0)
					unipi_mfd_int_status(&n_spi->channel, devstatus.interrupt);
				break;
			default:
				reply.ret = -EIO;
				n_spi->stat.errors_opcode1++;
				break;
		}
		if (context->operation_callback) {
			if (reply.ret > 0) {
				if ((context->simple_read) && (context->data))
					memmove(context->data, reply.data, context->simple_len);
//			} else if (context->tx_header[0] == UNIPI_SPI_OP_READSTR) {
//				/* combine len + remain */
//				ret = ret | (context->rx_header2[3] << 8);
//			}
//			} else {
//				ret = -EIO;
			}
		}
	}
	if (context->operation_callback)
		context->operation_callback(context->operation_callback_data, reply.ret, reply.data);
	kfree(context);
}

int unipi_spi_exec_context(struct spi_device* spi_dev, struct unipi_spi_context *context,
                           void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	unsigned long flags, bflags;
	int ret;

	context->operation_callback_data = cb_data;
	context->operation_callback = cb_function;
	context->message.complete = unipi_spi_op_complete;

	spin_lock_irqsave(&n_spi->firmware_lock, flags);
	if (n_spi->firmware_in_progress) {
		spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
		kfree(context);
		return -EACCES;
	}
	n_spi->stat.messages++;
	spin_lock_irqsave(&n_spi->busy_lock, bflags);
	if (n_spi->busy) {
		unipi_spi_trace(spi_dev, "Add to busyqueue txopcode:%x\n", context->tx_header[0]);
		list_add_tail(&context->message.queue, &n_spi->queue);
		spin_unlock_irqrestore(&n_spi->busy_lock, bflags);
		spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
		return 0;
	}
	if (context->interframe_nsec)
		n_spi->busy = 1;
	spin_unlock_irqrestore(&n_spi->busy_lock, bflags);

	ret = spi_async(spi_dev, &context->message);
	spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
	if (ret != 0) {
		n_spi->stat.errors_tx++;
		unipi_spi_trace(spi_dev, "Err=3 txopcode:%d\n", context->tx_header[0]);
		kfree(context);
	}
	return ret;
}


/*  Async op for READREG */
/*
int unipi_spi_read_regs_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	if (n_spi->hmode) {
		return unipi_spi2_read_simple(spi_dev, UNIPI_SPI_OP_READREG, reg, count, data, cb_data, cb_function);
	} else {
		return unipi_spi1_read_regs_async(spi_dev, reg, count, data, cb_data, cb_function);
	}
}
*/

void unipi_spi_populated(void * self)
{
	struct spi_device *spi_dev = (struct spi_device*) self;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	if (spi_dev->max_speed_hz) n_spi->frequency = spi_dev->max_speed_hz;
}


int unipi_spi_probe(struct spi_device *spi)
{
	struct unipi_spi_device *n_spi;
//	u16  first_probe[8];
	int alive = 0;
	int ret, i;   //, degraded=0; //, no_irq = 0;
	u32 probe_always_succeeds = 0;

	//unsigned long flags;

	n_spi = kzalloc(sizeof *n_spi, GFP_ATOMIC);
	if (!n_spi)
		return -ENOMEM;

	unipi_spi_trace(spi, "channel device probe start\n");

	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	spi->mode		    = spi->mode ? spi->mode : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? spi->max_speed_hz : 12000000;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,9,0)
	spi->rt = 1;
    spi->cs_inactive.value = 1;
    spi->cs_inactive.unit  = SPI_DELAY_UNIT_USECS;
    spi->cs_setup.value = 10;
    spi->cs_setup.unit  = SPI_DELAY_UNIT_NSECS;
    spi->cs_hold.value = 10;
    spi->cs_hold.unit  = SPI_DELAY_UNIT_NSECS;
#endif
	ret = spi_setup(spi);
	if (ret) {
        kfree(n_spi);
		return ret;
    }

	//n_spi->spi = spi; // back link used by hrtimer
	spin_lock_init(&n_spi->firmware_lock);
	n_spi->firmware_in_progress = 0;
	n_spi->frequency = UNIPI_SPI_PROBE_FREQ;
	n_spi->probe_mode = 1;

	INIT_LIST_HEAD(&n_spi->queue);
	spin_lock_init(&n_spi->busy_lock);
	n_spi->busy = 0;

	hrtimer_init(&n_spi->frame_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	n_spi->frame_timer.function = unipi_spi_timer_func;
	n_spi->spi_dev = spi; // required by timer

	unipi_spi_trace(spi, "Max Hz controller=%d device=%d\n", spi->master->max_speed_hz, spi->max_speed_hz);
	if (spi->dev.of_node) {
		of_property_read_u32(spi->dev.of_node, "probe-always-succeeds", &(probe_always_succeeds));
	}

	n_spi->channel.proto_self = spi;
	n_spi->channel.dev = &spi->dev;
	n_spi->channel.op = &spi_op_v1;

	spi_set_drvdata(spi, n_spi);
	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point
	//unipi_ping_sync(&n_spi->channel);
	for (i=0; i< 5; i++) {
		if (unipi_ping_sync(&n_spi->channel) == 0) {
			alive++;
			if (alive > 1)
				break;
		}
		/*
		int x= unipi_read_regs_sync(&n_spi->channel, UNIPI_MFD_REG_FW_VERSION, 5, first_probe);
		if (x==5)
			break;
		first_probe[0] = 0;
		*/
	}
	if (alive > 1) {
		dev_info(&spi->dev, "spi channel is alive\n");
	} else if (probe_always_succeeds) {
		// dummy channel
		dev_info(&spi->dev, "spi channel is forced up\n");
	} else {
		hrtimer_cancel(&n_spi->frame_timer);
		ret = -ENODEV;
		spi_set_drvdata(spi, NULL);
		kfree(n_spi);
		dev_err(&spi->dev, "spi channel looks empty\n");
		return ret;
	}
/*
	if (first_probe[0] == 0x0600) {
		degraded = 1;
		dev_warn(&spi->dev, "spi device is in degraded (bootloader) mode. Load operational firmware!\n");
	} else if (first_probe[0] != 0) {
		dev_info(&spi->dev, "spi channel is alive\n");
	} else if (probe_always_succeeds) {
		// dummy channel
		dev_info(&spi->dev, "spi channel is forced up\n");
	} else {
//   usleep_range(1000, 1100);
		hrtimer_cancel(&n_spi->frame_timer);
		ret = -ENODEV;
		spi_set_drvdata(spi, NULL);
		kfree(n_spi);
		dev_err(&spi->dev, "spi channel looks empty\n");
		return ret;
	}
*/
	n_spi->frequency = UNIPI_SPI_SLOWER_FREQ;
	if (spi->max_speed_hz && (spi->max_speed_hz < UNIPI_SPI_SLOWER_FREQ)) 
		n_spi->frequency = spi->max_speed_hz;

	n_spi->probe_mode = 0;
	n_spi->enable_v2 = 1;
	unipi_ping_sync(&n_spi->channel);

	n_spi->channel.op->populated = unipi_spi_populated;

	ret = unipi_channel_init(&n_spi->channel, &spi->dev);
	return ret;
}

int unipi_spi_remove(struct spi_device *spi)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);

	if (! n_spi) return 0;

	unipi_channel_exit(&n_spi->channel);
	kfree(n_spi);
	dev_info(&spi->dev, "spi channel removed\n");
	return 0;
}


/*********************
 * Final definitions *
 *********************/
static const struct spi_device_id unipi_spi_ids[] = {
	{ .name = "unipi-spi" },
	{ .name = "neuron" },
	{ .name = "axon" },
	{ .name = "unipispi" },
	{},
};
MODULE_DEVICE_TABLE(spi, unipi_spi_ids);

static const struct of_device_id unipi_spi_id_match[] = {
		{.compatible = "unipi,unipi-spi"},
		{.compatible = "unipi,neuron"},
		{.compatible = "unipi,axon"},
		{.compatible = "unipispi"},
		{}
};

MODULE_DEVICE_TABLE(of, unipi_spi_id_match);

struct spi_driver unipi_spi_driver =
{
	.driver =
	{
		.name			= "unipi-spi",
		.of_match_table	= of_match_ptr(unipi_spi_id_match),
		.dev_groups = unipi_spi_groups,
	},
	.probe				= unipi_spi_probe,
	.remove				= unipi_spi_remove,
	.id_table			= unipi_spi_ids,
};

MODULE_ALIAS("spi:unipispi");
MODULE_ALIAS("spi:unipi-spi");

struct unipi_channel* unipi_spi_get_channel(struct spi_device* spi_dev)
{
	struct unipi_spi_device *n_spi;

	/* test if device is driven by this driver */
	if ((spi_dev==NULL) || (spi_dev->dev.driver==NULL) || (strcmp(unipi_spi_driver.driver.name, spi_dev->dev.driver->name) !=0 ))
		return NULL;

	n_spi = spi_get_drvdata(spi_dev);
	if (!n_spi) 
		return NULL;
	return &n_spi->channel;
}
EXPORT_SYMBOL_GPL(unipi_spi_get_channel);

static int __init unipi_spi_init(void)
{
	int ret = 0;

//	ret = unipi_modbus_init();
//	if (ret) return (ret);

	//spin_lock_init(&unipi_spi_master_lock);

	ret = spi_register_driver(&unipi_spi_driver);

	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to register spi driver --> %d\n", unipi_spi_driver.driver.name, ret);
		return ret;
	}
	printk(KERN_INFO "%s: Driver registered. Major Version: %s\n", unipi_spi_driver.driver.name, UNIPI_MODULE_MAJOR_VERSIONSTRING);
	return ret;
}

static void __exit unipi_spi_exit(void)
{
	spi_unregister_driver(&unipi_spi_driver);
//	unipi_modbus_exit();
}

module_init(unipi_spi_init);
module_exit(unipi_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Spi Channel Driver");
