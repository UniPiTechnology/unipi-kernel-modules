/*
 * rtc-unipi.c - RTC driver for mcp794xx based on rtc-ds1307.c
 *               with calibration support
 *
 *  Copyright (C) 2005 James Chapman (ds1337 core)
 *  Copyright (C) 2006 David Brownell
 *  Copyright (C) 2009 Matthias Fuchs (rx8025 support)
 *  Copyright (C) 2012 Bertrand Achard (nvram access fixes)
 *  Copyright (C) 2019 Miroslav Ondra 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc/ds1307.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/version.h>


/* RTC registers don't differ much, except for the century flag */
#define DS1307_REG_SECS		0x00	/* 00-59 */
#	define DS1307_BIT_CH		0x80
#	define DS1340_BIT_nEOSC		0x80
#	define MCP794XX_BIT_ST		0x80
#define DS1307_REG_MIN		0x01	/* 00-59 */
#	define M41T0_BIT_OF		0x80
#define DS1307_REG_HOUR		0x02	/* 00-23, or 1-12{am,pm} */
#	define DS1307_BIT_12HR		0x40	/* in REG_HOUR */
#	define DS1307_BIT_PM		0x20	/* in REG_HOUR */
#	define DS1340_BIT_CENTURY_EN	0x80	/* in REG_HOUR */
#	define DS1340_BIT_CENTURY	0x40	/* in REG_HOUR */
#define DS1307_REG_WDAY		0x03	/* 01-07 */
#	define MCP794XX_BIT_VBATEN	0x08
#define DS1307_REG_MDAY		0x04	/* 01-31 */
#define DS1307_REG_MONTH	0x05	/* 01-12 */
#	define DS1337_BIT_CENTURY	0x80	/* in REG_MONTH */
#define DS1307_REG_YEAR		0x06	/* 00-99 */

/*
 * Other registers (control, status, alarms, trickle charge, NVRAM, etc)
 * start at 7, and they differ a LOT. Only control and status matter for
 * basic RTC date and time functionality; be careful using them.
 */
#define DS1307_REG_CONTROL	0x07		/* or ds1338 */
#	define DS1307_BIT_OUT		0x80
#	define DS1338_BIT_OSF		0x20
#	define DS1307_BIT_SQWE		0x10
#	define DS1307_BIT_RS1		0x02
#	define DS1307_BIT_RS0		0x01


#define MCP794XX_NVRAM_OFFSET	0x20
#define MCP794XX_NVRAM_SIZE		0x40


struct rtc_unipi {
	struct nvmem_config	nvmem_cfg;
	/*enum ds_type		type;*/
	unsigned long		flags;
#define HAS_NVRAM	0		/* bit 0 == sysfs file active */
#define HAS_ALARM	1		/* bit 1 == irq claimed */
	struct device		*dev;
	struct regmap		*regmap;
	const char			*name;
	struct rtc_device	*rtc;
/*
#ifdef CONFIG_COMMON_CLK
	struct clk_hw		clks[2];
#endif
*/
};

/*struct chip_desc { */
	/*unsigned		alarm:1;*/
	/*u16			nvram_offset;
	u16			nvram_size;*/
	/*u8			offset; *//* register's offset */
	/*u8			century_reg;
	u8			century_enable_bit;
	u8			century_bit;*/
	/*u8			bbsqi_bit; */
	/*irq_handler_t		irq_handler;*/
	/*const struct rtc_class_ops *rtc_ops;*/
/*};*/

static int mcp794xx_get_time(struct device *dev, struct rtc_time *t);
static int mcp794xx_set_time(struct device *dev, struct rtc_time *t);
static irqreturn_t mcp794xx_irq(int irq, void *dev_id);
static int mcp794xx_read_alarm(struct device *dev, struct rtc_wkalrm *t);
static int mcp794xx_set_alarm(struct device *dev, struct rtc_wkalrm *t);
static int mcp794xx_alarm_irq_enable(struct device *dev, unsigned int enabled);
static int mcp794xx_read_offset(struct device *dev, long *offset);
static int mcp794xx_set_offset(struct device *dev, long offset);


static const struct rtc_class_ops mcp794xx_rtc_ops = {
	.read_time      = mcp794xx_get_time,
	.set_time       = mcp794xx_set_time,
	.read_alarm     = mcp794xx_read_alarm,
	.set_alarm      = mcp794xx_set_alarm,
	.alarm_irq_enable = mcp794xx_alarm_irq_enable,
	.read_offset    = mcp794xx_read_offset,
	.set_offset     = mcp794xx_set_offset,
};

/*static const struct chip_desc mcpchip = {*/
		/*.alarm		= 1,*/
		/* this is battery backed SRAM */
		/* .nvram_offset	= 0x20,
		.nvram_size	= 0x40, */
		/*.irq_handler = mcp794xx_irq,*/
		/*.rtc_ops = &mcp794xx_rtc_ops,*/
/*};*/

static const struct i2c_device_id rtc_unipi_id[] = {
	{ "unipi-mcp7941x", 0 /*mcp794xx */},
	{ "rtc-unipi", 0 /*mcp794xx */},
	{ }
};
MODULE_DEVICE_TABLE(i2c, rtc_unipi_id);

#ifdef CONFIG_OF
static const struct of_device_id rtc_unipi_of_match[] = {
	{
		.compatible = "unipi,unipi-mcp7941x",
		.data = NULL /*(void *)mcp794xx*/
	},
	{
		.compatible = "unipi,rtc-unipi",
		.data = NULL /*(void *)mcp794xx*/
	},
	{ }
};
MODULE_DEVICE_TABLE(of, rtc_unipi_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id rtc_unipi_acpi_ids[] = {
	{ .id = "MCP7940X", .driver_data = 0 /*mcp794xx */},
	{ .id = "MCP7941X", .driver_data = 0 /*mcp794xx */},
	{ }
};
MODULE_DEVICE_TABLE(acpi, rtc_unipi_acpi_ids);
#endif


/*----------------------------------------------------------------------*/

static int mcp794xx_get_time(struct device *dev, struct rtc_time *t)
{
	struct rtc_unipi	*rtc_unipi = dev_get_drvdata(dev);
	int		tmp, ret;
	/*const struct chip_desc *chip = &chips[ds1307->type];*/
	u8 regs[7];

	/* read the RTC date and time registers all at once */
	ret = regmap_bulk_read(rtc_unipi->regmap, 0/*chip->offset*/, regs,
			       sizeof(regs));
	if (ret) {
		dev_err(dev, "%s error %d\n", "read", ret);
		return ret;
	}

	dev_dbg(dev, "%s: %7ph\n", "read", regs);

	t->tm_sec = bcd2bin(regs[DS1307_REG_SECS] & 0x7f);
	t->tm_min = bcd2bin(regs[DS1307_REG_MIN] & 0x7f);
	tmp = regs[DS1307_REG_HOUR] & 0x3f;
	t->tm_hour = bcd2bin(tmp);
	t->tm_wday = bcd2bin(regs[DS1307_REG_WDAY] & 0x07) - 1;
	t->tm_mday = bcd2bin(regs[DS1307_REG_MDAY] & 0x3f);
	tmp = regs[DS1307_REG_MONTH] & 0x1f;
	t->tm_mon = bcd2bin(tmp) - 1;
	t->tm_year = bcd2bin(regs[DS1307_REG_YEAR]) + 100;
	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"read", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	/* initial clock setting can be undefined */
	return rtc_valid_tm(t);
}

static int mcp794xx_set_time(struct device *dev, struct rtc_time *t)
{
	struct rtc_unipi	*rtc_unipi = dev_get_drvdata(dev);
	/*const struct chip_desc *chip = &chips[ds1307->type];*/
	int		result;
	int		tmp;
	u8		regs[7];

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"write", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	if (t->tm_year < 100)
		return -EINVAL;
//#ifdef CONFIG_RTC_DRV_DS1307_CENTURY
//	if (t->tm_year > (/*chip->century_bit ? 299 : */199))
//		return -EINVAL;
//#else
	if (t->tm_year > 199)
		return -EINVAL;
//#endif

	regs[DS1307_REG_SECS] = bin2bcd(t->tm_sec);
	regs[DS1307_REG_MIN] = bin2bcd(t->tm_min);
	regs[DS1307_REG_HOUR] = bin2bcd(t->tm_hour);
	regs[DS1307_REG_WDAY] = bin2bcd(t->tm_wday + 1);
	regs[DS1307_REG_MDAY] = bin2bcd(t->tm_mday);
	regs[DS1307_REG_MONTH] = bin2bcd(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	regs[DS1307_REG_YEAR] = bin2bcd(tmp);


	//if (ds1307->type == mcp794xx) {
		/*
		 * these bits were cleared when preparing the date/time
		 * values and need to be set again before writing the
		 * regsfer out to the device.
		 */
		regs[DS1307_REG_SECS] |= MCP794XX_BIT_ST;
		regs[DS1307_REG_WDAY] |= MCP794XX_BIT_VBATEN;
	//}

	dev_dbg(dev, "%s: %7ph\n", "write", regs);

	result = regmap_bulk_write(rtc_unipi->regmap, 0 /*chip->offset*/, regs,
				   sizeof(regs));
	if (result) {
		dev_err(dev, "%s error %d\n", "write", result);
		return result;
	}
	return 0;
}




/*----------------------------------------------------------------------*/

/*
 * Alarm support for mcp794xx devices.
 */

#define MCP794XX_REG_WEEKDAY		0x3
#define MCP794XX_REG_WEEKDAY_WDAY_MASK	0x7
#define MCP794XX_REG_CONTROL		0x07
#	define MCP794XX_BIT_ALM0_EN	0x10
#	define MCP794XX_BIT_ALM1_EN	0x20
#   define MCP794XX_BIT_EXTOSC  0x08
#define MCP794XX_REG_ALARM0_BASE	0x0a
#define MCP794XX_REG_ALARM0_CTRL	0x0d
#define MCP794XX_REG_ALARM1_BASE	0x11
#define MCP794XX_REG_ALARM1_CTRL	0x14
#	define MCP794XX_BIT_ALMX_IF	BIT(3)
#	define MCP794XX_BIT_ALMX_C0	BIT(4)
#	define MCP794XX_BIT_ALMX_C1	BIT(5)
#	define MCP794XX_BIT_ALMX_C2	BIT(6)
#	define MCP794XX_BIT_ALMX_POL	BIT(7)
#	define MCP794XX_MSK_ALMX_MATCH	(MCP794XX_BIT_ALMX_C0 | \
					 MCP794XX_BIT_ALMX_C1 | \
					 MCP794XX_BIT_ALMX_C2)

#define MCP794XX_REG_CALIBRATION	0x08

static irqreturn_t mcp794xx_irq(int irq, void *dev_id)
{
	struct rtc_unipi           *rtc_unipi = dev_id;
	struct mutex            *lock = &rtc_unipi->rtc->ops_lock;
	int reg, ret;

	mutex_lock(lock);

	/* Check and clear alarm 0 interrupt flag. */
	ret = regmap_read(rtc_unipi->regmap, MCP794XX_REG_ALARM0_CTRL, &reg);
	if (ret)
		goto out;
	if (!(reg & MCP794XX_BIT_ALMX_IF))
		goto out;
	reg &= ~MCP794XX_BIT_ALMX_IF;
	ret = regmap_write(rtc_unipi->regmap, MCP794XX_REG_ALARM0_CTRL, reg);
	if (ret)
		goto out;

	/* Disable alarm 0. */
	ret = regmap_update_bits(rtc_unipi->regmap, MCP794XX_REG_CONTROL,
				 MCP794XX_BIT_ALM0_EN, 0);
	if (ret)
		goto out;

	rtc_update_irq(rtc_unipi->rtc, 1, RTC_AF | RTC_IRQF);

out:
	mutex_unlock(lock);

	return IRQ_HANDLED;
}

static int mcp794xx_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rtc_unipi *rtc_unipi = dev_get_drvdata(dev);
	u8 regs[10];
	int ret;

	if (!test_bit(HAS_ALARM, &rtc_unipi->flags))
		return -EINVAL;

	/* Read control and alarm 0 registers. */
	ret = regmap_bulk_read(rtc_unipi->regmap, MCP794XX_REG_CONTROL, regs,
			       sizeof(regs));
	if (ret)
		return ret;

	t->enabled = !!(regs[0] & MCP794XX_BIT_ALM0_EN);

	/* Report alarm 0 time assuming 24-hour and day-of-month modes. */
	t->time.tm_sec = bcd2bin(regs[3] & 0x7f);
	t->time.tm_min = bcd2bin(regs[4] & 0x7f);
	t->time.tm_hour = bcd2bin(regs[5] & 0x3f);
	t->time.tm_wday = bcd2bin(regs[6] & 0x7) - 1;
	t->time.tm_mday = bcd2bin(regs[7] & 0x3f);
	t->time.tm_mon = bcd2bin(regs[8] & 0x1f) - 1;
	t->time.tm_year = -1;
	t->time.tm_yday = -1;
	t->time.tm_isdst = -1;

	dev_dbg(dev, "%s, sec=%d min=%d hour=%d wday=%d mday=%d mon=%d "
		"enabled=%d polarity=%d irq=%d match=%lu\n", __func__,
		t->time.tm_sec, t->time.tm_min, t->time.tm_hour,
		t->time.tm_wday, t->time.tm_mday, t->time.tm_mon, t->enabled,
		!!(regs[6] & MCP794XX_BIT_ALMX_POL),
		!!(regs[6] & MCP794XX_BIT_ALMX_IF),
		(regs[6] & MCP794XX_MSK_ALMX_MATCH) >> 4);

	return 0;
}

static int mcp794xx_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rtc_unipi *rtc_unipi = dev_get_drvdata(dev);
	unsigned char regs[10];
	int ret;

	if (!test_bit(HAS_ALARM, &rtc_unipi->flags))
		return -EINVAL;

	dev_dbg(dev, "%s, sec=%d min=%d hour=%d wday=%d mday=%d mon=%d "
		"enabled=%d pending=%d\n", __func__,
		t->time.tm_sec, t->time.tm_min, t->time.tm_hour,
		t->time.tm_wday, t->time.tm_mday, t->time.tm_mon,
		t->enabled, t->pending);

	/* Read control and alarm 0 registers. */
	ret = regmap_bulk_read(rtc_unipi->regmap, MCP794XX_REG_CONTROL, regs,
			       sizeof(regs));
	if (ret)
		return ret;

	/* Set alarm 0, using 24-hour and day-of-month modes. */
	regs[3] = bin2bcd(t->time.tm_sec);
	regs[4] = bin2bcd(t->time.tm_min);
	regs[5] = bin2bcd(t->time.tm_hour);
	regs[6] = bin2bcd(t->time.tm_wday + 1);
	regs[7] = bin2bcd(t->time.tm_mday);
	regs[8] = bin2bcd(t->time.tm_mon + 1);

	/* Clear the alarm 0 interrupt flag. */
	regs[6] &= ~MCP794XX_BIT_ALMX_IF;
	/* Set alarm match: second, minute, hour, day, date, month. */
	regs[6] |= MCP794XX_MSK_ALMX_MATCH;
	/* Disable interrupt. We will not enable until completely programmed */
	regs[0] &= ~MCP794XX_BIT_ALM0_EN;

	ret = regmap_bulk_write(rtc_unipi->regmap, MCP794XX_REG_CONTROL, regs,
				sizeof(regs));
	if (ret)
		return ret;

	if (!t->enabled)
		return 0;
	regs[0] |= MCP794XX_BIT_ALM0_EN;
	return regmap_write(rtc_unipi->regmap, MCP794XX_REG_CONTROL, regs[0]);
}

static int mcp794xx_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct rtc_unipi *rtc_unipi = dev_get_drvdata(dev);

	if (!test_bit(HAS_ALARM, &rtc_unipi->flags))
		return -EINVAL;

	return regmap_update_bits(rtc_unipi->regmap, MCP794XX_REG_CONTROL,
				  MCP794XX_BIT_ALM0_EN,
				  enabled ? MCP794XX_BIT_ALM0_EN : 0);
}

static int mcp794xx_read_offset(struct device *dev, long *offset)
{
	struct rtc_unipi *rtc_unipi = dev_get_drvdata(dev);
	int ret;
	int reg;

	ret = regmap_read(rtc_unipi->regmap, MCP794XX_REG_CALIBRATION, &reg);
	if (ret < 0)
		return ret;

	if (reg & 0x80) { 
		*offset = ((long) -(reg & 0x7f)); 
	} else {
		*offset = ((long) ((s8)(reg & 0x7f)));
	}

	return 0;
}

/*

 */
static int mcp794xx_set_offset(struct device *dev, long offset)
{
	struct rtc_unipi *rtc_unipi = dev_get_drvdata(dev);
	s8 reg;

	if (offset > 127)
		reg = 127;
	else if (offset < -127)
		reg = 127 | 0x80;
	else if (offset < 0)
		reg = ((s8)(-offset)) | 0x80;
	else
		reg = (s8)(offset);

	return regmap_write(rtc_unipi->regmap, MCP794XX_REG_CALIBRATION, reg);
}

/*----------------------------------------------------------------------*/

static int rtc_unipi_nvram_read(void *priv, unsigned int offset, void *val,
			     size_t bytes)
{
	struct rtc_unipi *rtc_unipi = priv;
	/*const struct chip_desc *chip = &chips[ds1307->type];*/

	return regmap_bulk_read(rtc_unipi->regmap, MCP794XX_NVRAM_OFFSET + offset,
				val, bytes);
}

static int rtc_unipi_nvram_write(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct rtc_unipi *rtc_unipi = priv;
	/*const struct chip_desc *chip = &chips[ds1307->type];*/

	return regmap_bulk_write(rtc_unipi->regmap, MCP794XX_NVRAM_OFFSET + offset,
				 val, bytes);
}

/*----------------------------------------------------------------------*/



static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int rtc_unipi_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct rtc_unipi		*rtc_unipi;
	int			err = -ENODEV;
	int			tmp, wday;
	/*const struct chip_desc	*chip;*/
	bool			want_irq;
	bool			rtc_unipi_can_wakeup_device = false;
	unsigned char		regs[8];
	/*struct rtc_unipi_platform_data *pdata = dev_get_platdata(&client->dev);*/
	struct rtc_time		tm;
	unsigned long		timestamp;

	rtc_unipi = devm_kzalloc(&client->dev, sizeof(struct rtc_unipi), GFP_KERNEL);
	if (!rtc_unipi)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, rtc_unipi);
	rtc_unipi->dev = &client->dev;
	rtc_unipi->name = client->name;

	rtc_unipi->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(rtc_unipi->regmap)) {
		dev_err(rtc_unipi->dev, "regmap allocation failed\n");
		return PTR_ERR(rtc_unipi->regmap);
	}

	i2c_set_clientdata(client, rtc_unipi);

	/*
	if (client->dev.of_node) {
		ds1307->type = (enum ds_type)
			of_device_get_match_data(&client->dev);
		//chip = &chips[ds1307->type];
	} else if (id) {
		//chip = &chips[id->driver_data];
		ds1307->type = id->driver_data;
	} else {
		const struct acpi_device_id *acpi_id;

		acpi_id = acpi_match_device(ACPI_PTR(ds1307_acpi_ids),
					    ds1307->dev);
		if (!acpi_id)
			return -ENODEV;
		//chip = &chips[acpi_id->driver_data];
		ds1307->type = acpi_id->driver_data;
	}
	*/
	want_irq = client->irq > 0/* && chip->alarm*/;

#ifdef CONFIG_OF
/*
 * For devices with no IRQ directly connected to the SoC, the RTC chip
 * can be forced as a wakeup source by stating that explicitly in
 * the device's .dts file using the "wakeup-source" boolean property.
 * If the "wakeup-source" property is set, don't request an IRQ.
 * This will guarantee the 'wakealarm' sysfs entry is available on the device,
 * if supported by the RTC.
 */
	if (/*chip->alarm && */ of_property_read_bool(client->dev.of_node, "wakeup-source"))
		rtc_unipi_can_wakeup_device = true;
#endif


read_rtc:
	/* read RTC registers */
	err = regmap_bulk_read(rtc_unipi->regmap, 0 /*chip->offset*/, regs,
			       sizeof(regs));
	if (err) {
		dev_dbg(rtc_unipi->dev, "read error %d\n", err);
		goto exit;
	}

	/*
	 * minimal sanity checking; some chips (like DS1340) don't
	 * specify the extra bits as must-be-zero, but there are
	 * still a few values that are clearly out-of-range.
	 */
	tmp = regs[DS1307_REG_SECS];

		/* make sure that the backup battery is enabled */
		if (!(regs[DS1307_REG_WDAY] & MCP794XX_BIT_VBATEN)) {
			regmap_write(rtc_unipi->regmap, DS1307_REG_WDAY,
				     regs[DS1307_REG_WDAY] |
				     MCP794XX_BIT_VBATEN);
		}

		/* clock halted?  turn it on, so clock can tick. */
		if (!(tmp & MCP794XX_BIT_ST)) {
			regmap_write(rtc_unipi->regmap, DS1307_REG_SECS,
				     MCP794XX_BIT_ST);
			dev_warn(rtc_unipi->dev, "SET TIME!\n");
			goto read_rtc;
		}
		if (regs[MCP794XX_REG_CONTROL] & MCP794XX_BIT_EXTOSC) {
			regmap_write(rtc_unipi->regmap, MCP794XX_REG_CONTROL,
				    regs[MCP794XX_REG_CONTROL] & ~MCP794XX_BIT_EXTOSC);
			dev_warn(rtc_unipi->dev, "BAD OSCILLATOR SETTING!\n");
		}


	tmp = regs[DS1307_REG_HOUR];
	if ((tmp & DS1307_BIT_12HR)) {
		/*
		 * Be sure we're in 24 hour mode.  Multi-master systems
		 * take note...
		 */
		tmp = bcd2bin(tmp & 0x1f);
		if (tmp == 12)
			tmp = 0;
		if (regs[DS1307_REG_HOUR] & DS1307_BIT_PM)
			tmp += 12;
		regmap_write(rtc_unipi->regmap, /*chip->offset + */DS1307_REG_HOUR,
			     bin2bcd(tmp));
	}
	/*
	 * Some IPs have weekday reset value = 0x1 which might not correct
	 * hence compute the wday using the current date/month/year values
	 */
	mcp794xx_get_time(rtc_unipi->dev, &tm);
	wday = tm.tm_wday;
	timestamp = rtc_tm_to_time64(&tm);
	rtc_time64_to_tm(timestamp, &tm);

	/*
	 * Check if reset wday is different from the computed wday
	 * If different then set the wday which we computed using
	 * timestamp
	 */
	if (wday != tm.tm_wday)
		regmap_update_bits(rtc_unipi->regmap, MCP794XX_REG_WEEKDAY,
				   MCP794XX_REG_WEEKDAY_WDAY_MASK,
				   tm.tm_wday + 1);

	if (want_irq || rtc_unipi_can_wakeup_device) {
		device_set_wakeup_capable(rtc_unipi->dev, true);
		set_bit(HAS_ALARM, &rtc_unipi->flags);
	}

	rtc_unipi->rtc = devm_rtc_allocate_device(rtc_unipi->dev);
	if (IS_ERR(rtc_unipi->rtc))
		return PTR_ERR(rtc_unipi->rtc);

	if (rtc_unipi_can_wakeup_device && !want_irq) {
		dev_info(rtc_unipi->dev,
			 "'wakeup-source' is set, request for an IRQ is disabled!\n");
		/* We cannot support UIE mode if we do not have an IRQ line */
		rtc_unipi->rtc->uie_unsupported = 1;
	}

	if (want_irq) {
		err = devm_request_threaded_irq(rtc_unipi->dev, client->irq, NULL,
						mcp794xx_irq,
						IRQF_SHARED | IRQF_ONESHOT,
						rtc_unipi->name, rtc_unipi);
		if (err) {
			client->irq = 0;
			device_set_wakeup_capable(rtc_unipi->dev, false);
			clear_bit(HAS_ALARM, &rtc_unipi->flags);
			dev_err(rtc_unipi->dev, "unable to request IRQ!\n");
		} else {
			dev_dbg(rtc_unipi->dev, "got IRQ %d\n", client->irq);
		}
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,17,0)
	/*if (chip->nvram_size) {*/
		rtc_unipi->nvmem_cfg.name = "rtc_unipi_nvram";
		rtc_unipi->nvmem_cfg.word_size = 1;
		rtc_unipi->nvmem_cfg.stride = 1;
		rtc_unipi->nvmem_cfg.size = MCP794XX_NVRAM_SIZE;
		rtc_unipi->nvmem_cfg.reg_read = rtc_unipi_nvram_read;
		rtc_unipi->nvmem_cfg.reg_write = rtc_unipi_nvram_write;
		rtc_unipi->nvmem_cfg.priv = rtc_unipi;

		rtc_unipi->rtc->nvmem_config = &rtc_unipi->nvmem_cfg;
		rtc_unipi->rtc->nvram_old_abi = true;
	/*}*/
#else
	    struct nvmem_config nvmem_cfg = {
		.name = "rtc_unipi_nvram",
		.word_size = 1,
		.stride = 1,
		.size = MCP794XX_NVRAM_SIZE,
		.reg_read = rtc_unipi_nvram_read,
		.reg_write = rtc_unipi_nvram_write,
		.priv = rtc_unipi,
	    };

	    rtc_unipi->rtc->nvram_old_abi = true;
	    rtc_nvmem_register(ds1307->rtc, &nvmem_cfg);

#endif
	rtc_unipi->rtc->ops = &mcp794xx_rtc_ops; /*chip->rtc_ops ?: &ds13xx_rtc_ops;*/
	err = rtc_register_device(rtc_unipi->rtc);
	if (err)
		return err;

	return 0;

exit:
	return err;
}

static struct i2c_driver rtc_unipi_driver = {
	.driver = {
		.name	= "rtc-unipi",
		.of_match_table = of_match_ptr(rtc_unipi_of_match),
		.acpi_match_table = ACPI_PTR(rtc_unipi_acpi_ids),
	},
	.probe		= rtc_unipi_probe,
	.id_table	= rtc_unipi_id,
};

module_i2c_driver(rtc_unipi_driver);

MODULE_DESCRIPTION("RTC driver for DS1307 and similar chips");
MODULE_LICENSE("GPL");
