#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/nvmem-consumer.h>
//#include <linux/device/bus.h>

#include "unipi_iogroup_bus.h"
#include "unipi_channel.h"

static void iogroupdev_release(struct device *dev)
{
	struct unipi_iogroup_device	*iogroup = to_unipi_iogroup_device(dev);

	/* iogroup channel may cleanup for released devices */
	//if (iogroup->channel->cleanup)
	//	iogroup->channel->cleanup(iogroup);

	unipi_channel_put(iogroup->channel);
	kfree(iogroup);
}

static int iogroup_match_device(struct device *dev, struct device_driver *drv)
{
	const struct unipi_iogroup_device	*iogroup = to_unipi_iogroup_device(dev);

	/* Attempt an OF style match */
	if (of_driver_match_device(dev, drv))
		return 1;

	return strcmp(iogroup->modalias, drv->name) == 0;
}
/* MODALIAS= must be add to environment of udev message. Action rule based on MODALIAS loads module
	MODALIAS is created according to compatible string, or iogroup:<modalias>, 
	         which is compatible without unipi,
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,3,0)
static int iogroup_uevent(struct device *dev, struct kobj_uevent_env *env)
#else
static int iogroup_uevent(const struct device *dev, struct kobj_uevent_env *env)
#endif
{
	const struct unipi_iogroup_device *iogroup = \
                   dev ? container_of(dev, struct unipi_iogroup_device, dev) : NULL;
	//const struct unipi_iogroup_device  *iogroup = to_unipi_iogroup_device(dev);
	if (of_device_uevent_modalias(dev, env) == 0)
		return 0;
	return add_uevent_var(env, "MODALIAS=%s%s", "iogroup:", iogroup->modalias);
}

struct bus_type iogroup_bus_type = {
	.name       = "iogroup",
	.match      = iogroup_match_device,
	//.dev_groups = spi_dev_groups,
	.uevent     = iogroup_uevent,
};
EXPORT_SYMBOL_GPL(iogroup_bus_type);


static int iogroup_drv_probe(struct device *dev)
{
	const struct unipi_iogroup_driver	*sdrv = to_unipi_iogroup_driver(dev->driver);
	struct unipi_iogroup_device			*iogroup = to_unipi_iogroup_device(dev);
	int ret;

	ret = dev_pm_domain_attach(dev, true);
	if (ret)
		return ret;

	if (dev->of_node) {
		iogroup->irq = of_irq_get(dev->of_node, 0);
		if (iogroup->irq == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (iogroup->irq < 0)
			iogroup->irq = 0;
	}

	if (sdrv->probe) {
		ret = sdrv->probe(iogroup);
		if (ret)
			dev_pm_domain_detach(dev, true);
	}
	return ret;
}


static int iogroup_drv_remove(struct device *dev)
{
	const struct unipi_iogroup_driver		*sdrv = to_unipi_iogroup_driver(dev->driver);
	int ret = 0;

	if (sdrv->remove)
		ret = sdrv->remove(to_unipi_iogroup_device(dev));
	dev_pm_domain_detach(dev, true);

	return ret;
}
static void iogroup_drv_shutdown(struct device *dev)
{
	const struct unipi_iogroup_driver *sdrv = to_unipi_iogroup_driver(dev->driver);

	sdrv->shutdown(to_unipi_iogroup_device(dev));
}

/**
 * __unipi_iogroup_register_driver - register a SPI driver
 * @owner: owner module of the driver to register
 * @sdrv: the driver to register
 * Context: can sleep
 *
 * Return: zero on success, else a negative error code.
 */
int __unipi_iogroup_register_driver(struct module *owner, struct unipi_iogroup_driver *sdrv)
{
	sdrv->driver.owner = owner;
	sdrv->driver.bus = &iogroup_bus_type;
	sdrv->driver.probe = iogroup_drv_probe;
	sdrv->driver.remove = iogroup_drv_remove;
	if (sdrv->shutdown)
		sdrv->driver.shutdown = iogroup_drv_shutdown;
	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(__unipi_iogroup_register_driver);


/*
 * Prevents addition of devices with same address
 */
static DEFINE_MUTEX(iogroup_add_lock);

/**
 * iogroup_alloc_device - Allocate a new iogroup device
 * @plc: PLC to which device is connected
 * Context: can sleep
 *
 * Allows a driver to allocate and initialize a unipi_iogroup_device without
 * registering it immediately.  This allows a driver to directly
 * fill the unipi_iogroup_device with device parameters before calling
 * iogroup_add_device() on it.
 *
 * Caller is responsible to call iogroup_add_device() on the returned
 * unipi_iogroup_device structure to add it to the SPI plc.  If the caller
 * needs to discard the unipi_iogroup_device without adding it, then it should
 * call spi_dev_put() on it.
 *
 * Return: a pointer to the new device, or NULL.
 */
struct unipi_iogroup_device *iogroup_alloc_device(struct unipi_channel *channel)
{
	struct unipi_iogroup_device	*iogroup;

	if (!unipi_channel_get(channel))
		return NULL;

	iogroup = kzalloc(sizeof(*iogroup), GFP_KERNEL);
	if (!iogroup) {
		unipi_channel_put(channel);
		return NULL;
	}

	iogroup->channel = channel;
	iogroup->dev.parent = channel->dev;
	iogroup->dev.bus = &iogroup_bus_type;
	iogroup->dev.release = iogroupdev_release;

	//spin_lock_init(&iogroup->statistics.lock);

	device_initialize(&iogroup->dev);
	return iogroup;
}
EXPORT_SYMBOL_GPL(iogroup_alloc_device);


static int iogroup_dev_check(struct device *dev, void *data)
{
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(dev);
	struct unipi_iogroup_device *new_iogroup = data;

	if (/*iogroup->plc == new_iogroup->plc &&*/
	    iogroup->address == new_iogroup->address)
		return -EBUSY;
	return 0;
}

/**
 * iogroup_add_device - Add unipi_iogroup_device allocated with iogroup_alloc_device
 * @iogroup: unipi_iogroup_device to register
 *
 * Companion function to iogroup_alloc_device.  Devices allocated with
 * iogroup_alloc_device can be added onto the spi bus with this function.
 *
 * Return: 0 on success; negative errno on failure
 */
int iogroup_add_device(struct unipi_iogroup_device *iogroup)
{
	struct device *dev = &iogroup->dev;
	int status;

	/* Set the bus ID string */
	//dev_set_name(&iogroup->dev, "%s.%u", dev_name(&iogroup->plc->pdev->dev),
	dev_set_name(dev, "iogroup%u", iogroup->address);

	//request_module("unipi_iogroup");
	/* We need to make sure there's no other device with this
	 * address **BEFORE** we call setup(), else we'll trash
	 * its configuration.  Lock against concurrent add() calls.
	 */
	mutex_lock(&iogroup_add_lock);

	status = bus_for_each_dev(&iogroup_bus_type, NULL, iogroup, iogroup_dev_check);
	if (status) {
		dev_err(dev, "address %d already in use\n", iogroup->address);
		goto done;
	}
	//of_device_request_module(&iogroup->dev);
	/* Device may be bound to an active driver when this returns */
	status = device_add(dev);
	if (status < 0)
		dev_err(dev->parent, "can't add %s, status %d\n", dev_name(dev), status);
	else
		dev_dbg(dev->parent, "registered child %s\n", dev_name(dev));

done:
	mutex_unlock(&iogroup_add_lock);
	return status;
}
EXPORT_SYMBOL_GPL(iogroup_add_device);

/**
 * iogroup_unregister_device - unregister a single Iogroup device
 * @iogroup: unipi_iogroup_device to unregister
 *
 * Start making the passed SPI device vanish. Normally this would be handled
 * by iogroup_unregister_plc().
 */
void iogroup_unregister_device(struct unipi_iogroup_device *iogroup)
{
	if (!iogroup)
		return;

	if (iogroup->dev.of_node) {
		of_node_clear_flag(iogroup->dev.of_node, OF_POPULATED);
		of_node_put(iogroup->dev.of_node);
	}
	device_unregister(&iogroup->dev);
}
EXPORT_SYMBOL_GPL(iogroup_unregister_device);


struct unipi_iogroup_device *
register_iogroup_device(struct unipi_channel *channel, int reg, const char* modalias)
{
	struct unipi_iogroup_device *iogroup;
	int rc;

	/* Alloc an spi_device */
	iogroup = iogroup_alloc_device(channel);
	if (!iogroup) {
		dev_err(channel->dev, "iogroup device alloc error for reg=%d\n", reg);
		rc = -ENOMEM;
		goto err_out;
	}

	/* Select device driver */
	strlcpy(iogroup->modalias, modalias, sizeof(iogroup->modalias));
	iogroup->address = reg;

	/* Register the new device */
	rc = iogroup_add_device(iogroup);
	if (rc) {
		dev_err(channel->dev, "iogroup register error for reg=%d\n", reg);
	}

	return iogroup;

err_out:
	unipi_iogroup_dev_put(iogroup);
	return ERR_PTR(rc);
}
EXPORT_SYMBOL_GPL(register_iogroup_device);

struct unipi_iogroup_device *
of_register_iogroup_device(struct unipi_channel *channel, struct device_node *nc)
{
	struct unipi_iogroup_device *iogroup;
	u32 value;
	int rc;

	/* Alloc an spi_device */
	iogroup = iogroup_alloc_device(channel);
	if (!iogroup) {
		dev_err(channel->dev, "iogroup device alloc error for %pOF\n", nc);
		rc = -ENOMEM;
		goto err_out;
	}

#ifdef CONFIG_OF
	/* Select device driver */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,6,0)
	rc = of_modalias_node(nc, iogroup->modalias,
				sizeof(iogroup->modalias));
#else
	rc = of_alias_from_compatible(nc, iogroup->modalias,
				sizeof(iogroup->modalias));
#endif
#else
	rc = -ENOENT;
#endif
	if (rc < 0) {
		dev_err(channel->dev, "cannot find modalias for %pOF\n", nc);
		goto err_out;
	}

	rc = of_property_read_u32(nc, "reg", &value);
	if (rc) {
		dev_err(channel->dev, "%pOF has no valid 'reg' property (%d)\n",
			nc, rc);
		goto err_out;
	}
	iogroup->address = value;

	/* Store a pointer to the node in the device structure */
	of_node_get(nc);
	iogroup->dev.of_node = nc;

	/* Register the new device */
	rc = iogroup_add_device(iogroup);
	if (rc) {
		dev_err(channel->dev, "iogroup register error %pOF\n", nc);
		goto err_of_node_put;
	}

	return iogroup;

err_of_node_put:
	of_node_put(nc);
err_out:
	unipi_iogroup_dev_put(iogroup);
	return ERR_PTR(rc);
}
EXPORT_SYMBOL_GPL(of_register_iogroup_device);


static int iogroup_check_and_unregister(struct device *dev, void *data)
{
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(dev);
	struct unipi_channel *channel = (struct unipi_channel*) data;
	if (iogroup->channel == channel)
	    iogroup_unregister_device(iogroup);
	return 0;
}


void iogroup_unregister_by_channel(struct unipi_channel *channel)
{
	bus_for_each_dev(&iogroup_bus_type, NULL, channel, iogroup_check_and_unregister);
}
EXPORT_SYMBOL_GPL(iogroup_unregister_by_channel);


/*
static int __unregister(struct device *dev, void *null)
{
	iogroup_unregister_device(to_unipi_iogroup_device(dev));
	return 0;
}
*/


static int __init unipi_iogroup_bus_init(void)
{
	return bus_register(&iogroup_bus_type);
}

static void __exit unipi_iogroup_bus_exit(void)
{
	bus_unregister(&iogroup_bus_type);
}

module_init(unipi_iogroup_bus_init);
module_exit(unipi_iogroup_bus_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Iogroup Bus Driver");
