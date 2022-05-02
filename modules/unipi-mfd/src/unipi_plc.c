#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/nvmem-consumer.h>
//#include <linux/device/bus.h>

#include "unipi_id.h"
#include "unipi_plc.h"

static void iogroupdev_release(struct device *dev)
{
	struct unipi_iogroup_device	*iogroup = to_unipi_iogroup_device(dev);

	/* iogroup plcs may cleanup for released devices */
	if (iogroup->plc->cleanup)
		iogroup->plc->cleanup(iogroup);

	unipi_plc_put(iogroup->plc);
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
static int iogroup_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct unipi_iogroup_device  *iogroup = to_unipi_iogroup_device(dev);
	if (of_device_uevent_modalias(dev, env) == 0) 
		return 0;
	return add_uevent_var(env, "MODALIAS=%s%s", "iogroup:", iogroup->modalias);
}

struct bus_type iogroup_bus_type = {
	.name		= "iogroup",
	.match		= iogroup_match_device,
	//.dev_groups	= spi_dev_groups,
	.uevent		= iogroup_uevent,
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
 * Prevents addition of devices with same chip select and
 * addition of devices below an unregistering plc.
 */
static DEFINE_MUTEX(iogroup_add_lock);

static int of_iogroup_parse_dt(struct unipi_plc *plc, struct unipi_iogroup_device *iogroup,
			   struct device_node *nc)
{
	u32 value;
	int rc;
	
	/* Mode (clock phase/polarity/etc.) */
	/*if (of_property_read_bool(nc, "spi-cpha"))
		spi->mode |= SPI_CPHA; */

	/* Device DUAL/QUAD mode */
	/*if (!of_property_read_u32(nc, "spi-tx-bus-width", &value)) 
	*/ 
	rc = of_property_read_u32(nc, "reg", &value);
	if (rc) {
		dev_err(&plc->pdev->dev, "%pOF has no valid 'reg' property (%d)\n",
			nc, rc);
		return rc;
	}
	iogroup->address = value;

	return 0;
}

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
struct unipi_iogroup_device *iogroup_alloc_device(struct unipi_plc *plc)
{
	struct unipi_iogroup_device	*iogroup;

	if (!unipi_plc_get(plc))
		return NULL;

	iogroup = kzalloc(sizeof(*iogroup), GFP_KERNEL);
	if (!iogroup) {
		unipi_plc_put(plc);
		return NULL;
	}

	iogroup->plc = plc;
	iogroup->dev.parent = &plc->pdev->dev;
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

	if (iogroup->plc == new_iogroup->plc &&
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
	struct unipi_plc *plc = iogroup->plc;
	struct device *dev = plc->pdev->dev.parent;
	int status;

	/* Set the bus ID string */
	//dev_set_name(&iogroup->dev, "%s.%u", dev_name(&iogroup->plc->pdev->dev),
	dev_set_name(&iogroup->dev, "iogroup%u", iogroup->address);

	//request_module("unipi_iogroup");
	/* We need to make sure there's no other device with this
	 * address **BEFORE** we call setup(), else we'll trash
	 * its configuration.  Lock against concurrent add() calls.
	 */
	mutex_lock(&iogroup_add_lock);

	status = bus_for_each_dev(&iogroup_bus_type, NULL, iogroup, iogroup_dev_check);
	if (status) {
		dev_err(dev, "address %d already in use\n",
				iogroup->address);
		goto done;
	}
	//of_device_request_module(&iogroup->dev);
	/* Device may be bound to an active driver when this returns */
	status = device_add(&iogroup->dev);
	if (status < 0)
		dev_err(dev, "can't add %s, status %d\n",
				dev_name(&iogroup->dev), status);
	else
		dev_dbg(dev, "registered child %s\n", dev_name(&iogroup->dev));

done:
	mutex_unlock(&iogroup_add_lock);
	return status;
}
EXPORT_SYMBOL_GPL(iogroup_add_device);

/**
 * iogroup_unregister_device - unregister a single SPI device
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


static struct unipi_iogroup_device *
of_register_iogroup_device(struct unipi_plc *plc, struct device_node *nc)
{
	struct unipi_iogroup_device *iogroup;
	int rc;

	/* Alloc an spi_device */
	iogroup = iogroup_alloc_device(plc);
	if (!iogroup) {
		dev_err(&plc->pdev->dev, "iogroup device alloc error for %pOF\n", nc);
		rc = -ENOMEM;
		goto err_out;
	}

	/* Select device driver */
	rc = of_modalias_node(nc, iogroup->modalias,
				sizeof(iogroup->modalias));
	if (rc < 0) {
		dev_err(&plc->pdev->dev, "cannot find modalias for %pOF\n", nc);
		goto err_out;
	}

	rc = of_iogroup_parse_dt(plc, iogroup, nc);
	if (rc)
		goto err_out;

	/* Store a pointer to the node in the device structure */
	of_node_get(nc);
	iogroup->dev.of_node = nc;

	/* Register the new device */
	rc = iogroup_add_device(iogroup);
	if (rc) {
		dev_err(&plc->pdev->dev, "spi_device register error %pOF\n", nc);
		goto err_of_node_put;
	}

	return iogroup;

err_of_node_put:
	of_node_put(nc);
err_out:
	unipi_iogroup_dev_put(iogroup);
	return ERR_PTR(rc);
}

/**
 * of_register_iogroup_devices() - Register child devices onto the SPI bus
 * @ctlr:	Pointer to spi_plc device
 *
 * Registers an iogroup for each child node of plc node 
 */
static void of_register_iogroup_devices(struct unipi_plc *plc)
{
	struct unipi_iogroup_device *iogroup;
	struct device_node *nc;

	if (!plc->pdev->dev.of_node)
		return;

	for_each_available_child_of_node(plc->pdev->dev.of_node, nc) {
		if (of_node_test_and_set_flag(nc, OF_POPULATED))
			continue;
		iogroup = of_register_iogroup_device(plc, nc);
		if (IS_ERR(iogroup)) {
			dev_warn(&plc->pdev->dev,
				 "Failed to create iogroup device for %pOF\n", nc);
			of_node_clear_flag(nc, OF_POPULATED);
		}
	}
}


static int __unregister(struct device *dev, void *null)
{
	iogroup_unregister_device(to_unipi_iogroup_device(dev));
	return 0;
}

/*
 ********************************************************************************
 *	sysfs attributes
 */

int device_match_unipi_id(struct device *dev, const void *unused)
{
	if (dev->driver && dev->driver->name && (strcmp(dev->driver->name,"unipi-id")==0))
		return 1;
	return 0;
}

static ssize_t model_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct device *unipi_dev;
	struct unipi_id_data *unipi_id;
	int ret;

	unipi_dev = bus_find_device(&platform_bus_type, NULL, NULL, device_match_unipi_id);
	if (unipi_dev) {
		unipi_id = dev_get_platdata(unipi_dev);
		ret =  scnprintf(buf, 255, "%s %.6s\n", unipi_id_get_family_name(unipi_id),
                     unipi_id->descriptor.product_info.model_str);
		put_device(unipi_dev);
		return ret;
	}
	return 0;
}
DEVICE_ATTR_RO(model_name);

static ssize_t sys_eeprom_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct device *unipi_dev;
	struct unipi_id_data *unipi_id;
	int ret;

	unipi_dev = bus_find_device(&platform_bus_type, NULL, NULL, device_match_unipi_id);
	if (unipi_dev) {
		unipi_id = dev_get_platdata(unipi_dev);
		ret = scnprintf(buf, 255, "%.6s\n", unipi_id->descriptor.product_info.model_str);
		put_device(unipi_dev);
		return ret;
	}
	return 0;
}
DEVICE_ATTR_RO(sys_eeprom_name);

static struct attribute *unipi_plc_attrs[] = {
		&dev_attr_model_name.attr,
		&dev_attr_sys_eeprom_name.attr,
		NULL,
};

ATTRIBUTE_GROUPS(unipi_plc);

/****************************************************!
 * This is the probe routine for the plc driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */

static int unipi_plc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *unipi_dev;
	struct device_node *root;
	const struct property *prop;
//	struct nvmem_cell * cell;
	struct unipi_plc plc;
//	struct device_node *np = dev->of_node;
//	struct i2c_adapter *adapter;
//	uint32_t main_id = 0x57;
//	int ret;

	dev_dbg(dev, "probe\n");

#if defined(NO_ID)
	root = of_find_node_by_path("/");
	if (root == NULL) {
		return  -ENODEV;
	}
	prop = of_find_property(tooy, "unipi-model", NULL);
	of_node_put(root);
#else
	unipi_dev = bus_find_device(&platform_bus_type, NULL, NULL, device_match_unipi_id);
	if (unipi_dev) {
		struct unipi_id_data *unipi_id = dev_get_platdata(unipi_dev);
		dev_info(dev, "Found PLC %s Model: %.6s\n",  unipi_id_get_family_name(unipi_id), unipi_id->descriptor.product_info.model_str);
		put_device(unipi_dev);
	} else {
		return  -EPROBE_DEFER;
	}
#endif
/*	np = of_parse_phandle(np, "id-channel", 0);
	if (!np) 
		return -EPROBE_DEFER;
		//goto deferred;
	adapter = of_find_i2c_adapter_by_node(np);
	// must be called put_device(adapter) after using adapter!
	of_node_put(np);
	if (adapter == NULL) {
		return -EPROBE_DEFER; //-ENOENT;
	}
	device_property_read_u32(dev, "id-main", &main_id);
*/
	memset(&plc, 0, sizeof(struct unipi_plc));
/*
	ret = unipi_id_probe(adapter, main_id, &plc.unipi_id);
	put_device(&adapter->dev);
	if (ret < 0)
		return ret;
*/
/*
	cell = devm_nvmem_cell_get(dev, "slots");
	if (IS_ERR(cell) || (cell == NULL)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER) 
			return -EPROBE_DEFER;
		dev_err(dev, "Cannot get nvmem slots\n");
		return -ENODEV;
	}
	ret = unipi_plc_do_refresh(dev, cell, &plc.description);
	if (ret < 0) 
		return -ENODEV;
*/

//	dev_info(dev, "Found PLC %s Model: %.6s\n",  unipi_id_get_family_name(&plc.unipi_id), plc.unipi_id.descriptor.product_info.model_str);
//	plc.slots = cell;
	plc.pdev = pdev;
	if (platform_device_add_data(pdev, &plc, sizeof(plc)) != 0) {
		dev_err(dev, "Memory error\n");
		return -ENOMEM;
	}

//	devm_device_add_group(dev, &unipi_slot_group_def);
//	sysfs_merge_group(&dev->kobj, &unipi_slot1_group_def);

//	unipi_id_add_group(dev, &plc.unipi_id);

	of_register_iogroup_devices(pdev->dev.platform_data);
	//of_platform_populate(np, NULL, NULL, dev);
	return 0;
}


static int unipi_plc_remove(struct platform_device *pdev)
{
//	struct unipi_plc *data = dev_get_platdata(&pdev->dev);
//	unipi_id_remove(&data->unipi_id);
	dev_info(&pdev->dev, "remove\n");
	return 0;
}

static const struct of_device_id unipi_plc_ids[] = {
    { .compatible = "unipi,unipi-plc", },
    { /*sentinel */}
};

MODULE_DEVICE_TABLE(of, unipi_plc_ids);

static struct platform_driver unipi_plc_driver = {
    .driver = {
        .name = "unipi-plc",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(unipi_plc_ids),
        .dev_groups = unipi_plc_groups,
        },
    .probe  = unipi_plc_probe,
    .remove = unipi_plc_remove,
};

static int __init unipi_plc_init(void)
{
    int ret;
	ret = bus_register(&iogroup_bus_type);
	if (ret < 0)
		return ret;

	ret = platform_driver_register(&unipi_plc_driver);
	if (ret)
		bus_unregister(&iogroup_bus_type);
	return ret;
}

static void __exit unipi_plc_exit(void)
{
	platform_driver_unregister(&unipi_plc_driver);
	bus_unregister(&iogroup_bus_type);
}

module_init(unipi_plc_init);
module_exit(unipi_plc_exit);

//module_platform_driver(unipi_plc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi PLC Platform Driver");
