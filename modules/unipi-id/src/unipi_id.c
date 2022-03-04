// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * unipi_id.c - handle Unipi PLC identity EPROMs
 *
 * Copyright (c) 2022, Unipi Technology
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/nvmem-consumer.h>
#include <crypto/hash.h>

#include "uniee.h"
#include "uniee_values.h"
#include "unipi_id.h"

/*
	names in /sys/bus/nvmem/devices used 
	to detect Unipi PLC and modules in slots
*/
static const char * const id_names[UNIPI_ID_MAX_IDS] = {
	"plcid0", "plcid1", "plcid2", "plcid3",
	"plcid4", "plcid5", "plcid6", "plcid7" };


struct unipi_id_family_data {
	uint16_t	family_id;
	char		name[32];
	uint8_t		i2caddr[UNIPI_ID_MAX_IDS];
	uint8_t		altaddr[UNIPI_ID_MAX_IDS];
	const char**iogroup;
};

static const char* iris_iogroups[] = {
 "12",  "11",  "21",  "22",  "32",  "42",  "52"
};

static const struct unipi_id_family_data unipi_id_family_ids[] = {
	{ UNIEE_PLATFORM_FAMILY_UNIPI1, "Unipi1" },
	{ UNIEE_PLATFORM_FAMILY_G1XX,   "Gate"   },
	{ UNIEE_PLATFORM_FAMILY_NEURON, "Neuron" },
	{ UNIEE_PLATFORM_FAMILY_AXON,   "Axon"   },
	{ UNIEE_PLATFORM_FAMILY_PATRON, "Patron" },
	{ UNIEE_PLATFORM_FAMILY_IRIS,   "Iris", {0x50,0x51,0x52,0x53,0x54,0x55,0x56},
	                                        {0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e},
	                                        iris_iogroups},
/*	{ UNIEE_PLATFORM_ID_IRISx1,     "Iris", {0x50,0x51},
	                                        {0x48,0x49},
	                                        iris_iogroups},
	{ UNIEE_PLATFORM_ID_IRISx2,     "Iris", {0x50,0x51,0x52,0x53},
	                                        {0x48,0x49,0x4a,0x4b},
	                                        iris_iogroups},*/
	{ /* END OF LIST */ 
	 0, "unknown", {0x50,0x51,0x52,0x53,0x54,0x55,0x56},
	                {0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e},
	                iris_iogroups }
};

static const struct unipi_id_family_data * get_family_data(platform_id_t platform_id)
{
	const struct unipi_id_family_data *fdata = NULL;
	const struct unipi_id_family_data *data = unipi_id_family_ids;
	while (data->family_id != 0) {
		if (data->family_id == platform_id.raw_id){
			return data;
		}
		if (data->family_id == platform_id.parsed.platform_family){
			fdata = data;
		}
		data++;
	}
	return fdata ? fdata:data;
}

const char* unipi_id_get_family_name(struct unipi_id_data *unipi_id)
{
	return unipi_id->family_data->name;
}
EXPORT_SYMBOL_GPL(unipi_id_get_family_name);

/*
	loads content of nvmem (using name id_names[nvmem_index]) into buf
	check unipi_id validity
	returns descriptor if valid content 
			NULL if content was read but not valid
			ERR if couldnot read data
*/
uniee_descriptor_area* unipi_id_load_nvmem(struct device *dev, int nvmem_index, size_t *size, uint8_t *buf)
{
	struct nvmem_device *nvmem;
	int ret;

	*size = 0;
	nvmem = nvmem_device_get(dev, id_names[nvmem_index]);
	//dev_info(dev, "Testing nvmem: %s = %p\n", labels[index][0], nvmem);
	if (IS_ERR(nvmem))
		return ERR_PTR(PTR_ERR(nvmem));
	ret = nvmem_device_read(nvmem, 0, UNIEE_MIN_EE_SIZE, buf);
	if (ret == UNIEE_MIN_EE_SIZE) {
		*size = UNIEE_MIN_EE_SIZE;
		ret = nvmem_device_read(nvmem, UNIEE_MIN_EE_SIZE, UNIEE_MIN_EE_SIZE, buf+UNIEE_MIN_EE_SIZE);
		if (ret == UNIEE_MIN_EE_SIZE) {
			if (memcmp(buf, buf+UNIEE_MIN_EE_SIZE, UNIEE_MIN_EE_SIZE) != 0)
				*size += UNIEE_MIN_EE_SIZE;
		}
	}
	//ret = nvmem_device_read(nvmem, 0, size, buf);
	nvmem_device_put(nvmem);
	//if (ret != size) 
	//	return NULL;

	if (*size == 0) return NULL;

	return uniee_get_valid_descriptor(buf, *size);
}


static uniee_descriptor_area* unipi_id_load_boardmem(struct device *dev,
				int nvmem_index, struct unipi_id_data * unipi_id, uint8_t *buf)
{
	uniee_descriptor_area *descriptor, *ndescriptor;
	size_t size;
	int i;

	descriptor = unipi_id_load_nvmem(dev, nvmem_index, &size, buf);
	if (IS_ERR(descriptor)) 
		return ERR_PTR(-1);

	if (nvmem_index==0) {
		if (descriptor) {
			uniee_fix_legacy_content(buf, size, descriptor);
		}
		unipi_id->family_data = get_family_data(descriptor->product_info.platform_id);
		ndescriptor = &unipi_id->descriptor;
	} else {
		if (!unipi_id->loaded_descriptor[nvmem_index]) {
			ndescriptor = kmalloc(sizeof(uniee_descriptor_area), GFP_KERNEL);
			if (!ndescriptor)
				return ERR_PTR(-1);
			unipi_id->loaded_descriptor[nvmem_index] = ndescriptor;
		} else {
			ndescriptor = unipi_id->loaded_descriptor[nvmem_index];
		}
	}
	if (descriptor) {
		memcpy(ndescriptor, descriptor, sizeof(uniee_descriptor_area));
		for (i=0; i<sizeof(ndescriptor->product_info.model_str); i++) 
			if (ndescriptor->product_info.model_str[i]==0xff) {
				ndescriptor->product_info.model_str[i] = '\0';
				break;
			}
	} else {
		memset(ndescriptor, 0, sizeof(uniee_descriptor_area));
	}
	return ndescriptor;
}


static uniee_descriptor_area* unipi_id_get_descriptor(struct device *dev,
				int nvmem_index, struct unipi_id_data *unipi_id)
{
	uint8_t dbuf[UNIEE_MAX_EE_SIZE];

	if (unipi_id->loaded_descriptor[nvmem_index])
		return unipi_id->loaded_descriptor[nvmem_index];

	return unipi_id_load_boardmem(dev, nvmem_index, unipi_id, dbuf);
}

/*
 ********************************************************************************
 *	sysfs attributes
 */


static int do_checksum(struct device *dev,struct unipi_id_data *unipi_id, unsigned char *digest);

static ssize_t product_model_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_3_t *bank3;
	if (unipi_id == NULL)
		return 0;
	bank3 = &unipi_id->descriptor.product_info;
	return scnprintf(buf, 255, "%.6s\n", bank3->model_str);
}
DEVICE_ATTR_RO(product_model);

static ssize_t product_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_3_t *bank3;
	if (unipi_id == NULL)
		return 0;
	bank3 = &unipi_id->descriptor.product_info;
	return scnprintf(buf, 255, "%d.%d", bank3->product_version.major, bank3->product_version.minor);
}
DEVICE_ATTR_RO(product_ver);

static ssize_t product_serial_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_3_t *bank3;
	if (unipi_id == NULL)
		return 0;
	bank3 = &unipi_id->descriptor.product_info;
	return scnprintf(buf, 255, "%d", bank3->product_serial);
}
DEVICE_ATTR_RO(product_serial);

static ssize_t product_description_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_3_t *bank3;
	if (unipi_id == NULL)
		return 0;
	bank3 = &unipi_id->descriptor.product_info;

	return scnprintf(buf, 2048,
				"Product string:  %.6s\n"
				"Product version: %u.%u\n"
				"Product serial:  %08u\n"
				"SKU:             %u\n"
				"Options:         0x%02X\n\n"
				"Platform family: %s (%d)\n"
				"Platform series: 0x%02X\n"
				"RAW platform ID: 0x%04X\n", 
				bank3->model_str,
				bank3->product_version.major, bank3->product_version.minor,
				bank3->product_serial,
				bank3->sku,
				bank3->mervis_license.bitmask,
				unipi_id_get_family_name(unipi_id),
				bank3->platform_id.parsed.platform_family,
				bank3->platform_id.parsed.platform_series,
				bank3->platform_id.raw_id);
}
DEVICE_ATTR_RO(product_description);

static ssize_t platform_family_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_3_t *bank3;
	if (unipi_id == NULL)
		return 0;
	bank3 = &unipi_id->descriptor.product_info;
	return scnprintf(buf, 255, "%d", bank3->platform_id.parsed.platform_family);
}
DEVICE_ATTR_RO(platform_family);

static ssize_t platform_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_3_t *bank3;
	if (unipi_id == NULL)
		return 0;
	bank3 = &unipi_id->descriptor.product_info;
	return scnprintf(buf, 255, "%04x", bank3->platform_id.raw_id);
}
DEVICE_ATTR_RO(platform_id);


static ssize_t baseboard_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_2_t *bank2;
	if (unipi_id == NULL)
		return 0;
	bank2 = &unipi_id->descriptor.board_info;
	return scnprintf(buf, 255, "%04x", bank2->board_model);
}
DEVICE_ATTR_RO(baseboard_id);

static ssize_t baseboard_description_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	uniee_bank_2_t *bank2;
	if (unipi_id == NULL)
		return 0;
	bank2 = &unipi_id->descriptor.board_info;

	return scnprintf(buf, 2048,
				"Modul string:  Baseboard\n"
				"Modul version: %u.%u\n"
				"Modul serial:  %08u\n"
				"Modul ID:      0x%04X\n"
				"Nvmem:         %s\n",
				bank2->board_version.major, bank2->board_version.minor,
				bank2->board_serial,
				bank2->board_model,
				id_names[0]);
}
DEVICE_ATTR_RO(baseboard_description);

static ssize_t fingerprint_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	int ret;

	if (unipi_id == NULL)
		return 0;
	if (!unipi_id->fingerprint_loaded) {
		ret = do_checksum(dev, unipi_id, unipi_id->fingerprint);
		if (ret < 0) return ret;
		unipi_id->fingerprint_loaded = 1;
	}
	return scnprintf(buf, 255, "%20phN", unipi_id->fingerprint);
}
DEVICE_ATTR_RO(fingerprint);

static int unipi_id_do_refresh(struct device *dev, struct unipi_id_data *unipi_id);

static ssize_t refresh_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	int ret = 0;

	if (unipi_id == NULL)
		return -ENODEV;
	if ((count > 0) && (buf[0] == '1')) {
		ret = unipi_id_do_refresh(dev, unipi_id);
		if (ret == 0)
			return count;
		return -ENODEV;
	}
	return -EINVAL;
}
DEVICE_ATTR_WO(refresh);

static struct attribute *unipi_id_attrs[] = {
		&dev_attr_product_description.attr,
		&dev_attr_product_model.attr,
		&dev_attr_product_serial.attr,
		&dev_attr_platform_family.attr,
		&dev_attr_platform_id.attr,
		&dev_attr_baseboard_description.attr,
		&dev_attr_baseboard_id.attr,
		&dev_attr_fingerprint.attr,
		&dev_attr_refresh.attr,
		NULL,
};

static const struct attribute_group unipi_id_group_def = {
	.name  = "unipi-id",
	.attrs  = unipi_id_attrs,
};

struct unipi_id_module_attribute {
	struct device_attribute dev_attr;
	int module_index;
};
#define to_unipi_id_attr(x) container_of(x, struct unipi_id_module_attribute, dev_attr)

static ssize_t module_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	struct unipi_id_module_attribute *ea = to_unipi_id_attr(attr);
	uniee_descriptor_area *descriptor;
	if (unipi_id == NULL)
		return 0;
	descriptor = unipi_id_get_descriptor(dev, unipi_id->active_slot[ea->module_index]+1, unipi_id);
	if (IS_ERR(descriptor) || (!descriptor))
		return 0;
	return scnprintf(buf, 255, "%04x", descriptor->board_info.board_model);
}

static ssize_t module_description_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct unipi_id_data *unipi_id = dev_get_platdata(dev);
	struct unipi_id_module_attribute *ea = to_unipi_id_attr(attr);
	int nvmem_index;
	uniee_descriptor_area *descriptor;
	if (unipi_id == NULL)
		return 0;
	nvmem_index =  unipi_id->active_slot[ea->module_index]+1;
	descriptor = unipi_id_get_descriptor(dev, nvmem_index, unipi_id);
	if (IS_ERR(descriptor) || (!descriptor))
		return 0;
	return scnprintf(buf, 2048,
				"Modul string:  %.6s\n"
				"Modul version: %u.%u\n"
				"Modul serial:  %08u\n"
				"Modul ID:      0x%04X\n"
				"SKU:           %u\n"
				"Nvmem:         %s\n"
				"Slot:          %s\n",
				descriptor->product_info.model_str,
				descriptor->board_info.board_version.major, descriptor->board_info.board_version.minor,
				descriptor->board_info.board_serial,
				descriptor->board_info.board_model,
				descriptor->product_info.sku,
				id_names[nvmem_index],
				unipi_id->family_data->iogroup[unipi_id->active_slot[ea->module_index]]);
}



const char module_id_name[] = "module_id.";
const char module_description_name[] = "module_description.";

int unipi_id_add_modules(struct device *dev, struct unipi_id_data *unipi_id)
{
	char* names;
	int names_len = 0;
	struct unipi_id_module_attribute *unipi_id_attrs;
	int i, count;
	struct attribute_group attr_group;
	struct attribute *attr_arr[2*UNIPI_ID_MAX_IDS+1];

	/* get memory size for strings */
	count = unipi_id->slot_count;
	for (i=0; i<count; i++) {
		names_len = strlen(unipi_id->family_data->iogroup[unipi_id->active_slot[i]])+1;
	}
	names_len += count * (strlen(module_id_name)+strlen(module_description_name));

	unipi_id_attrs = devm_kzalloc(dev,
		2*count * sizeof(struct unipi_id_module_attribute) +
		names_len,
		GFP_KERNEL);
	names = (char*) (unipi_id_attrs + 2*count);

	/* create attribute array with attributes module_id.XX module_description.XX */
	for (i=0; i<count; i++) {
		strcpy(names, module_description_name);
		strcat(names, unipi_id->family_data->iogroup[unipi_id->active_slot[i]]);
		unipi_id_attrs[2*i].dev_attr.attr.name = names;
		unipi_id_attrs[2*i].dev_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
		unipi_id_attrs[2*i].module_index = i;
		unipi_id_attrs[2*i].dev_attr.show = module_description_show;
		names += strlen(names)+1;

		strcpy(names, module_id_name);
		strcat(names, unipi_id->family_data->iogroup[unipi_id->active_slot[i]]);
		unipi_id_attrs[2*i+1].dev_attr.attr.name = names;
		unipi_id_attrs[2*i+1].dev_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
		unipi_id_attrs[2*i+1].module_index = i;
		unipi_id_attrs[2*i+1].dev_attr.show = module_id_show;
		names += strlen(names)+1;
	}
	attr_group.name = unipi_id_group_def.name;
	attr_group.attrs = attr_arr;
	for (i=0; i<count; i++) {
		attr_arr[2*i] = &unipi_id_attrs[2*i].dev_attr.attr;
		attr_arr[2*i+1] = &unipi_id_attrs[2*i+1].dev_attr.attr;
	}
	attr_arr[2*count] = NULL;
	sysfs_merge_group(&dev->kobj, &attr_group);
	return 0;
}

static int unipi_id_do_refresh(struct device *dev, struct unipi_id_data *unipi_id)
{
	int i;
	size_t size;
	const struct unipi_id_family_data *family_data;
	uint8_t buf[UNIEE_MAX_EE_SIZE];
	uniee_descriptor_area *descriptor;

	unipi_id->fingerprint_loaded = 0;
	for (i=0; i<UNIPI_ID_MAX_IDS; i++) {
		if (unipi_id->loaded_descriptor[i]) {
			kfree(unipi_id->loaded_descriptor[i]);
			unipi_id->loaded_descriptor[i] = NULL;
		}
	}
	/* Read and analyze main id eprom */
	descriptor = unipi_id_load_nvmem(dev, 0, &size, buf);
	if (IS_ERR(descriptor))
		return -EINVAL;
	if (descriptor != NULL) {
		uniee_fix_legacy_content(buf, size, descriptor);
		memcpy(&unipi_id->descriptor, descriptor, sizeof(uniee_descriptor_area));
	}

	family_data = get_family_data(unipi_id->descriptor.product_info.platform_id);
	unipi_id->family_data = family_data;
	return 0;
}

int unipi_id_add_group(struct device *dev, struct unipi_id_data *unipi_id)
{
	int ret;
	ret = devm_device_add_group(dev, &unipi_id_group_def);
	ret = unipi_id_add_modules(dev, unipi_id);
//	sysfs_merge_group(&dev->kobj, &unipi_slot1_group_def);
	return ret;
}

/*****************************************************
 Calc hash of unipi_id eeproms
*/

static int do_checksum(struct device *dev,struct unipi_id_data *unipi_id, unsigned char *digest)
{
	struct crypto_shash *alg;
	//char *hash_alg_name = "sha1-padlock-nano";
	char *hash_alg_name = "sha1"; // digest size = 20
	struct shash_desc *sdesc;
	size_t size;
	int nvmem_index;
	uniee_descriptor_area *desc;
	uint8_t data[UNIEE_MAX_EE_SIZE];
    int ret, i;

    alg = crypto_alloc_shash(hash_alg_name, CRYPTO_ALG_TYPE_SHASH, 0);
    if (IS_ERR(alg)) {
            pr_info("can't alloc alg %s\n", hash_alg_name);
            return PTR_ERR(alg);
    }

	size = sizeof(struct shash_desc) + crypto_shash_descsize(alg);
	sdesc = kmalloc(size, GFP_KERNEL);
	if (!sdesc)
        return -ENOMEM;
	sdesc->tfm = alg;

	ret = crypto_shash_init(sdesc);
	if(ret!=0) return ret;

	desc = unipi_id_load_nvmem(dev, 0, &size, data);
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	ret = crypto_shash_update(sdesc, data, size);
	if(ret!=0) return ret;

	for (i=0; i<unipi_id->slot_count; i++) {
		nvmem_index = unipi_id->active_slot[i]+1;
		desc = unipi_id_load_nvmem(dev, nvmem_index, &size, data);
		if (!IS_ERR(desc)) {
			ret = crypto_shash_update(sdesc, data, size);
			if(ret!=0) return ret;
		}
	}
	ret = crypto_shash_final(sdesc, digest);

	//ret = calc_hash(alg, data, datalen, digest);
	kfree(sdesc);
	crypto_free_shash(alg);
	return ret;
}



static const char * const labels[][UNIPI_ID_MAX_IDS] = {
	{ id_names[0] },{ id_names[1] },{ id_names[2] },{ id_names[3] },
	{ id_names[4] },{ id_names[5] },{ id_names[6] },{ id_names[7] },
};

static const struct property_entry label0_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[0]),{ }
};
static const struct property_entry label1_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[1]),	{ }
};
static const struct property_entry label2_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[2]),	{ }
};
static const struct property_entry label3_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[3]),	{ }
};
static const struct property_entry label4_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[4]),	{ }
};
static const struct property_entry label5_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[5]),	{ }
};
static const struct property_entry label6_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[6]),	{ }
};
static const struct property_entry label7_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("label", labels[7]),	{ }
};

static const struct property_entry* label_props_arr[8] = {
	label0_props, label1_props,
	label2_props, label3_props,
	label4_props, label5_props,
	label6_props, label7_props,
};

static struct i2c_client* unipi_id_load_client(struct i2c_adapter *adapter, unsigned int index, const char* eprom_type, unsigned short address)
{
	struct device *dev = &adapter->dev;
	union i2c_smbus_data dummy;
	struct nvmem_device *nvmem;
	int err;
	struct i2c_board_info info;

	nvmem = nvmem_device_get(dev, id_names[index]);
	//dev_info(dev, "Testing nvmem: %s = %p\n", id_names[index], nvmem);
	if (!IS_ERR(nvmem)) {
		// id eprom already loaded
		nvmem_device_put(nvmem);
		return NULL;
	}

	if (!(i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE)))
		return ERR_PTR(-EIO);

	memset(&info, 0, sizeof(info));
	info.addr = address;
	err = i2c_smbus_xfer(adapter, info.addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE, &dummy);
	if (err<0) 
		return ERR_PTR(err);
	
	strncpy(info.type, eprom_type, I2C_NAME_SIZE);
	//info.properties = label_props_arr[index];
	dev_info(dev, "TRY %s,  %d %d", info.type, info.addr, index);
	return i2c_new_client_device(adapter, &info);
}


int unipi_id_probe(struct platform_device *pdev)
//struct i2c_adapter *adapter, uint32_t main_id, struct unipi_id_data * unipi_id_data)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	uint32_t main_id = 0x57;
	const struct unipi_id_family_data *family_data;
	uint8_t buf[UNIEE_MAX_EE_SIZE];
	uniee_descriptor_area *descriptor;
	struct unipi_id_data unipi_id;
	//int ret;
	int i, slot_count;

	np = of_parse_phandle(dev->of_node, "id-channel", 0);
	if (!np) 
		return -EPROBE_DEFER;

	adapter = of_find_i2c_adapter_by_node(np);
	// must be called put_device(adapter) after using adapter!
	of_node_put(np);
	if (adapter == NULL) {
		return -EPROBE_DEFER; //-ENOENT;
	}
	device_property_read_u32(dev, "id-main", &main_id);

	memset(&unipi_id, 0, sizeof(struct unipi_id_data));

	/* Try to load primary unipi_id eprom on main_id */
	client = unipi_id_load_client(adapter, 0, "24c01", main_id);
	if (IS_ERR(client)) {
		put_device(&adapter->dev);
		return PTR_ERR(client);
	}
	unipi_id.loaded_clients[0] = client;
	/* Read and analyze main id eprom */
	descriptor = unipi_id_load_boardmem(dev, 0, &unipi_id, buf);
	if (IS_ERR(descriptor)) {
		put_device(&adapter->dev);
		return -EINVAL;
	}

	family_data = unipi_id.family_data;
	slot_count = 0;
	for (i=0; i<UNIPI_ID_MAX_IDS; i++) {
		if (family_data->i2caddr[i] == 0)
			break;

		client = unipi_id_load_client(adapter, i+1, "24c02", family_data->i2caddr[i]);
		if (IS_ERR(client) && (family_data->altaddr[i] != 0))
			client = unipi_id_load_client(adapter, i+1, "24c02", family_data->altaddr[i]);

		if (!IS_ERR(client)) {
			unipi_id.loaded_clients[slot_count+1] = client;
			unipi_id.active_slot[slot_count] = i;
			slot_count++;
		}
	}
	put_device(&adapter->dev);
	unipi_id.slot_count = slot_count;

	//dev_info(dev, "Found PLC %s (family %s)\n", unipi_id_data->description.model, family_data->name);
	if (slot_count > 0) {
		char buf[256] = "";
		for (i=0; i<slot_count; i++)
			scnprintf(buf, sizeof(buf), "%s %s,", buf, family_data->iogroup[unipi_id.active_slot[i]]);
		dev_info(dev, "Found %d modules in slots %s\n", slot_count,buf);
	}

	if (platform_device_add_data(pdev, &unipi_id, sizeof(unipi_id)) != 0) {
		dev_err(dev, "Memory error\n");
		return -ENOMEM;
	}

	unipi_id_add_group(dev, &unipi_id);

	return 0;
}


int unipi_id_remove(struct platform_device *pdev)
{
	struct unipi_id_data *unipi_id_data = dev_get_platdata(&pdev->dev);
	int i;

	for (i=0; i<unipi_id_data->slot_count+1; i++) {
		if (unipi_id_data->loaded_clients[i]) {
			i2c_unregister_device(unipi_id_data->loaded_clients[i]);
		}
	}
	for (i=0; i<UNIPI_ID_MAX_IDS; i++) {
		if (unipi_id_data->loaded_descriptor[i])
			kfree(unipi_id_data->loaded_descriptor[i]);
	}
	return 0;
}

static const struct of_device_id unipi_id_ids[] = {
    { .compatible = "unipi,unipi-id", },
    { /*sentinel */}
};

MODULE_DEVICE_TABLE(of, unipi_id_ids);

static struct platform_driver unipi_id_driver = {
    .driver = {
        .name = "unipi-id",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(unipi_id_ids),
        //.dev_groups = unipi_plc_groups,
        },
    .probe  = unipi_id_probe,
    .remove = unipi_id_remove,
	//.id_table = unipi_id_ids,
};


static int __init unipi_id_init(void)
{
	return platform_driver_register(&unipi_id_driver);
}

static void __exit unipi_id_exit(void)
{
	platform_driver_unregister(&unipi_id_driver);
}

module_init(unipi_id_init);
module_exit(unipi_id_exit);

MODULE_DESCRIPTION("Driver for ID EEPROMs in Unipi PLCs");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_LICENSE("GPL");
