
#include <linux/regmap.h>
#include "unipi_spi.h"

/* ==========================================
packed struct modbus_header {
    u8  op;
    u8  regcount;
    u16 reg;
}
*/

#define UNIPI_REGMAP_GET_BUFFER_SIZE_1  (UNIPI_MODBUS_REGISTER_SIZE + UNIPI_MODBUS_HEADER_SIZE + UNIPI_MODBUS_TAIL_SIZE)
#define UNIPI_REGMAP_GET_BUFFER_SIZE_2  (2*UNIPI_MODBUS_REGISTER_SIZE + UNIPI_MODBUS_HEADER_SIZE + UNIPI_MODBUS_TAIL_SIZE)
#define UNIPI_REGMAP_GET_BUFFER_SIZE(regcount) ((regcount)*UNIPI_MODBUS_REGISTER_SIZE + UNIPI_MODBUS_HEADER_SIZE + UNIPI_MODBUS_TAIL_SIZE)

/*** This structure is from /drivers/base/regmap/internal.h ****/
struct regmap_async {
	struct list_head list;
	struct regmap *map;
	void *work_buf;
	int padding[2];
};
void regmap_async_complete_cb(struct regmap_async *async, int ret);


struct unipi_regmap_async {
	u8 *buffer;
	struct regmap_async core;
};

static void unipi_regmap_complete(void *data, int result, u8* recv)
{
	struct unipi_regmap_async *async = data;
	regmap_async_complete_cb(&async->core, result);
    if (async->buffer) kfree(async->buffer);
}

int unipi_regmap_hw_read(void *context, const void *reg_buf, size_t reg_size, void *val_buf, size_t val_size)
{
	struct spi_device *spi = context;
	u8 *buffer;
	int regcount;

	buffer = kzalloc(val_size+4, GFP_KERNEL);
    if (buffer == NULL)
		return -ENOMEM;

    regcount = (val_size + (UNIPI_MODBUS_REGISTER_SIZE-1)) / UNIPI_MODBUS_REGISTER_SIZE;
	if (unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_READREG, *(u32*)reg_buf, regcount, buffer) != regcount)
		return -EINVAL;

	memmove(val_buf, buffer, val_size);
	kfree(buffer);
	return 0;
}

int unipi_regmap_hw_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct spi_device *spi = context;
	u16 buf[4];
	
	if (unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_READREG, reg, 1, (u8*)buf) != 1)
		return -EINVAL;

	*val = buf[0];
	return 0;
}

int unipi_regmap_hw_gather_write(void *context, const void *reg, size_t reg_size,
                                 const void *val, size_t val_size)
{
	struct spi_device *spi = context;
	u8 *buffer;
	int ret, regcount;

	if (reg_size != sizeof(u16)) return -EINVAL;

	buffer = kzalloc(val_size+4, GFP_KERNEL);
    if (buffer == NULL)
		return -ENOMEM;

    memcpy(buffer, val, val_size);
    regcount = (val_size + (UNIPI_MODBUS_REGISTER_SIZE-1)) / UNIPI_MODBUS_REGISTER_SIZE;
	ret = unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_WRITEREG, *(u32*)reg, regcount, buffer);
	if (ret == regcount) ret = 0;
    kfree(buffer);
    return ret;
}

int unipi_regmap_hw_write(void *context, const void *buf, size_t buf_size)
{
	return unipi_regmap_hw_gather_write(context, buf, sizeof(u16), (u8*)buf + sizeof(u16), buf_size-sizeof(u16));
}

int unipi_regmap_hw_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct spi_device *spi = context;
	if (unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_WRITEREG, reg, 1, (u8*)&val) == 1) 
		return 0;
	return -EINVAL;
	
}

static int unipi_regmap_async_write(void *context,  const void *reg, size_t reg_len,
                            const void *val, size_t val_len,  struct regmap_async *a)
{
	struct spi_device *spi = context;
	struct unipi_regmap_async *async = container_of(a, struct unipi_regmap_async, core);
	u8 *buffer;
	int ret, regcount;

	if (reg_len != sizeof(u16)) return -EINVAL;

	buffer = kzalloc(val_len+4, GFP_KERNEL);
    if (buffer == NULL)
		return -ENOMEM;

	async->buffer = buffer;
    memcpy(buffer, val, val_len);
    regcount = (val_len + (UNIPI_MODBUS_REGISTER_SIZE-1)) / UNIPI_MODBUS_REGISTER_SIZE;
	ret = unipi_spi_write_regs_async(spi, *(u32*)reg, regcount, buffer, async, unipi_regmap_complete);
	if (ret != 0) 
		kfree(buffer);
	return ret;
}

static struct regmap_async *unipi_regmap_async_alloc(void)
{
	struct unipi_regmap_async *async;

	async = kzalloc(sizeof(struct unipi_regmap_async), GFP_KERNEL);
	if (!async)
		return NULL;

	return &async->core;
}

static const struct regmap_range unipi_big_read_ranges[] = {
	{ .range_min = 0,   .range_max = 99 },
	{ .range_min = 500, .range_max = 547 },
	{ .range_min = 1000, .range_max = 1099 },
};

static const struct regmap_access_table unipi_big_read_table = {
	.yes_ranges = unipi_big_read_ranges,
	.n_yes_ranges = ARRAY_SIZE(unipi_big_read_ranges),
};


static const struct regmap_bus unipi_regmap_bus =
{
	.fast_io 					= 0,
	.write 						= unipi_regmap_hw_write, // must be defined else there is some error in regmap
	.gather_write 				= unipi_regmap_hw_gather_write,
	.reg_write					= unipi_regmap_hw_reg_write,
	.async_write				= unipi_regmap_async_write,
	.read						= unipi_regmap_hw_read,
	.reg_read					= unipi_regmap_hw_reg_read,
	.async_alloc				= unipi_regmap_async_alloc,
	.reg_format_endian_default  = REGMAP_ENDIAN_NATIVE, /* REGMAP_ENDIAN_LITTLE */
	.val_format_endian_default  = REGMAP_ENDIAN_NATIVE, /* REGMAP_ENDIAN_LITTLE */
	.max_raw_read				= 64,								// CRC and other overhead not included
	.max_raw_write				= 64,								// CRC and other overhead not included
    //.reg_update_bits            = unipi_regmap_hw_reg_update_bits,
};

static const struct regmap_config unipi_regmap_config_default =
{
		.name 					= "registers",
		.reg_bits				= 16,
		.reg_stride				= 0,
		.pad_bits				= 0,
		.val_bits				= 16,
		.max_register			= 65535,
		.cache_type				= REGCACHE_NONE, //RBTREE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,6)
		.use_single_read			= 0,
		.use_single_write			= 0,
#else
		.use_single_rw			= 0,
#endif
		.can_multi_write		= 0,
		.rd_table				= &unipi_big_read_table,
		.wr_table 				= &unipi_big_read_table,
};


int unipi_regmap_hw_read_coil(void *context, const void *reg_buf, size_t reg_size, void *val_buf, size_t val_size)
{
	struct spi_device *spi = context;
	u32 buffer[2];
	int i;
	
	if (unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_READBIT, *(u16*)reg_buf, 1, (u8*)buffer) < val_size) {
		return -EINVAL;
	}
	for (i=0; i<val_size; i++) {
		((u8*)val_buf)[i] = !!(buffer[0] & (1 << i));
	}
	return 0;
}

int unipi_regmap_hw_reg_read_coil(void *context, unsigned int reg, unsigned int *val)
{
	struct spi_device *spi = context;
	u32 buf[2] = {0,0};

	if (unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_READBIT, reg, 1, (u8*)buf) < 1)
		return -EINVAL;

	*val = buf[0] & 1;
	return 0;
}

int unipi_regmap_hw_reg_write_coil(void *context, unsigned int reg, unsigned int val)
{
	struct spi_device *spi = context;

	unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_WRITEBITS, reg, 1, (u8*)&val);
    return 0;
}

int unipi_regmap_hw_gather_write_coil(void *context, const void *reg, size_t reg_len,
                                 const void *val, size_t val_len)
{
	/* Max coil count is 32 -> use buffer with 32 bits data */
	struct spi_device *spi = context;
	int ret, i;
	u32 buffer[2] = {0,0};

	if (reg_len < sizeof(u16))
		return -EINVAL;
	if (reg_len > sizeof(u16)) {
		val_len = reg_len - sizeof(u16);
		val = (u8*)reg + sizeof(u16);
	}

	for (i=0; i<val_len; i++) {
		if (((u8*)val)[i]!=0) buffer[0] |= 1 << i;
	}
	ret = unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_WRITEBITS, *(u16*)reg, val_len, (u8*)buffer);
    return ret >= 0;
}

int unipi_regmap_hw_write_coil(void *context, const void *buf, size_t buf_size)
{
	return unipi_regmap_hw_gather_write_coil(context, buf, sizeof(u16), (u8*)buf + sizeof(u16), buf_size-sizeof(u16));
}

static int unipi_regmap_async_write_coil(void *context,  const void *reg, size_t reg_len,
                            const void *val, size_t val_len,  struct regmap_async *a)
{
	/* Max coil count is 32 -> use buffer with 32 bits data 
	 * incoming data are bytes oriented - must be compressed 
	 */
	struct spi_device *spi = context;
	struct unipi_regmap_async *async = container_of(a, struct unipi_regmap_async, core);
	u32 buffer[2] = {0,0};
	int ret, i;

	if (reg_len < sizeof(u16)) return -EINVAL;
	if (reg_len > sizeof(u16)) {
		val_len = reg_len - sizeof(u16);
		val = (u8*)reg + sizeof(u16);
	}
	for (i=0; i<val_len; i++) {
		if (((u8*)val)[i]!=0) buffer[0] |= 1 << i;
	}
	//printk("Async r=%d, v=%d l=%ld", *(u16*)reg, buffer[0], val_len);
	async->buffer = NULL;
	ret = unipi_spi_write_bits_async(spi, *(u16*)reg, val_len, (u8*)buffer, async, unipi_regmap_complete);
	return ret;

//	printk("async write coil rlen=%ld vlen=%ld reg=", reg_len, val_len);
/*
	for (i=0; i< reg_len; i++) printk("%02x ", *((u8*)reg+i)); 
	if (val) {
		for (i=0; i< val_len; i++) printk("%02x ", *((u8*)val+i)); 
	}
*/
}


static const struct regmap_range unipi_coil_big_read_ranges[] = {
	{ .range_min = 0,   .range_max = 63 },
	{ .range_min = 1000, .range_max = 1063 },
};

static const struct regmap_access_table unipi_coil_big_read_table = {
	.yes_ranges = unipi_coil_big_read_ranges,
	.n_yes_ranges = ARRAY_SIZE(unipi_coil_big_read_ranges),
};

static const struct regmap_bus unipi_regmap_coil_bus =
{
	.fast_io 					= 0,
	.write						= unipi_regmap_hw_write_coil,
	.gather_write 				= unipi_regmap_hw_gather_write_coil,
	.reg_write					= unipi_regmap_hw_reg_write_coil,
	.async_write				= unipi_regmap_async_write_coil,
	.read						= unipi_regmap_hw_read_coil,
	.reg_read					= unipi_regmap_hw_reg_read_coil,
	.async_alloc				= unipi_regmap_async_alloc,
	.reg_format_endian_default  = REGMAP_ENDIAN_NATIVE, /* REGMAP_ENDIAN_LITTLE */
	.val_format_endian_default  = REGMAP_ENDIAN_NATIVE, /* REGMAP_ENDIAN_LITTLE */
	.max_raw_read				= 32,
	.max_raw_write				= 32,
    //.reg_update_bits            = unipi_regmap_hw_reg_update_bits,
};

static const struct regmap_config unipi_regmap_coil_config_default =
{
		.name 					= "coils",
		.reg_bits				= 16,
		.reg_stride				= 0,
		.pad_bits				= 0,
		.val_bits				= 8,
		.max_register			= 65535,
		.cache_type				= REGCACHE_NONE, //REGCACHE_RBTREE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,6)
		.use_single_read			= 0,
		.use_single_write			= 0,
#else
		.use_single_rw			= 0,
#endif
		.can_multi_write		= 0,
		.rd_table				= &unipi_coil_big_read_table,
		.wr_table 				= &unipi_coil_big_read_table,
};

struct regmap *devm_regmap_init_unipi_regs(struct spi_device *spi,
					const struct regmap_config *config)
{
	return devm_regmap_init(&(spi->dev), &unipi_regmap_bus, spi, config ? config : &unipi_regmap_config_default);
}


struct regmap *devm_regmap_init_unipi_coils(struct spi_device *spi,
					const struct regmap_config *config)
{
	return devm_regmap_init(&(spi->dev), &unipi_regmap_coil_bus, spi, config ? config : &unipi_regmap_coil_config_default);
}

int unipi_reg_read_async(void *context, unsigned int reg, void* cb_data, OperationCallback callback)
{
	struct spi_device *spi = context;
	
    return unipi_spi_read_regs_async(spi, reg, 1, NULL, cb_data, callback);
}

