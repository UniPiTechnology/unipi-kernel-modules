/*
 *
 * Copyright (c) 2021  Faster CZ, ondra@faster.cz
 *
 * SPDX-License-Identifier: LGPL-2.1+
 *
 */

#ifndef UNIPI_ID_H_
#define UNIPI_ID_H_

#ifdef CONFIG_MODULES
 #include <linux/types.h>
 #include <linux/string.h>
 #include <asm/byteorder.h>
#else
 #include <sys/types.h>
 #include <string.h>
 #include <stdint.h>
#endif

#include "uniee.h"

#define UNIPI_ID_MAX_IDS     8
#define UNIPI_ID_FINGERPRINT_SIZE     20

/*
typedef struct {
//	const char* platform_family_name;
	uint8_t     platform_family;
	uint8_t     platform_series;
	uint16_t    platform_raw_id;

	uint8_t     product_version_major;
	uint8_t     product_version_minor;
	uint32_t  product_serial;
	uint8_t   options;
	char      model[8];
	uint32_t  sku;
} unipi_description_t;

typedef struct {
	char      model[8];
	uint32_t  sku;

	uint32_t  board_serial;
	uint8_t   board_version_major;
	uint8_t   board_version_minor;
	uint16_t  board_model;
} unipi_board_description_t;
*/

#ifdef CONFIG_MODULES
#include <linux/i2c.h>

struct unipi_id_data 
{
	int slot_count;
	struct i2c_client * loaded_clients[UNIPI_ID_MAX_IDS];
	const struct unipi_id_family_data *family_data;
	uniee_descriptor_area descriptor;
	int active_slot[UNIPI_ID_MAX_IDS];
	int fingerprint_loaded;
	uint8_t fingerprint[UNIPI_ID_FINGERPRINT_SIZE];
	uniee_descriptor_area *loaded_descriptor[UNIPI_ID_MAX_IDS];
};

const char* unipi_id_get_family_name(struct unipi_id_data *unipi_id);

#endif

#endif /* UNIPI_ID_H_*/
