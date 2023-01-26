/*
 * eestruct.h
 *
 *  Created on: Nov 12, 2021
 *      Author: martyy
 */

#ifndef UNIEE_H_
#define UNIEE_H_

#if defined (CONFIG_MODULES) || defined (CONFIG_SYS_BOARD)
 #include <linux/types.h>
 #include <linux/string.h>
 #include <asm/byteorder.h>
 #define htobe16(x) cpu_to_be16(x)
#else
 #include <stdint.h>
 #include <string.h>
 #include <endian.h>
#endif

 #include "uniee_values.h"

#define UNIEE_SIGNATURE htobe16(0xFA55)
#define UNIEE_MIN_EE_SIZE 128
#define UNIEE_MAX_EE_SIZE 256
#define UNIEE_BANK_CNT 4
#define UNIEE_BANK_SIZE 32
#define UNIEE_INFO_AREA_SIZE (2*UNIEE_BANK_SIZE)
#define UNIEE_MAX_SPECDATA_AREA_SIZE (UNIEE_MAX_EE_SIZE - UNIEE_INFO_AREA_SIZE)


typedef struct __attribute__((__packed__)){
	uint8_t major;
	uint8_t minor;
}version16_t;

typedef struct __attribute__((__packed__)){
	uint8_t bitmask;
	uint8_t bitmask_inverted;
}mervic_options_t;

typedef union __attribute__((__packed__)){
	struct {
		uint8_t platform_family;
		uint8_t platform_series;
	} parsed;
	uint16_t raw_id;
}platform_id_t;

typedef struct __attribute__((__packed__)){
	uint8_t field_type;
	uint8_t field_len;
} specdata_header_t;

typedef uint16_t board_model_t;

/* Bank 0 - Board specific data*/
typedef struct{
	uint8_t dummy_data [32];
}uniee_bank_0_t;

/* Bank 1 - Board specific data */
typedef struct {
	uint8_t dummy_data [32];
}uniee_bank_1_t;

/* Bank 2 - Board info */
typedef struct __attribute__((__packed__)){
	uint32_t board_serial;
	version16_t board_version;
	board_model_t board_model;
	specdata_header_t specdata_headers_table[12];
}uniee_bank_2_t;

typedef struct __attribute__((__packed__)){
	uint32_t board_serial;
	version16_t board_version;
	int16_t rtc_calibration;
	board_model_t board_model;
	specdata_header_t specdata_headers_table[11];
}uniee_bank_2_legacy_t;

/* Bank 3 - Product info */
typedef struct __attribute__((__packed__)){
	uint16_t ee_signature;
	version16_t product_version;
	uint32_t product_serial;
	mervic_options_t mervis_license;
	uint8_t model_str [6];
	uint32_t sku;
	uint8_t dummy_data [8];
	platform_id_t platform_id;
	uint16_t checksum;
}uniee_bank_3_t;


///* 128 bytes eeprom space */
//typedef struct {
//	//uniee_bank_0_t bank_0;
//	//uniee_bank_1_t bank_1;
//	uint8_t board_specific_area[64];
//	uniee_bank_2_t bank_2;
//	uniee_bank_3_t bank_3;
//}uniee_content_128_t;
//

///* 256 bytes eeprom space */
//typedef struct {
//	//uniee_bank_0_t bank_0;
//	//uniee_bank_1_t bank_1;
//	uint8_t board_specific_area[64+128];
//	uniee_bank_2_t bank_2;
//	uniee_bank_3_t bank_3;
//}uniee_content_256_t;

/* 256 bytes eeprom space */
/*
typedef struct {
	//uniee_bank_0_t bank_0;
	//uniee_bank_1_t bank_1;
	//uint8_t board_specific_area[64+128];
	uint8_t board_specific_area[];
	uniee_bank_2_t bank_2;
	uniee_bank_3_t bank_3;
}uniee_content_t;
*/

typedef struct {
	uniee_bank_2_t board_info;
	uniee_bank_3_t product_info;
}uniee_descriptor_area;


/**
 * Compute checksum from loaded EEPROM data and puts it to the right place in the buffer.
 * This function does not write the checksum physically to the memory, @see uniee_write_bank()
 * @return Checksum value
 */
uint16_t uniee_checksum_fill(void);

/**
 * Compute checksum from loaded EEPROM data and compare with the stored value.
 * @return Zero if success, -1 otherwise
 */
int uniee_checksum_verify(void);

/**
 * Check if EEPROM is virgin
 * @return
 */
int uniee_is_empty(void);

int uniee_fix_content(void);


static inline uniee_descriptor_area* uniee_get_valid_descriptor(uint8_t* buff, int size)
{
	uniee_descriptor_area *descriptor;
	if (size < sizeof(uniee_descriptor_area) || buff==NULL){
		// EEprom has to have minimal size for descriptor area
		return NULL;
	}

	/* ToDo: check alignment or verify size leading banks */
	descriptor = (uniee_descriptor_area*) (buff + (size - sizeof(uniee_descriptor_area)));
	if (descriptor->product_info.ee_signature != UNIEE_SIGNATURE){
		return NULL;
	}
	/* ToDo: softly check CRC */

	return descriptor;
}

static inline void uniee_fix_legacy_content(uint8_t* buff, int size, uniee_descriptor_area *descriptor)
{
	/* Check and fix mervis option */
	if ((~descriptor->product_info.mervis_license.bitmask) != descriptor->product_info.mervis_license.bitmask_inverted) {
		descriptor->product_info.mervis_license.bitmask = 0;
		descriptor->product_info.mervis_license.bitmask_inverted = 0xff;
	}
	/* Fix platform */
	if (descriptor->product_info.platform_id.raw_id == 0xffff) {
		char firstch = descriptor->product_info.model_str[0];
		char proc_ch = descriptor->product_info.model_str[3];
		if ((firstch=='S')||(firstch=='M')||(firstch=='L')) {
			if (proc_ch=='3')
				descriptor->product_info.platform_id.parsed.platform_family = UNIEE_PLATFORM_FAMILY_NEURON;
			else if (proc_ch=='5')
				descriptor->product_info.platform_id.parsed.platform_family = UNIEE_PLATFORM_FAMILY_AXON;
			else if (proc_ch=='7')
				descriptor->product_info.platform_id.parsed.platform_family = UNIEE_PLATFORM_FAMILY_PATRON;
		} else if ((firstch=='G')) {
			descriptor->product_info.platform_id.parsed.platform_family = UNIEE_PLATFORM_FAMILY_G1XX;
		}
	} else if (descriptor->product_info.platform_id.raw_id == 0x0000) {
		/* probably Unipi 1 */
		if ((descriptor->product_info.product_version.major==1)
			&& ((descriptor->product_info.product_version.minor==1)
			  ||(descriptor->product_info.product_version.minor==0x11))) {
			descriptor->product_info.platform_id.parsed.platform_family = UNIEE_PLATFORM_FAMILY_UNIPI1;
			/* Fix AI1 AI2 calibration for unipi 1 */
			memcpy(buff, &descriptor->product_info.sku, 2*sizeof(float));
			descriptor->product_info.sku = 0xffffffff;
			descriptor->board_info.specdata_headers_table[0].field_type = UNIEE_FIELD_TYPE_AICAL;
			descriptor->board_info.specdata_headers_table[0].field_len = sizeof(float);
			descriptor->board_info.specdata_headers_table[1].field_type = UNIEE_FIELD_TYPE_AICAL;
			descriptor->board_info.specdata_headers_table[1].field_len = sizeof(float);
		}
	}
	/* Fix rtc_calibration on old G1xx and Iris proto-motherboards */
	if ((descriptor->board_info.specdata_headers_table[0].field_type < 4) &&
	    (descriptor->board_info.specdata_headers_table[0].field_type != 0) &&
	    (descriptor->board_info.specdata_headers_table[0].field_len == 0)) {
		board_model_t rtc_cal = descriptor->board_info.board_model;
		descriptor->board_info.board_model = descriptor->board_info.specdata_headers_table[0].field_type;
		descriptor->board_info.specdata_headers_table[0].field_type = UNIEE_FIELD_TYPE_RTC;
		descriptor->board_info.specdata_headers_table[0].field_len = sizeof(rtc_cal);
		memcpy(buff, &rtc_cal, sizeof(rtc_cal));
	}
}


#endif /* UNIEE_H_ */
