/*
 * unipi_platform.c
 *
 *  Created on: 23 Feb 2018
 *      Author: Tom Knot <knot@faster.cz>
 */

#include "unipi_platform.h"
#include "unipi_spi.h"
#include "unipi_common.h"

s32 neuronspi_regmap_invalidate(void *data)
{
	int i;
	int freq_cnt = 0;
	while (!kthread_should_stop()) {
		usleep_range(15000,25000);
		if (freq_cnt == 450001) freq_cnt = 0;
		for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
			if (neuronspi_s_dev[i] != NULL) {
				struct neuronspi_driver_data *d_data = spi_get_drvdata(neuronspi_s_dev[i]);
				if (d_data->combination_id == 0xFF) {
					continue;
				}
				neuronspi_regmap_invalidate_device(d_data->reg_map, NEURONSPI_BOARDTABLE[d_data->combination_id].definition, freq_cnt);
			}
		}
		freq_cnt++;
	}
	return 0;
}

void neuronspi_regmap_invalidate_device(struct regmap *reg_map, struct neuronspi_board_combination *device_def, u32 period_counter)
{
	int block_start, block_len, block_counter, current_period, period_len;
	int i;
	block_start = -1;
	block_len = -1;
	block_counter = 0;
	period_len = 1;

	for (i = 0; i < device_def->block_count; i++) {
		if (block_start == -1) {
			block_start = device_def->blocks[i];
			block_counter = 0;
		} else if (block_len == -1) {
			block_len = device_def->blocks[i];
		} else if ((block_counter != block_len)) {
			current_period = device_def->blocks[i] & 0x00FF0000;
			if ((block_counter + 1 != block_len) && ((device_def->blocks[i + 1] & 0x00FF0000) == current_period)) {
				period_len++;
			} else {
				switch (current_period) {
				case NEURONSPI_REGFLAG_ACC_AFAP: {
					regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					break;
				}
				case NEURONSPI_REGFLAG_ACC_10HZ: {
					if ((period_counter) % 5) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_1HZ: {
					if ((period_counter % 50) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_6SEC: {
					if ((period_counter % 300) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_1MIN: {
					if ((period_counter % 3000) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_15MIN: {
					if ((period_counter % 45000) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				default:
					break;
				}
				period_len = 1;
			}
			block_counter++;
		}
		if (block_counter == block_len) {
			block_counter = 0;
			block_start = -1;
			block_len = -1;
		}
	}
}

int neuronspi_regmap_hw_reg_read(void *context, unsigned int reg, unsigned int *val) {
	struct spi_device *spi = context;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u8 *inp_buf;
	u8 *outp_buf;
	int write_length;
	printk(KERN_INFO "NEURONSPI: RM_REG_READ\n");
	write_length = neuronspi_spi_compose_single_register_read(reg, &inp_buf, &outp_buf);
	neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1);
	memcpy(val, &outp_buf[NEURONSPI_HEADER_LENGTH], sizeof(u16));
	kfree(inp_buf);
	kfree(outp_buf);
	return 0;
}

int neuronspi_regmap_hw_write(void *context, const void *data, size_t count) {
	BUG_ON(count < 1);
	return neuronspi_regmap_hw_gather_write(context, data, 1, data + 1, count - 1);
}

int neuronspi_regmap_hw_reg_write(void *context, unsigned int reg, unsigned int val) {
	struct spi_device *spi = context;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u8 *inp_buf;
	u8 *outp_buf;
	int write_length;
	write_length = neuronspi_spi_compose_single_register_write(reg, &inp_buf, &outp_buf, (val >> 8));
	printk(KERN_INFO "HW_REG_WRITE l:%d, r:%d, v:%d\n", write_length, reg, (val >> 8));
	neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1);
	memcpy(&val, &outp_buf[NEURONSPI_HEADER_LENGTH], sizeof(u16));
	kfree(inp_buf);
	kfree(outp_buf);
	return 0;
}

int neuronspi_regmap_hw_gather_write(void *context, const void *reg, size_t reg_size, const void *val, size_t val_size) {
	u16 *mb_reg_buf = (u16*)reg;
	u32 *mb_val_buf = (u32*)val;
	struct spi_device *spi = context;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u8 *inp_buf;
	u8 *outp_buf;
	int i, write_length;
	int block_counter = 0;
	printk(KERN_INFO "HW_REG_GATHER_WRITE:%d, %d, %x, %x\n", val_size, reg_size, mb_reg_buf[0], mb_val_buf[0]);
	if (reg_size == 1) {
		neuronspi_regmap_hw_reg_write(context,mb_reg_buf[0],mb_val_buf[0]);
	} else {
		for (i = 0; i < reg_size; i++) {
			// Swap endianness
			cpu_to_be16s((u16*)&(mb_val_buf[i]));
			// Check for continuity and read the largest possible continuous block
			if (block_counter == (reg_size - 1) || ((mb_reg_buf[i] + 1) != mb_reg_buf[i + 1]))  {
				write_length = neuronspi_spi_compose_multiple_register_write(block_counter, mb_reg_buf[i - block_counter], &inp_buf, &outp_buf,
						                                                     (u8*)(&mb_val_buf[i - block_counter]));
				neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 125, 1);
				block_counter = 0;
				kfree(inp_buf);
				kfree(outp_buf);
			} else {
				block_counter++;
			}
		}
	}
	return 0;
}

int neuronspi_create_reg_starts(struct neuronspi_board_regstart_table *out_table, struct neuronspi_board_combination *board) {
	if (board->features.di_count > 0) {
		out_table->di_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DI_READ);
		out_table->di_counter_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DI_COUNTER_UPPER);
		out_table->di_deboun_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DI_DEBOUNCE);
		out_table->di_direct_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DS_ENABLE);
		out_table->di_polar_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DS_POLARITY);
		out_table->di_toggle_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DS_TOGGLE);
	}
	if (board->features.do_count > 0) {
		out_table->do_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DO_RW);
		out_table->do_pwm_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_PWM_DUTY);
		out_table->do_pwm_c_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_PWM_CYCLE);
		out_table->do_pwm_ps_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_PWM_PRESCALE);
	}
	if (board->features.led_count > 0) {
		out_table->led_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_LED_RW);
	}
	if (board->features.light_count > 0) {
		// TODO: Fill in light bus registers
	}
	if (board->features.ro_count > 0) {
		out_table->ro_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DO_RW);
	}
	if (board->features.sec_ai_count > 0) {
		out_table->sec_ai_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_VER2_READ_LOWER);
		out_table->sec_ai_mode_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_VER2_MODE);
	}
	if (board->features.sec_ao_count > 0) {
		out_table->sec_ao_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_VER2_RW);
	}
	if (board->features.stm_ao_count > 0) {
		out_table->stm_ao_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN);
		out_table->stm_ao_mode_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_MODE);
		out_table->stm_ao_vol_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_V_ERR);
		out_table->stm_ao_vol_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_V_OFF);
		out_table->stm_ao_curr_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_I_ERR);
		out_table->stm_ao_curr_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_I_OFF);
	}
	if (board->features.stm_ai_count > 0) {
		out_table->stm_ai_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN);
		out_table->stm_ai_mode_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_MODE);
		out_table->stm_ai_vol_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_V_ERR);
		out_table->stm_ai_vol_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_V_OFF);
		out_table->stm_ai_curr_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_I_ERR);
		out_table->stm_ai_curr_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_I_OFF);
		out_table->stm_aio_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AIO_BRAIN);
		out_table->stm_aio_vol_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AIO_BRAIN_ERR);
		out_table->stm_aio_vol_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AIO_BRAIN_OFF);
		out_table->vref_inp = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_V_REF_INP);
	}
	out_table->uart_queue_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_TX_QUEUE_LEN);
	out_table->uart_conf_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_RS485_CONFIG);
	out_table->vref_int = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_V_REF_INT);
	out_table->wd_timeout_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_MWD_TO);
	out_table->sys_serial_num = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_SERIAL_NR_LOWER);
	out_table->sys_sw_ver = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_SW_VER);
	out_table->sys_hw_ver = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_HW_VER);
	out_table->sys_hw_flash_ver = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_FLASH_HW_VER);
	return 0;
}

s32 neuronspi_find_reg_start(struct neuronspi_board_combination *board, u16 regfun) {
	int i;
	int block_start = -1;
	int block_len = -1;
	int block_counter = 0;
	for (i = 0; i < board->block_count; i++) {
		if (block_start == -1) {
			block_start = board->blocks[i];
		} else if (block_len == -1) {
			block_len = board->blocks[i];
		} else if ((board->blocks[i] & 0xFFFF) == regfun) {
			//printk(KERN_INFO "NEURONSPI: Reg Start Fun: %x RegS: %d", regfun, block_start + block_counter);
			return block_start + block_counter;
		} else {
			block_counter++;
		}
		if (block_counter == block_len) {
			block_counter = 0;
			block_start = -1;
			block_len = -1;
		}
	}
	return -1;
}

int neuronspi_regmap_hw_read(void *context, const void *reg_buf, size_t reg_size, void *val_buf, size_t val_size) {
	const u16 *mb_reg_buf = reg_buf;
	u16 *mb_val_buf = val_buf;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	u8 *inp_buf;
	u8 *outp_buf;
	int i, write_length;
	int block_counter = 0;
	if (context == NULL) {
		return 0;
	}
	spi = context;
	n_spi = spi_get_drvdata(spi);
	if (n_spi == NULL) {
		return 0;
	}
	for (i = 0; i < (reg_size / 2); i++) {
		// Check for continuity and read the largest possible continuous block
		if (block_counter == ((reg_size / 2) - 1) || ((mb_reg_buf[i] + 1) != mb_reg_buf[i + 1])) {
			write_length = neuronspi_spi_compose_multiple_register_read(block_counter + 1, mb_reg_buf[i - block_counter], &inp_buf, &outp_buf);
			neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 125, 1);
			memcpy(&mb_val_buf[i - block_counter], &outp_buf[NEURONSPI_HEADER_LENGTH], (block_counter + 1) * 2);
			kfree(inp_buf);
			kfree(outp_buf);
			block_counter = 0;
		} else {
			block_counter++;
		}
	}
	printk(KERN_INFO "NEURONSPI: RM_READ %d %x %d %x\n", reg_size, mb_reg_buf[0], val_size, mb_val_buf[0]);
	return 0;
}

s32 neuronspi_find_model_id(u32 probe_count)
{
	struct neuronspi_driver_data *n_spi;
	int i,j, ret = -1;
	u8 *inv = kzalloc(sizeof(*inv) * NEURONSPI_MODELTABLE_LEN, GFP_KERNEL);
	for (i = 0; i < probe_count; i++) {
		if (neuronspi_s_dev[i]) {
			n_spi = spi_get_drvdata(neuronspi_s_dev[i]);
			for (j = 0; j < NEURONSPI_MODELTABLE_LEN; j++) {
				if (i + 1 > NEURONSPI_MODELTABLE[j].combination_count) {
					inv[j] = 1;
				} else if (NEURONSPI_MODELTABLE[j].combinations[i].combination_board_id != n_spi->combination_id) {
					inv[j] = 1;
				}
			}
		} else {
			for (j = 0; j < NEURONSPI_MODELTABLE_LEN; j++) {
				if (i + 1 < NEURONSPI_MODELTABLE[j].combination_count) {
					inv[j] = 1;
				}
			}
		}
	}
	for (i = 0; i < NEURONSPI_MODELTABLE_LEN; i++) {
		if (inv[i] != 1) {
			ret = i;
			break;
		}
	}
	kfree(inv);
	return ret;
}


