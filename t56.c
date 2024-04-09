/*
 * t56.c - Low level ops for T56
 *
 * This file is a part of Minipro.
 *
 * Minipro is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Minipro is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>
#include <time.h>

#include "database.h"
#include "minipro.h"
#include "t56.h"
#include "bitbang.h"
#include "usb.h"

#define T56_BEGIN_TRANS		0x03
#define T56_END_TRANS		0x04
#define T56_READID		0x05
#define T56_WRITE_BITSTREAM 0x26

#define T56_REQUEST_STATUS	0x39

/* clang-format off */
#define ALGO_COUNT (sizeof((t56_algo_table))/(sizeof(t56_algo_table[0])))
static const char t56_algo_table[][32] = {
	"IIC24C",   "MW93ALG", "SPI25F", "AT45D",    "F29EE",	"W29F32P",
	"ROM28P",   "ROM32P",  "ROM40P", "R28TO32P", "ROM24P",	"ROM44",
	"EE28C32P", "RAM32",   "SPI25F", "28F32P",   "FWH",	     "T48",
	"T40A",	   	"T40B",    "T88V",	 "PIC32X",   "P18F87J", "P16F",
	"P18F2",    "P16F5X",  "P16CX",	 "",	     "ATMGA_",	"ATTINY_",
	"AT89P20_", "",	       "AT89C_", "P87C_",    "SST89_",	"W78E_",
	"",	        "",	       "ROM24P", "ROM28P",   "RAM32",	"GAL16",
	"GAL20",	"GAL22",   "NAND_",	 "PIC32X",   "RAM36",	"KB90",
	"EMMC_",    "VGA_",    "CPLD_",	 "GEN_",     "ITE_"
};

/* clang-format on */
int t56_begin_transaction(minipro_handle_t *handle)
{
	uint8_t msg[64];
	uint8_t ovc;
	device_t *device = handle->device;

	/* Get the required FPGA bitstream algorithm */
	uint8_t algo_number = (uint8_t)(device->variant >> 8);
	if (algo_number == 0 || device->protocol_id > ALGO_COUNT ||
	    !*t56_algo_table[device->protocol_id]) {
		fprintf(stderr, "Invalid algorithm number found.\n");
		return EXIT_FAILURE;
	}

	db_data_t db_data;
	memset(&db_data, 0, sizeof(db_data));
	db_data.algo_path = handle->cmdopts->algo_path;

	char algo_name[64];
	snprintf(algo_name, sizeof(algo_name), "%s%02X",
		 t56_algo_table[device->protocol_id - 1], algo_number);
	db_data.device_name = algo_name;

	algorithm_t *algorithm = get_algorithm(&db_data);
	if (!algorithm){
		fprintf(stderr, "T56 initialization error.\n");
		return EXIT_FAILURE;
	}

	fprintf(stderr, "Using %s algorithm..\n", db_data.device_name);;

	memset(msg, 0x00, sizeof(msg));

	/* Send the bitstream algorithm to the T56 */
	msg[0] = T56_WRITE_BITSTREAM;
	format_int(&msg[4], algorithm->length, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8)){
		free(algorithm->bitstream);
		free(algorithm);
		return EXIT_FAILURE;
	}
	if (msg_send(handle->usb_handle, algorithm->bitstream,
		     algorithm->length)){
		free(algorithm->bitstream);
		free(algorithm);
		return EXIT_FAILURE;
	}

	if (!handle->device->flags.custom_protocol) {
		msg[0] = T56_BEGIN_TRANS;
		msg[1] = device->protocol_id;
		msg[2] = (uint8_t)handle->device->variant;
		msg[3] = handle->icsp;

		format_int(&(msg[4]), device->voltages.raw_voltages, 2,
			   MP_LITTLE_ENDIAN);
		msg[6] = (uint8_t)device->chip_info;
		msg[7] = (uint8_t)device->pin_map;
		format_int(&(msg[8]), device->data_memory_size, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[10]), device->page_size, 2, MP_LITTLE_ENDIAN);
		format_int(&(msg[12]), device->pulse_delay, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[14]), device->data_memory2_size, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[16]), device->code_memory_size, 4,
			   MP_LITTLE_ENDIAN);

		msg[20] = (uint8_t)(device->voltages.raw_voltages >> 16);

		if ((device->voltages.raw_voltages & 0xf0) == 0xf0) {
			msg[22] = (uint8_t)device->voltages.raw_voltages;
		} else {
			msg[21] = (uint8_t)device->voltages.raw_voltages & 0x0f;
			msg[22] = (uint8_t)device->voltages.raw_voltages & 0xf0;
		}
		if (device->voltages.raw_voltages & 0x80000000)
			msg[22] = (device->voltages.raw_voltages >> 16) & 0x0f;

		format_int(&(msg[40]),
			   handle->device->package_details.packed_package, 4,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[44]), handle->device->read_buffer_size, 2,
			   MP_LITTLE_ENDIAN);
		format_int(&(msg[56]), handle->device->flags.raw_flags, 4,
			   MP_LITTLE_ENDIAN);

		if (msg_send(handle->usb_handle, msg, sizeof(msg)))
			return EXIT_FAILURE;
	} else {
		if (bb_begin_transaction(handle)) {
			return EXIT_FAILURE;
		}
	}

	if (t56_get_ovc_status(handle, NULL, &ovc))
		return EXIT_FAILURE;
	if (ovc) {
		fprintf(stderr, "Overcurrent protection!\007\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}


int t56_end_transaction(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_end_transaction(handle);
	}
	uint8_t msg[8];
	memset(msg, 0x00, sizeof(msg));
	msg[0] = T56_END_TRANS;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
	return EXIT_SUCCESS;
}


int t56_get_chip_id(minipro_handle_t *handle, uint8_t *type,
		uint32_t *device_id)
{
	if (handle->device->flags.custom_protocol) {
		return bb_get_chip_id(handle, device_id);
	}
	uint8_t msg[32], format, id_length;
	memset(msg, 0xd0, sizeof(msg));
	msg[0] = T56_READID;
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;

	*type = msg[0]; /* The Chip ID type (1-5) */

	format = (*type == MP_ID_TYPE3 || *type == MP_ID_TYPE4 ?
			  MP_LITTLE_ENDIAN :
			  MP_BIG_ENDIAN);

	/* The length byte is always 1-4 but never know,
	 * truncate to max. 4 bytes. */
	id_length = handle->device->chip_id_bytes_count > 4 ?
			    4 :
			    handle->device->chip_id_bytes_count;
	*device_id = (id_length ? load_int(&(msg[2]), id_length, format) :
				  0); /* Check for positive length. */
	return EXIT_SUCCESS;
}


int t56_get_ovc_status(minipro_handle_t *handle,
		minipro_status_t *status, uint8_t *ovc)
{
	uint8_t msg[32];
	memset(msg, 0, sizeof(msg));
	msg[0] = T56_REQUEST_STATUS;
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;
	if (status && !handle->device->flags.custom_protocol) {
		/* This is verify while writing feature. */
		status->error = msg[0];
		status->address = load_int(&msg[8], 4, MP_LITTLE_ENDIAN);
		status->c1 = load_int(&msg[2], 2, MP_LITTLE_ENDIAN);
		status->c2 = load_int(&msg[4], 2, MP_LITTLE_ENDIAN);
	}
	*ovc = msg[12]; /* return the ovc status */
	return EXIT_SUCCESS;
}

