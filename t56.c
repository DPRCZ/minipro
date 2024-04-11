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
#define T56_READ_USER		 0x06
#define T56_WRITE_USER		 0x07
#define T56_READ_CFG		 0x08
#define T56_WRITE_CFG		 0x09
#define T56_WRITE_USER_DATA	 0x0A
#define T56_READ_USER_DATA	 0x0B
#define T56_WRITE_CODE		 0x0C
#define T56_READ_CODE		 0x0D
#define T56_ERASE		 0x0E
#define T56_READ_DATA		 0x10
#define T56_WRITE_DATA		 0x11
#define T56_WRITE_LOCK		 0x14
#define T56_READ_LOCK		 0x15
#define T56_READ_CALIBRATION	 0x16
#define T56_PROTECT_OFF		 0x18
#define T56_PROTECT_ON		 0x19
#define T56_READ_JEDEC		 0x1D
#define T56_WRITE_JEDEC		 0x1E
#define T56_WRITE_BITSTREAM 0x26
#define T56_LOGIC_IC_TEST_VECTOR 0x28
#define T56_AUTODETECT		 0x37
#define T56_UNLOCK_TSOP48	 0x38
#define T56_REQUEST_STATUS	0x39
#define T56_PIN_DETECTION	0x3E

static int t56_send_bitstream(minipro_handle_t *handle, algorithm_t *algorithm)
{
	uint8_t msg[64];
	/* Send the bitstream algorithm to the T56 */
	msg[0] = T56_WRITE_BITSTREAM;
	format_int(&msg[4], algorithm->length, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8)) {
		return EXIT_FAILURE;
	}
	if (msg_send(handle->usb_handle, algorithm->bitstream,
		     algorithm->length)) {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int t56_begin_transaction(minipro_handle_t *handle)
{
	uint8_t msg[64];
	uint8_t ovc;
	device_t *device = handle->device;
	static uint8_t bitstream_uploaded = 0;

	memset(msg, 0x00, sizeof(msg));

	/* Don't upload the bitstream again if we are in the same session */
	if (!bitstream_uploaded) {
		/* Get the required FPGA bitstream algorithm */
		if (get_algorithm(device, handle->cmdopts->algo_path,
				   handle->icsp, handle->vopt)) {
			fprintf(stderr, "T56 initialization error.\n");
			return EXIT_FAILURE;
		}
		fprintf(stderr, "Using %s algorithm..\n",
			device->algorithm.name);

		if (t56_send_bitstream(handle, &device->algorithm)) {
			free(device->algorithm.bitstream);
			fprintf(stderr,
				"An error occurred while sending bitstream.\n");
			return EXIT_FAILURE;
		}
		bitstream_uploaded = 1;
	}

	/* T56 FPGA was initialized, we can send the normal begin_transaction command */
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

int t56_read_block(minipro_handle_t *handle, uint8_t type,
			   uint32_t addr, uint8_t *buf, size_t len)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_block(handle, type, addr, buf, len);
	}
	uint8_t msg[64];

	if (type == MP_CODE) {
		type = T56_READ_CODE;
	} else if (type == MP_DATA) {
		type = T56_READ_DATA;
	} else if (type == MP_USER) {
		type = T56_READ_USER_DATA;
	} else {
		fprintf(stderr, "Unknown type for read_block (%d)\n", type);
		return EXIT_FAILURE;
	}

	memset(msg, 0x00, sizeof(msg));
	msg[0] = type;
	/* msg[1] = 1; */
	format_int(&(msg[2]), len, 2, MP_LITTLE_ENDIAN);
	format_int(&(msg[4]), addr, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;

	return msg_recv(handle->usb_handle, buf, len);
}

int t56_write_block(minipro_handle_t *handle, uint8_t type,
			    uint32_t addr, uint8_t *buf, size_t len)
{
	if (handle->device->flags.custom_protocol) {
		return bb_write_block(handle, type, addr, buf, len);
	}
	uint8_t msg[64];

	if (type == MP_CODE) {
		type = T56_WRITE_CODE;
	} else if (type == MP_DATA) {
		type = T56_WRITE_DATA;
	} else if (type == MP_USER) {
		type = T56_WRITE_USER_DATA;
	} else {
		fprintf(stderr, "Unknown type for write_block (%d)\n", type);
		return EXIT_FAILURE;
	}

	memset(msg, 0x00, sizeof(msg));
	msg[0] = type;
	format_int(&(msg[2]), len, 2, MP_LITTLE_ENDIAN);
	format_int(&(msg[4]), addr, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_send(handle->usb_handle, buf,
				handle->device->write_buffer_size))
		return EXIT_FAILURE;
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

int t56_erase(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_erase(handle);
	}
	uint8_t msg[64];
	memset(msg, 0, sizeof(msg));
	msg[0] = T56_ERASE;

	fuse_decl_t *fuses = (fuse_decl_t *)handle->device->config;
	if (!fuses || fuses->num_fuses)
		msg[2] = 1;
	else
		msg[2] = (fuses->num_fuses > 4) ? 1 : fuses->num_fuses;

	if (msg_send(handle->usb_handle, msg, 15))
		return EXIT_FAILURE;
	memset(msg, 0x00, sizeof(msg));
	return msg_recv(handle->usb_handle, msg, sizeof(msg));
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

