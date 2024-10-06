/*
 * t48.c - Low level ops for T48
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
#include "t48.h"
#include "bitbang.h"
#include "usb.h"

#define T48_NAND_INIT		 0x02
#define T48_BEGIN_TRANS		 0x03
#define T48_END_TRANS		 0x04
#define T48_READID		 0x05
#define T48_READ_USER		 0x06
#define T48_WRITE_USER		 0x07
#define T48_READ_CFG		 0x08
#define T48_WRITE_CFG		 0x09
#define T48_WRITE_USER_DATA	 0x0A
#define T48_READ_USER_DATA	 0x0B
#define T48_WRITE_CODE		 0x0C
#define T48_READ_CODE		 0x0D
#define T48_ERASE		 0x0E
#define T48_READ_DATA		 0x10
#define T48_WRITE_DATA		 0x11
#define T48_WRITE_LOCK		 0x14
#define T48_READ_LOCK		 0x15
#define T48_READ_CALIBRATION	 0x16
#define T48_PROTECT_OFF		 0x18
#define T48_PROTECT_ON		 0x19
#define T48_READ_JEDEC		 0x1D
#define T48_WRITE_JEDEC		 0x1E
#define T48_LOGIC_IC_TEST_VECTOR 0x28
#define T48_AUTODETECT		 0x37
#define T48_UNLOCK_TSOP48	 0x38
#define T48_REQUEST_STATUS	 0x39

#define T48_BOOTLOADER_WRITE	 0x3B
#define T48_BOOTLOADER_ERASE	 0x3C
#define T48_SWITCH		 0x3D

/* Hardware Bit Banging */
#define T48_SET_VCC_VOLTAGE	 0x1B
#define T48_SET_VPP_VOLTAGE	 0x1C
#define T48_RESET_PIN_DRIVERS	 0x2D
#define T48_SET_VCC_PIN		 0x2E
#define T48_SET_VPP_PIN		 0x2F
#define T48_SET_GND_PIN		 0x30
#define T48_SET_PULLDOWNS	 0x31
#define T48_SET_PULLUPS		 0x32
#define T48_SET_DIR		 0x34
#define T48_READ_PINS		 0x35
#define T48_SET_OUT		 0x36

/* Firmware */
#define T48_BTLDR_MAGIC  0xCDEF89AB45670123


/* #define DEBUG_USB_MSG */
#ifdef DEBUG_USB_MSG
static void print_msg(uint8_t *buffer, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		printf("%.2x ", buffer[i]);
		if (i % 16 == 15)
			printf("\n");
	}
	printf("---\n");
}

static int wrap_msg_recv(void *handle, uint8_t *buffer, size_t size)
{
	printf("%s\n", __func__);
	int status = msg_recv(handle, buffer, size);
	print_msg(buffer, size);
	return status;
}

static int wrap_msg_send(void *handle, uint8_t *buffer, size_t size)
{
	printf("%s\n", __func__);
	print_msg(buffer, size);
	return msg_send(handle, buffer, size);
}

#define msg_send wrap_msg_send
#define msg_recv wrap_msg_recv
#endif

/* clang-format on */
int t48_begin_transaction(minipro_handle_t *handle)
{
	uint8_t msg[64];
	uint8_t ovc;
	device_t *device = handle->device;

	memset(msg, 0x00, sizeof(msg));
	if (!handle->device->flags.custom_protocol) {
		msg[0] = T48_BEGIN_TRANS;
		msg[1] = device->protocol_id;
		msg[2] = (uint8_t)handle->device->variant;
		msg[3] = handle->cmdopts->icsp;

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
	if (t48_get_ovc_status(handle, NULL, &ovc))
		return EXIT_FAILURE;
	if (ovc) {
		fprintf(stderr, "Overcurrent protection!\007\n");
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int t48_end_transaction(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_end_transaction(handle);
	}
	uint8_t msg[8];
	memset(msg, 0x00, sizeof(msg));
	msg[0] = T48_END_TRANS;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
	return EXIT_SUCCESS;
}

int t48_read_block(minipro_handle_t *handle, uint8_t type,
			   uint32_t addr, uint8_t *buf, size_t len)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_block(handle, type, addr, buf, len);
	}
	uint8_t msg[64];

	if (type == MP_CODE) {
		type = T48_READ_CODE;
	} else if (type == MP_DATA) {
		type = T48_READ_DATA;
	} else if (type == MP_USER) {
		type = T48_READ_USER_DATA;
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

	return read_payload2(handle->usb_handle, buf, len, 0);
}

int t48_write_block(minipro_handle_t *handle, uint8_t type,
			    uint32_t addr, uint8_t *buf, size_t len)
{
	if (handle->device->flags.custom_protocol) {
		return bb_write_block(handle, type, addr, buf, len);
	}
	uint8_t msg[64];

	if (type == MP_CODE) {
		type = T48_WRITE_CODE;
	} else if (type == MP_DATA) {
		type = T48_WRITE_DATA;
	} else if (type == MP_USER) {
		type = T48_WRITE_USER_DATA;
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
	if (write_payload2(handle->usb_handle, buf,
				handle->device->write_buffer_size, 0))
		return EXIT_FAILURE; /* And payload to the endp.2 */
	return EXIT_SUCCESS;
}

int t48_read_fuses(minipro_handle_t *handle, uint8_t type,
			   size_t length, uint8_t items_count, uint8_t *buffer)
{
	if (handle->device->flags.custom_protocol) {
		return bb_read_fuses(handle, type, length, items_count, buffer);
	}
	uint8_t msg[64];

	if (type == MP_FUSE_USER) {
		type = T48_READ_USER;
	} else if (type == MP_FUSE_CFG) {
		type = T48_READ_CFG;
	} else if (type == MP_FUSE_LOCK) {
		type = T48_READ_LOCK;
	} else {
		fprintf(stderr, "Unknown type for read_fuses (%d)\n", type);
		return EXIT_FAILURE;
	}

	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	msg[1] = handle->device->protocol_id;
	msg[2] = items_count;
	format_int(&msg[4], handle->device->code_memory_size, 4,
		   MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, sizeof(msg)))
		return EXIT_FAILURE;
	memcpy(buffer, &(msg[8]), length);
	return EXIT_SUCCESS;
}

int t48_write_fuses(minipro_handle_t *handle, uint8_t type,
			    size_t length, uint8_t items_count, uint8_t *buffer)
{
	if (handle->device->flags.custom_protocol) {
		return bb_write_fuses(handle, type, length, items_count,
				      buffer);
	}
	uint8_t msg[64];

	if (type == MP_FUSE_USER) {
		type = T48_WRITE_USER;
	} else if (type == MP_FUSE_CFG) {
		type = T48_WRITE_CFG;
	} else if (type == MP_FUSE_LOCK) {
		type = T48_WRITE_LOCK;
	} else {
		fprintf(stderr, "Unknown type for write_fuses (%d)\n", type);
	}

	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	if (buffer) {
		msg[1] = handle->device->protocol_id;
		msg[2] = items_count;
		format_int(&msg[4], handle->device->code_memory_size - 0x38, 4,
			   MP_LITTLE_ENDIAN); /* 0x38, firmware bug? */
		memcpy(&(msg[8]), buffer, length);
	}
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

int t48_get_chip_id(minipro_handle_t *handle, uint8_t *type,
			    uint32_t *device_id)
{
	if (handle->device->flags.custom_protocol) {
		return bb_get_chip_id(handle, device_id);
	}
	uint8_t msg[32], format, id_length;
	memset(msg, 0x00, sizeof(msg));
	msg[0] = T48_READID;
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

int t48_spi_autodetect(minipro_handle_t *handle, uint8_t type,
			       uint32_t *device_id)
{
	if (handle->device != NULL && handle->device->flags.custom_protocol) {
		return bb_spi_autodetect(handle, type, device_id);
	}
	uint8_t msg[64];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_AUTODETECT;
	msg[8] = type;
	if (msg_send(handle->usb_handle, msg, 10))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, 32))
		return EXIT_FAILURE;
	*device_id = load_int(&(msg[2]), 3, MP_BIG_ENDIAN);
	return EXIT_SUCCESS;
}

int t48_erase(minipro_handle_t *handle)
{
	if (handle->device->flags.custom_protocol) {
		return bb_erase(handle);
	}
	uint8_t msg[64];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_ERASE;

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

int t48_protect_off(minipro_handle_t *handle)
{
	uint8_t msg[8];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_PROTECT_OFF;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

int t48_protect_on(minipro_handle_t *handle)
{
	uint8_t msg[8];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_PROTECT_ON;
	return msg_send(handle->usb_handle, msg, sizeof(msg));
}

int t48_get_ovc_status(minipro_handle_t *handle,
			       minipro_status_t *status, uint8_t *ovc)
{
	uint8_t msg[32];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_REQUEST_STATUS;
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

int t48_write_jedec_row(minipro_handle_t *handle, uint8_t *buffer,
				uint8_t row, uint8_t flags, size_t size)
{
	/* Warning: Using JEDEC algorithm for TL866ii+. Results in T48 may be unexpected */
	if (handle->device->flags.custom_protocol) {
		return bb_write_jedec_row(handle, buffer, row, flags, size);
	}
	uint8_t msg[64];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_WRITE_JEDEC;
	msg[1] = handle->device->protocol_id;
	msg[2] = size;
	msg[4] = row;
	msg[5] = flags;
	memcpy(&msg[8], buffer, (size + 7) / 8);
	return msg_send(handle->usb_handle, msg, 64);
}

int t48_read_jedec_row(minipro_handle_t *handle, uint8_t *buffer,
			       uint8_t row, uint8_t flags, size_t size)
{
	/* Warning: Using JEDEC algorithm for TL866ii+. Results in T48 may be unexpected */
	if (handle->device->flags.custom_protocol) {
		return bb_read_jedec_row(handle, buffer, row, flags, size);
	}
	uint8_t msg[32];
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_READ_JEDEC;
	msg[1] = handle->device->protocol_id;
	msg[2] = size;
	msg[4] = row;
	msg[5] = flags;
	if (msg_send(handle->usb_handle, msg, 8))
		return EXIT_FAILURE;
	if (msg_recv(handle->usb_handle, msg, 32))
		return EXIT_FAILURE;
	memcpy(buffer, msg, (size + 7) / 8);
	return EXIT_SUCCESS;
}

/* Pull: 0=Pull-up, 1=Pull-down */
static uint8_t *do_ic_test(minipro_handle_t *handle, int pull)
{
	uint8_t *vector = handle->device->vectors;
	uint8_t msg[32];
	uint8_t *result;
	uint8_t pin_count = handle->device->package_details.pin_count;

	result = calloc(pin_count, handle->device->vector_count);
	if (!result)
		return NULL;

	uint8_t *out = result;
	int n;
	for (n = 0; n < handle->device->vector_count; n++) {
		memset(msg, 0xff, sizeof(msg));

		msg[0] = T48_LOGIC_IC_TEST_VECTOR;
		msg[1] = handle->device->voltages.vcc;
		msg[1] |= pull << 7; /* Set the pull-up/pull-down */
		format_int(&msg[2], pin_count, 2, MP_LITTLE_ENDIAN);
		format_int(&msg[4], n, 4, MP_LITTLE_ENDIAN);

		int i;
		/* Pack the vector to 2 pin/byte */
		for (i = 0; i < handle->device->package_details.pin_count;
		     i++) {
			if (i & 1)
				msg[8 + i / 2] |= *vector << 4;
			else
				msg[8 + i / 2] = *vector;
			vector++;
		}

		/* Send the test vector and read the pin status */
		if (msg_send(handle->usb_handle, msg, sizeof(msg))) {
			free(result);
			return NULL;
		}
		if (msg_recv(handle->usb_handle, msg, sizeof(msg))) {
			free(result);
			return NULL;
		}

		/* Unpack the result from 2 pin/byte to 1 pin/byte */
		for (i = 0; i < handle->device->package_details.pin_count; i++)
			*out++ = (msg[8 + i / 2] >> (4 * (i & 1))) & 0xf;
	}

	return result;
}

/* Performing a logic test. This is accomplished in two steps.
 * The first step will set a pull-up resistor on all chip outputs (L, H, Z).
 * The second step will set a pull-down resistor on all chip outputs.
 * According to the vector table then each output is compared against the two
 * result array. Considering the weak pull-up/pull-down resistors we can detect
 * L(low) state as 0 in both steps, H(high) state as 1 in both steps and
 * Z(high impedance) state as 1 in step 1 when the pull-up is activated and
 * 0 in step 2 when the pull-down is activated.
 * While for chips with open collector/open drain output we need to perform
 * these two steps to detect the Z state, for chips with totem-pole outputs
 * this is not really necessary but, sometimes internal issues can be
 * detected this way like burned H side or L side output transistors.
 * The C (clock) state is performed in firmware by first pulsing the pin
 * marked as C and then all pins are read back.
 * The X (don't care) state will leave the pin unconnected.
 * The V (VCC) and G (Ground) state will designate the power supply pins.
 */

int t48_logic_ic_test(minipro_handle_t *handle)
{
	uint8_t *vector = handle->device->vectors;
	uint8_t *first_step = NULL;
	uint8_t *second_step = NULL;
	int ret = EXIT_FAILURE;

	if (!(first_step = do_ic_test(handle, 0))) { /* Pull-up active */
		fprintf(stderr,
			"Error running the first step of logic test.\n");
	} else if (!(second_step = do_ic_test(handle, 1))) { /* Pull-down active */
		fprintf(stderr,
			"Error running the second step of logic test.\n");
	} else if (handle->cmdopts->logicic_out) {
		ret = write_logic_file(handle, first_step, second_step);
	} else {
		int errors = 0, err;
		static const char pst[] = "01LHCZXGV";
		size_t n = 0;

		printf("      ");
		for (int pin = 1;
		     pin <= handle->device->package_details.pin_count; pin++)
			printf("%-3d", pin);
		putchar('\n');

		for (int i = 0; i < handle->device->vector_count; i++) {
			printf("%04d: ", i);
			for (int pin = 0;
			     pin < handle->device->package_details.pin_count;
			     pin++) {
				err = 0;
				switch (vector[n]) {
				case LOGIC_L: /* Pin must be 0 in both steps */
					if (first_step[n] || second_step[n])
						err = 1;
					break;
				case LOGIC_H: /* Pin must be 1 in both steps */
					if (!first_step[n] || !second_step[n])
						err = 1;
					break;
				case LOGIC_Z: /* Pin must be 1 in step 1 and 0 in step 2 */
					if (!first_step[n] || second_step[n])
						err = 1;
					break;
				}
				printf("%s%c%c ", err ? "\e[0;91m" : "\e[0m",
				       pst[vector[n]], err ? '-' : ' ');
				errors += err;
				n++;
			}
			printf("\e[0m\n");
		}

		if (errors) {
			fprintf(stderr,
				"Logic test failed: %d errors encountered.\n",
				errors);
		} else {
			fprintf(stderr, "Logic test successful.\n");
			ret = EXIT_SUCCESS;
		}
	}

	free(second_step);
	free(first_step);
	if (t48_end_transaction(handle))
		return EXIT_FAILURE;

	return ret;
}


/*****************************************************************************
 * Firmware updater section
 *****************************************************************************

This is the updateT48.dat file structure.
It has a fixed 16 byte header, followed by a number of 276-byte blocks
|============|===========|============|==============|
|File version| File CRC  | Unknown    | Blocks count |
|============|===========|============|==============|
|  4 bytes   | 4 bytes   | 4 bytes    | 4 bytes      |
|============|===========|============|==============|
|  offset 0  | offset 4  | offset 8   | offset 12    |
|============|===========|============|==============|

*/

/* Performing a firmware update */
int t48_firmware_update(minipro_handle_t *handle, const char *firmware)
{
	uint8_t msg[288];

	struct stat st;
	if (stat(firmware, &st)) {
		fprintf(stderr, "%s open error!: ", firmware);
		perror("");
		return EXIT_FAILURE;
	}

	off_t file_size = st.st_size;
	/* Check the updateT48.dat size */
	if (file_size < 16 || file_size > 1048576) {
		fprintf(stderr, "%s file size error!\n", firmware);
		return EXIT_FAILURE;
	}

	/* Open the updateT48.dat firmware file */
	FILE *file = fopen(firmware, "rb");
	if (!file) {
		fprintf(stderr, "%s open error!: ", firmware);
		perror("");
		return EXIT_FAILURE;
	}
	uint8_t *update_dat = malloc(file_size);
	if (!update_dat) {
		fprintf(stderr, "Out of memory!\n");
		fclose(file);
		return EXIT_FAILURE;
	}

	/* Read the updateT48.dat file */
	if (fread(update_dat, sizeof(char), st.st_size, file) != st.st_size) {
		fprintf(stderr, "%s file read error!\n", firmware);
		fclose(file);
		free(update_dat);
		return EXIT_FAILURE;
	}
	fclose(file);

	uint32_t version = load_int(update_dat, 4, MP_LITTLE_ENDIAN);
	if ((version & 0xFFFF0000) != 0xf0480000) {
		fprintf(stderr, "%s file version error!\n", firmware);
		free(update_dat);
		return EXIT_FAILURE;
	}
	version &= 0x0000FFFF;

	/* Read the blocks count and check if correct */
	uint32_t blocks = load_int(update_dat + 12, 4, MP_LITTLE_ENDIAN);
	if (blocks * 0x114 + 16 != file_size) {
		fprintf(stderr, "%s file size error!\n", firmware);
		free(update_dat);
		return EXIT_FAILURE;
	}

	/* Compute the file CRC and compare */
	uint32_t crc = crc_32(update_dat + 16, blocks * 0x114, 0xFFFFFFFF);
	if (crc != load_int(update_dat + 4, 4, MP_LITTLE_ENDIAN)) {
		fprintf(stderr, "%s file CRC error!\n", firmware);
		free(update_dat);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "%s contains firmware version %02u.%02u.%02u", firmware,
		0, (version >> 8) & 0xFF, (version & 0xFF));

	if (handle->firmware > version)
		fprintf(stderr, " (older)");
	else if (handle->firmware < version)
		fprintf(stderr, " (newer)");

	fprintf(stderr, "\n\nDo you want to continue with firmware update? y/n:");
	fflush(stderr);
	char c = getchar();
	if (c != 'Y' && c != 'y') {
		free(update_dat);
		fprintf(stderr, "Firmware update aborted.\n");
		return EXIT_FAILURE;
	}

	/* Switching to boot mode if necessary */
	if (handle->status == MP_STATUS_NORMAL) {
		fprintf(stderr, "Switching to bootloader... ");
		fflush(stderr);

		memset(msg, 0, sizeof(msg));
		msg[0] = T48_SWITCH;
		format_int(&msg[8], T48_BTLDR_MAGIC, 8, MP_LITTLE_ENDIAN);
		if (msg_send(handle->usb_handle, msg, 16)) {
			free(update_dat);
			return EXIT_FAILURE;
		}

		memset(msg, 0, sizeof(msg));
		if (msg_recv(handle->usb_handle, msg, 32)) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (msg[0]) {
			fprintf(stderr, "failed (code: %d)!\n", msg[0]);
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (minipro_reset(handle)) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		handle = minipro_open(VERBOSE);
		if (!handle) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (handle->status == MP_STATUS_NORMAL) {
			fprintf(stderr, "failed!\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		fprintf(stderr, "OK\n");
	}

	/* Erase device */
	fprintf(stderr, "Erasing... ");
	fflush(stderr);

	memset(msg, 0, sizeof(msg));
	msg[0] = T48_BOOTLOADER_ERASE;
	format_int(&msg[8], T48_BTLDR_MAGIC, 8, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 16)) {
		fprintf(stderr, "\nErase failed!\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	memset(msg, 0, sizeof(msg));
	if (msg_recv(handle->usb_handle, msg, 32)) {
		fprintf(stderr, "\nErase failed!\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	if (msg[1]) {
		fprintf(stderr, "\nfailed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "OK\n");

	/* Reflash firmware */
	fprintf(stderr, "Reflashing... ");
	fflush(stderr);

	/* Begin write blocks */
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_BOOTLOADER_WRITE;
	msg[1] = 1;
	format_int(&msg[8], T48_BTLDR_MAGIC, 8, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 16)) {
		fprintf(stderr, "\nReflash failed!\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	for (uint32_t i = 0; i < blocks; i++) {
		memset(msg, 0, sizeof(msg));
		msg[0] = T48_BOOTLOADER_WRITE;
		msg[1] = 0;
		msg[2] = 0x14;	/* Data Length LSB */
		msg[3] = 0x01; 	/* Data length MSB */
		memcpy(&msg[8], update_dat + 16 + i * 0x114, 0x114); /* Block */

		/* Write firmware block */
		if (msg_send(handle->usb_handle, msg, 0x11c)) {
			fprintf(stderr, "\nReflash failed\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		/* Check if the firmware block was successfully written */
		memset(msg, 0, sizeof(msg));
		if (msg_recv(handle->usb_handle, msg, 32)) {
			fprintf(stderr, "\nReflash... Failed\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		if (msg[1]) {
			fprintf(stderr, "\nReflash... Failed\n");
			free(update_dat);
			return EXIT_FAILURE;
		}

		fprintf(stderr, "\r\e[KReflashing... %2d%%", i * 100 / blocks);
		fflush(stderr);
	}

	/* Write last block */
	memset(msg, 0, sizeof(msg));
	msg[0] = T48_BOOTLOADER_WRITE;
	msg[1] = 3;
	msg[3] = 1;
	msg[5] = 0xFF;
	msg[6] = 3;
	msg[7] = 8;
	format_int(&msg[260], 0xCDEF8668, 4, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 264)) {
		fprintf(stderr, "\nReflash failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	/* Check if the last block was successfully written */
	memset(msg, 0, sizeof(msg));
	if (msg_recv(handle->usb_handle, msg, 32)) {
		fprintf(stderr, "\nReflash... Failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	if (msg[1]) {
		fprintf(stderr, "\nReflash... Failed\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	memset(msg, 0, sizeof(msg));
	msg[0] = T48_BOOTLOADER_WRITE;
	msg[1] = 2;
	format_int(&msg[8], T48_BTLDR_MAGIC, 8, MP_LITTLE_ENDIAN);
	if (msg_send(handle->usb_handle, msg, 16)) {
		fprintf(stderr, "\nReflash failed!\n");
		free(update_dat);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "\r\e[KReflashing... 100%%\n");

	/* Switching back to normal mode */
	fprintf(stderr, "Resetting device... ");
	fflush(stderr);
	if (minipro_reset(handle)) {
		fprintf(stderr, "failed!\n");
		return EXIT_FAILURE;
	}

	handle = minipro_open(VERBOSE);
	if (!handle) {
		fprintf(stderr, "failed!\n");
		return EXIT_FAILURE;
	}
	fprintf(stderr, "OK\n");
	if (handle->status != MP_STATUS_NORMAL) {
		fprintf(stderr, "Reflash... failed\n");
		return EXIT_FAILURE;
	}

	fprintf(stderr, "Reflash... OK\n");
	return EXIT_SUCCESS;
}
