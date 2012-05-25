/*
 *  Copyright (C) 2009 LSI Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/io.h>
#include <linux/module.h>

#include "ncr.h"

static void *nca_reg;

#define NCA  nca_reg

typedef union {
	unsigned long raw;
	struct {
		unsigned long start_done:1;
		unsigned long unused:6;
		unsigned long local_bit:1;
		unsigned long status:2;
		unsigned long byte_swap_enable:1;
		unsigned long cfg_cmpl_int_enable:1;
		unsigned long cmd_type:4;
		unsigned long dbs:16;
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) command_data_register_0_t;

typedef union {
	unsigned long raw;
	struct {
		unsigned long target_address:32;
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) command_data_register_1_t;

typedef union {
	unsigned long raw;
	struct {
		unsigned long unused:16;
		unsigned long target_node_id:8;
		unsigned long target_id_address_upper:8;
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) command_data_register_2_t;

/*
  ----------------------------------------------------------------------
  ncr_register_read
*/

static __inline__ unsigned long
ncr_register_read(unsigned *address)
{
	unsigned long value;

	value = in_be32(address);

	return value;
}

/*
  ----------------------------------------------------------------------
  ncr_register_write
*/

static __inline__ void
ncr_register_write(const unsigned value, unsigned *address)
{
	out_be32(address, value);
}

/*
  ======================================================================
  ======================================================================
  Public Interface
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  ncr_read
*/

int
ncr_read(unsigned long region, unsigned long address, int number,
	 void *buffer)
{
	command_data_register_0_t cdr0;
	command_data_register_1_t cdr1;
	command_data_register_2_t cdr2;

	/*
	  Set up the read command.
	*/

	cdr2.raw = 0;
	cdr2.bits.target_node_id = NCP_NODE_ID(region);
	cdr2.bits.target_id_address_upper = NCP_TARGET_ID(region);
	ncr_register_write(cdr2.raw, (unsigned *) (NCA + 0xf8));

	cdr1.raw = 0;
	cdr1.bits.target_address = (address >> 2);
	ncr_register_write(cdr1.raw, (unsigned *) (NCA + 0xf4));

	cdr0.raw = 0;
	cdr0.bits.start_done = 1;

	if (0xff == cdr2.bits.target_id_address_upper)
		cdr0.bits.local_bit = 1;

	cdr0.bits.cmd_type = 4;
	/* TODO: Verify number... */
	cdr0.bits.dbs = (number - 1);
	ncr_register_write(cdr0.raw, (unsigned *) (NCA + 0xf0));
	mb();

	/*
	  Wait for completion.
	*/

	/* TODO: Handle failure cases. */
#if 0
	while (0x80000000 ==
	       (ncr_register_read((unsigned *) (NCA + 0xf0)) & 0x80000000))
		;
#else
	{
		volatile unsigned long value;

		do {
			value = ncr_register_read((unsigned *) (NCA + 0xf0));
		} while (0x80000000 == (value & 0x80000000));
	}
#endif

	/*
	  Copy data words to the buffer.
	*/

	address = (unsigned long)(NCA + 0x1000);
	while (4 <= number) {
		*((unsigned long *) buffer) =
			ncr_register_read((unsigned *) address);
		address += 4;
		number -= 4;
	}

	if (0 < number) {
		unsigned long temp =
			ncr_register_read((unsigned *) address);
		memcpy((void *) buffer, &temp, number);
	}

	return 0;
}

/*
  ----------------------------------------------------------------------
  ncr_write
*/

int
ncr_write(unsigned long region, unsigned long address, int number,
	  void *buffer)
{
	command_data_register_0_t cdr0;
	command_data_register_1_t cdr1;
	command_data_register_2_t cdr2;
	unsigned long data_word_base;
	int dbs = (number - 1);

	/*
	  Set up the write.
	*/

	cdr2.raw = 0;
	cdr2.bits.target_node_id = NCP_NODE_ID(region);
	cdr2.bits.target_id_address_upper = NCP_TARGET_ID(region);
	ncr_register_write(cdr2.raw, (unsigned *) (NCA + 0xf8));

	cdr1.raw = 0;
	cdr1.bits.target_address = (address >> 2);
	ncr_register_write(cdr1.raw, (unsigned *) (NCA + 0xf4));

	/*
	  Copy from buffer to the data words.
	*/

	data_word_base = (unsigned long)(NCA + 0x1000);

	while (4 <= number) {
		ncr_register_write(*((unsigned long *) buffer),
				   (unsigned *) data_word_base);
		data_word_base += 4;
		buffer += 4;
		number -= 4;
	}

	if (0 < number) {
		unsigned long temp = 0;

		memcpy((void *) &temp, (void *) buffer, number);
		ncr_register_write(temp, (unsigned *) data_word_base);
		data_word_base += number;
		buffer += number;
		number = 0;
	}

	cdr0.raw = 0;
	cdr0.bits.start_done = 1;

	if (0xff == cdr2.bits.target_id_address_upper) {
		cdr0.bits.local_bit = 1;
	}

	cdr0.bits.cmd_type = 5;
	/* TODO: Verify number... */
	cdr0.bits.dbs = dbs;
	ncr_register_write(cdr0.raw, (unsigned *) (NCA + 0xf0));
	mb();

	/*
	  Wait for completion.
	*/

	/* TODO: Handle failure cases. */
	while (0x80000000 ==
	       (ncr_register_read((unsigned *) (NCA + 0xf0)) & 0x80000000))
		;

	return 0;
}

/*
  ----------------------------------------------------------------------
  ncr_init
*/

int
ncr_init( void )
{
    /* We need this to be a module so that the functions can be exported
     * as module symbols.
     */
	nca_reg = ioremap(0x2000520000ull, 0x2000);
	if (!nca_reg) {
		pr_err("Can't iomap for nca registers\n");
		return -1;
	}

	return 0;
}

postcore_initcall( ncr_init );

/*
  ----------------------------------------------------------------------
  ncr_exit
*/

void __exit
ncr_exit( void )
{
	iounmap(nca_reg);
}

module_exit( ncr_exit );

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Register Ring access for LSI's ACP board");

EXPORT_SYMBOL(ncr_read);
EXPORT_SYMBOL(ncr_write);
