/*
 *  Fujitsu MB85RS256B FRAM MTD Driver
 *
 *  This code is based on m25p80.c and the IMM FRAM MTD Driver.
 *
 *  Copyright 2015-2017 Crown Equipment Corp.
 */

#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#define MTDRAM_ERASE_SIZE (32 * 1024)

/* Driver name for the Fujitsu MB85RS256B */
#define DRIVER_NAME "mb85rs256"

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_WRDI		0x04	/* Reset write enable latch */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_READ		0x03	/* Read data bytes (low frequency) */
#define	OPCODE_WRITE		0x02	/* Write memory */
#define OPCODE_RDID		0x9f    /* Read device ID */
#define	OPCODE_FSTRD		0x0b	/* Read data bytes (high frequency) */
#define OPCODE_SIZE		24      /* OPCODE and Address size */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/*  */
#define	MAX_CMD_SIZE		4


#define ceil(x, y) \
	({ unsigned long __x = (x), __y = (y); (__x + __y - 1) / __y; })

/****************************************************************************/

struct mb85rs {
	struct spi_device *spi;
	struct mutex lock;
	struct mtd_info mtd;
	unsigned partitioned:1;
	u16 page_size;
	u16 addr_width;
	u8 erase_opcode;
	u8 *command;
};

static inline struct mb85rs* mtd_to_mb85rs(struct mtd_info *mtd) {
	return container_of(mtd, struct mb85rs, mtd);
}


// NOTE: (May 20, 2016)
// Currently this section of code is not used or needed. We don't
// fuss with the status register to see if we need to wait to
// write. Currently the read and write functions use a mutex to
// lock other process out of the FRAM.
#if 0

/* Read the status register, returning its value in the location
 * Return the status register value
 */
static u8 read_status_register(struct mb85rs *flash)
{
	ssize_t retval;
	u8 code = OPCODE_RDSR;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n", (int)retval);
		return retval;
	}

	return val;
}

/* Write status register 1 byte
 * Returns negative if error occurred
 */
static int write_status_register(struct mb85rs *flash, u8 val)
{
	flash->command[0] = OPCODE_WRSR;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct mb85rs *flash)
{
	return 0;
//	unsigned long deadline;
//	int sr;
//
//	deadline = jiffies + MAX_READY_WAIT_JIFFIES;
//	dev_err(&flash->spi->dev, "%s: %s start\n", __func__);
//
//	do {
//		sr = read_status_register(flash);
//		dev_err(&flash->spi->dev, "%s: %d\n", __func__, sr);
//		if (sr < 0)
//			break;
//		else if (!(sr & SR_WIP))
//			return 0;
//
//		cond_resched();
//	}
//	while (!time_after_eq(jiffies, deadline));
//
//	return 1;
}

#endif

/****************************************************************************/
/** Internal helper functions */

/* Set Write Enable Latch (WEL) with Write Enable command
 * Returns negative if error occurred
 */
static inline int write_enable(struct mb85rs *flash)
{
	u8 code = OPCODE_WREN;
	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}


/* Send write disable instruction to the chip, resets the WEL
 * Returns negative if error occurred
 */
static inline int write_disable(struct mb85rs *flash)
{
	u8 code = OPCODE_WRDI;
	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/************************************************************************/
/** MTD implementation */

/*
 * Write an address range to the flash chip. The address range may be
 * any size provided it is within the physical boundaries.
 */
static int mb85rs_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct mb85rs 	*flash = mtd_to_mb85rs(mtd);
	size_t 		n_tx;     // length of transmit buffer
	u8 		*txbuf;   // transmit buffer
	u8 		*p_txbuf; // pointer to head of data
	int 		status;   // return code

	mutex_lock(&flash->lock);
	write_enable(flash);
	n_tx = len + 3;
	txbuf = kzalloc(n_tx, GFP_KERNEL);
	txbuf[0] = OPCODE_WRITE;
	txbuf[1] = 0x00FF & (to >> 8);
	txbuf[2] = 0x00FF & to;
	p_txbuf = &txbuf[3];
	memcpy(p_txbuf, buf, len);

	/* Write to FRAM */
	status = spi_write(flash->spi, txbuf, n_tx);
	if (status != 0)
		printk("MB85RS256: failed on write: %d\n", status);
	mutex_unlock(&flash->lock);

 	return 0;
 }

/* Read an address range from the flash chip. The address range
 * may be any size provided it is within the physical boundaries.
 */
static int mb85rs_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct mb85rs 		*flash = mtd_to_mb85rs(mtd);
	u8 			txbuf[3]; // transmit buffer for OPCODE+Addr
	int 			status;   // return code
	struct spi_message	message;
	struct spi_transfer	x[2];

	mutex_lock(&flash->lock);
	txbuf[0] = OPCODE_READ;
	txbuf[1] = 0x00FF & (from >> 8);
	txbuf[2] = 0x00FF & from;

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	x[0].len = 3;
	spi_message_add_tail(&x[0], &message);
	x[1].len = len;
	spi_message_add_tail(&x[1], &message);
	x[0].tx_buf = txbuf;
	x[1].rx_buf = buf;

	/* do the i/o */
	status = spi_sync(flash->spi, &message);
	*retlen = len;
	mutex_unlock(&flash->lock);

	return 0;
}

/* TODO:
 * Erase an address range on the flash chip.
 * Return an error if there is a problem erasing.
 */
static int mb85rs_erase(struct mtd_info *mtd, struct erase_info *instr) {
	return 0;
}

/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32 jedec_id;
	u16 ext_id;

	/* The size listed here isn't necessarily called a "sector" by the
	 * vendor. For this FRAM it would be the size of the part, in bytes.
	 */
	unsigned sector_size;
	/* Number of sectors. For this FRAM it will be 1. */
	u16 n_sectors;

	u16 page_size;
	u16 addr_width;

	u16 flags;
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
	 	.jedec_id = (_jedec_id),				\
	 	.ext_id = (_ext_id),					\
	 	.sector_size = (_sector_size),				\
	 	.n_sectors = (_n_sectors),				\
	 	.page_size = 256,					\
	 	.addr_width = 3,					\
	 	.flags = (_flags),					\
	 })

static const struct spi_device_id mb85rs_ids[] = {
	{ DRIVER_NAME, INFO(0x202014, 0, 32 * 1024, 1, 0) },
	{ },
};

#define FAST_READ_DUMMY_BYTE 1

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit mb85rs_probe(struct spi_device *spi) {
	const struct spi_device_id	*id = spi_get_device_id(spi);
	struct flash_platform_data	*data;
	struct mb85rs			*flash;
	char				*name = DRIVER_NAME;
	struct flash_info		*info;
	struct mtd_part_parser_data	ppdata;
	u8				opcode;
	u8				opid[5];
	int 				opret;

	printk("MB85RS256 probe start\n");
	data = spi->dev.platform_data;
	info = (void *)id->driver_data;

	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash) return -ENOMEM;
	flash->command =
		kmalloc(MAX_CMD_SIZE + FAST_READ_DUMMY_BYTE, GFP_KERNEL);
	if (!flash->command) {
		kfree(flash);
		return -ENOMEM;
	}

	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

	/* Setup the MTD structure */
	flash->mtd.name = name;
	flash->mtd.type = MTD_RAM;
	flash->mtd.flags = MTD_CAP_RAM;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd.writesize = 1;
	flash->mtd.erasesize = 1;
	flash->mtd.flags |= MTD_NO_ERASE;
	flash->mtd.erase = mb85rs_erase;
	flash->mtd.read = mb85rs_read;
	flash->mtd.write = mb85rs_write;
	flash->mtd.dev.parent = &spi->dev;
	flash->page_size = info->page_size;
	ppdata.of_node = spi->dev.of_node;

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", flash->mtd.name,
			(long long)flash->mtd.size >> 10);

	opcode = OPCODE_RDID;
	opret = spi_write_then_read(spi, &opcode, 1, opid, 5);
	if (opret < 0) {
		printk("Read of OPCODE_RDID failed\n");
	} else {
		printk("%s: 1:%d 2:%d 3:%d 4:%d 5:%d\n",
			dev_name(&spi->dev), opid[0],opid[1],opid[2],opid[3],opid[4]);
	}

	/* partitions should match sector boundaries */
	return mtd_device_parse_register(&flash->mtd, NULL, &ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
}

static int __devexit mb85rs_remove(struct spi_device *spi) {
	struct mb85rs	*flash = dev_get_drvdata(&spi->dev);
	int		status = 0;

	/* Clean up MTD stuff. */
	status = mtd_device_unregister(&flash->mtd);
	if (status == 0) {
		kfree(flash->command);
		kfree(flash);
	}
	return 0;
}

static struct spi_driver mb85rs256_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.id_table = mb85rs_ids,
	.probe = mb85rs_probe,
	.remove = __devexit_p(mb85rs_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

static int __init mb85rs256_init(void) {
	return spi_register_driver(&mb85rs256_driver);
}

static void __exit mb85rs256_exit(void) {
	spi_unregister_driver(&mb85rs256_driver);
}

module_init(mb85rs256_init);
module_exit(mb85rs256_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael John <michael.john@crown.com>");
MODULE_DESCRIPTION("Fujitsu MB85RS256B FRAM MTD driver");
