/*
 *  Fujitsu MB85RS256B FRAM MTD Driver
 */

#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#define MTDRAM_ERASE_SIZE (32 * 1024)

/* Driver name for the Fujitsu MB85RS256B */
#define DRIVER_NAME "mb85rs256"

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_WRDI		0x04	/* Reset write enable latch */
#define OPCODE_RDID	        0x9f    /* Read device ID */
#define	OPCODE_WR		0x02	/* Write memory */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define OPCODE_SIZE             24      /* OPCODE and Address size */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/*  */
#define	MAX_CMD_SIZE		4

#define FAST_READ_DUMMY_BYTE 0

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

static inline struct mb85rs* mtd_to_mb85rs(struct mtd_info *mtd)
{
	return container_of(mtd, struct mb85rs, mtd);
}

static int mb85rs_cmdsz(struct mb85rs *flash)
{
	return 1 + flash->addr_width; /* OPCODE + Address Width */
}

/* Maximum bytes shifted out in a single SPI burst */
#define  BURST_LENGTH_BYTES 512


/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct mb85rs *flash)
{
	u8 code = OPCODE_WREN;
	struct spi_message m;
	struct spi_transfer t;

	/* This sets the Burst Length (bitcount) in the Control Register. */
	flash->spi->bits_per_burst = 8;
	spi_setup(flash->spi);

	spi_message_init(&m);
	memset(&t, 0, sizeof t);

	t.tx_buf = &code;
	t.rx_buf = &code;
	t.len = 1;

	spi_message_add_tail(&t, &m);

	if (spi_sync(flash->spi, &m) != 0) {
		printk(KERN_ERR "%s: error\n", __func__);
		return -1;
	}

	return 0;
}

#if 0
/* We currently aren't making use of block or write protection in FRAM. */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct mb85rs *flash)
{
	ssize_t retval;
	struct spi_message m;
	struct spi_transfer t;
	u16 val;

	flash->spi->bits_per_burst = 16
	val = OPCODE_RDSR << 8;
	
	spi_setup(flash->spi);
	spi_message_init(&m);
	memset(&t, 0, sizeof t);

	t.tx_buf = (u8 *)&val;
	t.rx_buf = (u8 *)&val;
	t.len = 2;

	spi_message_add_tail(&t, &m);

	if (spi_sync(flash->spi, &m) != 0) {
		printk(KERN_ERR "%s: error\n", __func__);
		return -1;
	}

	return val & 0x00ff;
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct mb85rs *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		sr = read_sr(flash);
		if (sr < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		cond_resched();
	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

#endif

/*
 * The main logic behind reads on the SPI bus.
 * Refer to the CSPI_CONREG (Control Register) in the Reference Manual to understand
 * the logic here. The CSPI reads 32bits at a time from the FIFO. If the burst does
 * not contain a multiple of 32bits the first word contains the unused bits, the
 * unused bits are not shifted out, and the used bits should start at the LSB.
 */
static int _fsl_spi_read(struct mb85rs *flash, void *buf, int addr, int len)
{
	u8			txer[BURST_LENGTH_BYTES] = { 0 }; /* transfer buffer */
	struct spi_message	m; /* spi message */
	struct spi_transfer	t; /* spi transfer */
	u8			*d = (u8 *)buf; /* pointer to destination */
	u8			*s; /* pointer to source */
	int			count; /* how many bytes left to read */
	int			l; /* number of bytes to read for a given transaction */
	int			i;
	int			j; /* index in current word */

	/* sanity checks */
	if (len == 0)
		return 0;

	if (addr + len > flash->mtd.size)
		return -EINVAL;
	
	// TODO: need to wait until flash is ready.
	// wait_till_ready(flash);

	count = len;

	while (count > 0) {
		s = (u8 *)txer;

		l = count > ((BURST_LENGTH_BYTES) - mb85rs_cmdsz(flash)) ?
			(BURST_LENGTH_BYTES) - mb85rs_cmdsz(flash) : count;

		switch((l+mb85rs_cmdsz(flash))%4) {
			case 1:
				txer[0] = OPCODE_NORM_READ;
				txer[7] = 0xff & (addr >> 8);
				txer[6] = 0xff & addr;
				j=1;
				s+=4;
				break;
			case 2:
				txer[1] = OPCODE_NORM_READ;
				txer[0] = 0xff & (addr >> 8);
				txer[7] = 0xff & addr;
				j=2;
				s+=4;
				break;
			case 3:
				txer[2] = OPCODE_NORM_READ;
				txer[1] = 0xff & (addr >> 8);
				txer[0] = 0xff & addr;
				j=3;
				s+=4;
				break;
			default:
				txer[3] = OPCODE_NORM_READ;
				txer[2] = 0xff & (addr >> 8);
				txer[1] = 0xff & addr;
				j=0;
		}

		/* convert bytes to bits */
		flash->spi->bits_per_burst = (l+mb85rs_cmdsz(flash)) << 3;

		/* setup the spi transaction */
		spi_setup(flash->spi);
		spi_message_init(&m);
		memset(&t, 0, sizeof t);
		t.tx_buf = txer;
		t.rx_buf = txer;
		t.len = ((l+mb85rs_cmdsz(flash))+3)/4;
		spi_message_add_tail(&t, &m);

		/* shift some bits */
		if (spi_sync(flash->spi, &m) != 0) {
			printk(KERN_ERR "%s: error\n", __func__);
			return -1;
		}

		/* copy bytes read from FRAM to the destination buffer */
		for(i = 0; i < l; i++, j--) {
			if (j < 0) {
				j = 3;
				s+=4;
			}
			*d++ = s[j];
		}

		count -= l;
		addr += l;
	}
	return len;
}

/*
 * Erase is not required on the mb85rs256.
 */
static int mb85rs_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	return 0;
}

/*
 * Read an address range from the flash chip, provided it is within the
 * physical boundaries.
 */
static int mb85rs_read(struct mtd_info *mtd, loff_t from, size_t len,
		       size_t *retlen, u_char *buf)
{
	struct mb85rs		*flash = mtd_to_mb85rs(mtd);

	mutex_lock(&flash->lock);
	*retlen = _fsl_spi_read(flash, buf, from, len);
	mutex_unlock(&flash->lock);

	return 0;
}


/*
 * Write an address range to the flash chip. The address range may be any size provided
 * it is within the physical boundaries.
 */
static int mb85rs_write(struct mtd_info *mtd, loff_t to, size_t len,
			size_t *retlen, const u_char *buf)
{
	struct mb85rs		*flash = mtd_to_mb85rs(mtd);
	u8			txbuf[BURST_LENGTH_BYTES] = { 0 }; /* transfer buffer */
	int			status; /* return code */
	u8			*s = (u8 *) buf; /* source */
	u8			*d = (u8 *) txbuf; /* destination */
	int			count; /* how many bytes left to write */
	int			l; /* number of bytes to write for a transaction */
	int 			i;
	int			j; /* index in current word */

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return 0;

	if (to + len > flash->mtd.size)
		return -EINVAL;

	mutex_lock(&flash->lock);

	count = len;

	while (count > 0) {
		d = (u8 *)txbuf;

		l = count > ((BURST_LENGTH_BYTES) - mb85rs_cmdsz(flash)) ?
			(BURST_LENGTH_BYTES) - mb85rs_cmdsz(flash) : count;

		switch ((l+mb85rs_cmdsz(flash))%4) {
			case 1:
				d[0] = OPCODE_WR;
				d[7] = 0xff & (to >> 8);
				d[6] = 0xff & to;
				d+=4;
				j=1;
				break;
			case 2:
				d[1] = OPCODE_WR;
				d[0] = 0xff & (to >> 8);
				d[7] = 0xff & to;
				d+=4;
				j=2;
				break;
			case 3:
				d[2] = OPCODE_WR;
				d[1] = 0xff & (to >> 8);
				d[0] = 0xff & to;
				d+=4;
				j=3;
				break;
			default:
				d[3] = OPCODE_WR;
				d[2] = 0xff & (to >> 8);
				d[1] = 0xff & to;
				j=0;
		}
		
		/* write data to transfer buffer */
		for (i = 0; i < l; i++, j--) {
			if (j < 0) {
				j = 3;
				d+=4;
			}
			d[j] = *s++;
		}

		write_enable(flash);

		/* convert bytes to bits */
		flash->spi->bits_per_burst = (l+mb85rs_cmdsz(flash)) << 3;

		/* start the transaction, length is the number of words(32bits) to be written */
		status = spi_write(flash->spi, txbuf, (l+mb85rs_cmdsz(flash)+3)/4);
		if (status) {
			printk("MB85RS::WRITE: failed, fatal: %d\n", status);
			return status;
		}

		/* update */
		count -= l;
		to += l;
	}

	*retlen = len;
	mutex_unlock(&flash->lock);
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

	/* The size listed here is what works with OPCODE_SE, which isn't
	* necessarily called a "sector" by the vendor.
	*/
	unsigned sector_size;
	u16 n_sectors;

	u16 page_size;
	u16 addr_width;

	u16 flags;
#define	SECT_4K		0x01	/* OPCODE_BE_4K works uniformly */
#define	M25P_NO_ERASE	0x02	/* No erase command needed */
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.jedec_id = (_jedec_id),				\
		.ext_id = (_ext_id),					\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.addr_width = 2,					\
		.flags = (_flags),					\
	})

static const struct spi_device_id mb85rs_ids[] = {
	{ DRIVER_NAME, INFO(0x202014, 0, MTDRAM_ERASE_SIZE, 1, 0) },
	{ },
};

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit mb85rs_probe(struct spi_device *spi)
{
	const struct spi_device_id	*id = spi_get_device_id(spi);
	struct flash_platform_data	*data;
	struct flash_info		*info;
	struct mb85rs			*flash;
	char				*name = DRIVER_NAME;

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
	flash->addr_width = 2;

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", flash->mtd.name,
		(long long)flash->mtd.size >> 10);

	return add_mtd_device(&flash->mtd) == 1 ? -ENODEV : 0;
}

static int __devexit mb85rs_remove(struct spi_device *spi)
{
	struct mb85rs *flash = dev_get_drvdata(&spi->dev);
	//struct mb85rs *flash;
	int status = 0;

	/* Clean up MTD stuff. */
	status = del_mtd_device(&flash->mtd);
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

static int __init mb85rs256_init(void)
{
	return spi_register_driver(&mb85rs256_driver);
}

static void __exit mb85rs256_exit(void)
{
	spi_unregister_driver(&mb85rs256_driver);
}

module_init(mb85rs256_init);
module_exit(mb85rs256_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ezell Young <ezell.young@crown.com>, Michael John <michael.john@crown.com>");
MODULE_DESCRIPTION("Fujitsu MB85RS256B FRAM MTD driver"); 
