// SPDX-License-Identifier: GPL-2.0
/* Available on SOLUTION_URL */

#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <uapi/linux/serial_reg.h>

/* Specific OMAP 8250 UART fields */
#define OMAP_UART_SCR_DMAMODE_CTL3 0x7
#define OMAP_UART_SCR_TX_TRIG_GRANU1 BIT(6)

/* IOCTL definitions (ideally declared in a header in uapi/linux/) */
#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER 1

#define SERIAL_BUFSIZE 16

/* Per device structure */
struct serial_dev {
	void __iomem *regs;
	struct resource *res;
	struct device *dev;
	struct miscdevice miscdev;
	atomic_t counter;
	char rx_buf[SERIAL_BUFSIZE];
	char tx_buf[SERIAL_BUFSIZE];
	unsigned int buf_rd;
	unsigned int buf_wr;
	wait_queue_head_t wait;
	/*
	 * Spinlock that protects against concurrent accesses to the
	 * serial port hardware and the circular buffer
	 */
	spinlock_t lock;
	struct dma_chan *txchan;
	dma_addr_t fifo_dma_addr;
	bool txongoing;
	struct completion txcomplete;
};

/* Utility functions */
static u32 reg_read(struct serial_dev *serial, unsigned int reg)
{
	return readl(serial->regs + (reg * 4));
}

static void reg_write(struct serial_dev *serial, u32 value, unsigned int reg)
{
	writel(value, serial->regs + (reg * 4));
}

static void serial_write_char(struct serial_dev *serial, unsigned char c)
{
	unsigned long flags;

retry:
	while ((reg_read(serial, UART_LSR) & UART_LSR_THRE) == 0)
		cpu_relax();

	spin_lock_irqsave(&serial->lock, flags);
	if ((reg_read(serial, UART_LSR) & UART_LSR_THRE) == 0) {
		spin_unlock_irqrestore(&serial->lock, flags);
		goto retry;
	}

	reg_write(serial, c, UART_TX);
	spin_unlock_irqrestore(&serial->lock, flags);
}

/* Interrupt handler routine */
static irqreturn_t serial_irq_handler(int irq, void *dev_id)
{
	struct serial_dev *serial = dev_id;
	unsigned char c;

	spin_lock(&serial->lock);

	c = reg_read(serial, UART_RX);
	/* pr_info("%s: received %c\n", __func__, c); */

	serial->rx_buf[serial->buf_wr++] = c;

	if (serial->buf_wr >= SERIAL_BUFSIZE)
		serial->buf_wr = 0;

	spin_unlock(&serial->lock);

	wake_up(&serial->wait);

	return IRQ_HANDLED;
}

/* Misc device operations */
static ssize_t serial_read(struct file *file, char __user *buf,
			   size_t sz, loff_t *ppos)
{
	struct miscdevice *miscdev_ptr = file->private_data;
	struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev,
						 miscdev);
	unsigned long flags;
	unsigned char c;
	int ret;

retry:
	ret = wait_event_interruptible(serial->wait,
				       serial->buf_wr != serial->buf_rd);
	if (ret)
		return ret;

	spin_lock_irqsave(&serial->lock, flags);

	/*
	 * Now we are holding the lock, make sure there is really
	 * something to read in the circular buffer. If not, release
	 * the lock and wait again.
	 */
	if (serial->buf_wr == serial->buf_rd) {
		spin_unlock_irqrestore(&serial->lock, flags);
		goto retry;
	}

	/* Now we are sure we can read at least 1 character */
	c = serial->rx_buf[serial->buf_rd++];

	if (serial->buf_rd >= SERIAL_BUFSIZE)
		serial->buf_rd = 0;

	spin_unlock_irqrestore(&serial->lock, flags);

	/* put_user can sleep, so it must be done without the lock taken */
	ret = put_user(c, buf);
	if (ret)
		return ret;

	*ppos += 1;
	return 1;
}

static ssize_t __maybe_unused
serial_write_pio(struct file *file, const char __user *buf, size_t sz, loff_t *ppos)
{
	struct miscdevice *miscdev_ptr = file->private_data;
	struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev,
						 miscdev);
	int i;

	for (i = 0; i < sz; i++) {
		unsigned char c;

		if (get_user(c, buf + i))
			return -EFAULT;
		serial_write_char(serial, c);
		atomic_inc(&serial->counter);

		if (c == '\n')
			serial_write_char(serial, '\r');
	}

	*ppos += sz;
	return sz;
}

static void serial_dma_tx_complete(void *param)
{
	struct serial_dev *serial = param;

	complete(&serial->txcomplete);
}

static ssize_t serial_write_dma(struct file *file, const char __user *buf,
				size_t sz, loff_t *ppos)
{
	struct miscdevice *miscdev_ptr = file->private_data;
	struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev,
						 miscdev);
	struct dma_async_tx_descriptor *desc;
	dma_addr_t dma_addr;
	dma_cookie_t cookie;
	unsigned long flags;
	unsigned int len;
	char first;
	int ret;

	if (!serial->txchan)
		return -EOPNOTSUPP;

	/* Prevent concurrent Tx */
	spin_lock_irqsave(&serial->lock, flags);
	if (serial->txongoing) {
		spin_unlock_irqrestore(&serial->lock, flags);
		return -EBUSY;
	}
	serial->txongoing = true;
	spin_unlock_irqrestore(&serial->lock, flags);

	/* Copy the user buffer in a kernel buffer */
	len = min_t(unsigned int, sz, SERIAL_BUFSIZE);
	ret = copy_from_user(serial->tx_buf, buf, len);
	if (ret)
		goto unlock_dma;

	/* 8250 OMAP UART quirk: need to write the first byte manually */
	first = serial->tx_buf[0];

	/* dma: Map the memory buffer to get a usable "bus address" for the
	 * DMA controller
	 */
	dma_addr = dma_map_single(serial->dev, serial->tx_buf, SERIAL_BUFSIZE,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(serial->dev, dma_addr)) {
		ret = -ENOMEM;
		goto unlock_dma;
	}

	/* dmaengine: Provide the configuration for the next transfer:
	 * - We will copy data from memory to the device
	 * - When doing so, the memory can be accessed through the dma_addr mapping
	 */
	desc = dmaengine_prep_slave_single(serial->txchan, dma_addr + 1, len - 1,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -EBUSY;
		goto unmap_dma;
	}

	/* dmaengine: Register a callback */
	desc->callback = serial_dma_tx_complete;
	desc->callback_param = serial;
	reinit_completion(&serial->txcomplete);

	/* dmaengine: Add the next operation to the DMA controller pending queue */
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto unmap_dma;

	/* dmaengine: Trigger the next transfer */
	dma_async_issue_pending(serial->txchan);

	/* Wait for DMA transfer to be over */
	reg_write(serial, first, UART_TX);
	ret = wait_for_completion_timeout(&serial->txcomplete, msecs_to_jiffies(2000));
	if (!ret) {
		/* 0 is returned on timeout */
		ret = -ETIMEDOUT;
		goto cancel_dma;
	}

	/* dma: Unmap the buffer */
	dma_unmap_single(serial->dev, dma_addr, SERIAL_BUFSIZE,
			 DMA_TO_DEVICE);

	/* Release exclusive access to the write helper */
	spin_lock_irqsave(&serial->lock, flags);
	serial->txongoing = false;
	spin_unlock_irqrestore(&serial->lock, flags);

	atomic_add(len, &serial->counter);
	*ppos += len;
	return len;

cancel_dma:
	dmaengine_terminate_sync(serial->txchan);
unmap_dma:
	dma_unmap_single(serial->dev, dma_addr, SERIAL_BUFSIZE,
			 DMA_TO_DEVICE);
unlock_dma:
	spin_lock_irqsave(&serial->lock, flags);
	serial->txongoing = false;
	spin_unlock_irqrestore(&serial->lock, flags);

	return ret;
}

static long serial_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *miscdev_ptr = file->private_data;
	struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev,
						 miscdev);
	unsigned int __user *argp = (unsigned int __user *)arg;

	switch (cmd) {
	case SERIAL_RESET_COUNTER:
		atomic_set(&serial->counter, 0);
		break;
	case SERIAL_GET_COUNTER:
		if (put_user(atomic_read(&serial->counter), argp))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static const struct file_operations serial_fops_dma = {
	.owner = THIS_MODULE,
	.write = serial_write_dma,
	.read = serial_read,
	.unlocked_ioctl = serial_ioctl,
};

static const struct file_operations serial_fops_pio = {
	.owner = THIS_MODULE,
	.write = serial_write_pio,
	.read = serial_read,
	.unlocked_ioctl = serial_ioctl,
};

static int serial_init_dma(struct serial_dev *serial)
{
	struct dma_slave_config txconf = {};
	int ret;

	init_completion(&serial->txcomplete);

	/* dmaengine: get the DMA channel for UART Tx */
	serial->txchan = dma_request_chan(serial->dev, "tx");
	if (IS_ERR(serial->txchan)) {
		dev_warn(serial->dev, "No Tx channel (%pe)", serial->txchan);
		ret = PTR_ERR(serial->txchan);
		serial->txchan = NULL;
		return ret;
	}

	/* dma: Map the UART FIFO for DMA access */
	serial->fifo_dma_addr = dma_map_resource(serial->dev,
						 serial->res->start + UART_TX * 4,
						 4, DMA_TO_DEVICE, 0);
	if (dma_mapping_error(serial->dev, serial->fifo_dma_addr)) {
		ret = -ENOMEM;
		goto release_chan;
	}

	/* dmaengine: configure the DMA controller with details from the UART controller:
	 * - When copying data from memory to the device
	 * - Write the bytes one after the other
	 * - Into the UART FIFO
	 */
	txconf.direction = DMA_MEM_TO_DEV;
	txconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	txconf.dst_addr = serial->fifo_dma_addr;
	ret = dmaengine_slave_config(serial->txchan, &txconf);
	if (ret)
		goto unmap_res;

	/* Enable DMA */
	reg_write(serial, OMAP_UART_SCR_DMAMODE_CTL3 | OMAP_UART_SCR_TX_TRIG_GRANU1,
		  UART_OMAP_SCR);

	return 0;

unmap_res:
	dma_unmap_resource(serial->dev, serial->fifo_dma_addr, 4, DMA_TO_DEVICE, 0);
release_chan:
	dma_release_channel(serial->txchan);

	return ret;
}

static void serial_cleanup_dma(struct serial_dev *serial)
{
	if (serial->txchan) {
		/* dmaengine: Stop ongoing transfers */
		dmaengine_terminate_sync(serial->txchan);
		/* dma: Unmap the UART FIFO */
		dma_unmap_resource(serial->dev, serial->fifo_dma_addr,
				   4, DMA_TO_DEVICE, 0);
		/* dmaengine: Release the DMA channel */
		dma_release_channel(serial->txchan);
	}
}

/* Driver registration */
static int serial_probe(struct platform_device *pdev)
{
	unsigned int baud_divisor, uartclk;
	struct serial_dev *serial;
	int irq;
	int ret;

	pr_info("Called %s\n", __func__);

	/* Per device structure allocation */
	serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
	if (!serial)
		/* No message necessary here, already issued by allocation functions */
		return -ENOMEM;

	serial->dev = &pdev->dev;

	/* Get a virtual address for the device registers */
	serial->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(serial->regs))
		return PTR_ERR(serial->regs);

	/* Enable power management - Needed to operate the device */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* Configure the baud rate to 115200 */
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				   &uartclk);
	if (ret) {
		dev_err(&pdev->dev,
			"clock-frequency property not found in Device Tree\n");
		goto disable_runtime_pm;
	}

	baud_divisor = uartclk / 16 / 115200;
	reg_write(serial, 0x07, UART_OMAP_MDR1);
	reg_write(serial, 0x00, UART_LCR);
	reg_write(serial, UART_LCR_DLAB, UART_LCR);
	reg_write(serial, baud_divisor & 0xff, UART_DLL);
	reg_write(serial, (baud_divisor >> 8) & 0xff, UART_DLM);
	reg_write(serial, UART_LCR_WLEN8, UART_LCR);
	reg_write(serial, 0x00, UART_OMAP_MDR1);

	/* Clear UART FIFOs */
	reg_write(serial, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);

	/* Initialize the spin lock */
	spin_lock_init(&serial->lock);

	/* Initialize wait queue */
	init_waitqueue_head(&serial->wait);

	/* Get IRQ number from the Device Tree */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto disable_runtime_pm;
	}

	/* Register interrupt handler - Do it before enabling interrupts! */
	ret = devm_request_irq(&pdev->dev, irq, serial_irq_handler, 0,
			       pdev->name, serial);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register interrupt handler\n");
		goto disable_runtime_pm;
	}

	/* Enable RX interrupts */
	reg_write(serial, UART_IER_RDI, UART_IER);

	serial->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!serial->res) {
		ret = -EINVAL;
		goto disable_runtime_pm;
	}

	/* Declare misc device */
	serial->miscdev.minor = MISC_DYNAMIC_MINOR;
	serial->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					      "serial-%x", serial->res->start);
	serial->miscdev.fops = &serial_fops_dma;
	serial->miscdev.parent = &pdev->dev;

	/* Setup pointer from physical to per device data */
	platform_set_drvdata(pdev, serial);

	/* Initialize DMA */
	ret = serial_init_dma(serial);
	if (ret == -ENODEV)
		serial->miscdev.fops = &serial_fops_pio;
	else if (ret)
		goto disable_runtime_pm;

	/* Everything is ready, register the misc device */
	ret = misc_register(&serial->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register misc device (%d)\n", ret);
		goto cleanup_dma;
	}

	return 0;

cleanup_dma:
	serial_cleanup_dma(serial);
disable_runtime_pm:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *serial = platform_get_drvdata(pdev);

	pr_info("Called %s\n", __func__);
	misc_deregister(&serial->miscdev);
	serial_cleanup_dma(serial);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

/* Declaration of supported DT devices */
static const struct of_device_id serial_dt_match[] = {
	{ .compatible = "bootlin,serial" },
	{ },
};
MODULE_DEVICE_TABLE(of, serial_dt_match);

/* Driver declaration and registration */
static struct platform_driver serial_driver = {
	.driver = {
		.name = "serial",
		.of_match_table = of_match_ptr(serial_dt_match),
		.owner = THIS_MODULE,
	},
	.probe = serial_probe,
	.remove = serial_remove,
};
module_platform_driver(serial_driver);

MODULE_LICENSE("GPL");
