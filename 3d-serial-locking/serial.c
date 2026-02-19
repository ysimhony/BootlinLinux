// SPDX-License-Identifier: GPL-2.0
/* Available on SOLUTION_URL */

#include <linux/atomic.h>
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

/* IOCTL definitions (ideally declared in a header in uapi/linux/) */
#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER 1

#define SERIAL_BUFSIZE 16

/* Per device structure */
struct serial_dev {
	void __iomem *regs;
	struct miscdevice miscdev;
	atomic_t counter;
	char rx_buf[SERIAL_BUFSIZE];
	unsigned int buf_rd;
	unsigned int buf_wr;
	wait_queue_head_t wait;
	/*
	 * Spinlock that protects against concurrent accesses to the
	 * serial port hardware and the circular buffer
	 */
	spinlock_t lock;
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

static ssize_t serial_write(struct file *file, const char __user *buf,
			    size_t sz, loff_t *ppos)
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

static const struct file_operations serial_fops = {
	.owner = THIS_MODULE,
	.write = serial_write,
	.read = serial_read,
	.unlocked_ioctl = serial_ioctl,
};

/* Driver registration */
static int serial_probe(struct platform_device *pdev)
{
	unsigned int baud_divisor, uartclk;
	struct serial_dev *serial;
	struct resource *res;
	int irq;
	int ret;

	pr_info("Called %s\n", __func__);

	/* Per device structure allocation */
	serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
	if (!serial)
		/* No message necessary here, already issued by allocation functions */
		return -ENOMEM;

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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -EINVAL;
		goto disable_runtime_pm;
	}

	/* Declare misc device */
	serial->miscdev.minor = MISC_DYNAMIC_MINOR;
	serial->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					      "serial-%x", res->start);
	serial->miscdev.fops = &serial_fops;
	serial->miscdev.parent = &pdev->dev;

	/* Setup pointer from physical to per device data */
	platform_set_drvdata(pdev, serial);

	/* Everything is ready, register the misc device */
	ret = misc_register(&serial->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register misc device (%d)\n", ret);
		goto disable_runtime_pm;
	}

	return 0;

disable_runtime_pm:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *serial = platform_get_drvdata(pdev);

	pr_info("Called %s\n", __func__);
	misc_deregister(&serial->miscdev);
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
