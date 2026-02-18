// SPDX-License-Identifier: GPL-2.0
/* Available on SOLUTION_URL */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>

static int nunchuk_read_registers(struct i2c_client *client, u8 *recv)
{
	u8 buf[1];
	int ret;

	/* Ask the device to get ready for a read */
	fsleep(10000);

	buf[0] = 0x00;
	ret = i2c_master_send(client, buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "i2c send failed (%d)\n", ret);
		return ret;
	}

	fsleep(10000);

	/* Now read registers */
	ret = i2c_master_recv(client, recv, 6);
	if (ret < 0) {
		dev_err(&client->dev, "i2c recv failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int nunchuk_probe(struct i2c_client *client)
{
	int ret, zpressed, cpressed;
	u8 recv[6], buf[2];

	/* Initialize device */
	buf[0] = 0xf0;
	buf[1] = 0x55;

	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "i2c send failed (%d)\n", ret);
		return ret;
	}

	fsleep(1000);

	buf[0] = 0xfb;
	buf[1] = 0x00;

	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "i2c send failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Make a dummy read from the device. The device seems to update the
	 * state of its internal registers only once they have been read.
	 */
	ret = nunchuk_read_registers(client, recv);
	if (ret < 0)
		return ret;

	/* Now get the real state of the device */
	ret = nunchuk_read_registers(client, recv);
	if (ret < 0)
		return ret;

	zpressed = (recv[5] & BIT(0)) ? 0 : 1;
	if (zpressed)
		dev_info(&client->dev, "Z button pressed\n");

	cpressed = (recv[5] & BIT(1)) ? 0 : 1;
	if (cpressed)
		dev_info(&client->dev, "C button pressed\n");

	return 0;
}

static void nunchuk_remove(struct i2c_client *client)
{
	return;
}

/* Specification of supported Device Tree devices */
static const struct of_device_id nunchuk_dt_match[] = {
	{ .compatible = "nintendo,nunchuk" },
	{ },
};
MODULE_DEVICE_TABLE(of, nunchuk_dt_match);

/* Driver declaration */
static struct i2c_driver nunchuk_driver = {
	.driver = {
		.name = "nunchuk",
		.of_match_table = nunchuk_dt_match,
	},
	.probe = nunchuk_probe,
	.remove = nunchuk_remove,
};
module_i2c_driver(nunchuk_driver);

MODULE_LICENSE("GPL");
