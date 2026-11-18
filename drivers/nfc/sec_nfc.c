// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S3FWRN5 (N5) NFC Controller I2C Char Driver (/dev/sec-nfc)
 * Ported for Samsung Galaxy S6 (Exynos 7420 / zeroflte) on Linux 6.6.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define SEC_NFC_DRIVER_NAME	"sec-nfc"
#define SEC_NFC_MAX_BUFFER_SIZE	512
#define SEC_NFC_MSG_MAX_SIZE	(256 + 4)
#define SEC_NFC_VEN_WAIT_TIME	100

/* Match universal7420-common/hardware/nfc/halimpl/include/sec_nfc.h */
#define SEC_NFC_MAGIC		'S'
#define SEC_NFC_GET_MODE	_IOW(SEC_NFC_MAGIC, 0, unsigned int)
#define SEC_NFC_SET_MODE	_IOW(SEC_NFC_MAGIC, 1, unsigned int)
#define SEC_NFC_SLEEP		_IOW(SEC_NFC_MAGIC, 2, unsigned int)
#define SEC_NFC_WAKEUP		_IOW(SEC_NFC_MAGIC, 3, unsigned int)
#define SEC_NFC_SET_NPT_MODE	_IOW(SEC_NFC_MAGIC, 4, unsigned int)

enum sec_nfc_mode {
	SEC_NFC_MODE_OFF = 0,
	SEC_NFC_MODE_FIRMWARE,
	SEC_NFC_MODE_BOOTLOADER,
	SEC_NFC_MODE_COUNT,
};

enum sec_nfc_irq_state {
	SEC_NFC_NONE = 0,
	SEC_NFC_INT,
	SEC_NFC_SKIP,
};

/* S3FWRN5 (N5): VEN is active-low (0 = ON, 1 = OFF) */
#define SEC_NFC_PW_ON		0
#define SEC_NFC_PW_OFF		1
#define SEC_NFC_FW_OFF		0
#define SEC_NFC_FW_ON		1
#define SEC_NFC_WAKE_SLEEP	0
#define SEC_NFC_WAKE_UP		1

#define SEC_NFC_NPT_CMD_ON	0x7E
#define SEC_NFC_NPT_CMD_OFF	0x7F

struct sec_nfc_info {
	struct miscdevice miscdev;
	struct i2c_client *client;
	struct device *dev;
	struct mutex mutex;
	struct mutex read_mutex;
	wait_queue_head_t read_wait;

	struct gpio_desc *ven_gpio;
	struct gpio_desc *firm_gpio;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *clk_req_gpio;
	struct regulator *vdd;
	struct clk *clk;

	int clk_req_irq;
	bool clk_ctl;
	bool clk_state;
	bool irq_wake_enabled;

	enum sec_nfc_mode mode;
	enum sec_nfc_irq_state read_irq;
	u8 *buf;
};

static irqreturn_t sec_nfc_irq_thread_fn(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;

	mutex_lock(&info->read_mutex);
	if (info->read_irq == SEC_NFC_SKIP) {
		mutex_unlock(&info->read_mutex);
		return IRQ_HANDLED;
	}
	info->read_irq = SEC_NFC_INT;
	mutex_unlock(&info->read_mutex);

	wake_up_interruptible(&info->read_wait);
	return IRQ_HANDLED;
}

static void sec_nfc_apply_clk_state(struct sec_nfc_info *info)
{
	bool req;

	if (!info->clk || !info->clk_req_gpio)
		return;

	req = gpiod_get_value_cansleep(info->clk_req_gpio) > 0;
	if (req == info->clk_state)
		return;

	if (req) {
		if (!clk_prepare_enable(info->clk))
			info->clk_state = true;
	} else {
		clk_disable_unprepare(info->clk);
		info->clk_state = false;
	}
}

static irqreturn_t sec_nfc_clk_irq_thread(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;

	sec_nfc_apply_clk_state(info);
	return IRQ_HANDLED;
}

static void sec_nfc_clk_ctl_enable(struct sec_nfc_info *info)
{
	int ret;

	if (info->clk_ctl || !info->clk || !info->clk_req_gpio || info->clk_req_irq <= 0)
		return;

	info->clk_state = false;
	ret = request_threaded_irq(info->clk_req_irq, NULL, sec_nfc_clk_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "sec-nfc-clk", info);
	if (ret < 0) {
		dev_err(info->dev, "failed to request clk_req irq %d: %d\n",
			info->clk_req_irq, ret);
		return;
	}
	info->clk_ctl = true;
	sec_nfc_apply_clk_state(info);
}

static void sec_nfc_clk_ctl_disable(struct sec_nfc_info *info)
{
	if (!info->clk_ctl)
		return;

	free_irq(info->clk_req_irq, info);
	if (info->clk_state && info->clk) {
		clk_disable_unprepare(info->clk);
		info->clk_state = false;
	}
	info->clk_ctl = false;
}

static void sec_nfc_set_mode(struct sec_nfc_info *info, enum sec_nfc_mode mode)
{
	if (info->mode == mode)
		return;

	info->mode = mode;

	mutex_lock(&info->read_mutex);
	info->read_irq = SEC_NFC_SKIP;
	mutex_unlock(&info->read_mutex);

	gpiod_set_value_cansleep(info->ven_gpio, SEC_NFC_PW_OFF);
	if (info->firm_gpio)
		gpiod_set_value_cansleep(info->firm_gpio, SEC_NFC_FW_OFF);

	if (mode == SEC_NFC_MODE_BOOTLOADER && info->firm_gpio)
		gpiod_set_value_cansleep(info->firm_gpio, SEC_NFC_FW_ON);

	if (mode != SEC_NFC_MODE_OFF) {
		msleep(SEC_NFC_VEN_WAIT_TIME);
		gpiod_set_value_cansleep(info->ven_gpio, SEC_NFC_PW_ON);
		sec_nfc_clk_ctl_enable(info);
		if (!info->irq_wake_enabled && info->client->irq > 0) {
			enable_irq_wake(info->client->irq);
			info->irq_wake_enabled = true;
		}
		msleep(SEC_NFC_VEN_WAIT_TIME / 2);
	} else {
		sec_nfc_clk_ctl_disable(info);
		if (info->irq_wake_enabled && info->client->irq > 0) {
			disable_irq_wake(info->client->irq);
			info->irq_wake_enabled = false;
		}
		mutex_lock(&info->read_mutex);
		info->read_irq = SEC_NFC_NONE;
		mutex_unlock(&info->read_mutex);
	}
}

static ssize_t sec_nfc_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						 struct sec_nfc_info, miscdev);
	enum sec_nfc_irq_state irq;
	int ret;

	mutex_lock(&info->mutex);
	if (info->mode == SEC_NFC_MODE_OFF) {
		ret = -ENODEV;
		goto out;
	}

	mutex_lock(&info->read_mutex);
	irq = info->read_irq;
	mutex_unlock(&info->read_mutex);

	if (irq == SEC_NFC_NONE && (file->f_flags & O_NONBLOCK)) {
		ret = -EAGAIN;
		goto out;
	}

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&info->read_mutex);
	memset(info->buf, 0, count);
	ret = i2c_master_recv(info->client, info->buf, count);
	info->read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->read_mutex);

	if (ret == -EREMOTEIO) {
		ret = -ERESTART;
		goto out;
	}
	if (ret != count)
		goto out;

	if (copy_to_user(buf, info->buf, ret))
		ret = -EFAULT;

out:
	mutex_unlock(&info->mutex);
	return ret;
}

static ssize_t sec_nfc_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						 struct sec_nfc_info, miscdev);
	int ret;

	mutex_lock(&info->mutex);
	if (info->mode == SEC_NFC_MODE_OFF) {
		ret = -ENODEV;
		goto out;
	}
	if (count > SEC_NFC_MSG_MAX_SIZE) {
		ret = -EINVAL;
		goto out;
	}
	if (copy_from_user(info->buf, buf, count)) {
		ret = -EFAULT;
		goto out;
	}

	mutex_lock(&info->read_mutex);
	ret = i2c_master_send(info->client, info->buf, count);
	if (info->read_irq == SEC_NFC_SKIP)
		info->read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->read_mutex);

	if (ret == -EREMOTEIO)
		ret = -ERESTART;
	else if (ret != count)
		ret = -EREMOTEIO;

out:
	mutex_unlock(&info->mutex);
	return ret;
}

static __poll_t sec_nfc_poll(struct file *file, poll_table *wait)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						 struct sec_nfc_info, miscdev);
	__poll_t mask = 0;

	mutex_lock(&info->mutex);
	if (info->mode == SEC_NFC_MODE_OFF) {
		mutex_unlock(&info->mutex);
		return EPOLLERR;
	}
	poll_wait(file, &info->read_wait, wait);

	mutex_lock(&info->read_mutex);
	if (info->read_irq == SEC_NFC_INT)
		mask = EPOLLIN | EPOLLRDNORM;
	mutex_unlock(&info->read_mutex);

	mutex_unlock(&info->mutex);
	return mask;
}

static long sec_nfc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						 struct sec_nfc_info, miscdev);
	unsigned int new_mode = (unsigned int)arg;
	long ret = 0;

	mutex_lock(&info->mutex);
	switch (cmd) {
	case SEC_NFC_SET_MODE:
		if (new_mode >= SEC_NFC_MODE_COUNT) {
			ret = -EINVAL;
			break;
		}
		sec_nfc_set_mode(info, new_mode);
		break;
	case SEC_NFC_GET_MODE:
		ret = info->mode;
		break;
	case SEC_NFC_SLEEP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER && info->firm_gpio)
			gpiod_set_value_cansleep(info->firm_gpio, SEC_NFC_WAKE_SLEEP);
		break;
	case SEC_NFC_WAKEUP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER && info->firm_gpio)
			gpiod_set_value_cansleep(info->firm_gpio, SEC_NFC_WAKE_UP);
		break;
	case SEC_NFC_SET_NPT_MODE:
		if (new_mode == SEC_NFC_NPT_CMD_ON) {
			info->mode = SEC_NFC_MODE_FIRMWARE;
			mutex_lock(&info->read_mutex);
			info->read_irq = SEC_NFC_SKIP;
			mutex_unlock(&info->read_mutex);
			gpiod_set_value_cansleep(info->ven_gpio, SEC_NFC_PW_ON);
			sec_nfc_clk_ctl_enable(info);
			if (info->firm_gpio)
				gpiod_set_value_cansleep(info->firm_gpio, SEC_NFC_FW_ON);
		} else if (new_mode == SEC_NFC_NPT_CMD_OFF) {
			info->mode = SEC_NFC_MODE_OFF;
			if (info->firm_gpio)
				gpiod_set_value_cansleep(info->firm_gpio, SEC_NFC_FW_OFF);
			gpiod_set_value_cansleep(info->ven_gpio, SEC_NFC_PW_OFF);
			sec_nfc_clk_ctl_disable(info);
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&info->mutex);
	return ret;
}

static int sec_nfc_open(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						 struct sec_nfc_info, miscdev);
	int ret = 0;

	mutex_lock(&info->mutex);
	if (info->mode != SEC_NFC_MODE_OFF)
		ret = -EBUSY;
	else
		sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	mutex_unlock(&info->mutex);
	return ret;
}

static int sec_nfc_close(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						 struct sec_nfc_info, miscdev);

	mutex_lock(&info->mutex);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	mutex_unlock(&info->mutex);
	return 0;
}

static const struct file_operations sec_nfc_fops = {
	.owner = THIS_MODULE,
	.read = sec_nfc_read,
	.write = sec_nfc_write,
	.poll = sec_nfc_poll,
	.open = sec_nfc_open,
	.release = sec_nfc_close,
	.unlocked_ioctl = sec_nfc_ioctl,
	.compat_ioctl = sec_nfc_ioctl,
};

static int sec_nfc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_nfc_info *info;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->buf = devm_kzalloc(dev, SEC_NFC_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!info->buf)
		return -ENOMEM;

	info->client = client;
	info->dev = dev;
	info->mode = SEC_NFC_MODE_OFF;
	info->read_irq = SEC_NFC_NONE;
	mutex_init(&info->mutex);
	mutex_init(&info->read_mutex);
	init_waitqueue_head(&info->read_wait);
	i2c_set_clientdata(client, info);

	info->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(info->vdd)) {
		if (PTR_ERR(info->vdd) == -ENODEV)
			info->vdd = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(info->vdd), "vdd supply\n");
	}
	if (info->vdd) {
		ret = regulator_enable(info->vdd);
		if (ret)
			return ret;
	}

	/* VEN starts OFF (HIGH on S3FWRN5) */
	info->ven_gpio = devm_gpiod_get(dev, "ven", GPIOD_OUT_HIGH);
	if (IS_ERR(info->ven_gpio))
		return dev_err_probe(dev, PTR_ERR(info->ven_gpio), "ven gpio\n");

	info->firm_gpio = devm_gpiod_get_optional(dev, "firm", GPIOD_OUT_LOW);
	if (IS_ERR(info->firm_gpio))
		return dev_err_probe(dev, PTR_ERR(info->firm_gpio), "firm gpio\n");

	info->irq_gpio = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
	if (IS_ERR(info->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(info->irq_gpio), "irq gpio\n");

	info->clk_req_gpio = devm_gpiod_get_optional(dev, "clk-req", GPIOD_IN);
	if (IS_ERR(info->clk_req_gpio))
		return dev_err_probe(dev, PTR_ERR(info->clk_req_gpio), "clk-req gpio\n");

	if (info->clk_req_gpio)
		info->clk_req_irq = gpiod_to_irq(info->clk_req_gpio);

	info->clk = devm_clk_get_optional(dev, "sclk_isp_sensor1");
	if (IS_ERR(info->clk))
		return dev_err_probe(dev, PTR_ERR(info->clk), "nfc clock\n");

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sec_nfc_irq_thread_fn,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						SEC_NFC_DRIVER_NAME, info);
		if (ret)
			return dev_err_probe(dev, ret, "request nfc irq\n");
	}

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = SEC_NFC_DRIVER_NAME;
	info->miscdev.fops = &sec_nfc_fops;
	info->miscdev.parent = dev;

	ret = misc_register(&info->miscdev);
	if (ret)
		return dev_err_probe(dev, ret, "misc_register\n");

	dev_info(dev, "Samsung S3FWRN5 NFC (/dev/%s) ready at 0x%02x (irq=%d, clk_req_irq=%d)\n",
		 SEC_NFC_DRIVER_NAME, client->addr, client->irq, info->clk_req_irq);
	return 0;
}

static void sec_nfc_remove(struct i2c_client *client)
{
	struct sec_nfc_info *info = i2c_get_clientdata(client);

	misc_deregister(&info->miscdev);
	mutex_lock(&info->mutex);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	mutex_unlock(&info->mutex);
	if (info->vdd)
		regulator_disable(info->vdd);
}

static const struct of_device_id sec_nfc_of_match[] = {
	{ .compatible = "samsung,sec-nfc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sec_nfc_of_match);

static const struct i2c_device_id sec_nfc_id[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_nfc_id);

static struct i2c_driver sec_nfc_driver = {
	.driver = {
		.name = SEC_NFC_DRIVER_NAME,
		.of_match_table = sec_nfc_of_match,
	},
	.probe = sec_nfc_probe,
	.remove = sec_nfc_remove,
	.id_table = sec_nfc_id,
};
module_i2c_driver(sec_nfc_driver);

MODULE_DESCRIPTION("Samsung S3FWRN5 NFC Controller Driver (/dev/sec-nfc)");
MODULE_LICENSE("GPL");
