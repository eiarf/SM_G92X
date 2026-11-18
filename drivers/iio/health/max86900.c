// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim MAX86900 / MAX86900A Optical Heart-Rate & SpO2 Sensor IIO Driver
 * for Samsung Galaxy S6 (Exynos 7420 / zeroflte) on Linux 6.6.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#define MAX86900_REG_INT_STATUS		0x00
#define MAX86900_REG_INT_ENABLE		0x01
#define MAX86900_REG_FIFO_WR_PTR	0x02
#define MAX86900_REG_OVF_COUNTER	0x03
#define MAX86900_REG_FIFO_RD_PTR	0x04
#define MAX86900_REG_FIFO_DATA		0x05
#define MAX86900_REG_MODE_CONFIG	0x06
#define MAX86900_REG_SPO2_CONFIG	0x07
#define MAX86900_REG_LED_CONFIG		0x09
#define MAX86900_REG_TEMP_INT		0x16
#define MAX86900_REG_TEMP_FRAC		0x17
#define MAX86900_REG_REV_ID		0xFE
#define MAX86900_REG_PART_ID		0xFF

#define MAX86900_MODE_RESET		0x40
#define MAX86900_MODE_SHDN		0x80
#define MAX86900_MODE_SPO2_EN		0x03
#define MAX86900_MODE_SPO2_TEMP		0x0B

/* 100 Hz sample rate, 400 us pulse width (16-bit ADC) */
#define MAX86900_SPO2_100HZ_400US	0x47
#define MAX86900_DEFAULT_LED_CURR	0x55
#define MAX86900_FINGER_IR_THRESH	6000

struct max86900_data {
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct regulator *vdd;
	struct regulator *led;
	struct mutex lock;
	struct delayed_work poll_work;

	bool streaming;
	u8 part_id;
	u8 rev_id;
	u8 led_current;

	/* Last raw readings */
	u16 last_ir;
	u16 last_red;
	s32 last_bpm;

	/* Simple PPG peak detector state (100 Hz -> 25 Hz averaged) */
	s64 dc_ir;
	s32 prev_ac;
	s32 prev2_ac;
	s64 last_peak_ns;
	s32 bpm_hist[4];
	int bpm_hist_idx;
	int bpm_hist_cnt;
	int sample_div;
};

struct max86900_scan {
	s32 bpm;
	u16 ir;
	u16 red;
	s64 ts __aligned(8);
};

static const struct iio_chan_spec max86900_channels[] = {
	{
		.type = IIO_COUNT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_RED,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 2,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = -1,
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int max86900_write_reg(struct max86900_data *st, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(st->client, reg, val);
	if (ret < 0)
		dev_err(&st->client->dev, "write reg 0x%02x=0x%02x failed: %d\n",
			reg, val, ret);
	return ret;
}

static int max86900_read_reg(struct max86900_data *st, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(st->client, reg);

	if (ret < 0)
		dev_err(&st->client->dev, "read reg 0x%02x failed: %d\n",
			reg, ret);
	return ret;
}

static int max86900_power_on(struct max86900_data *st)
{
	int ret;

	if (st->vdd) {
		ret = regulator_enable(st->vdd);
		if (ret)
			return ret;
	}
	if (st->led) {
		ret = regulator_enable(st->led);
		if (ret) {
			if (st->vdd)
				regulator_disable(st->vdd);
			return ret;
		}
	}
	msleep(15);
	return 0;
}

static void max86900_power_off(struct max86900_data *st)
{
	if (st->led)
		regulator_disable(st->led);
	if (st->vdd)
		regulator_disable(st->vdd);
}

static int max86900_hw_start(struct max86900_data *st)
{
	int ret;

	ret = max86900_power_on(st);
	if (ret)
		return ret;

	ret = max86900_write_reg(st, MAX86900_REG_MODE_CONFIG, MAX86900_MODE_RESET);
	if (ret)
		goto err_off;
	usleep_range(2000, 3000);

	(void)max86900_read_reg(st, MAX86900_REG_INT_STATUS);

	ret = max86900_write_reg(st, MAX86900_REG_SPO2_CONFIG, MAX86900_SPO2_100HZ_400US);
	if (ret)
		goto err_off;

	ret = max86900_write_reg(st, MAX86900_REG_LED_CONFIG, st->led_current);
	if (ret)
		goto err_off;

	max86900_write_reg(st, MAX86900_REG_FIFO_WR_PTR, 0x00);
	max86900_write_reg(st, MAX86900_REG_OVF_COUNTER, 0x00);
	max86900_write_reg(st, MAX86900_REG_FIFO_RD_PTR, 0x00);
	max86900_write_reg(st, MAX86900_REG_INT_ENABLE, 0x10);

	ret = max86900_write_reg(st, MAX86900_REG_MODE_CONFIG, MAX86900_MODE_SPO2_TEMP);
	if (ret)
		goto err_off;

	return 0;

err_off:
	max86900_power_off(st);
	return ret;
}

static void max86900_hw_stop(struct max86900_data *st)
{
	max86900_write_reg(st, MAX86900_REG_INT_ENABLE, 0x00);
	max86900_write_reg(st, MAX86900_REG_LED_CONFIG, 0x00);
	max86900_write_reg(st, MAX86900_REG_MODE_CONFIG, MAX86900_MODE_SHDN);
	max86900_power_off(st);
}

static int max86900_read_fifo_sample(struct max86900_data *st, u16 *ir, u16 *red)
{
	u8 buf[4];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(st->client, MAX86900_REG_FIFO_DATA,
					    sizeof(buf), buf);
	if (ret < 0)
		return ret;
	if (ret != sizeof(buf))
		return -EIO;

	*ir = ((u16)buf[0] << 8) | buf[1];
	*red = ((u16)buf[2] << 8) | buf[3];
	return 0;
}

static void max86900_update_bpm(struct max86900_data *st, u16 ir, s64 now_ns)
{
	s32 ac;

	if (ir < MAX86900_FINGER_IR_THRESH) {
		st->dc_ir = 0;
		st->prev_ac = 0;
		st->prev2_ac = 0;
		st->last_peak_ns = 0;
		st->bpm_hist_cnt = 0;
		st->bpm_hist_idx = 0;
		st->last_bpm = 0;
		return;
	}

	if (st->dc_ir == 0)
		st->dc_ir = (s64)ir << 8;
	else
		st->dc_ir += (((s64)ir << 8) - st->dc_ir) >> 4;

	ac = (s32)ir - (s32)(st->dc_ir >> 8);

	/* Local peak detection with refractory period (300 ms .. 1500 ms -> 40..200 BPM) */
	if (st->prev_ac > st->prev2_ac && st->prev_ac > ac && st->prev_ac > 15) {
		if (st->last_peak_ns > 0) {
			s64 dt_ms = div_s64(now_ns - st->last_peak_ns, 1000000);

			if (dt_ms >= 300 && dt_ms <= 1500) {
				s32 inst_bpm = (s32)div_s64(60000, dt_ms);
				int i, sum = 0;

				st->bpm_hist[st->bpm_hist_idx] = inst_bpm;
				st->bpm_hist_idx = (st->bpm_hist_idx + 1) & 3;
				if (st->bpm_hist_cnt < 4)
					st->bpm_hist_cnt++;
				for (i = 0; i < st->bpm_hist_cnt; i++)
					sum += st->bpm_hist[i];
				st->last_bpm = sum / st->bpm_hist_cnt;
				st->last_peak_ns = now_ns;
			} else if (dt_ms > 1500) {
				st->last_peak_ns = now_ns;
			}
		} else {
			st->last_peak_ns = now_ns;
		}
	}

	st->prev2_ac = st->prev_ac;
	st->prev_ac = ac;
}

static void max86900_poll_work(struct work_struct *work)
{
	struct max86900_data *st = container_of(work, struct max86900_data,
						poll_work.work);
	struct max86900_scan scan;
	int wr, rd, count, i;
	s64 now_ns;

	mutex_lock(&st->lock);
	if (!st->streaming) {
		mutex_unlock(&st->lock);
		return;
	}

	wr = max86900_read_reg(st, MAX86900_REG_FIFO_WR_PTR);
	rd = max86900_read_reg(st, MAX86900_REG_FIFO_RD_PTR);
	if (wr < 0 || rd < 0)
		goto reschedule;

	count = (wr - rd) & 0x0F;
	if (count == 0)
		count = 1;

	now_ns = ktime_get_boottime_ns();
	for (i = 0; i < count; i++) {
		u16 ir = 0, red = 0;

		if (max86900_read_fifo_sample(st, &ir, &red) < 0)
			break;
		st->last_ir = ir;
		st->last_red = red;
		max86900_update_bpm(st, ir, now_ns);
	}

	/* Push ~10 Hz frames into IIO kfifo */
	memset(&scan, 0, sizeof(scan));
	scan.bpm = st->last_bpm;
	scan.ir = st->last_ir;
	scan.red = st->last_red;
	iio_push_to_buffers_with_timestamp(st->indio_dev, &scan, now_ns);

reschedule:
	if (st->streaming)
		schedule_delayed_work(&st->poll_work, msecs_to_jiffies(40));
	mutex_unlock(&st->lock);
}

static int max86900_buffer_postenable(struct iio_dev *indio_dev)
{
	struct max86900_data *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	st->dc_ir = 0;
	st->prev_ac = 0;
	st->prev2_ac = 0;
	st->last_peak_ns = 0;
	st->bpm_hist_cnt = 0;
	st->bpm_hist_idx = 0;
	st->last_bpm = 0;

	ret = max86900_hw_start(st);
	if (!ret) {
		st->streaming = true;
		schedule_delayed_work(&st->poll_work, msecs_to_jiffies(40));
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int max86900_buffer_predisable(struct iio_dev *indio_dev)
{
	struct max86900_data *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	st->streaming = false;
	mutex_unlock(&st->lock);

	cancel_delayed_work_sync(&st->poll_work);

	mutex_lock(&st->lock);
	max86900_hw_stop(st);
	mutex_unlock(&st->lock);
	return 0;
}

static const struct iio_buffer_setup_ops max86900_buffer_ops = {
	.postenable = max86900_buffer_postenable,
	.predisable = max86900_buffer_predisable,
};

static int max86900_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max86900_data *st = iio_priv(indio_dev);
	bool started_temp = false;
	int ret = 0;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (!st->streaming) {
		ret = max86900_hw_start(st);
		if (ret) {
			mutex_unlock(&st->lock);
			return ret;
		}
		msleep(30);
		(void)max86900_read_fifo_sample(st, &st->last_ir, &st->last_red);
		started_temp = true;
	}

	switch (chan->type) {
	case IIO_COUNT:
		*val = st->last_bpm;
		ret = IIO_VAL_INT;
		break;
	case IIO_INTENSITY:
		*val = (chan->channel2 == IIO_MOD_LIGHT_IR) ?
		       st->last_ir : st->last_red;
		ret = IIO_VAL_INT;
		break;
	case IIO_TEMP: {
		int ti = max86900_read_reg(st, MAX86900_REG_TEMP_INT);
		int tf = max86900_read_reg(st, MAX86900_REG_TEMP_FRAC);

		if (ti < 0 || tf < 0)
			ret = -EIO;
		else {
			*val = (s8)ti * 1000 + (tf & 0x0F) * 62;
			ret = IIO_VAL_INT;
		}
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	if (started_temp)
		max86900_hw_stop(st);
	mutex_unlock(&st->lock);
	return ret;
}

static const struct iio_info max86900_iio_info = {
	.read_raw = max86900_read_raw,
};

static int max86900_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct max86900_data *st;
	int part, rev, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	st->indio_dev = indio_dev;
	st->led_current = MAX86900_DEFAULT_LED_CURR;
	mutex_init(&st->lock);
	INIT_DELAYED_WORK(&st->poll_work, max86900_poll_work);
	i2c_set_clientdata(client, indio_dev);

	st->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(st->vdd)) {
		if (PTR_ERR(st->vdd) == -ENODEV)
			st->vdd = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(st->vdd), "vdd regulator\n");
	}
	st->led = devm_regulator_get_optional(dev, "led");
	if (IS_ERR(st->led)) {
		if (PTR_ERR(st->led) == -ENODEV)
			st->led = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(st->led), "led regulator\n");
	}

	/* Probe chip ID with rails enabled: try MAX86900A/B/C (0x57) first, then MAX86900 (0x51) */
	ret = max86900_power_on(st);
	if (ret)
		return ret;

	client->addr = 0x57;
	rev = i2c_smbus_read_byte_data(client, MAX86900_REG_REV_ID);
	part = i2c_smbus_read_byte_data(client, MAX86900_REG_PART_ID);
	if (rev < 0 || part < 0 || part != 0x11) {
		client->addr = 0x51;
		rev = i2c_smbus_read_byte_data(client, MAX86900_REG_REV_ID);
		part = i2c_smbus_read_byte_data(client, MAX86900_REG_PART_ID);
	}
	if (rev >= 0 && part >= 0) {
		if (client->addr == 0x57 && rev == 0x05)
			st->led_current = 0x0F;
		else if (client->addr == 0x57)
			st->led_current = 0x55;
		max86900_write_reg(st, MAX86900_REG_MODE_CONFIG, MAX86900_MODE_SHDN);
	}
	max86900_power_off(st);

	if (rev < 0 || part < 0)
		return dev_err_probe(dev, rev < 0 ? rev : part,
				     "failed to read MAX86900 ID at 0x57/0x51\n");

	st->rev_id = rev;
	st->part_id = part;
	dev_info(dev, "MAX86900 detected at 0x%02x (part=0x%02x, rev=0x%02x, led=0x%02x)\n", client->addr,
		 st->part_id, st->rev_id, st->led_current);

	indio_dev->name = "max86900-hrm";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = max86900_channels;
	indio_dev->num_channels = ARRAY_SIZE(max86900_channels);
	indio_dev->info = &max86900_iio_info;

	ret = iio_device_set_clock(indio_dev, CLOCK_BOOTTIME);
	if (ret)
		return ret;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev, &max86900_buffer_ops);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static void max86900_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct max86900_data *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	st->streaming = false;
	mutex_unlock(&st->lock);
	cancel_delayed_work_sync(&st->poll_work);
}

static const struct of_device_id max86900_of_match[] = {
	{ .compatible = "maxim,max86900" },
	{ }
};
MODULE_DEVICE_TABLE(of, max86900_of_match);

static const struct i2c_device_id max86900_id[] = {
	{ "max86900", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max86900_id);

static struct i2c_driver max86900_driver = {
	.driver = {
		.name = "max86900",
		.of_match_table = max86900_of_match,
	},
	.probe = max86900_probe,
	.remove = max86900_remove,
	.id_table = max86900_id,
};
module_i2c_driver(max86900_driver);

MODULE_DESCRIPTION("Maxim MAX86900 Optical Heart-Rate & SpO2 IIO Driver");
MODULE_LICENSE("GPL");
