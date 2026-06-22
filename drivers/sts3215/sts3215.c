// SPDX-License-Identifier: GPL-2.0
/*
 * Feetech STS3215 serial bus servo driver (serdev, half-duplex 1-wire)
 *
 * Target: ROCK 3C / RK3566, kernel 5.10.160 (no CONFIG_OF_OVERLAY)
 * Bound to uart3 (serial@fe670000) via a "malus,sts3215" child node in the DT.
 *
 * Protocol: FeeTech SCS/STS packet
 *   0xFF 0xFF  ID  LEN  INSTR  PARAM...  CHECKSUM
 *   LEN      = nparams + 2   (instruction + params + checksum)
 *   CHECKSUM = ~(ID + LEN + INSTR + sum(PARAM)) & 0xFF
 *   STS3215 multi-byte values are LITTLE-endian.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/device.h>

/* ---- bus / servo defaults ---- */
#define STS_BAUD               1000000
#define STS_DEFAULT_ID         1
#define STS_TX_TIMEOUT_MS      100

/* ---- SMS/STS register map ---- */
#define STS_REG_TORQUE_ENABLE  0x28   /* 40, 1 byte                */
#define STS_REG_ACC            0x29   /* 41, 1 byte                */
#define STS_REG_GOAL_POSITION  0x2A   /* 42, 2 bytes LE (0..4095)  */
#define STS_REG_GOAL_TIME      0x2C   /* 44, 2 bytes LE            */
#define STS_REG_GOAL_SPEED     0x2E   /* 46, 2 bytes LE            */

/* ---- instructions ---- */
#define STS_INST_PING          0x01
#define STS_INST_READ          0x02
#define STS_INST_WRITE         0x03

/* ---- motion defaults ---- */
#define STS_POS_MIN            0
#define STS_POS_MAX            4095
#define STS_DEFAULT_SPEED      2400
#define STS_DEFAULT_ACC        50

static bool selftest = true;
module_param(selftest, bool, 0444);
MODULE_PARM_DESC(selftest, "Run a position sweep at probe to verify motion (default on)");

struct sts3215 {
	struct serdev_device *serdev;
	struct device        *dev;
	struct mutex          lock;
	u8                    id;
	u16                   last_pos;
	bool                  torque_on;
};

/*
 * Build one packet and send it. For a WRITE, params[0] is the register
 * address followed by the data bytes.
 */
static int sts3215_send(struct sts3215 *st, u8 instr,
			const u8 *params, u8 nparams)
{
	u8 buf[16];
	u8 len, csum;
	int i, n = 0, ret;

	if (nparams > sizeof(buf) - 6)
		return -EINVAL;

	len = nparams + 2;		/* instruction + params + checksum */

	buf[n++] = 0xFF;
	buf[n++] = 0xFF;
	buf[n++] = st->id;
	buf[n++] = len;
	buf[n++] = instr;
	for (i = 0; i < nparams; i++)
		buf[n++] = params[i];

	csum = st->id + len + instr;
	for (i = 0; i < nparams; i++)
		csum += params[i];
	buf[n++] = ~csum;

	ret = serdev_device_write(st->serdev, buf, n,
				  msecs_to_jiffies(STS_TX_TIMEOUT_MS));
	if (ret < 0) {
		dev_err(st->dev, "serdev_device_write failed: %d\n", ret);
		return ret;
	}
	if (ret < n) {
		dev_err(st->dev, "short write %d/%d\n", ret, n);
		return -EIO;
	}

	/* flush the UART FIFO before returning (matters for half-duplex RX) */
	serdev_device_wait_until_sent(st->serdev,
				      msecs_to_jiffies(STS_TX_TIMEOUT_MS));
	return 0;
}

static int sts3215_write_reg8(struct sts3215 *st, u8 reg, u8 val)
{
	u8 p[2] = { reg, val };

	return sts3215_send(st, STS_INST_WRITE, p, 2);
}

static int sts3215_torque(struct sts3215 *st, bool on)
{
	int ret = sts3215_write_reg8(st, STS_REG_TORQUE_ENABLE, on ? 1 : 0);

	if (!ret)
		st->torque_on = on;
	return ret;
}

/*
 * Move to position [0..4095]. Writes ACC, then position(2)+time(2)+speed(2)
 * starting at reg 0x2A in one WRITE, mirroring FeeTech WritePosEx().
 */
static int sts3215_move(struct sts3215 *st, u16 pos, u16 speed, u8 acc)
{
	u8 p[8];
	int ret;

	if (pos > STS_POS_MAX)
		pos = STS_POS_MAX;

	ret = sts3215_write_reg8(st, STS_REG_ACC, acc);
	if (ret)
		return ret;

	p[0] = STS_REG_GOAL_POSITION;
	p[1] = pos & 0xFF;		/* position L */
	p[2] = (pos >> 8) & 0xFF;	/* position H */
	p[3] = 0;			/* time L     */
	p[4] = 0;			/* time H     */
	p[5] = speed & 0xFF;		/* speed L    */
	p[6] = (speed >> 8) & 0xFF;	/* speed H    */

	ret = sts3215_send(st, STS_INST_WRITE, p, 7);
	if (!ret)
		st->last_pos = pos;
	return ret;
}

/* ---------------- sysfs (live control, no reboot needed) ---------------- */

static ssize_t torque_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", st->torque_on ? 1 : 0);
}

static ssize_t torque_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = sts3215_torque(st, on);
	mutex_unlock(&st->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(torque_enable);

static ssize_t goal_position_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", st->last_pos);
}

static ssize_t goal_position_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret)
		return ret;
	if (v > STS_POS_MAX)
		return -ERANGE;

	mutex_lock(&st->lock);
	if (!st->torque_on)
		sts3215_torque(st, true);
	ret = sts3215_move(st, (u16)v, STS_DEFAULT_SPEED, STS_DEFAULT_ACC);
	mutex_unlock(&st->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(goal_position);

static struct attribute *sts3215_attrs[] = {
	&dev_attr_torque_enable.attr,
	&dev_attr_goal_position.attr,
	NULL,
};
static const struct attribute_group sts3215_group = {
	.attrs = sts3215_attrs,
};

/* ---------------- serdev callbacks ---------------- */

static int sts3215_receive_buf(struct serdev_device *serdev,
			       const unsigned char *buf, size_t count)
{
	struct sts3215 *st = serdev_device_get_drvdata(serdev);

	/*
	 * 1-wire half-duplex: RX sees our own TX echo plus any servo reply.
	 * Milestone 2 is write-only, so we just consume and log.
	 */
	if (st)
		dev_dbg(st->dev, "rx %zu bytes\n", count);
	return count;
}

static const struct serdev_device_ops sts3215_serdev_ops = {
	.receive_buf  = sts3215_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,   /* <-- the fix for -EINVAL */
};

/* ---------------- probe / remove ---------------- */

static void sts3215_selftest(struct sts3215 *st)
{
	static const u16 seq[] = { 2048, 1024, 3072, 2048 };
	int i;

	mutex_lock(&st->lock);
	sts3215_torque(st, true);
	mutex_unlock(&st->lock);

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		mutex_lock(&st->lock);
		sts3215_move(st, seq[i], STS_DEFAULT_SPEED, STS_DEFAULT_ACC);
		mutex_unlock(&st->lock);
		msleep(600);
	}
}

static int sts3215_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct sts3215 *st;
	u32 baud;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->serdev = serdev;
	st->dev    = dev;
	st->id     = STS_DEFAULT_ID;
	mutex_init(&st->lock);
	serdev_device_set_drvdata(serdev, st);

	serdev_device_set_client_ops(serdev, &sts3215_serdev_ops);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(dev, "serdev_device_open failed: %d\n", ret);
		return ret;
	}

	baud = serdev_device_set_baudrate(serdev, STS_BAUD);
	dev_info(dev, "baud requested %d, got %u\n", STS_BAUD, baud);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	ret = devm_device_add_group(dev, &sts3215_group);
	if (ret) {
		dev_err(dev, "sysfs group failed: %d\n", ret);
		goto err_close;
	}

	dev_info(dev, "sts3215 ready (id=%d)\n", st->id);

	if (selftest)
		sts3215_selftest(st);

	return 0;

err_close:
	serdev_device_close(serdev);
	return ret;
}

static void sts3215_remove(struct serdev_device *serdev)
{
	struct sts3215 *st = serdev_device_get_drvdata(serdev);

	if (st) {
		mutex_lock(&st->lock);
		sts3215_torque(st, false);
		mutex_unlock(&st->lock);
	}
	serdev_device_close(serdev);
}

static const struct of_device_id sts3215_of_match[] = {
	{ .compatible = "malus,sts3215" },
	{ }
};
MODULE_DEVICE_TABLE(of, sts3215_of_match);

static struct serdev_device_driver sts3215_driver = {
	.probe  = sts3215_probe,
	.remove = sts3215_remove,
	.driver = {
		.name           = "sts3215",
		.of_match_table = sts3215_of_match,
	},
};
module_serdev_device_driver(sts3215_driver);

MODULE_AUTHOR("malus");
MODULE_DESCRIPTION("Feetech STS3215 serial bus servo serdev driver");
MODULE_LICENSE("GPL");
