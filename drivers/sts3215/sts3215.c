// SPDX-License-Identifier: GPL-2.0
/*
 * Feetech STS3215 serial bus servo driver (serdev, half-duplex 1-wire)
 *
 * Target: ROCK 3C / RK3566, kernel 5.10.160 (no CONFIG_OF_OVERLAY)
 * Bound to uart3 (serial@fe670000) via a "malus,sts3215" child node in the DT.
 *
 * Protocol: FeeTech SCS/STS half-duplex packet
 *   request : 0xFF 0xFF  ID  LEN  INSTR  PARAM...  CHECKSUM
 *   response: 0xFF 0xFF  ID  LEN  ERR    PARAM...  CHECKSUM
 *   LEN      = nparams + 2
 *   CHECKSUM = ~(ID + LEN + INSTR/ERR + sum(PARAM)) & 0xFF
 *   STS3215 multi-byte values are LITTLE-endian.
 *
 * Single-wire half-duplex: every byte we transmit is echoed back on RX before
 * the servo's reply arrives.  We drop exactly TX-length bytes (rx_skip), then a
 * byte-wise state machine reassembles the reply frame and a completion wakes the
 * waiting reader.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/device.h>

/* ---- bus / servo defaults ---- */
#define STS_BAUD               115200	/* servo EPROM reg 0x06 set to 4 (115200) */
#define STS_DEFAULT_ID         1
#define STS_TX_TIMEOUT_MS      100
#define STS_RX_TIMEOUT_MS      50

/* ---- register map (write) ---- */
#define STS_REG_TORQUE_ENABLE  0x28   /* 40, 1 byte                */
#define STS_REG_ACC            0x29   /* 41, 1 byte                */
#define STS_REG_GOAL_POSITION  0x2A   /* 42, 2 bytes LE (0..4095)  */

/* ---- register map (read / feedback, SRAM) ---- */
#define STS_REG_PRESENT_POSITION 0x38 /* 56, 2 bytes LE            */
#define STS_REG_PRESENT_SPEED    0x3A /* 58, 2 bytes, sign-mag     */
#define STS_REG_PRESENT_LOAD     0x3C /* 60, 2 bytes, sign-mag     */
#define STS_REG_PRESENT_VOLTAGE  0x3E /* 62, 1 byte, 0.1V units    */
#define STS_REG_PRESENT_TEMP     0x3F /* 63, 1 byte, deg C         */
#define STS_REG_PRESENT_CURRENT  0x45 /* 69, 2 bytes LE, 6.5mA/LSB */

/* ---- instructions ---- */
#define STS_INST_PING          0x01
#define STS_INST_READ          0x02
#define STS_INST_WRITE         0x03

/* ---- motion defaults ---- */
#define STS_POS_MIN            0
#define STS_POS_MAX            4095
#define STS_DEFAULT_SPEED      2400
#define STS_DEFAULT_ACC        50

/* ---- rx frame state machine ---- */
enum sts_rx_state {
	RX_H1 = 0,	/* waiting for first  0xFF */
	RX_H2,		/* waiting for second 0xFF */
	RX_ID,
	RX_LEN,
	RX_ERR,
	RX_DATA,
	RX_CSUM,
};

static bool selftest = true;
module_param(selftest, bool, 0444);
MODULE_PARM_DESC(selftest, "Run a position sweep at probe to verify motion (default on)");

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Dump raw RX bytes on read/ping for protocol debugging (default off)");

struct sts3215 {
	struct serdev_device *serdev;
	struct device        *dev;
	struct mutex          lock;	/* serialize whole bus transactions */
	u8                    id;
	u16                   last_pos;
	bool                  torque_on;

	/* rx path, shared with receive_buf */
	spinlock_t            rx_lock;
	struct completion     rx_done;
	int                   rx_skip;	/* TX-echo bytes still to drop */
	int                   rx_state;
	u8                    rx_id;
	u8                    rx_flen;
	u8                    rx_err;
	u8                    rx_sum;
	int                   rx_ndata;
	int                   rx_dcount;
	u8                    rx_params[16];
	bool                  rx_ok;

	/* debug capture (filled per-transaction, dumped on timeout) */
	int                   rx_total;
	int                   dbg_len;
	u8                    dbg[32];
};

/*
 * Low-level transmit. Always primes rx_skip with the TX length so the byte
 * echoed back on the single wire is discarded, and resets the frame parser.
 */
static int sts3215_tx(struct sts3215 *st, const u8 *buf, int n)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&st->rx_lock, flags);
	st->rx_skip   = n;
	st->rx_state  = RX_H1;
	st->rx_ndata  = 0;
	st->rx_dcount = 0;
	st->rx_ok     = false;
	st->rx_total  = 0;
	st->dbg_len   = 0;
	spin_unlock_irqrestore(&st->rx_lock, flags);

	reinit_completion(&st->rx_done);

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

	/* flush UART FIFO before the line turns around for RX */
	serdev_device_wait_until_sent(st->serdev,
				      msecs_to_jiffies(STS_TX_TIMEOUT_MS));
	return 0;
}

/* Build and send one packet. params[] already includes the register address. */
static int sts3215_send(struct sts3215 *st, u8 instr,
			const u8 *params, u8 nparams)
{
	u8 buf[16];
	u8 len, csum;
	int i, n = 0;

	if (nparams > sizeof(buf) - 6)
		return -EINVAL;

	len = nparams + 2;

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

	return sts3215_tx(st, buf, n);
}

/*
 * Issue a READ and wait for the reply. Returns rlen bytes in out[].
 * Caller must hold st->lock.
 */
static int sts3215_read_block(struct sts3215 *st, u8 reg, u8 rlen, u8 *out)
{
	u8 buf[8];
	u8 csum;
	int n = 0, ret;
	unsigned long flags;
	long t;

	if (rlen == 0 || rlen > sizeof(st->rx_params))
		return -EINVAL;

	buf[n++] = 0xFF;
	buf[n++] = 0xFF;
	buf[n++] = st->id;
	buf[n++] = 4;			/* params (reg,rlen) = 2, +2 */
	buf[n++] = STS_INST_READ;
	buf[n++] = reg;
	buf[n++] = rlen;
	csum = st->id + 4 + STS_INST_READ + reg + rlen;
	buf[n++] = ~csum;

	ret = sts3215_tx(st, buf, n);
	if (ret)
		return ret;

	t = wait_for_completion_timeout(&st->rx_done,
					msecs_to_jiffies(STS_RX_TIMEOUT_MS));
	if (t == 0) {
		if (debug) {
			char hex[3 * 32 + 1];
			int k, p = 0;

			spin_lock_irqsave(&st->rx_lock, flags);
			for (k = 0; k < st->dbg_len; k++)
				p += scnprintf(hex + p, sizeof(hex) - p,
					       "%02x ", st->dbg[k]);
			dev_warn(st->dev,
				 "read 0x%02x timeout: total=%d state=%d skip=%d bytes=[%s]\n",
				 reg, st->rx_total, st->rx_state, st->rx_skip, hex);
			spin_unlock_irqrestore(&st->rx_lock, flags);
		} else {
			dev_warn(st->dev, "read 0x%02x: no response (timeout)\n", reg);
		}
		return -ETIMEDOUT;
	}

	spin_lock_irqsave(&st->rx_lock, flags);
	if (!st->rx_ok || st->rx_id != st->id || st->rx_ndata < rlen) {
		spin_unlock_irqrestore(&st->rx_lock, flags);
		dev_warn(st->dev, "read reg 0x%02x: bad frame (ok=%d id=%d n=%d)\n",
			 reg, st->rx_ok, st->rx_id, st->rx_ndata);
		return -EIO;
	}
	memcpy(out, st->rx_params, rlen);
	spin_unlock_irqrestore(&st->rx_lock, flags);
	return 0;
}

static int sts3215_read_u8(struct sts3215 *st, u8 reg, u8 *val)
{
	u8 b[1];
	int ret = sts3215_read_block(st, reg, 1, b);

	if (!ret)
		*val = b[0];
	return ret;
}

static int sts3215_read_u16(struct sts3215 *st, u8 reg, u16 *val)
{
	u8 b[2];
	int ret = sts3215_read_block(st, reg, 2, b);

	if (!ret)
		*val = b[0] | (b[1] << 8);
	return ret;
}

/* speed: 15-bit magnitude + sign bit (bit15) */
static int sts3215_decode_speed(u16 raw)
{
	int mag = raw & 0x7FFF;

	return (raw & 0x8000) ? -mag : mag;
}

/* load: 10-bit magnitude (0..1000) + direction bit (bit10) */
static int sts3215_decode_load(u16 raw)
{
	int mag = raw & 0x3FF;

	return (raw & 0x400) ? -mag : mag;
}

/* ---------------- write helpers ---------------- */

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
	p[1] = pos & 0xFF;
	p[2] = (pos >> 8) & 0xFF;
	p[3] = 0;			/* time L */
	p[4] = 0;			/* time H */
	p[5] = speed & 0xFF;
	p[6] = (speed >> 8) & 0xFF;

	ret = sts3215_send(st, STS_INST_WRITE, p, 7);
	if (!ret)
		st->last_pos = pos;
	return ret;
}

/* ---------------- sysfs: control (RW) ---------------- */

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

/* ---------------- sysfs: feedback (RO) ---------------- */

static ssize_t present_position_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u16 v;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_u16(st, STS_REG_PRESENT_POSITION, &v);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR_RO(present_position);

static ssize_t present_speed_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u16 v;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_u16(st, STS_REG_PRESENT_SPEED, &v);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%d\n", sts3215_decode_speed(v));
}
static DEVICE_ATTR_RO(present_speed);

static ssize_t present_load_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u16 v;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_u16(st, STS_REG_PRESENT_LOAD, &v);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%d\n", sts3215_decode_load(v));
}
static DEVICE_ATTR_RO(present_load);

static ssize_t voltage_mv_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u8 v;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_u8(st, STS_REG_PRESENT_VOLTAGE, &v);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%u\n", v * 100);	/* 0.1V -> mV */
}
static DEVICE_ATTR_RO(voltage_mv);

static ssize_t current_ma_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u16 v;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_u16(st, STS_REG_PRESENT_CURRENT, &v);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%u\n", (v * 13) / 2);	/* 6.5mA/LSB */
}
static DEVICE_ATTR_RO(current_ma);

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u8 v;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_u8(st, STS_REG_PRESENT_TEMP, &v);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%u\n", v);		/* deg C */
}
static DEVICE_ATTR_RO(temperature);

static ssize_t feedback_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct sts3215 *st = dev_get_drvdata(dev);
	u8 blk[8], cur[2];
	u16 pos, spd_raw, load_raw, cur_raw;
	u8 volt, temp;
	int ret;

	mutex_lock(&st->lock);
	ret = sts3215_read_block(st, STS_REG_PRESENT_POSITION, 8, blk);
	if (!ret)
		ret = sts3215_read_block(st, STS_REG_PRESENT_CURRENT, 2, cur);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;

	pos      = blk[0] | (blk[1] << 8);
	spd_raw  = blk[2] | (blk[3] << 8);
	load_raw = blk[4] | (blk[5] << 8);
	volt     = blk[6];
	temp     = blk[7];
	cur_raw  = cur[0] | (cur[1] << 8);

	return scnprintf(buf, PAGE_SIZE,
		"position=%u speed=%d load=%d voltage=%u.%uV current=%umA temperature=%uC\n",
		pos,
		sts3215_decode_speed(spd_raw),
		sts3215_decode_load(load_raw),
		volt / 10, volt % 10,
		(cur_raw * 13) / 2,
		temp);
}
static DEVICE_ATTR_RO(feedback);

static struct attribute *sts3215_attrs[] = {
	&dev_attr_torque_enable.attr,
	&dev_attr_goal_position.attr,
	&dev_attr_present_position.attr,
	&dev_attr_present_speed.attr,
	&dev_attr_present_load.attr,
	&dev_attr_voltage_mv.attr,
	&dev_attr_current_ma.attr,
	&dev_attr_temperature.attr,
	&dev_attr_feedback.attr,
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
	unsigned long flags;
	size_t i;

	if (!st)
		return count;

	spin_lock_irqsave(&st->rx_lock, flags);
	for (i = 0; i < count; i++) {
		u8 b = buf[i];

		/* capture everything (echo + reply) for diagnostics */
		st->rx_total++;
		if (st->dbg_len < (int)sizeof(st->dbg))
			st->dbg[st->dbg_len++] = b;

		/* discard our own TX echo (single-wire half-duplex) */
		if (st->rx_skip > 0) {
			st->rx_skip--;
			continue;
		}

		switch (st->rx_state) {
		case RX_H1:
			if (b == 0xFF)
				st->rx_state = RX_H2;
			break;
		case RX_H2:
			st->rx_state = (b == 0xFF) ? RX_ID : RX_H1;
			break;
		case RX_ID:
			st->rx_id  = b;
			st->rx_sum = b;
			st->rx_state = RX_LEN;
			break;
		case RX_LEN:
			st->rx_flen = b;
			st->rx_sum += b;
			st->rx_state = RX_ERR;
			break;
		case RX_ERR:
			st->rx_err = b;
			st->rx_sum += b;
			st->rx_dcount = 0;
			st->rx_ndata  = (st->rx_flen >= 2) ? (st->rx_flen - 2) : 0;
			if (st->rx_ndata > (int)sizeof(st->rx_params))
				st->rx_ndata = sizeof(st->rx_params);
			st->rx_state = st->rx_ndata > 0 ? RX_DATA : RX_CSUM;
			break;
		case RX_DATA:
			if (st->rx_dcount < (int)sizeof(st->rx_params))
				st->rx_params[st->rx_dcount] = b;
			st->rx_sum += b;
			if (++st->rx_dcount >= st->rx_ndata)
				st->rx_state = RX_CSUM;
			break;
		case RX_CSUM:
			st->rx_ok = (b == (u8)~st->rx_sum);
			st->rx_state = RX_H1;
			complete(&st->rx_done);
			break;
		}
	}
	spin_unlock_irqrestore(&st->rx_lock, flags);
	return count;
}

static const struct serdev_device_ops sts3215_serdev_ops = {
	.receive_buf  = sts3215_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/* ---------------- probe / remove ---------------- */

/*
 * Diagnostic PING: send a bare PING and dump whatever comes back. PING reads
 * and writes nothing, so it is a zero-risk probe of "does the servo answer
 * anything at all". Caller must hold st->lock.
 */
static int sts3215_ping(struct sts3215 *st)
{
	u8 buf[6];
	u8 csum;
	int n = 0, ret;
	long t;
	unsigned long flags;

	buf[n++] = 0xFF;
	buf[n++] = 0xFF;
	buf[n++] = st->id;
	buf[n++] = 2;			/* nparams(0) + 2 */
	buf[n++] = STS_INST_PING;
	csum = st->id + 2 + STS_INST_PING;
	buf[n++] = ~csum;

	ret = sts3215_tx(st, buf, n);
	if (ret)
		return ret;

	t = wait_for_completion_timeout(&st->rx_done,
					msecs_to_jiffies(STS_RX_TIMEOUT_MS));

	spin_lock_irqsave(&st->rx_lock, flags);
	ret = (t && st->rx_ok) ? 0 : -ETIMEDOUT;
	if (debug) {
		char hex[3 * 32 + 1];
		int k, p = 0;

		for (k = 0; k < st->dbg_len; k++)
			p += scnprintf(hex + p, sizeof(hex) - p,
				       "%02x ", st->dbg[k]);
		dev_info(st->dev, "PING %s: total=%d state=%d bytes=[%s]\n",
			 ret ? "NO RESPONSE" : "OK",
			 st->rx_total, st->rx_state, hex);
	} else {
		dev_info(st->dev, "PING %s\n", ret ? "NO RESPONSE" : "OK");
	}
	spin_unlock_irqrestore(&st->rx_lock, flags);

	return ret;
}

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
	st->rx_state = RX_H1;
	mutex_init(&st->lock);
	spin_lock_init(&st->rx_lock);
	init_completion(&st->rx_done);
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

	mutex_lock(&st->lock);
	sts3215_ping(st);
	mutex_unlock(&st->lock);

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
MODULE_DESCRIPTION("Feetech STS3215 serial bus servo serdev driver with feedback read");
MODULE_LICENSE("GPL");
