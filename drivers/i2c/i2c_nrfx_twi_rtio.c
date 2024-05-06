/*
 * Copyright (c) 2018, Nordic Semiconductor ASA
 * Copyright (c) 2024, Croxel Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/rtio.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <nrfx_twi.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_nrfx_twi, CONFIG_I2C_LOG_LEVEL);

#if CONFIG_I2C_NRFX_TRANSFER_TIMEOUT
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(CONFIG_I2C_NRFX_TRANSFER_TIMEOUT)
#else
#define I2C_TRANSFER_TIMEOUT_MSEC K_FOREVER
#endif

struct i2c_nrfx_twi_data {
	struct i2c_rtio *ctx;
	bool twi_enabled;
	uint32_t dev_config;
};

struct i2c_nrfx_twi_config {
	nrfx_twi_t twi;
	nrfx_twi_config_t config;
	const struct pinctrl_dev_config *pcfg;
};

static int i2c_nrfx_twi_recover_bus(const struct device *dev);
static void i2c_nrfx_twi_complete(const struct device *dev, int status);

static bool i2c_nrfx_twi_msg_start(const struct device *dev, uint8_t flags,
				   uint8_t *buf, size_t buf_len, uint16_t i2c_addr)
{
	const struct i2c_nrfx_twi_config *config = dev->config;
	struct i2c_nrfx_twi_data *const dev_data = dev->data;
	struct i2c_rtio *ctx = dev_data->ctx;
	int ret = 0;

	/** Enabling while already enabled ends up in a failed assertion: skip it. */
	if (!dev_data->twi_enabled) {
		nrfx_twi_enable(&config->twi);
		dev_data->twi_enabled = true;
	}

	if (flags & I2C_MSG_ADDR_10_BITS) {
		LOG_ERR("10-bit I2C Addr devices not supported");
		return i2c_rtio_complete(ctx, -ENOTSUP);
	}

	nrfx_twi_xfer_desc_t cur_xfer = {
		.p_primary_buf = buf,
		.primary_length = buf_len,
		.address = i2c_addr,
		.type = (flags & I2C_MSG_READ) ?
			 NRFX_TWI_XFER_RX : NRFX_TWI_XFER_TX,
	};
	uint32_t xfer_flags = 0;
	nrfx_err_t res;

	/* In case the STOP condition is not supposed to appear after
	 * the current message, check what is requested further:
	 */
	if (!(flags & I2C_MSG_STOP)) {
		/* - if the transfer consists of more messages
		 *   finish the transfer without generating the STOP
		 *   condition, unless the current message is an RX
		 *   request, for which such feature is not supported
		 */
		if (flags & I2C_MSG_READ) {
			ret = -ENOTSUP;
		} else {
			xfer_flags |= NRFX_TWI_FLAG_TX_NO_STOP;
		}
	}

	if (!ret) {
		res = nrfx_twi_xfer(&config->twi, &cur_xfer, xfer_flags);
		switch (res) {
		case NRFX_SUCCESS:
			break;
		case NRFX_ERROR_BUSY:
			ret = -EBUSY;
			break;
		default:
			ret = -EIO;
			break;
		}
	}

	if (ret != 0) {
		nrfx_twi_disable(&config->twi);
		dev_data->twi_enabled = false;

		return i2c_rtio_complete(ctx, ret);
	}

	return false;
}

static int i2c_nrfx_twi_configure(const struct device *dev,
				  uint32_t dev_config)
{
	const struct i2c_nrfx_twi_config *config = dev->config;
	struct i2c_nrfx_twi_data *data = dev->data;
	nrfx_twi_t const *inst = &config->twi;

	if (I2C_ADDR_10_BITS & dev_config) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		nrf_twi_frequency_set(inst->p_twi, NRF_TWI_FREQ_100K);
		break;
	case I2C_SPEED_FAST:
		nrf_twi_frequency_set(inst->p_twi, NRF_TWI_FREQ_400K);
		break;
	default:
		LOG_ERR("unsupported speed");
		return -EINVAL;
	}
	data->dev_config = dev_config;

	return 0;
}

static bool i2c_nrfx_twi_start(const struct device *dev)
{
	struct i2c_nrfx_twi_data *const dev_data = dev->data;
	struct i2c_rtio *ctx = dev_data->ctx;
	struct rtio_sqe *sqe = &ctx->txn_curr->sqe;
	struct i2c_dt_spec *dt_spec = sqe->iodev->data;

	switch (sqe->op) {
	case RTIO_OP_RX:
		return i2c_nrfx_twi_msg_start(dev, I2C_MSG_READ | sqe->iodev_flags,
					sqe->buf, sqe->buf_len, dt_spec->addr);
	case RTIO_OP_TINY_TX:
		return i2c_nrfx_twi_msg_start(dev, I2C_MSG_WRITE | sqe->iodev_flags,
					sqe->tiny_buf, sqe->tiny_buf_len, dt_spec->addr);
	case RTIO_OP_TX:
		return i2c_nrfx_twi_msg_start(dev, I2C_MSG_WRITE | sqe->iodev_flags,
					sqe->buf, sqe->buf_len, dt_spec->addr);
	case RTIO_OP_I2C_CONFIGURE:
		(void)i2c_nrfx_twi_configure(dev, sqe->i2c_config);
		return false;
	case RTIO_OP_I2C_RECOVER:
		(void)i2c_rtio_recover(ctx);
		return false;
	default:
		LOG_ERR("Invalid op code %d for submission %p\n", sqe->op, (void *)sqe);
		return i2c_rtio_complete(ctx, -EINVAL);
	}
}

static void i2c_nrfx_twi_complete(const struct device *dev, int status)
{
	/** Finalize if there are no more pending xfers */
	const struct i2c_nrfx_twi_config *config = dev->config;
	struct i2c_nrfx_twi_data *data = dev->data;
	struct i2c_rtio *const ctx = data->ctx;

	if (i2c_rtio_complete(ctx, status)) {
		(void)i2c_nrfx_twi_start(dev);
	} else {
		nrfx_twi_disable(&config->twi);
		data->twi_enabled = false;
	}
}

static int i2c_nrfx_twi_transfer(const struct device *dev,
				 struct i2c_msg *msgs,
				 uint8_t num_msgs, uint16_t addr)
{
	struct i2c_rtio *const ctx = ((struct i2c_nrfx_twi_data *)
		dev->data)->ctx;

	return i2c_rtio_transfer(ctx, msgs, num_msgs, addr);
}

static void event_handler(nrfx_twi_evt_t const *p_event, void *p_context)
{
	const struct device *dev = p_context;
	nrfx_err_t res;
	int status = 0;

	switch (p_event->type) {
	case NRFX_TWI_EVT_DONE:
		res = NRFX_SUCCESS;
		break;
	case NRFX_TWI_EVT_ADDRESS_NACK:
		res = NRFX_ERROR_DRV_TWI_ERR_ANACK;
		break;
	case NRFX_TWI_EVT_DATA_NACK:
		res = NRFX_ERROR_DRV_TWI_ERR_DNACK;
		break;
	default:
		res = NRFX_ERROR_INTERNAL;
		break;
	}

	if (res != NRFX_SUCCESS) {
		status = -EIO;
	}

	i2c_nrfx_twi_complete(dev, status);
}

static int i2c_nrfx_twi_recover_bus(const struct device *dev)
{
	const struct i2c_nrfx_twi_config *config = dev->config;
	uint32_t scl_pin;
	uint32_t sda_pin;
	nrfx_err_t err;

	scl_pin = nrf_twi_scl_pin_get(config->twi.p_twi);
	sda_pin = nrf_twi_sda_pin_get(config->twi.p_twi);

	err = nrfx_twi_bus_recover(scl_pin, sda_pin);
	return (err == NRFX_SUCCESS ? 0 : -EBUSY);
}

static void i2c_nrfx_twi_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_seq)
{
	struct i2c_nrfx_twi_data *data = dev->data;
	struct i2c_rtio *const ctx = data->ctx;

	if (i2c_rtio_submit(ctx, iodev_seq)) {
		(void)i2c_nrfx_twi_start(dev);
	}
}

static const struct i2c_driver_api i2c_nrfx_twi_driver_api = {
	.configure   = i2c_nrfx_twi_configure,
	.transfer    = i2c_nrfx_twi_transfer,
	.recover_bus = i2c_nrfx_twi_recover_bus,
	.iodev_submit = i2c_nrfx_twi_submit,
};

static int init_twi(const struct device *dev)
{
	const struct i2c_nrfx_twi_config *config = dev->config;
	nrfx_err_t result = nrfx_twi_init(&config->twi, &config->config,
					  event_handler, (void *)dev);
	if (result != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize device: %s",
			    dev->name);
		return -EBUSY;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int twi_nrfx_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	const struct i2c_nrfx_twi_config *config = dev->config;
	struct i2c_nrfx_twi_data *data = dev->data;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
		init_twi(dev);
		if (data->dev_config) {
			i2c_nrfx_twi_configure(dev, data->dev_config);
		}
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		nrfx_twi_uninit(&config->twi);

		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if (ret < 0) {
			return ret;
		}
		break;

	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

#define I2C_NRFX_TWI_INVALID_FREQUENCY  ((nrf_twi_frequency_t)-1)
#define I2C_NRFX_TWI_FREQUENCY(bitrate)					       \
	 (bitrate == I2C_BITRATE_STANDARD ? NRF_TWI_FREQ_100K		       \
	: bitrate == 250000               ? NRF_TWI_FREQ_250K		       \
	: bitrate == I2C_BITRATE_FAST     ? NRF_TWI_FREQ_400K		       \
					  : I2C_NRFX_TWI_INVALID_FREQUENCY)
#define I2C(idx) DT_NODELABEL(i2c##idx)
#define I2C_FREQUENCY(idx)						       \
	I2C_NRFX_TWI_FREQUENCY(DT_PROP(I2C(idx), clock_frequency))

#define I2C_NRFX_TWI_DEVICE(idx)					       \
	NRF_DT_CHECK_NODE_HAS_PINCTRL_SLEEP(I2C(idx));			       \
	BUILD_ASSERT(I2C_FREQUENCY(idx)	!=				       \
		     I2C_NRFX_TWI_INVALID_FREQUENCY,			       \
		     "Wrong I2C " #idx " frequency setting in dts");	       \
	static int twi_##idx##_init(const struct device *dev)		       \
	{								       \
		IRQ_CONNECT(DT_IRQN(I2C(idx)), DT_IRQ(I2C(idx), priority),     \
			    nrfx_isr, nrfx_twi_##idx##_irq_handler, 0);	       \
		const struct i2c_nrfx_twi_config *config = dev->config;	       \
		const struct i2c_nrfx_twi_data *dev_data = dev->data;	       \
		int err = pinctrl_apply_state(config->pcfg,		       \
					      PINCTRL_STATE_DEFAULT);	       \
		if (err < 0) {						       \
			return err;					       \
		}							       \
		i2c_rtio_init(dev_data->ctx, dev);			       \
		return init_twi(dev);					       \
	}								       \
	I2C_RTIO_DEFINE(_i2c##idx##_twi_rtio,				       \
			DT_INST_PROP_OR(n, sq_size, CONFIG_I2C_RTIO_SQ_SIZE),  \
			DT_INST_PROP_OR(n, cq_size, CONFIG_I2C_RTIO_CQ_SIZE)); \
	static struct i2c_nrfx_twi_data twi_##idx##_data = {		       \
		.ctx = &_i2c##idx##_twi_rtio,				       \
	};								       \
	PINCTRL_DT_DEFINE(I2C(idx));					       \
	static const struct i2c_nrfx_twi_config twi_##idx##z_config = {	       \
		.twi = NRFX_TWI_INSTANCE(idx),				       \
		.config = {						       \
			.skip_gpio_cfg = true,				       \
			.skip_psel_cfg = true,				       \
			.frequency = I2C_FREQUENCY(idx),		       \
		},							       \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(I2C(idx)),		       \
	};								       \
	PM_DEVICE_DT_DEFINE(I2C(idx), twi_nrfx_pm_action);		       \
	I2C_DEVICE_DT_DEFINE(I2C(idx),					       \
		      twi_##idx##_init,					       \
		      PM_DEVICE_DT_GET(I2C(idx)),			       \
		      &twi_##idx##_data,				       \
		      &twi_##idx##z_config,				       \
		      POST_KERNEL,					       \
		      CONFIG_I2C_INIT_PRIORITY,				       \
		      &i2c_nrfx_twi_driver_api)

#ifdef CONFIG_HAS_HW_NRF_TWI0
I2C_NRFX_TWI_DEVICE(0);
#endif

#ifdef CONFIG_HAS_HW_NRF_TWI1
I2C_NRFX_TWI_DEVICE(1);
#endif
