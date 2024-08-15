/*
 * Copyright (c) 2023 Intel Corporation
 * Copyright (c) 2024 Croxel Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/sys/mpsc_lockfree.h>

const struct rtio_iodev_api spi_iodev_api = {
	.submit = spi_iodev_submit,
};

/**
 * @brief Lock the SPI RTIO spinlock
 *
 * This is used internally for controlling the SPI RTIO context, and is
 * exposed to the user as it's required for safely implementing
 * iodev_start API, specific to each driver.
 *
 * @param ctx SPI RTIO context
 *
 * @retval Spinlock key
 */
static inline k_spinlock_key_t spi_spin_lock(struct spi_rtio *ctx)
{
	return k_spin_lock(&ctx->lock);
}

/**
 * @brief Unlock the previously obtained SPI RTIO spinlock
 *
 * @param ctx SPI RTIO context
 * @param key Spinlock key
 */
static inline void spi_spin_unlock(struct spi_rtio *ctx, k_spinlock_key_t key)
{
	k_spin_unlock(&ctx->lock, key);
}

void spi_rtio_init(struct spi_rtio *ctx,
		   const struct device *dev)
{
	mpsc_init(&ctx->io_q);
	ctx->txn_head = NULL;
	ctx->txn_curr = NULL;
	ctx->dt_spec.bus = dev;
	ctx->iodev.data = &ctx->dt_spec;
	ctx->iodev.api = &spi_iodev_api;
}

/**
 * @private
 * @brief Setup the next transaction (could be a single op) if needed
 *
 * @retval true New transaction to start with the hardware is setup
 * @retval false No new transaction to start
 */
static bool spi_rtio_next(struct spi_rtio *ctx, bool completion)
{
	k_spinlock_key_t key = spi_spin_lock(ctx);

	if (!completion && ctx->txn_curr != NULL) {
		spi_spin_unlock(ctx, key);
		return false;
	}

	struct mpsc_node *next = mpsc_pop(&ctx->io_q);

	if (next != NULL) {
		struct rtio_iodev_sqe *next_sqe = CONTAINER_OF(next, struct rtio_iodev_sqe, q);

		ctx->txn_head = next_sqe;
		ctx->txn_curr = next_sqe;
	} else {
		ctx->txn_head = NULL;
		ctx->txn_curr = NULL;
	}

	spi_spin_unlock(ctx, key);

	return (ctx->txn_curr != NULL);
}

bool spi_rtio_complete(struct spi_rtio *ctx, int status)
{
	struct rtio_iodev_sqe *txn_head = ctx->txn_head;
	bool result;

	result = spi_rtio_next(ctx, true);

	if (status < 0) {
		rtio_iodev_sqe_err(txn_head, status);
	} else {
		rtio_iodev_sqe_ok(txn_head, status);
	}

	return result;
}

bool spi_rtio_submit(struct spi_rtio *ctx,
		     struct rtio_iodev_sqe *iodev_sqe)
{
	/** Done */
	mpsc_push(&ctx->io_q, &iodev_sqe->q);
	return spi_rtio_next(ctx, false);
}

int spi_rtio_transceive(struct spi_rtio *ctx,
			const struct spi_config *config,
			const struct spi_buf_set *tx_bufs,
			const struct spi_buf_set *rx_bufs)
{
	struct spi_dt_spec *dt_spec = &ctx->dt_spec;
	struct rtio_sqe *sqe;
	struct rtio_cqe *cqe;
	int err = 0;
	int ret;

	dt_spec->config = *config;

	ret = spi_rtio_copy(ctx->r, &ctx->iodev, tx_bufs, rx_bufs, &sqe);
	if (ret < 0) {
		return ret;
	}

	/** Submit request and wait */
	rtio_submit(ctx->r, ret);

	while (ret > 0) {
		cqe = rtio_cqe_consume(ctx->r);
		if (cqe->result < 0) {
			err = cqe->result;
		}

		rtio_cqe_release(ctx->r, cqe);
		ret--;
	}

	return err;
}
