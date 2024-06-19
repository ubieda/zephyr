/*
 * Copyright (c) 2024 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTIO_ASYNC_SHIM_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTIO_ASYNC_SHIM_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/p4wq.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback API to execute synchronous operation decoupled from
 * its submission.
 *
 * @param dev Device associated to RTIO operation.
 * @param iodev_sqe Associated SQE operation.
 */
typedef void (*rtio_async_shim_submit_t)(const struct device *dev,
					 struct rtio_iodev_sqe *iodev_sqe);

/**
 * @brief RTIO Asynchronous Shim request.
 *
 * This Request serves as a shim layer to decouple a syncronous
 * operation without breaking the RTIO paradigm of non-blocking,
 * asynchronous operations.
 */
struct rtio_async_shim_req {
	/** Work item used to submit unit of work. */
	struct k_p4wq_work work;

	/** Handle to device associated with operation.
	 * This is filled inside @ref rtio_async_shim_req_submit.
	 */
	const struct device *dev;

	/** Handle to IODEV SQE containing the operation.
	 * This is filled inside @ref rtio_async_shim_req_submit.
	 */
	struct rtio_iodev_sqe *iodev_sqe;

	/** Callback handler where synchronous operation may be executed.
	 * This is filled inside @ref rtio_async_shim_req_submit.
	 */
	rtio_async_shim_submit_t handler;
};

/**
 * @brief Allocate request item to perform an asynchronous RTIO request.
 *
 * @details This allocation utilizes its internal memory slab with
 * pre-allocated elements.
 *
 * @return Pointer to allocated item if successful.
 * @return NULL if allocation failed.
 */
struct rtio_async_shim_req *rtio_async_shim_req_alloc(void);

/**
 * @brief Submit non-blocking request to perform an asynchronous RTIO request.
 *
 * @param req Item to fill with request information.
 * @param dev Device associated with RTIO operation.
 * @param iodev_sqe RTIO Operation information.
 * @param handler Callback to handler where Synchronous operation may be performed.
 */
void rtio_async_shim_req_submit(struct rtio_async_shim_req *req,
				const struct device *dev,
				struct rtio_iodev_sqe *iodev_sqe,
				rtio_async_shim_submit_t handler);

/**
 * @brief Obtain number of currently used items from the pre-allocated pool.
 *
 * @return Number of used items.
 */
uint32_t rtio_async_shim_req_used_count_get(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTIO_ASYNC_SHIM_H_ */
