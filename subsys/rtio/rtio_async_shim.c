/*
 * Copyright (c) 2024 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/async_shim.h>
#include <zephyr/kernel.h>

#define RTIO_ASYNC_SHIM_WQ_PRIO_MED		CONFIG_RTIO_ASYNC_SHIM_WQ_PRIO_MED
#define RTIO_ASYNC_SHIM_WQ_PRIO_HIGH		RTIO_ASYNC_SHIM_WQ_PRIO_MED - 1
#define RTIO_ASYNC_SHIM_WQ_PRIO_LOW		RTIO_ASYNC_SHIM_WQ_PRIO_MED + 1

K_P4WQ_DEFINE(rtio_async_shim_wq,
	      CONFIG_RTIO_ASYNC_SHIM_WQ_THREADS_POOL,
	      CONFIG_RTIO_ASYNC_SHIM_WQ_STACK_SIZE);

K_MEM_SLAB_DEFINE_STATIC(rtio_async_shim_wq_slab,
			 sizeof(struct rtio_async_shim_req),
			 CONFIG_RTIO_ASYNC_SHIM_WQ_POOL_ITEMS,
			 4);

static void rtio_async_shim_work_handler(struct k_p4wq_work *work)
{
	struct rtio_async_shim_req *req = CONTAINER_OF(work,
						 struct rtio_async_shim_req,
						 work);
	const struct device *dev = req->dev;
	struct rtio_iodev_sqe *iodev_sqe = req->iodev_sqe;

	req->handler(dev, iodev_sqe);

	k_mem_slab_free(&rtio_async_shim_wq_slab, req);
}

struct rtio_async_shim_req *rtio_async_shim_req_alloc(void)
{
	struct rtio_async_shim_req *req;
	int err;

	err = k_mem_slab_alloc(&rtio_async_shim_wq_slab, (void **)&req, K_NO_WAIT);
	if (err) {
		return NULL;
	}

	(void)k_sem_init(&req->work.done_sem, 1, 1);

	return req;
}

void rtio_async_shim_req_submit(struct rtio_async_shim_req *req,
				const struct device *dev,
				struct rtio_iodev_sqe *iodev_sqe,
				rtio_async_shim_submit_t handler)
{
	if (!req) {
		return;
	}

	if (!dev || !iodev_sqe || !handler) {
		k_mem_slab_free(&rtio_async_shim_wq_slab, req);
		return;
	}

	struct k_p4wq_work *work = &req->work;
	struct rtio_sqe *sqe = &iodev_sqe->sqe;

	/** Link the relevant info so that we can get it on the k_p4wq_work work item.
	 */
	req->dev = dev;
	req->iodev_sqe = iodev_sqe;
	req->handler = handler;

	/** Set the required information to handle the action */
	work->handler = rtio_async_shim_work_handler;
	work->deadline = 0;

	if (sqe->prio == RTIO_PRIO_LOW) {
		work->priority = RTIO_ASYNC_SHIM_WQ_PRIO_LOW;
	} else if (sqe->prio == RTIO_PRIO_HIGH) {
		work->priority = RTIO_ASYNC_SHIM_WQ_PRIO_HIGH;
	} else {
		work->priority = RTIO_ASYNC_SHIM_WQ_PRIO_MED;
	}

	/** Decoupling action: Let the P4WQ execute the action. */
	k_p4wq_submit(&rtio_async_shim_wq, work);
}

uint32_t rtio_async_shim_req_used_count_get(void)
{
	return k_mem_slab_num_used_get(&rtio_async_shim_wq_slab);
}
