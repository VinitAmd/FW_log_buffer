// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, Advanced Micro Devices, Inc.
 */

#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <drm/drm_cache.h>
#include "aie2_msg_priv.h"
#include "aie2_pci.h"
#include "amdxdna_trace.h"
#include "amdxdna_mailbox.h"

#define LOG_BUFFER_IRQ 27
#define MPIPU_IOHUB_INT_27_ALIAS 0xD7008
#define LOG_BUFFER_MAILBOX MPIPU_IOHUB_INT_27_ALIAS

uint8_t g_fwLogBuf[TRACE_EVENT_BUFFER_SIZE];

struct trace_event_metadata
{
	uint64_t tail_offset;
	uint64_t head_offset;
	uint32_t padding[12];
};

struct trace_event_log_data
{
	uint64_t counter;
	uint16_t payload_hi;
	uint16_t type;
	uint32_t payload_low;
};

struct event_trace_req_buf
{
	struct amdxdna_dev_hdl *ndev;
	struct start_event_trace_req trace_req;
	int log_ch_irq;
	u8 *buf;
};

uint32_t get_trace_event_Content(struct event_trace_req_buf *trace_req_buf, uint8_t *kern_buf)
{
	u8 *sysBuf;
	uint32_t rd_ptr, wr_ptr, wr_ptr_wrap;
	// Log size is the size of the log content in the trace buff in bytes.
	uint32_t total_log_size = 0, log_size = 0;
	uint32_t rb_size, offset = 0;
	struct trace_event_metadata *trace_metadata;
	struct amdxdna_dev_hdl *ndev = trace_req_buf->ndev;

	sysBuf = (u8 *)trace_req_buf->buf;
	rb_size = TRACE_EVENT_BUFFER_SIZE - 64;
	if (rb_size == 0)
	{
		return 0;
	}

	trace_metadata = (struct trace_event_metadata *)(sysBuf + rb_size);
	if (!trace_metadata)
	{
		printk(KERN_ERR "FW trace buffer metadata is null!");
		return 0;
	}

	// Get the ring buffer read and write pointers, update the ring buffer content size
	rd_ptr = (uint32_t)(trace_metadata->head_offset % rb_size);
	wr_ptr = (uint32_t)(trace_metadata->tail_offset);
	wr_ptr_wrap = wr_ptr % rb_size;

	// Update the Ring Buffer read pointer
	trace_metadata->head_offset = wr_ptr;
	// Clear the log buffer interrupt
	clear_logbuff_irq(LOG_BUFFER_MAILBOX, ndev);

	do
	{
		if (wr_ptr_wrap > rd_ptr)
			log_size = wr_ptr_wrap - rd_ptr;
		else if (wr_ptr_wrap < rd_ptr)
			log_size = rb_size - rd_ptr;
		else
			return 0;

		if (log_size > rb_size)
		{
			printk(KERN_ERR "log_size > rb_size");
			return 0;
		}
		// Copy the ring buffer content to the kernel buffer
		memcpy(kern_buf + offset, (u8 *)(sysBuf + rd_ptr), log_size);

		offset += log_size;
		total_log_size += log_size;
		rd_ptr = (rd_ptr + log_size) % rb_size;
	} while (rd_ptr < wr_ptr_wrap); // If the buffer is rolling over, continue to copy the content

	return total_log_size;
}

void print_trace_event_log(struct amdxdna_dev_hdl *ndev)
{
	uint32_t log_size;
	uint64_t payload;
	struct event_trace_req_buf *trace_req_buf;
	struct trace_event_log_data *log_content;
	trace_req_buf = ndev->event_trace_req;

	if (!trace_req_buf)
	{
		XDNA_ERR(ndev->xdna, "FW resp trace buffer is null!");
		return;
	}

	log_size = get_trace_event_Content(trace_req_buf, g_fwLogBuf);
	XDNA_INFO(ndev->xdna, "FW log size in bytes %u", log_size);

	if (log_size)
	{
		uint64_t fwTicks;
		char *str = (char *)g_fwLogBuf;
		char *end = ((char *)str + log_size);
		g_fwLogBuf[log_size] = 0;

		// Print the raw counter value from NPU first
		while (str < end)
		{
			log_content = (struct trace_event_log_data *)str;
			payload = (uint64_t)((uint64_t)(log_content->payload_hi) << 32 | log_content->payload_low);
			fwTicks = log_content->counter;
			printk(KERN_ERR "[%llu][FW] type: 0x%04x payload:0x%016llx", fwTicks, log_content->type, payload);
			str += MAX_ONE_TIME_LOG_INFO_LEN;
		}
	}
}

static irqreturn_t log_buffer_irq_handler(int irq, void *data)
{
	struct amdxdna_dev_hdl *ndev = (struct amdxdna_dev_hdl *)data;

	if (!ndev) {
		printk(KERN_ERR "xdna dev is null !\n");
		return IRQ_NONE;
	}

	trace_mbox_irq_handle("LOG_BUFFER", irq);
	print_trace_event_log(ndev);
	return IRQ_HANDLED;
}

int aie2_stop_event_trace_send(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->xdna;
	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));
	return aie2_stop_event_trace(ndev);
}

int aie2_event_trace_alloc(struct amdxdna_dev_hdl *ndev)
{
	int ret;
	struct amdxdna_dev *xdna = ndev->xdna;
	struct event_trace_req_buf *req_buf;

	req_buf = kzalloc(sizeof(struct event_trace_req_buf), GFP_KERNEL);
	if (!req_buf)
		return -ENOMEM;

	req_buf->buf = dma_alloc_noncoherent(xdna->ddev.dev, TRACE_EVENT_BUFFER_SIZE, (dma_addr_t *)&req_buf->trace_req.dram_buffer_address,
						DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (!req_buf->buf)
	{
		ret = -ENOMEM;
		goto free_event_trace_req_buf;
	}
	req_buf->trace_req.dram_buffer_size = TRACE_EVENT_BUFFER_SIZE;
	ndev->event_trace_req = req_buf;
	req_buf->ndev = ndev;

	req_buf->log_ch_irq = pci_irq_vector(to_pci_dev(xdna->ddev.dev), LOG_BUFFER_IRQ);
	ret = request_irq(req_buf->log_ch_irq, log_buffer_irq_handler, 0, "LOG_BUFFER", ndev);
	if (ret)
	{
		printk(KERN_INFO "Failed to register irq %d ret %d", LOG_BUFFER_IRQ, ret);
		goto free_event_trace_req_buf;
	}

	XDNA_INFO(xdna, "Start event trace buf addr: 0x%llx size 0x%x", req_buf->trace_req.dram_buffer_address,
			  req_buf->trace_req.dram_buffer_size);
	return 0;

free_event_trace_req_buf:
	kfree(req_buf);
	return ret;
}

void aie2_event_trace_free(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->xdna;
	struct event_trace_req_buf *req_buf = ndev->event_trace_req;

	if (!req_buf)
		return;

	dma_free_noncoherent(xdna->ddev.dev, req_buf->trace_req.dram_buffer_size, req_buf->buf,
					(dma_addr_t)req_buf->trace_req.dram_buffer_address, DMA_BIDIRECTIONAL);
	free_irq(req_buf->log_ch_irq, ndev);
	kfree(req_buf);
}

int aie2_start_event_trace_send(struct amdxdna_dev_hdl *ndev)
{
	int ret;
	struct event_trace_req_buf *trace_req_buf = NULL;
	struct amdxdna_dev *xdna = ndev->xdna;

	ret = aie2_event_trace_alloc(ndev);

	if (!ret)
	{
		trace_req_buf = ndev->event_trace_req;
		drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));
		drm_clflush_virt_range(trace_req_buf->buf, trace_req_buf->trace_req.dram_buffer_size);
		ret = aie2_start_event_trace(ndev, trace_req_buf->trace_req.dram_buffer_address,
									 trace_req_buf->trace_req.dram_buffer_size, &trace_req_buf->trace_req);
	}
	else
	{
		XDNA_ERR(xdna, "Failed to allocate and register event trace");
	}

	return ret;
}
