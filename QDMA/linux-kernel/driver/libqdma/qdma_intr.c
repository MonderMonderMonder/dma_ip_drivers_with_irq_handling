/*
 * This file is part of the Xilinx DMA IP Core driver for Linux
 *
 * Copyright (c) 2017-2022, Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022, Advanced Micro Devices, Inc. All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include "qdma_descq.h"
#include "qdma_device.h"
#include "qdma_regs.h"
#include "thread.h"
#include "version.h"
#include "qdma_mbox_protocol.h"
#include "qdma_intr.h"
#include "qdma_access_common.h"

static off_t GIRQ_BASE_ADDR = 0x101000; // interrupt generation registers base address

// interrupt generation register pointers
static void __iomem *reg_girq_ctrl; 
static void __iomem *reg_girq_trig; 
static void __iomem *reg_girq_stat;
static void __iomem *reg_girq_cmpt; 
static void __iomem *reg_girq_ts0;
static void __iomem *reg_girq_ts1;
static void __iomem *reg_girq_ts2;
static void __iomem *reg_girq_counter;

static struct pci_dev *pdev; // PCIe device
static void __iomem *bar2; // BAR pointer

// Initializes memory-mapped registers
static int mm_registers_init(void) {
	unsigned long bar_start, bar_len;
	// get PCIe device based on vendor
	pdev = pci_get_device((uint16_t)(0x10ee), PCI_ANY_ID, NULL); 
	if (!pdev) { // device not found
		pr_err("1"); 
		return -1;
	}
	// initialize PCIe device memory space
	if (pci_enable_device_mem(pdev)) { 
		pr_err("2");
		return -1;
	}

	// set PCIe flags
	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN); // relaxed ordering
	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_EXT_TAG); // extended tag field
	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_NOSNOOP_EN); // no snoop transactions
	// pcie_capability_set_word(pdev, PCI_EXP_LNKCTL, PCI_EXP_LNKCTL_ASPM_L0S); // active state power management
	// pcie_capability_set_word(pdev, PCI_EXP_LNKCTL, PCI_EXP_LNKCTL_ASPM_L1); // L1 state, power-saving

	pci_set_master(pdev); // set device to bus master
	// if (!dma_set_mask(pdev, DMA_BIT_MASK(64))) dma_set_mask_and_coherent(pdev, DMA_BIT_MASK(64));
	// else if (!dma_set_mask(pdev, DMA_BIT_MASK(32))) dma_set_mask_and_coherent(pdev, DMA_BIT_MASK(32));
	// else return -1;

	// map BAR2 to kernel virtual address space
	bar_start = pci_resource_start(pdev, 2);
    bar_len = pci_resource_len(pdev, 2);
    bar2 = ioremap(bar_start, bar_len);
    if (!bar2) {
        pci_release_region(pdev, 2); // release BAR2 if ioremap fails
        pci_disable_device(pdev);
        return -1;
    }

	// initialize interrupt generation register pointers
	reg_girq_ctrl = bar2 + GIRQ_BASE_ADDR + 0*4;
	reg_girq_trig = bar2 + GIRQ_BASE_ADDR + 1*4;
	reg_girq_stat = bar2 + GIRQ_BASE_ADDR + 2*4;
	reg_girq_cmpt = bar2 + GIRQ_BASE_ADDR + 3*4;
	reg_girq_ts0 = bar2 + GIRQ_BASE_ADDR + 4*4;
	reg_girq_ts1 = bar2 + GIRQ_BASE_ADDR + 5*4;
	reg_girq_ts2 = bar2 + GIRQ_BASE_ADDR + 6*4; 
	reg_girq_counter = bar2 + GIRQ_BASE_ADDR + 6*4;
    return 0;
}

// Releases PCIe resources
static void mm_registers_teardown(void) {
	iounmap(bar2);
    pci_release_region(pdev, 2);
    pci_disable_device(pdev);
}


// Top Half: simple completion signal write
static irqreturn_t top(int irq, void *dev) {
	iowrite32(0X1, reg_girq_cmpt);
	return IRQ_HANDLED; 
}

// Top Half: indefinetly busy-wait and monopolize CPU
static irqreturn_t top_busy(int irq, void *dev) {
	unsigned long start_jiffies = jiffies;
	while (time_before(jiffies, start_jiffies + HZ * 30)) cpu_relax();
	iowrite32(0X1, reg_girq_cmpt);
	return IRQ_HANDLED; 
}


// Top Half: wake up Threaded Handler (BH)
static irqreturn_t threaded_top(int irq, void *dev) {
	return IRQ_WAKE_THREAD; 
}

// Bottom Half: Threaded Handler
static irqreturn_t threaded_bottom(int irq, void *dev) {
	iowrite32(0X1, reg_girq_cmpt); // write completion signal
	return IRQ_HANDLED; 
}


static struct tasklet_struct tk1, tk2;

// Top Half: schedule Tasklet BH
static irqreturn_t tasklet_top(int irq, void *dev_id) {
	tasklet_schedule(&tk1);
	return IRQ_HANDLED; 
}

// Top Half: schedule high priority Tasklet (BH)
static irqreturn_t tasklet_hi_top(int irq, void *dev_id) {
	tasklet_hi_schedule(&tk2);
	return IRQ_HANDLED; 
}

// Bottom Half: (high priority) Tasklet
static void tasklet_func(unsigned long data) {
    iowrite32(0X1, reg_girq_cmpt);
}

// Bottom Half: indefinetly busy-waiting (high priority) Tasklet, can be stopped by Top Half
static void tasklet_func_busy(unsigned long data) {
	unsigned long start_jiffies = jiffies;
    while (time_before(jiffies, start_jiffies + HZ * 30)) cpu_relax();
    iowrite32(0X1, reg_girq_cmpt);
}


static struct workqueue_struct *wq1, *wq2, *wq3, *wq4, *wq5;
static struct work_struct *work; // wrapper

// Top Half: enqueue Work into Workqueue (BH)

static irqreturn_t wq1_top(int irq, void *dev) {
	queue_work(wq1, work);
	return IRQ_HANDLED; 
}

static irqreturn_t wq2_top(int irq, void *dev) {
	queue_work(wq2, work);
	return IRQ_HANDLED; 
}

static irqreturn_t wq3_top(int irq, void *dev) {
	queue_work(wq3, work);
	return IRQ_HANDLED; 
}

static irqreturn_t wq4_top(int irq, void *dev) {
	queue_work(wq4, work);
	return IRQ_HANDLED; 
}

static irqreturn_t wq5_top(int irq, void *dev) {
	queue_work(wq5, work);
	return IRQ_HANDLED; 
}

// Bottom Half: Work to be enqueued in Workqueue 
static void w_func(struct work_struct *work) {
    iowrite32(0X1, reg_girq_cmpt);
}

// Register IRQ for MSI-X vector indexed by idx, with ISR of type type 
static int usr_intr_vector_init(struct xlnx_dma_dev *xdev, int idx, enum intr_type_list type) {
	int rv;
	snprintf(xdev->dev_intr_info_list[idx].msix_name, QDMA_DEV_NAME_MAXLEN + 16, "%s-user", xdev->conf.name);

	// initialize BH structs
	work = kzalloc(sizeof(struct work_struct), GFP_ATOMIC);
	INIT_WORK(work, w_func); // wrap simple function with Work struct 

	switch (type) {
		case INTR_TYPE_USER_TOP: // Top Half without Bottom Half
			rv = request_irq(xdev->msix[idx].vector, top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_THREADED:
			rv = request_threaded_irq(xdev->msix[idx].vector, threaded_top, threaded_bottom, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_TASKLET:
			tasklet_init(&tk1, tasklet_func_busy, 0);
			rv = request_irq(xdev->msix[idx].vector, tasklet_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_TASKLET_HIGHPRI:
			tasklet_init(&tk2, tasklet_func, 0);
			rv = request_irq(xdev->msix[idx].vector, tasklet_hi_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_WQ: // refer to Workqueue API https://docs.kernel.org/core-api/workqueue.html#flags
			wq1 = alloc_workqueue("wq1", 0, 1);
			if (!wq1) pr_debug("Failed at creating INTR_TYPE_USER_WQ"); 
			rv = request_irq(xdev->msix[idx].vector, wq1_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_WQ_UNBOUND: 
			wq2 = alloc_workqueue("wq2", WQ_UNBOUND, 1);
			if (!wq2) pr_debug("Failed at creating INTR_TYPE_USER_WQ_UNBOUND"); 
			rv = request_irq(xdev->msix[idx].vector, wq2_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_WQ_HIGHPRI:
			wq3 = alloc_workqueue("wq3", WQ_HIGHPRI, 1);
			if (!wq3) pr_debug("Failed at creating INTR_TYPE_USER_WQ_HIGHPRI"); 
			rv = request_irq(xdev->msix[idx].vector, wq3_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_WQ_UNBOUND_HIGHPRI:
			wq4 = alloc_workqueue("wq4", WQ_UNBOUND | WQ_HIGHPRI, 1);
			if (!wq4) pr_debug("Failed at creating INTR_TYPE_USER_WQ_UNBOUND_HIGHPRI"); 
			rv = request_irq(xdev->msix[idx].vector, wq4_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		case INTR_TYPE_USER_WQ_CPUINTENSIVE:
			wq5 = alloc_workqueue("wq5", WQ_CPU_INTENSIVE, 1);
			if (!wq5) pr_debug("Failed at creating INTR_TYPE_USER_WQ_CPUINTENSIVE"); 
			rv = request_irq(xdev->msix[idx].vector, wq5_top, 0, xdev->dev_intr_info_list[idx].msix_name, 0);
			break;
		// Can only be used for Kernel >= 6.9:
		case INTR_TYPE_USER_WQ_BH:
			break;
		case INTR_TYPE_USER_WQ_BH_HIGHPRI:
			break;
		// TODO: Alternative to IRQ Affinity at Workqueue level, see https://docs.kernel.org/core-api/workqueue.html#affinity-scopes
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_CPU_STRICT: 
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_SMT_STRICT:
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_CACHE_STRICT:
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_NUMA_STRICT:
		case INTR_TYPE_USER_WQ_ORDERED:
			break;
		default:
			pr_warn("Unhandled interrupt type in initialization: %d\n", type);
	}
	// log IRQ request
	pr_debug("%s requesting IRQ vector #%d: vec %d, type %d, %s.\n", xdev->conf.name, idx, xdev->msix[idx].vector, type, xdev->dev_intr_info_list[idx].msix_name);
	if (rv) pr_err("%s requesting IRQ vector #%d: vec %d failed %d.\n", xdev->conf.name, idx, xdev->msix[idx].vector, rv);
	return rv;
}

// Free IRQs and their allocated resources
static void usr_intr_vector_teardown(enum intr_type_list type) {
	switch (type) {
		case INTR_TYPE_USER_TASKLET:
			tasklet_kill(&tk1);
			break;
		case INTR_TYPE_USER_TASKLET_HIGHPRI:
			tasklet_kill(&tk2);
			break;
		case INTR_TYPE_USER_WQ:
			flush_workqueue(wq1);
			destroy_workqueue(wq1);
			break;
		case INTR_TYPE_USER_WQ_UNBOUND:
			flush_workqueue(wq2);
			destroy_workqueue(wq2);
			break;
		case INTR_TYPE_USER_WQ_HIGHPRI:
			flush_workqueue(wq3);
			destroy_workqueue(wq3);
			break;
		case INTR_TYPE_USER_WQ_UNBOUND_HIGHPRI:
			flush_workqueue(wq4);
			destroy_workqueue(wq4);
			break;
		case INTR_TYPE_USER_WQ_CPUINTENSIVE:
			flush_workqueue(wq5);
			destroy_workqueue(wq5);
			break;
		case INTR_TYPE_USER_WQ_BH:
		case INTR_TYPE_USER_WQ_BH_HIGHPRI:
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_CPU_STRICT:
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_SMT_STRICT:
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_CACHE_STRICT:
		case INTR_TYPE_USER_WQ_UNBOUND_AFFNT_NUMA_STRICT:
		case INTR_TYPE_USER_WQ_ORDERED:
			break;
		default:
			pr_warn("Unhandled interrupt type in teardown: %d\n", type);
	}
}


int intr_setup(struct xlnx_dma_dev *xdev) {
	int rv=0, i=0, num_vecs=0, num_vecs_req=0;

	// if device is in poll or legacy interrupt mode, exit early
	if ((xdev->conf.qdma_drv_mode == POLL_MODE) || (xdev->conf.qdma_drv_mode == LEGACY_INTR_MODE)) goto exit;

	// get the number of MSI-X vectors supported by the device PF
	num_vecs = pci_msix_vec_count(xdev->conf.pdev);
	pr_debug("dev %s, xdev->num_vecs = %d\n", dev_name(&xdev->conf.pdev->dev), xdev->num_vecs);
	if (num_vecs == 0) {
		pr_warn("MSI-X not supported, running in polled mode\n");
		return 0;
	}

	// maximum available vectors allowed by configuration
	xdev->num_vecs = min_t(int, num_vecs, xdev->conf.msix_qvec_max);
	// total requested vectors = user vectors + data vectors (+ 1 error vector + 1 mailbox vector)
	num_vecs_req = xdev->conf.user_msix_qvec_max + xdev->conf.data_msix_qvec_max;
	// if master PF, 1 vector for error interrupt
	if (xdev->conf.master_pf) num_vecs_req++; 
	// if mailbox available, 1 vector for mailbox interrupt
#ifndef MBOX_INTERRUPT_DISABLE
	if (qdma_mbox_is_irq_availabe(xdev)) num_vecs_req++;
#endif
	// check if total requested vectors exceeds available vectors
	if (num_vecs_req > xdev->num_vecs) {
		pr_warn("Available vectors(%u) is less than Requested vectors(%u) [u:%u|d:%u]\n", xdev->num_vecs, num_vecs_req, xdev->conf.user_msix_qvec_max, xdev->conf.data_msix_qvec_max);
		return -EINVAL;
	}

	// initialize MSI-X and interrupt information entries based on number of vectors
	xdev->msix = kzalloc((sizeof(struct msix_entry) * xdev->num_vecs), GFP_KERNEL);
	if (!xdev->msix) {
		pr_err("dev %s xdev->msix OOM.\n", dev_name(&xdev->conf.pdev->dev));
		rv = -ENOMEM;
		goto exit;
	}
	xdev->dev_intr_info_list = kzalloc((sizeof(struct intr_info_t) * xdev->num_vecs), GFP_KERNEL);
	if (!xdev->dev_intr_info_list) {
		pr_err("dev %s xdev->dev_intr_info_list OOM.\n", dev_name(&xdev->conf.pdev->dev));
		rv = -ENOMEM;
		goto free_msix;
	}
	for (i = 0; i < xdev->num_vecs; i++) {
		xdev->msix[i].entry = i;
		INIT_LIST_HEAD(&xdev->dev_intr_info_list[i].intr_list);
		spin_lock_init(&xdev->dev_intr_info_list[i].vec_q_list);
	}

	// enable MSI-X vectors based on kernel version
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
	rv = pci_enable_msix_exact(xdev->conf.pdev, xdev->msix, xdev->num_vecs);
#else
	rv = pci_enable_msix(xdev->conf.pdev, xdev->msix, xdev->num_vecs);
#endif
	if (rv < 0) {
		pr_err("Error enabling MSI-X (%d)\n", rv);
		goto free_intr_info;
	}

	// initialize affinity mask
	cpumask_var_t mask;
	if (!zalloc_cpumask_var(&mask, GFP_KERNEL)) {
		pr_err("Failed to allocate cpumask\n");
		return -ENOMEM;
	}
	// example: bind all interrupts to CPU 11 only (interrupts can be bound to a subset of the available cores also)
	int irq;
	for (i = 0; i < xdev->num_vecs; i++) {
		// get MSI-X vector 
		irq = pci_irq_vector(xdev->conf.pdev, i); 
		// set affinity mask for CPU 11
		cpumask_clear(mask);
		cpumask_set_cpu(11, mask); 
		rv = irq_set_affinity_and_hint(irq, mask);
		if (rv) pr_err("Failed to set affinity for vector %d\n", i);
	}
	free_cpumask_var(mask);

	// initialize memory-mapped register pointers
	if (mm_registers_init()) pr_err("Error when allocating memory mapped registers.");

	// set up user interrupt vectors with different ISRs
	i = 0;
#ifndef USER_INTERRUPT_DISABLE
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_TOP)) goto cleanup_irq;
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_TASKLET)) goto cleanup_irq;
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_TASKLET_HIGHPRI)) goto cleanup_irq;
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_THREADED)) goto cleanup_irq;
 	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_WQ)) goto cleanup_irq;
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_WQ_UNBOUND)) goto cleanup_irq;
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_WQ_HIGHPRI)) goto cleanup_irq;
	if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_WQ_UNBOUND_HIGHPRI)) goto cleanup_irq;
	// if (usr_intr_vector_init(xdev, i++, INTR_TYPE_USER_WQ_CPUINTENSIVE)) goto cleanup_irq;
#endif

	xdev->flags |= XDEV_FLAG_IRQ;
	return 0;

cleanup_irq: // free IRQs in case of failure
	while (--i >= 0) free_irq(xdev->msix[i].vector, xdev);
	pci_disable_msix(xdev->conf.pdev);
	xdev->num_vecs = 0;

free_intr_info:
	kfree(xdev->dev_intr_info_list);

free_msix:
	kfree(xdev->msix); // free MSI-X entries

exit:
	return rv;
}

// Teardown function to clean up interrupt resources
void intr_teardown(struct xlnx_dma_dev *xdev) {
	int i = 7; // quick fix with with hardcoded number of vectors instead of xdev->num_vecs  
	while (--i >= 0) free_irq(xdev->msix[i].vector, xdev);
	if (xdev->num_vecs) pci_disable_msix(xdev->conf.pdev);
	kfree(xdev->msix);
	kfree(xdev->dev_intr_info_list);
	// clean up ISR resources
	usr_intr_vector_teardown(INTR_TYPE_USER_TOP);
	usr_intr_vector_teardown(INTR_TYPE_USER_TASKLET);
	usr_intr_vector_teardown(INTR_TYPE_USER_TASKLET_HIGHPRI);
	usr_intr_vector_teardown(INTR_TYPE_USER_THREADED);
	usr_intr_vector_teardown(INTR_TYPE_USER_WQ);
	usr_intr_vector_teardown(INTR_TYPE_USER_WQ_UNBOUND);
	usr_intr_vector_teardown(INTR_TYPE_USER_WQ_HIGHPRI);
	usr_intr_vector_teardown(INTR_TYPE_USER_WQ_UNBOUND_HIGHPRI);
	// usr_intr_vector_teardown(INTR_TYPE_USER_WQ_CPUINTENSIVE);
	// tear down memory-mapped registers
	mm_registers_teardown();
}


// The rest of the file is out of the scope of our thesis


#ifndef __QDMA_VF__
static LIST_HEAD(legacy_intr_q_list);
static spinlock_t legacy_intr_lock;
static spinlock_t legacy_q_add_lock;
static unsigned long legacy_intr_flags = IRQF_SHARED;
#endif

static inline void intr_ring_free(struct xlnx_dma_dev *xdev, int ring_sz,
			int intr_desc_sz, u8 *intr_desc, dma_addr_t desc_bus)
{
	unsigned int len = ring_sz * intr_desc_sz;

	pr_debug("free %u(0x%x)=%d*%u, 0x%p, bus 0x%llx.\n",
		len, len, intr_desc_sz, ring_sz, intr_desc, desc_bus);

	dma_free_coherent(&xdev->conf.pdev->dev, (size_t)ring_sz * intr_desc_sz,
			intr_desc, desc_bus);
}

static void *intr_ring_alloc(struct xlnx_dma_dev *xdev, int ring_sz,
				int intr_desc_sz, dma_addr_t *bus)
{
	unsigned int len = ring_sz * intr_desc_sz;
	u8 *p = dma_alloc_coherent(&xdev->conf.pdev->dev, len, bus, GFP_KERNEL);

	if (!p) {
		pr_err("%s, OOM, sz ring %d, intr_desc %d.\n",
			xdev->conf.name, ring_sz, intr_desc_sz);
		return NULL;
	}

	memset(p, 0, len);

	pr_debug("alloc %u(0x%x)=%d*%u, bus 0x%llx .\n",
		len, len, intr_desc_sz, ring_sz, *bus);

	return p;
}

#ifdef __QDMA_VF__
static void intr_context_invalidate(struct xlnx_dma_dev *xdev)
{
	int i = 0;
	struct mbox_msg *m;
	int rv = 0;
	struct mbox_msg_intr_ctxt ictxt;
	struct intr_coal_conf  *ring_entry;

	m = qdma_mbox_msg_alloc();
	if (!m)
		return;
	memset(&ictxt, 0, sizeof(struct mbox_msg_intr_ctxt));
	ictxt.num_rings = QDMA_NUM_DATA_VEC_FOR_INTR_CXT;

	for (i = 0; i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		ictxt.ring_index_list[i] =
			get_intr_ring_index(xdev, xdev->dvec_start_idx + i);
	}
	qdma_mbox_compose_vf_intr_ctxt_invalidate(xdev->func_id,
			&ictxt, m->raw);
	rv = qdma_mbox_msg_send(xdev, m, 1, QDMA_MBOX_MSG_TIMEOUT_MS);
	if (rv < 0) {
		pr_err("%s invalidate interrupt context failed %d.\n",
			xdev->conf.name, rv);
	}

	qdma_mbox_msg_free(m);

	for (i = 0; i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT; i++) {
		ring_entry = (xdev->intr_coal_list + i);
		if (ring_entry) {
			intr_ring_free(xdev,
				ring_entry->intr_rng_num_entries,
				sizeof(union qdma_intr_ring),
				(u8 *)ring_entry->intr_ring_base,
				ring_entry->intr_ring_bus);
		}
	}

}
#else
static void intr_context_invalidate(struct xlnx_dma_dev *xdev)
{
	int i = 0;
	unsigned int ring_index = 0;
	struct intr_coal_conf  *ring_entry;
	int rv = 0;

	while (i < QDMA_NUM_DATA_VEC_FOR_INTR_CXT) {
		ring_index = get_intr_ring_index(xdev,
				(i + xdev->dvec_start_idx));
		rv = xdev->hw.qdma_indirect_intr_ctx_conf(xdev, ring_index,
				NULL, QDMA_HW_ACCESS_INVALIDATE);
		if (rv < 0) {
			pr_err("Intr ctxt invalidate failed, err = %d",
						rv);
			return;
		}
		ring_entry = (xdev->intr_coal_list + i);
		if (ring_entry) {
			intr_ring_free(xdev,
				ring_entry->intr_rng_num_entries,
				sizeof(union qdma_intr_ring),
				(u8 *)ring_entry->intr_ring_base,
				ring_entry->intr_ring_bus);
		}
		i++;
	}

}
#endif

void intr_ring_teardown(struct xlnx_dma_dev *xdev)
{
	intr_context_invalidate(xdev);
	kfree(xdev->intr_coal_list);
}

#ifdef __PCI_MSI_VEC_COUNT__

#define msix_table_size(flags)	((flags & PCI_MSIX_FLAGS_QSIZE) + 1)

static int pci_msix_vec_count(struct pci_dev *dev)
{
	u16 control;

	if (!dev->msix_cap)
		return 0;

	pci_read_config_word(dev, dev->msix_cap + PCI_MSIX_FLAGS, &control);
	return msix_table_size(control);
}
#endif

#ifndef __QDMA_VF__
static irqreturn_t irq_top(int irq, void *dev_id)
{
	struct xlnx_dma_dev *xdev = dev_id;

	if (xdev->conf.fp_q_isr_top_dev) {
		xdev->conf.fp_q_isr_top_dev((unsigned long)xdev,
					xdev->conf.uld);
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t irq_legacy(int irq, void *irq_data)
{
	struct list_head *entry, *tmp;
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)irq_data;
	irqreturn_t ret = IRQ_NONE;

	if (!xdev) {
		pr_err("Invalid Xdev");
		goto irq_return;
	}

	spin_lock_irqsave(&legacy_intr_lock, legacy_intr_flags);
	if (!xdev->hw.qdma_is_legacy_intr_pend(xdev)) {

		list_for_each_safe(entry, tmp, &legacy_intr_q_list) {
			struct qdma_descq *descq =
					container_of(entry,
						     struct qdma_descq,
						     legacy_intr_q_list);

			qdma_descq_service_cmpl_update(descq, 0, 1);
		}
		xdev->hw.qdma_clear_pend_legacy_intr(xdev);
		xdev->hw.qdma_legacy_intr_conf(xdev, ENABLE);
		ret = IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&legacy_intr_lock, legacy_intr_flags);

irq_return:
	return ret;
}

void intr_legacy_clear(struct qdma_descq *descq)
{

	if (!descq) {
		pr_err("Invalid descq received");
		return;
	}
	list_del(&descq->legacy_intr_q_list);

	if (list_empty(&legacy_intr_q_list)) {

		pr_info("un-registering legacy interrupt from qdma%05x\n",
			descq->xdev->conf.bdf);

		descq->xdev->hw.qdma_legacy_intr_conf(descq->xdev, DISABLE);

		free_irq(descq->xdev->conf.pdev->irq, descq->xdev);
	}
}

int intr_legacy_setup(struct qdma_descq *descq)
{
	int req_irq = 0;
	int rv = 0;

	if (!descq) {
		pr_err("Invalid descq received");
		return -EINVAL;
	}

	spin_lock(&legacy_q_add_lock);
	req_irq = list_empty(&legacy_intr_q_list);
	rv = req_irq ? 0 : 1;

	if (req_irq != 0) {
		spin_lock_init(&legacy_intr_lock);
		pr_debug("registering legacy interrupt for irq-%d from qdma%05x\n",
			descq->xdev->conf.pdev->irq, descq->xdev->conf.bdf);

		if (descq->xdev->hw.qdma_legacy_intr_conf(descq->xdev,
								DISABLE)) {
			spin_unlock(&legacy_q_add_lock);
			return -EINVAL;
		}

		rv = request_threaded_irq(descq->xdev->conf.pdev->irq, irq_top,
					  irq_legacy, legacy_intr_flags,
					  "qdma legacy intr",
					  descq->xdev);

		if (rv < 0)
			goto exit_intr_setup;
		else {
			list_add_tail(&descq->legacy_intr_q_list,
				      &legacy_intr_q_list);
			rv = 0;
		}
		if (descq->xdev->hw.qdma_legacy_intr_conf(descq->xdev,
								ENABLE)) {
			spin_unlock(&legacy_q_add_lock);
			return -EINVAL;
		}
	} else
		list_add_tail(&descq->legacy_intr_q_list,
			      &legacy_intr_q_list);

exit_intr_setup:
	spin_unlock(&legacy_q_add_lock);
	return rv;
}
#endif

int intr_ring_setup(struct xlnx_dma_dev *xdev)
{
	int num_entries = 0;
	int counter = 0;
	struct intr_coal_conf  *intr_coal_list;
	struct intr_coal_conf  *intr_coal_list_entry;

	if ((xdev->conf.qdma_drv_mode != INDIRECT_INTR_MODE) &&
			(xdev->conf.qdma_drv_mode != AUTO_MODE)) {
		pr_debug("skipping interrupt aggregation: driver is loaded in %s mode\n",
			mode_name_list[xdev->conf.qdma_drv_mode].name);
		xdev->intr_coal_list = NULL;
		return 0;
	}

	/** For master_pf, vec1 and vec2 is used for
	 *  error and user interrupts
	 *  for other pfs, vec0 is used for user interrupts
	 */
	if (xdev->num_vecs != 0) {
		pr_debug("dev %s num_vectors[%d] < num_queues [%d]\n",
					dev_name(&xdev->conf.pdev->dev),
					xdev->num_vecs,
					xdev->conf.qsets_max);
		pr_debug("Enabling Interrupt aggregation\n");

		/** obtain the number of queue entries
		 * in each inr_ring based on ring size
		 */
		num_entries = ((xdev->conf.intr_rngsz + 1) * 512);

		pr_debug("%s interrupt coalescing ring with %d entries\n",
			dev_name(&xdev->conf.pdev->dev), num_entries);
		/**
		 * Initially assuming that each vector has the same size of the
		 * ring, In practical it is possible to have different ring
		 * size of different vectors (?)
		 */
		intr_coal_list = kzalloc(
				sizeof(struct intr_coal_conf) *
				QDMA_NUM_DATA_VEC_FOR_INTR_CXT,
				GFP_KERNEL);
		if (!intr_coal_list) {
			pr_err("dev %s num_vecs %d OOM.\n",
				dev_name(&xdev->conf.pdev->dev),
				QDMA_NUM_DATA_VEC_FOR_INTR_CXT);
			return -ENOMEM;
		}

		for (counter = 0;
			counter < QDMA_NUM_DATA_VEC_FOR_INTR_CXT;
			counter++) {
			intr_coal_list_entry = (intr_coal_list + counter);
			intr_coal_list_entry->intr_rng_num_entries =
							num_entries;
			intr_coal_list_entry->intr_ring_base = intr_ring_alloc(
					xdev, num_entries,
					sizeof(union qdma_intr_ring),
					&intr_coal_list_entry->intr_ring_bus);
			if (!intr_coal_list_entry->intr_ring_base) {
				pr_err("dev %s, sz %u, intr_desc ring OOM.\n",
				xdev->conf.name,
				intr_coal_list_entry->intr_rng_num_entries);
				goto err_out;
			}

			intr_coal_list_entry->vec_id =
			xdev->msix[counter + xdev->dvec_start_idx].entry;
			intr_coal_list_entry->intr_cidx_info.sw_cidx = 0;
			intr_coal_list_entry->color = 1;
			intr_coal_list_entry->intr_cidx_info.rng_idx =
					get_intr_ring_index(xdev,
					    intr_coal_list_entry->vec_id);
			pr_debug("ring_number = %d, vector_index = %d, ring_size = %d, ring_base = 0x%08x",
			    counter, intr_coal_list_entry->vec_id,
			    intr_coal_list_entry->intr_rng_num_entries,
			    (unsigned int)intr_coal_list_entry->intr_ring_bus);
		}

		pr_debug("dev %s interrupt coalescing ring setup successful\n",
					dev_name(&xdev->conf.pdev->dev));

		xdev->intr_coal_list = intr_coal_list;
	} else {
		pr_info("dev %s intr vec[%d] >= queues[%d], No aggregation\n",
			dev_name(&xdev->conf.pdev->dev),
			(xdev->num_vecs - xdev->dvec_start_idx),
			xdev->conf.qsets_max);

		xdev->intr_coal_list = NULL;
		/* Fallback from indirect interrupt mode */
		xdev->conf.qdma_drv_mode = POLL_MODE;
	}
	return 0;

err_out:
	while (--counter >= 0) {
		intr_coal_list_entry = (intr_coal_list + counter);
		intr_ring_free(xdev, intr_coal_list_entry->intr_rng_num_entries,
				sizeof(union qdma_intr_ring),
				(u8 *)intr_coal_list_entry->intr_ring_base,
				intr_coal_list_entry->intr_ring_bus);
	}
	kfree(intr_coal_list);
	return -ENOMEM;
}

/**
 * qdma_queue_service - service the queue
 * in the case of irq handler is registered by the user, the user should
 * call qdma_queue_service() in its interrupt handler to service the queue
 * @dev_hndl: hndl retured from qdma_device_open()
 * @qhndl: hndl retured from qdma_queue_add()
 */
int qdma_queue_service(unsigned long dev_hndl, unsigned long id, int budget,
			bool c2h_upd_cmpl)
{
	struct xlnx_dma_dev *xdev = (struct xlnx_dma_dev *)dev_hndl;
	struct qdma_descq *descq;

	/** make sure that the dev_hndl passed is Valid */
	if (!xdev) {
		pr_err("dev_hndl is NULL");
		return -EINVAL;
	}

	if (xdev_check_hndl(__func__, xdev->conf.pdev, dev_hndl) < 0) {
		pr_err("Invalid dev_hndl passed");
		return -EINVAL;
	}

	descq = qdma_device_get_descq_by_id(xdev, id, NULL, 0, 0);
	if (descq)
		return qdma_descq_service_cmpl_update(descq,
					budget, c2h_upd_cmpl);

	return -EINVAL;
}

static u8 get_intr_vec_index(struct xlnx_dma_dev *xdev, u8 intr_type)
{
	int i = 0;

	for (i = 0; i < xdev->num_vecs; i++) {
		if (xdev->dev_intr_info_list[i].intr_vec_map.intr_type ==
		    intr_type) {
			struct intr_info_t *dev_intr_info_list =
					&xdev->dev_intr_info_list[i];
			return dev_intr_info_list->intr_vec_map.intr_vec_index;
		}
	}
	return 0;
}

int qdma_err_intr_setup(struct xlnx_dma_dev *xdev)
{
	int rv = 0;
	u8  err_intr_index = 0;

	err_intr_index = get_intr_vec_index(xdev, INTR_TYPE_ERROR);

	rv = xdev->hw.qdma_hw_error_intr_setup(xdev, xdev->func_id,
					    err_intr_index);
	if (rv < 0) {
		pr_err("Failed to setup error interrupt, err = %d", rv);
		return -EINVAL;
	}

	return 0;
}

int get_intr_ring_index(struct xlnx_dma_dev *xdev, u32 vector_index)
{
	int ring_index = 0;

	ring_index = (vector_index - xdev->dvec_start_idx) +
			(xdev->func_id * QDMA_NUM_DATA_VEC_FOR_INTR_CXT);
	pr_debug("func_id = %d, vector_index = %d, ring_index = %d\n",
			xdev->func_id, vector_index, ring_index);

	return ring_index;
}

void intr_legacy_init(void)
{
#ifndef __QDMA_VF__
	spin_lock_init(&legacy_q_add_lock);
#endif
}

void intr_work(struct work_struct *work)
{
	struct qdma_descq *descq;

	descq = container_of(work, struct qdma_descq, work);
	qdma_descq_service_cmpl_update(descq, 0, 1);
}
