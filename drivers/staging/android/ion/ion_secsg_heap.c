/*
 * /ion/ion_secsg_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "secsg: " fmt

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/genalloc.h>
#include <linux/mutex.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/cma.h>
#include <linux/sizes.h>
#include <linux/memblock.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include <linux/hisi/hisi_cma.h>
#include <linux/hisi/hisi_ion.h>
#include <linux/hisi/hisi_mm.h>
#include <teek_client_api.h>
#include <teek_client_id.h>
#include <teek_client_constants.h>

#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#include "ion.h"
#include "ion_priv.h"

/*uuid to TA: f8028dca-aba0-11e6-80f5-76304dec7eb7*/
#define UUID_TEEOS_TZMP2_IonMemoryManagement \
{ \
	0xf8028dca,\
	0xaba0,\
	0x11e6,\
	{ \
		0x80, 0xf5, 0x76, 0x30, 0x4d, 0xec, 0x7e, 0xb7 \
	} \
}

enum SECSG_HEAP_TYPE {
	HEAP_NORMAL = 0,
	HEAP_PROTECT = 1,
	HEAP_SECURE = 2,
	HEAP_MAX,
};

#define ION_PBL_SHIFT 12
#define ION_NORMAL_SHIFT 16
#define DEVICE_MEMORY PROT_DEVICE_nGnRE

#define SECBOOT_CMD_ID_MEM_ALLOCATE 0x1

#ifdef CONFIG_HISI_ION_SECSG_DEBUG
#define secsg_debug(fmt, ...) \
	pr_info(fmt, ##__VA_ARGS__)
#else
#define secsg_debug(fmt, ...)
#endif

struct ion_secsg_heap {
	struct ion_heap heap;
	struct cma *cma;
	struct gen_pool *pool;
	/* heap mutex */
	struct mutex mutex;
	/* heap attr: secure, protect, non_sec */
	u32 heap_attr;
	u32 pool_shift;
	/* heap total size*/
	size_t heap_size;
	/* heap allocated size*/
	unsigned long alloc_size;
	/* heap flag */
	u64 flag;
	u64 per_alloc_sz;
	/* align size = 64K*/
	u64 per_bit_sz;
	u64 water_mark;
	phys_addr_t pgtable_phys;
	u32 pgtable_size;
	struct list_head allocate_head;
	TEEC_Context *context;
	TEEC_Session *session;
	int iova_init;
	int TA_init;
};

enum SEC_Task{
	SEC_TASK_DRM = 0x0,
	SEC_TASK_SEC,
	SEC_TASK_MAX,
};

struct mem_chunk_list {
	u32 protect_id;
	union {
		u32 nents;
		u32 buff_id;
	};
	u32 va;
	void *phys_addr;  /* Must be the start addr of struct tz_pageinfo */
	u32 size;
};

struct tz_pageinfo {
	u64 addr;
	u32 nr_pages;
} __aligned(8);

struct alloc_list {
	u64 addr;
	u32 size;
	struct list_head list;
};

struct cma *hisi_secsg_cma;

int hisi_sec_cma_set_up(struct reserved_mem *rmem)
{
	phys_addr_t align = PAGE_SIZE << max(MAX_ORDER - 1, pageblock_order);
	phys_addr_t mask = align - 1;
	unsigned long node = rmem->fdt_node;
	struct cma *cma;
	int err;

	if (!of_get_flat_dt_prop(node, "reusable", NULL) ||
	    of_get_flat_dt_prop(node, "no-map", NULL))
		return -EINVAL;

	if ((rmem->base & mask) || (rmem->size & mask)) {
		pr_err("Reserved memory: incorrect alignment of CMA region\n");
		return -EINVAL;
	}

	if(!memblock_is_memory(rmem->base)){
		memblock_free(rmem->base, rmem->size);
		pr_err("memory is invalid(0x%llx), size(0x%llx)\n",
			rmem->base, rmem->size);
		return -EINVAL;
	}
	err = cma_init_reserved_mem(rmem->base, rmem->size, 0, &cma);
	if (err) {
		pr_err("Reserved memory: unable to setup CMA region\n");
		return err;
	}

	hisi_secsg_cma = cma;
	ion_register_dma_camera_cma((void *)cma);

	return 0;
}
/*lint -e528 -esym(528,RESERVEDMEM_OF_DECLARE)*/
RESERVEDMEM_OF_DECLARE(hisi_secsg_cma, "hisi-cma-pool", hisi_sec_cma_set_up);//lint !e611
/*lint -e528 +esym(528,RESERVEDMEM_OF_DECLARE)*/

static int secsg_tee_init(struct ion_secsg_heap *secsg_heap)
{
	u32 root_id = 2000;
	const char *package_name = "sec_mem";
	TEEC_UUID svc_id = UUID_TEEOS_TZMP2_IonMemoryManagement;
	TEEC_Operation op = {0};
	TEEC_Result result;
	u32 origin = 0;

	if (secsg_heap->TA_init) {
		pr_err("TA had been opened before(%d).\n", secsg_heap->TA_init);
		return 0;
	}
	/* initialize TEE environment */
	result = TEEK_InitializeContext(NULL, secsg_heap->context);
	if(result != TEEC_SUCCESS) {
		pr_err("InitializeContext failed, ReturnCode=0x%x\n", result);
		goto cleanup_1;
	} else {
		secsg_debug("InitializeContext success\n");
	}
	/* operation params create  */
	op.started = 1;
	op.cancel_flag = 0;
	/*open session*/
	op.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE,
			    TEEC_NONE,
			    TEEC_MEMREF_TEMP_INPUT,
			    TEEC_MEMREF_TEMP_INPUT);/*lint !e845*/

	op.params[2].tmpref.buffer = (void *)&root_id;/*lint !e789*/
	op.params[2].tmpref.size = sizeof(root_id);
	op.params[3].tmpref.buffer = (void *)package_name;/*lint !e789*/
	op.params[3].tmpref.size = (size_t)(strlen(package_name) + 1);

	result = TEEK_OpenSession(secsg_heap->context, secsg_heap->session,
				  &svc_id, TEEC_LOGIN_IDENTIFY, NULL,
				  &op, &origin);
	if(result != TEEC_SUCCESS) {
		pr_err("OpenSession fail, RC=0x%x, RO=0x%x\n", result, origin);
		goto cleanup_2;
	} else {
		secsg_debug("OpenSession success\n");
	}
	secsg_heap->TA_init = 1;
	return 0;
cleanup_2:
	TEEK_FinalizeContext(secsg_heap->context);
cleanup_1:
	return -1;
}
static int secsg_tee_exec_cmd(struct ion_secsg_heap *secsg_heap,
				struct mem_chunk_list *mcl,
				u32 cmd)
{
	TEEC_Result result;
	TEEC_Operation op = {0};
	u32 protect_id = mcl->protect_id;
	u32 origin = 0;

	if (!secsg_heap->TA_init) {
		pr_err("[%s] TA not inited.\n", __func__);
		return -EINVAL;
	}

	op.started = 1;
	op.cancel_flag = 0;
	op.params[0].value.a = cmd;
	op.params[0].value.b = protect_id;

	switch (cmd) {
	case ION_SEC_CMD_PGATBLE_INIT:
		op.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_INPUT,
			TEEC_VALUE_INPUT,
			TEEC_NONE,
			TEEC_NONE);
		op.params[1].value.a = (u32)secsg_heap->pgtable_phys;
		op.params[1].value.b = (u32)secsg_heap->pgtable_size;
		break;
	case ION_SEC_CMD_ALLOC:
		op.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_INPUT,
			TEEC_VALUE_INOUT,
			TEEC_MEMREF_TEMP_INPUT,
			TEEC_NONE);
		op.params[1].value.a = mcl->nents;
		/* op.params[1].value.b receive the return value*/
		/* number of list in CMD buffer alloc/table set/table clean*/
		op.params[2].tmpref.buffer = mcl->phys_addr;
		op.params[2].tmpref.size =
			mcl->nents * sizeof(struct tz_pageinfo);
		break;
	case ION_SEC_CMD_MAP_IOMMU:
		op.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_INPUT,
			TEEC_VALUE_INOUT,
			TEEC_NONE,
			TEEC_NONE);
		op.params[1].value.a = mcl->buff_id;
		op.params[1].value.b = mcl->size;
		break;
	case ION_SEC_CMD_FREE:
	case ION_SEC_CMD_UNMAP_IOMMU:
		op.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_INPUT,
			TEEC_VALUE_INPUT,
			TEEC_NONE,
			TEEC_NONE);
		op.params[1].value.a = mcl->buff_id;
		break;
	case ION_SEC_CMD_TABLE_SET:
	case ION_SEC_CMD_TABLE_CLEAN:
		op.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_INPUT,
			TEEC_VALUE_INPUT,
			TEEC_MEMREF_TEMP_INPUT,
			TEEC_NONE);
		op.params[1].value.a = mcl->nents;
		op.params[2].tmpref.buffer = mcl->phys_addr;
		op.params[2].tmpref.size =
			mcl->nents * sizeof(struct tz_pageinfo);
		break;
	default:
		pr_err("Invalid cmd\n");
		break;
	}

	result = TEEK_InvokeCommand(secsg_heap->session,
				    SECBOOT_CMD_ID_MEM_ALLOCATE,
				    &op, &origin);
	if (result != TEEC_SUCCESS) {
		pr_err("Invoke CMD fail, RC=0x%x, RO=0x%x\n", result, origin);
		return -1;
	}

	secsg_debug("Exec TEE CMD success.\n");

	if (cmd == ION_SEC_CMD_ALLOC) {
		mcl->buff_id = op.params[1].value.b;
		secsg_debug("TEE return secbuf id 0x%x\n", mcl->buff_id);
	}

	if (cmd == ION_SEC_CMD_MAP_IOMMU) {
		mcl->va = op.params[1].value.b;
		secsg_debug("TEE return iova 0x%x\n", mcl->va);
	}

	return 0;
}

static void secsg_tee_destory(struct ion_secsg_heap *secsg_heap)
{
	if (!secsg_heap->TA_init)
		return;

	TEEK_CloseSession(secsg_heap->session);
	TEEK_FinalizeContext(secsg_heap->context);
	secsg_heap->TA_init = 0;
	secsg_debug("TA closed !\n");
}

static inline void free_alloc_list(struct list_head *head)
{
	struct alloc_list *pos = NULL;
	while (!list_empty(head)) {
		pos = list_first_entry(head, struct alloc_list, list);
		if (pos->addr || pos->size)
			pr_err("in %s %llx %x failed\n", __func__,
				pos->addr, pos->size);

		list_del(&(pos->list));
		kfree(pos);
	}
}

static u32 count_list_nr(struct ion_secsg_heap *secsg_heap)
{
	u32 nr = 0;
	struct list_head *head = &secsg_heap->allocate_head;
	struct list_head *pos;

	list_for_each(pos, head)
		nr++;
	return nr;
}

static int cons_phys_struct(struct ion_secsg_heap *secsg_heap,
			    u32 nents,
			    struct list_head *head,
			    u32 cmd_to_ta)
{
	u32 i = 0;
	int ret = 0;
	u32 protect_id = SEC_TASK_MAX;
	struct tz_pageinfo *pageinfo;
	struct mem_chunk_list mcl;
	struct list_head *pos = head->prev;
	struct alloc_list *tmp_list = NULL;
	unsigned long size = nents * sizeof(*pageinfo);

	pageinfo = (struct tz_pageinfo *)kzalloc(size, GFP_KERNEL);
	if (!pageinfo) {
		pr_err("[%s], pageinfo failed(nents = %d)\n", __func__, nents);
		return -ENOMEM;
	}

	for (i = 0; (i < nents) && (pos != (head)); i++) {
		tmp_list = list_entry(pos, struct alloc_list, list);
		pageinfo[i].addr = tmp_list->addr;
		pageinfo[i].nr_pages = tmp_list->size / PAGE_SIZE;
		pos = pos->prev;
	}

	if (i < nents) {
		pr_err("[%s], invalid nents(%d) or head!\n", __func__, nents);
		ret = -EINVAL;
		goto out;
	}

	if (secsg_heap->heap_attr == HEAP_SECURE) {
		protect_id = SEC_TASK_SEC;
	} else if (secsg_heap->heap_attr == HEAP_PROTECT) {
		protect_id = SEC_TASK_DRM;
	} else {
		pr_err("not sec heap, return.!\n");
		ret = -EINVAL;
		goto out;
	}
	/*Call TZ Driver here*/
	mcl.nents = nents;
	mcl.phys_addr = (void *)pageinfo;
	mcl.protect_id = protect_id;
	mcl.size = nents * sizeof(struct tz_pageinfo);
	ret = secsg_tee_exec_cmd(secsg_heap, &mcl, cmd_to_ta);/*lint !e732*/
out:
	kfree(pageinfo);
	return ret;
}

static int change_scatter_prop(struct ion_secsg_heap *secsg_heap,
			       struct ion_buffer *buffer,
			       u32 cmd)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct sg_table *table = buffer->priv_virt;
#else
	struct sg_table *table = buffer->sg_table;
#endif
	struct scatterlist *sg;
	struct page *page;
	struct mem_chunk_list mcl;
	struct tz_pageinfo *pageinfo = NULL;
	unsigned int nents = table->nents;
	int ret = 0;
	int i;

	if (cmd == ION_SEC_CMD_ALLOC) {
		pageinfo = (struct tz_pageinfo *)kzalloc(sizeof(*pageinfo) * nents,
							 GFP_KERNEL);
		if (!pageinfo)
			return -ENOMEM;

		for_each_sg(table->sgl, sg, table->nents, i) {/*lint !e574*/
			page = sg_page(sg);
			pageinfo[i].addr = page_to_phys(page);
			pageinfo[i].nr_pages = sg->length / PAGE_SIZE;
		}

		mcl.phys_addr = (void *)pageinfo;
		mcl.nents = nents;
		mcl.protect_id = SEC_TASK_DRM;
	} else if (cmd == ION_SEC_CMD_FREE) {
		mcl.protect_id = SEC_TASK_DRM;
		mcl.buff_id = buffer->id;
		mcl.phys_addr = NULL;
	} else {
		pr_err("%s: Error cmd\n", __func__);
		return -EINVAL;
	}

	ret = secsg_tee_exec_cmd(secsg_heap, &mcl, cmd);
	if (ret) {
		pr_err("%s:exec cmd[%d] fail\n", __func__, cmd);
		ret = -EINVAL;
	} else {
		if (cmd == ION_SEC_CMD_ALLOC)
			buffer->id = mcl.buff_id;
	}

	if (pageinfo) {
		kfree(pageinfo);
		pageinfo = NULL;
	}


	return ret;

}

static int __secsg_pgtable_init(struct ion_secsg_heap *secsg_heap)
{
	int ret = 0;
	struct mem_chunk_list mcl;

	if (secsg_heap->heap_attr != HEAP_PROTECT) {
		pr_err("normal/sec heap can't init secure iommu!\n");
		return -EINVAL;
	}

	mcl.protect_id = SEC_TASK_DRM;

	ret = secsg_tee_exec_cmd(secsg_heap, &mcl, ION_SEC_CMD_PGATBLE_INIT);
	if (ret)
		pr_err("init secure iommu fail!\n");

	return ret;
}

static int __secsg_cma_alloc(struct ion_secsg_heap *secsg_heap,
			     unsigned long user_alloc_size)
{
	int ret = 0;
	unsigned long size_remain;
	unsigned long allocated_size;
	unsigned long cma_remain;
	u64 size = secsg_heap->per_alloc_sz;
	u64 per_bit_sz = secsg_heap->per_bit_sz;
	u64 cma_size;
	/* Add for TEEOS ++, per_alloc_size = 64M */
	unsigned long virt;
	struct page *pg;
	struct alloc_list *alloc = NULL;/*lint !e429*/
	unsigned int count = 0;
	/* Add for TEEOS -- */
#ifdef CONFIG_HISI_KERNELDUMP
	unsigned int k;
	struct page *tmp_page = NULL;
#endif

	secsg_debug("into %s \n", __func__);
	/* add 64M for every times
	 * per_alloc_sz = 64M, per_bit_sz = 16M(the original min_size)
	 */
	allocated_size = secsg_heap->alloc_size;
	size_remain = gen_pool_avail(secsg_heap->pool);
	cma_size = cma_get_size(secsg_heap->cma);
	cma_remain = cma_size - (allocated_size + size_remain );
	if (secsg_heap->heap_size <= (allocated_size + size_remain )) {
		pr_err("heap full! allocated(0x%lx), heap_size(0x%lx))\n",
			allocated_size, secsg_heap->heap_size);
		return -ENOMEM;
	}

	/* we allocated more than 1M for SMMU page table before.
	 * then, for the last cma alloc , there is no 64M in
	 * cma pool. So, we allocate as much contiguous memory
	 * as we can.
	 */
retry:
	count = 1UL << get_order(size);
	pg = cma_alloc(secsg_heap->cma, (size_t)count,
		       get_order(per_bit_sz));/*lint !e516 !e866 !e834 !e732 !e747*/
	if (!pg) {
		size = size >> 1;
		if ((size >= user_alloc_size) && (size >= per_bit_sz)) {/*lint !e516 !e866 !e834 !e747*/
			pr_err("retry 0x%llx\n", size);
			goto retry;
		} else {
			pr_err("out of memory\n");
			return -ENOMEM;
		}
	}

#ifdef CONFIG_HISI_KERNELDUMP
	tmp_page = pg;
	for (k = 0; k < count; k++) {
		SetPageMemDump(tmp_page);
		tmp_page++;
	}
#endif

	alloc = kzalloc(sizeof(struct alloc_list), GFP_KERNEL);
	if(!alloc){
		pr_err("alloc list failed.\n");
		ret = -ENOMEM;
		goto err_out1;
	}
	alloc->addr = page_to_phys(pg);
	alloc->size = size;
	list_add_tail(&alloc->list, &secsg_heap->allocate_head);

	if (secsg_heap->flag & ION_FLAG_SECURE_BUFFER) {
		ion_flush_all_cpus_caches();
		virt = (unsigned long)__va(alloc->addr);/*lint !e834 !e648*/
		/*lint -save -e747*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		create_mapping_late(alloc->addr, virt, size,
				    __pgprot(DEVICE_MEMORY));
#else
		change_secpage_range(alloc->addr, virt, size,
				     __pgprot(DEVICE_MEMORY));
#endif
		/*lint -restore*/
		flush_tlb_all();
		if(cons_phys_struct(secsg_heap, 1,
			&(secsg_heap->allocate_head), ION_SEC_CMD_TABLE_SET)){
			pr_err("cons_phys_struct failed \n");
			ret = -EINVAL;
			goto err_out2;
		}
	}else {
		memset(page_address(pg), 0x0, size);/*lint !e747*/ /* unsafe_function_ignore: memset  */
		ion_flush_all_cpus_caches();
	}
	gen_pool_free(secsg_heap->pool, page_to_phys(pg), size);/*lint !e747*/
	secsg_debug("out %s %llu MB memory(ret = %d). \n",
			__func__, size / SZ_1M, ret);
	return 0;/*lint !e429*/
err_out2:
	/*lint -save -e747 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	create_mapping_late(alloc->addr, virt, size,
			    PAGE_KERNEL);
#else
	change_secpage_range(alloc->addr, virt, size,
			     PAGE_KERNEL);
#endif
	flush_tlb_all();
	/*lint -restore*/
	list_del(&alloc->list);
	kfree(alloc);
err_out1:
	cma_release(secsg_heap->cma, pg, count);
	return ret;
}

static int __secsg_heap_input_check(struct ion_secsg_heap *secsg_heap,
				    unsigned long size, unsigned long flag)
{
	if ((secsg_heap->alloc_size + size) > secsg_heap->heap_size) {
		pr_err("alloc size = 0x%lx, size = 0x%lx, heap size = 0x%lx\n",
			secsg_heap->alloc_size, size, secsg_heap->heap_size);
		return -EINVAL;
	}

	if (size > secsg_heap->per_alloc_sz) {
		pr_err("size too large! size 0x%lx, per_alloc_sz 0x%llx",
			size, secsg_heap->per_alloc_sz);
		return -EINVAL;
	}

	if (((secsg_heap->heap_attr == HEAP_SECURE) ||
	    (secsg_heap->heap_attr == HEAP_PROTECT)) &&
	    !(flag & ION_FLAG_SECURE_BUFFER)) {
		pr_err("alloc mem w/o sec flag in sec heap(%u) flag(%lx)\n",
		       secsg_heap->heap_attr, flag);
		return -EINVAL;
	}

	if ((secsg_heap->heap_attr == HEAP_NORMAL) &&
	    (flag & ION_FLAG_SECURE_BUFFER)) {
		pr_err("invalid allocate sec in sec heap(%u) flag(%lx)\n",
				secsg_heap->heap_attr, flag);
		return -EINVAL;
	}

	secsg_heap->flag = flag;
	return 0;
}
static int __secsg_create_pool(struct ion_secsg_heap *secsg_heap)
{
	u64 cma_base;
	u64 cma_size;
	int ret = 0;

	secsg_debug("into %s\n", __func__);
	/* Allocate on 4KB boundaries (1 << ION_PBL_SHIFT)*/
	secsg_heap->pool = gen_pool_create(secsg_heap->pool_shift, -1);

	if (!secsg_heap->pool) {
		pr_err("in __secsg_create_pool create failed\n");
		return -ENOMEM;
	}
	gen_pool_set_algo(secsg_heap->pool, gen_pool_best_fit, NULL);

	/* Add all memory to genpool first，one chunk only*/
	cma_base = cma_get_base(secsg_heap->cma);
	cma_size = cma_get_size(secsg_heap->cma);
	if (gen_pool_add(secsg_heap->pool, cma_base, cma_size, -1)) {
		pr_err("cma_base 0x%llx cma_size 0x%llx\n", cma_base, cma_size);
		ret = -ENOMEM;
		goto err_add;
	}

	/* Alloc the 512M memory first*/
	if (!gen_pool_alloc(secsg_heap->pool, cma_size)) {
		pr_err("in __secsg_create_pool alloc failed\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	return 0;
err_alloc:
	gen_pool_destroy(secsg_heap->pool);
err_add:
	secsg_heap->pool = NULL;
	return ret;
}

static bool gen_pool_bulk_free(struct gen_pool *pool, u32 size)
{
	int i;
	unsigned long offset = 0;

	for (i = 0; i < (size / PAGE_SIZE); i++) {/*lint !e574*/
		offset = gen_pool_alloc(pool, PAGE_SIZE);/*lint !e747*/
		if (!offset) {
			pr_err("%s:%d:gen_pool_alloc failed!\n",
			       __func__, __LINE__);
			return false;
		}
	}
	return true;
}

static void __secsg_pool_release(struct ion_secsg_heap *secsg_heap)
{
	u32 nents;
	u64 addr;
	u32 size;
	unsigned long virt;
	unsigned long size_remain = 0;
	unsigned long offset = 0;/*lint !e438 !e529 */
	struct alloc_list *pos;

	if (secsg_heap->flag & ION_FLAG_SECURE_BUFFER) {
		nents = count_list_nr(secsg_heap);
		if (nents &&
		    cons_phys_struct(secsg_heap, nents,
				     &secsg_heap->allocate_head,
				     ION_SEC_CMD_TABLE_CLEAN)) {
			pr_err("heap_type:(%u)unconfig failed!!!\n",
			       secsg_heap->heap_attr);
			goto out;
		}
	}

	if (!list_empty(&secsg_heap->allocate_head)) {
		list_for_each_entry(pos, &secsg_heap->allocate_head, list) {
			addr = pos->addr;
			size = pos->size;
			offset = gen_pool_alloc(secsg_heap->pool, size);/*lint !e747*/
			if (!offset) {
				pr_err("%s:%d:gen_pool_alloc failed! but it's nothing %llx %x\n",
				       __func__, __LINE__, addr, size);
				continue;
			}
			virt = (unsigned long)__va(addr);/*lint !e834 !e648*/
			/*lint -save -e747 */
			if (secsg_heap->flag & ION_FLAG_SECURE_BUFFER)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
				create_mapping_late(addr, virt, size,
						    PAGE_KERNEL);
#else
				change_secpage_range(addr, virt, size,
						     PAGE_KERNEL);
#endif

			/*lint -restore*/
			cma_release(secsg_heap->cma, phys_to_page(addr),
				    (1U << get_order(size)));/*lint !e516 !e866 !e834 !e747*/
			pos->addr = 0;
			pos->size = 0;
		}
	}

	if (!list_empty(&secsg_heap->allocate_head)) {
		list_for_each_entry(pos, &secsg_heap->allocate_head, list) {
			addr = pos->addr;
			size = pos->size;
			if (!addr || !size)
				continue;

			if (unlikely(!gen_pool_bulk_free(secsg_heap->pool,
							 size))) {
				pr_err("%s:%d:gen_poo_bulk_free failed! %llx %x\n",
				       __func__, __LINE__, addr, size);
				continue;
			}

			virt = (unsigned long)__va(addr);/*lint !e834 !e648*/
			/*lint -save -e747 */
			if (secsg_heap->flag & ION_FLAG_SECURE_BUFFER)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
				create_mapping_late(addr, virt, size,
						    PAGE_KERNEL);
#else
				change_secpage_range(addr, virt, size,
						     PAGE_KERNEL);
#endif
			/*lint -restore*/
			cma_release(secsg_heap->cma, phys_to_page(addr),
				    (1U << get_order(size)));/*lint !e516 !e866 !e834 !e747*/
			pos->addr = 0;
			pos->size = 0;
		}
		free_alloc_list(&secsg_heap->allocate_head);
	}
	flush_tlb_all();
out:
	size_remain = gen_pool_avail(secsg_heap->pool);
	if (size_remain)
		pr_err("out %s, size_remain = 0x%lx(0x%lx)\n",
			__func__, size_remain, offset);
	return;/*lint !e438*/
}/*lint !e550*/

static struct page *__secsg_alloc_large(struct ion_secsg_heap *secsg_heap,
					struct ion_buffer *buffer,
					unsigned long size,
					unsigned long align)
{
	struct page *page;
	int count = 0;
#ifdef CONFIG_HISI_KERNELDUMP
	int k;
	struct page *tmp_page = NULL;
#endif

	count = size / PAGE_SIZE;
	page = cma_alloc(secsg_heap->cma, count, get_order(align));
	if (!page) {
		pr_err("alloc cma fail, count:0x%x\n", count);
		return NULL;
	}

#ifdef CONFIG_HISI_KERNELDUMP
	tmp_page = page;
	for (k = 0; k < count; k++) {
		SetPageMemDump(tmp_page);
		tmp_page++;
	}
#endif
	return page;
}

static int __secsg_alloc_scatter(struct ion_secsg_heap *secsg_heap,
				 struct ion_buffer *buffer,
				 unsigned long size)
{
	struct sg_table *table;
	struct scatterlist *sg;
	struct page *page;
	unsigned long per_bit_sz = secsg_heap->per_bit_sz;
	unsigned long size_remaining = ALIGN(size, per_bit_sz);
	unsigned long alloc_size = 0;
	unsigned long nents = ALIGN(size, SZ_2M) / SZ_2M;
	int ret = 0;
	int i = 0;

	secsg_debug("%s: enter, ALIGN size 0x%lx\n", __func__, size_remaining);
	table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	if (sg_alloc_table(table, nents, GFP_KERNEL))
		goto free_table;

	sg = table->sgl;
	while (size_remaining) {
		if (size_remaining > SZ_2M)
			alloc_size = SZ_2M;
		else
			alloc_size = size_remaining;

		page = __secsg_alloc_large(secsg_heap, buffer,
					   alloc_size, per_bit_sz);
		if (!page) {
			pr_err("%s: alloc largest avaliable failed!\n",
				__func__);
			goto free_pages;
		}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		create_mapping_late(page_to_phys(page),
				    (unsigned long)page_address(page),
				    alloc_size, __pgprot(DEVICE_MEMORY));
#else
		change_secpage_range(page_to_phys(page),
				    (unsigned long)page_address(page),
				    alloc_size, __pgprot(DEVICE_MEMORY));
#endif
		size_remaining -= alloc_size;
		sg_set_page(sg, page, alloc_size, 0);
		sg = sg_next(sg);
		i++;
	}
	flush_tlb_all();
	ion_flush_all_cpus_caches();

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	buffer->priv_virt = table;
#else
	buffer->sg_table = table;
#endif

	ret = change_scatter_prop(secsg_heap, buffer, ION_SEC_CMD_ALLOC);
	if (ret)
		goto free_pages;

	secsg_debug("%s: exit\n", __func__);

	return 0;
free_pages:
	nents = i;
	pr_err("free %ld pages in err runtime\n", nents);
	for_each_sg(table->sgl, sg, nents, i) {/*lint !e574*/
		page = sg_page(sg);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		create_mapping_late(page_to_phys(page),
				    (unsigned long)page_address(page),
				    sg->length, PAGE_KERNEL);
#else
		change_secpage_range(page_to_phys(page),
				    (unsigned long)page_address(page),
				    sg->length, PAGE_KERNEL);
#endif
		cma_release(secsg_heap->cma, page, sg->length / PAGE_SIZE);
	}
	flush_tlb_all();
	sg_free_table(table);
free_table:
	kfree(table);

	return -ENOMEM;
}

static int __secsg_alloc_contig(struct ion_secsg_heap *secsg_heap,
				struct ion_buffer *buffer,
				unsigned long size)
{
	int ret = 0;
	unsigned long offset = 0;
	struct sg_table *table;

	table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (!table){
		pr_err("[%s] kzalloc failed .\n", __func__);
		return -ENOMEM;
	}

	if (sg_alloc_table(table, 1, GFP_KERNEL)){
		pr_err("[%s] sg_alloc_table failed .\n", __func__);
		ret = -ENOMEM;
		goto err_out1;
	}
	/*align size*/
	offset = gen_pool_alloc(secsg_heap->pool, size);
	if(!offset){
		ret = __secsg_cma_alloc(secsg_heap, size);
		if (ret)
			goto err_out2;
		offset = gen_pool_alloc(secsg_heap->pool, size);
		if (!offset) {
			ret = -ENOMEM;
			pr_err("line = %d, in __secsg_alloc, gen_pool_alloc failed!\n", __LINE__);
			goto err_out2;
		}
	}
	sg_set_page(table->sgl, pfn_to_page(PFN_DOWN(offset)), size, 0);/*lint !e712 !e747*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	buffer->priv_virt = table;
#else
	buffer->sg_table = table;
#endif
	if (secsg_heap->heap_attr == HEAP_SECURE)
		pr_info("__secsg_alloc sec buffer phys %lx, size %lx\n",
			offset, size);

	secsg_debug(" out [%s] .\n", __func__);
	return ret;
err_out2:
	sg_free_table(table);
err_out1:
	kfree(table);
	return ret;
}

static int secsg_alloc(struct ion_secsg_heap *secsg_heap,
			struct ion_buffer *buffer,
			unsigned long size)
{
	int ret = 0;

	if (secsg_heap->heap_attr == HEAP_PROTECT)
		ret = __secsg_alloc_scatter(secsg_heap, buffer, size);
	else
		ret = __secsg_alloc_contig(secsg_heap, buffer, size);

	return ret;
}

static void __secsg_free_pool(struct ion_secsg_heap *secsg_heap,
			      struct sg_table *table,
			      struct ion_buffer *buffer)
{
	struct page *page = sg_page(table->sgl);
	ion_phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));
	struct platform_device *hisi_ion_dev = get_hisi_ion_platform_device();

	if (!(buffer->flags & ION_FLAG_SECURE_BUFFER)) {
		(void)ion_heap_buffer_zero(buffer);
		if (buffer->flags & ION_FLAG_CACHED)
			dma_sync_sg_for_device(&hisi_ion_dev->dev, table->sgl,
					table->nents, DMA_BIDIRECTIONAL);
	}
	gen_pool_free(secsg_heap->pool, paddr, buffer->size);
	if (secsg_heap->heap_attr == HEAP_SECURE)
		pr_info("__secsg_free sec buffer phys %lx, size %zx\n",
			paddr, buffer->size);

	sg_free_table(table);
	kfree(table);

	secsg_debug("out %s\n", __func__);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
int ion_secmem_heap_phys(struct ion_heap *heap,
		struct ion_buffer *buffer,
		ion_phys_addr_t *addr, size_t *len)
{
	struct sg_table *table = buffer->sg_table;
	struct page *page = sg_page(table->sgl);
	struct ion_secsg_heap *secsg_heap;

	if (heap->type != ION_HEAP_TYPE_SECSG) {
		pr_err("%s: not secsg mem!\n", __func__);
		return -EINVAL;
	}

	secsg_heap = container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/

	if(secsg_heap->heap_attr == HEAP_PROTECT) {
		*addr = buffer->id;
		*len = buffer->size;
	} else {
		ion_phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));
		*addr = paddr;
		*len = buffer->size;
	}

	return 0;
}
#endif

static int __secsg_fill_watermark(struct ion_secsg_heap *secsg_heap)
{
	struct page *pg;
	u64 size = secsg_heap->water_mark;
	u64 per_bit_sz = secsg_heap->per_bit_sz;
	struct alloc_list *alloc;
	unsigned int count = 1UL << get_order(size);
	unsigned int align = get_order(per_bit_sz);
#ifdef CONFIG_HISI_KERNELDUMP
	unsigned int k;
	struct page *tmp_page = NULL;
#endif

	if (!size || size > secsg_heap->per_alloc_sz)
		return -EINVAL;

	pg = cma_alloc(secsg_heap->cma, (size_t)count, align);
	if (!pg) {
		pr_err("%s:alloc cma fail\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_HISI_KERNELDUMP
	tmp_page = pg;
	for (k = 0; k < count; k++) {
		SetPageMemDump(tmp_page);
		tmp_page++;
	}
#endif

	alloc = kzalloc(sizeof(*alloc), GFP_KERNEL);
	if (!alloc)
		goto err;

	alloc->addr = page_to_phys(pg);
	alloc->size = size;
	list_add_tail(&alloc->list, &secsg_heap->allocate_head);

	memset(page_address(pg), 0x0, size);/*lint !e747*//* unsafe_function_ignore: memset */
	gen_pool_free(secsg_heap->pool, page_to_phys(pg), size);/*lint !e747*/
	secsg_debug("out %s %llu MB memory.\n",
		    __func__, (size) / SZ_1M);
	return 0;/*lint !e429*/
err:
	cma_release(secsg_heap->cma, pg, count);
	return -ENOMEM;
}

static void __secsg_free_scatter(struct ion_secsg_heap *secsg_heap,
				 struct ion_buffer *buffer)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct sg_table *table = buffer->priv_virt;
#else
	struct sg_table *table = buffer->sg_table;
#endif
	struct scatterlist *sg;
	int ret = 0;
	int i;

	ret = change_scatter_prop(secsg_heap, buffer, ION_SEC_CMD_FREE);
	if (ret) {
		pr_err("release MPU protect fail! Need check DRM runtime\n");
		return;
	}

	for_each_sg(table->sgl, sg, table->nents, i) {/*lint !e574*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		create_mapping_late(page_to_phys(sg_page(sg)),
				    (unsigned long)page_address(sg_page(sg)),
				    sg->length, PAGE_KERNEL);
#else
		change_secpage_range(page_to_phys(sg_page(sg)),
				    (unsigned long)page_address(sg_page(sg)),
				    sg->length, PAGE_KERNEL);
#endif
		cma_release(secsg_heap->cma, sg_page(sg),
			    sg->length / PAGE_SIZE);
	}
	flush_tlb_all();
	sg_free_table(table);
	kfree(table);
	secsg_heap->alloc_size -= buffer->size;
}

static void __secsg_free_contig(struct ion_secsg_heap *secsg_heap,
				struct ion_buffer *buffer)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct sg_table *table = buffer->priv_virt;
#else
	struct sg_table *table = buffer->sg_table;
#endif

	__secsg_free_pool(secsg_heap, table, buffer);
	WARN_ON(secsg_heap->alloc_size < buffer->size);
	secsg_heap->alloc_size -= buffer->size;
	if (!secsg_heap->alloc_size) {
		__secsg_pool_release(secsg_heap);
		if (secsg_heap->water_mark &&
		    __secsg_fill_watermark(secsg_heap))
			pr_err("__secsg_fill_watermark failed!\n");
	}
}

static void secsg_free(struct ion_secsg_heap *secsg_heap,
			struct ion_buffer *buffer)
{
	secsg_debug("%s: enter, free size 0x%zx\n", __func__, buffer->size);
	if (secsg_heap->heap_attr == HEAP_PROTECT)
		__secsg_free_scatter(secsg_heap, buffer);
	else
		__secsg_free_contig(secsg_heap, buffer);
	secsg_debug("%s: exit\n", __func__);
}

static int secsg_map_iommu(struct ion_secsg_heap *secsg_heap,
			   struct ion_buffer *buffer,
			   struct ion_iommu_map *map_data)
{
	struct mem_chunk_list mcl;
	int ret;

	mcl.protect_id = SEC_TASK_DRM;
	mcl.buff_id = buffer->id;
	mcl.phys_addr = NULL;
	mcl.size = (u32)buffer->size;
	if (mcl.size != buffer->size) {
		pr_err("%s:size(0x%zx) too large\n", __func__, buffer->size);
		return -EINVAL;
	}

	mutex_lock(&secsg_heap->mutex);
	ret = secsg_tee_exec_cmd(secsg_heap, &mcl, ION_SEC_CMD_MAP_IOMMU);
	mutex_unlock(&secsg_heap->mutex);
	if (ret) {
		pr_err("%s:exec map iommu cmd fail\n", __func__);
		return -EINVAL;
	}

	map_data->format.iova_start = mcl.va;
	map_data->format.iova_size = buffer->size;

	return 0;
}

static void secsg_unmap_iommu(struct ion_secsg_heap *secsg_heap,
			      struct ion_buffer *buffer,
			      struct ion_iommu_map *map_data)
{
	struct mem_chunk_list mcl;
	int ret;

	mcl.protect_id = SEC_TASK_DRM;
	mcl.buff_id = buffer->id;
	mcl.phys_addr = NULL;

	mutex_lock(&secsg_heap->mutex);
	ret = secsg_tee_exec_cmd(secsg_heap, &mcl, ION_SEC_CMD_UNMAP_IOMMU);
	mutex_unlock(&secsg_heap->mutex);
	if (ret)
		pr_err("%s:exec unmap iommu cmd fail\n", __func__);
}

static int ion_secsg_heap_allocate(struct ion_heap *heap,
				   struct ion_buffer *buffer,
				   unsigned long size, unsigned long align,
				   unsigned long flags)
{
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/
	int ret = 0;

	secsg_debug("enter %s  size 0x%lx heap id %u\n",
		    __func__, size, heap->id);
	mutex_lock(&secsg_heap->mutex);

	if (__secsg_heap_input_check(secsg_heap, size, flags)){
		pr_err("input params failed\n");
		ret = -EINVAL;
		goto out;
	}

	/*init the TA conversion here*/
	if(!secsg_heap->TA_init &&
	   (flags & ION_FLAG_SECURE_BUFFER) &&
	   secsg_tee_init(secsg_heap)) {
		pr_err("[%s] TA init failed\n", __func__);
		ret = -1;
		goto out;
	}

	/* SMMU pgtable init */
	if (!secsg_heap->iova_init && secsg_heap->pgtable_phys) {
		ret = __secsg_pgtable_init(secsg_heap);
		if (ret)
			goto out;
		secsg_heap->iova_init = 1;
	}

	if (secsg_alloc(secsg_heap, buffer, size)) {/*lint !e838*/
		pr_err("[%s] secsg_alloc failed, size = 0x%lx.\n",
			__func__, secsg_heap->alloc_size);
		ret = -ENOMEM;
		goto out;
	}
	secsg_heap->alloc_size += size;
	secsg_debug("secsg heap alloc succ, heap all alloc_size 0x%lx\n",
		    secsg_heap->alloc_size);
	mutex_unlock(&secsg_heap->mutex);
	return 0;
out:
	secsg_debug("secsg heap alloc fail, heap all alloc_size 0x%lx\n",
		    secsg_heap->alloc_size);
	mutex_unlock(&secsg_heap->mutex);
	return ret;
}/*lint !e715*/

static void ion_secsg_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/

	secsg_debug("%s:enter, heap %d, free size:0x%zx,\n",
		    __func__, heap->id, buffer->size);
	mutex_lock(&secsg_heap->mutex);
	secsg_free(secsg_heap, buffer);
	secsg_debug("out %s:heap remaining allocate %lx\n",
		    __func__, secsg_heap->alloc_size);
	mutex_unlock(&secsg_heap->mutex);
}/*lint !e715*/

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
static int ion_secsg_heap_phys(struct ion_heap *heap,
				struct ion_buffer *buffer,
				ion_phys_addr_t *addr, size_t *len)
{
	/* keep the input parames for compatible with other heaps*/
	/* TZ driver can call "ion_phys" with ion_handle input*/
	struct sg_table *table = buffer->priv_virt;
	struct page *page = sg_page(table->sgl);
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/

	if(secsg_heap->heap_attr == HEAP_PROTECT) {
		*addr = buffer->id;
		*len = buffer->size;
	} else {
		ion_phys_addr_t paddr = PFN_PHYS(page_to_pfn(page));
		*addr = paddr;
		*len = buffer->size;
	}

	return 0;
}/*lint !e715*/
#endif

static int ion_secsg_heap_map_user(struct ion_heap *heap,
				   struct ion_buffer *buffer,
				   struct vm_area_struct *vma)
{
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/
	if ((secsg_heap->heap_attr == HEAP_SECURE) ||
	    (secsg_heap->heap_attr == HEAP_PROTECT)) {
		pr_err("secure buffer, can not call %s\n", __func__);
		return -EINVAL;
	}
	return ion_heap_map_user(heap, buffer, vma);
}

static void *ion_secsg_heap_map_kernel(struct ion_heap *heap,
					struct ion_buffer *buffer)
{
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/
	if ((secsg_heap->heap_attr == HEAP_SECURE) ||
	    (secsg_heap->heap_attr == HEAP_PROTECT)) {
		pr_err("secure buffer, can not call %s\n", __func__);
		return NULL;
	}
	return ion_heap_map_kernel(heap, buffer);
}

static void ion_secsg_heap_unmap_kernel(struct ion_heap *heap,
					struct ion_buffer *buffer)
{
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/
	if ((secsg_heap->heap_attr == HEAP_SECURE) ||
	    (secsg_heap->heap_attr == HEAP_PROTECT)) {
		pr_err("secure buffer, can not call %s\n", __func__);
		return;
	}
	ion_heap_unmap_kernel(heap, buffer);
}

static int ion_secsg_heap_map_iommu(struct ion_buffer *buffer,
				    struct ion_iommu_map *map_data)
{
	struct ion_heap *heap = buffer->heap;
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/
	int ret = 0;

	if(secsg_heap->heap_attr == HEAP_PROTECT) {
		ret = secsg_map_iommu(secsg_heap, buffer, map_data);
		if (ret)
			pr_err("%s:protect map iommu fail\n", __func__);
		return ret;
	} else if (secsg_heap->heap_attr == HEAP_SECURE) {
		pr_err("%s:sec or protect buffer can't map iommu\n", __func__);
		return -EINVAL;
	} else {
		return ion_heap_map_iommu(buffer, map_data);
	}
}

static void ion_secsg_heap_unmap_iommu(struct ion_iommu_map *map_data)
{
	struct ion_buffer *buffer = map_data->buffer;
	struct ion_heap *heap = buffer->heap;
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/

	if(secsg_heap->heap_attr == HEAP_PROTECT)
		secsg_unmap_iommu(secsg_heap, buffer, map_data);
	else if (secsg_heap->heap_attr == HEAP_SECURE)
		pr_err("[%s]secure buffer, do nothing.\n", __func__);
	else
		ion_heap_unmap_iommu(map_data);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
/*lint -save -e715 */
static struct sg_table *ion_secsg_heap_map_dma(struct ion_heap *heap,
						struct ion_buffer *buffer)
{
	return buffer->priv_virt;
}

static void ion_secsg_heap_unmap_dma(struct ion_heap *heap,
				     struct ion_buffer *buffer)
{
}
/*lint -restore*/
#endif

static struct ion_heap_ops secsg_heap_ops = {
	.allocate = ion_secsg_heap_allocate,
	.free = ion_secsg_heap_free,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	.phys = ion_secsg_heap_phys,
	.map_dma = ion_secsg_heap_map_dma,
	.unmap_dma = ion_secsg_heap_unmap_dma,
#endif
	.map_user = ion_secsg_heap_map_user,
	.map_kernel = ion_secsg_heap_map_kernel,
	.unmap_kernel = ion_secsg_heap_unmap_kernel,
	.map_iommu = ion_secsg_heap_map_iommu,
	.unmap_iommu = ion_secsg_heap_unmap_iommu,
};/*lint !e785*/

static int __secsg_parse_dt(struct device *dev,
			    struct ion_platform_heap *heap_data,
			    struct ion_secsg_heap *secsg_heap)
{
	struct device_node *nd;
	struct device_node *mem_region;
	struct resource res;
	u64 per_bit_sz = 0;
	u64 per_alloc_sz = 0;
	u64 water_mark = 0;
	u32 heap_attr = 0;
	u32 pool_shift = ION_PBL_SHIFT;
	int ret = 0;

	nd = of_get_child_by_name(dev->of_node, heap_data->name);
	if (!nd) {
		pr_err("can't of_get_child_by_name %s\n", heap_data->name);
		ret = -EINVAL;
		goto out;
	}

	mem_region = of_parse_phandle(nd, "memory-region", 0);
	if (mem_region) {
		ret = of_address_to_resource(mem_region, 0, &res);
		of_node_put(mem_region);
		if (ret) {
			pr_err("failed to parse memory-region: %d\n", ret);
			goto out;
		}
		secsg_heap->pgtable_phys = res.start;
		secsg_heap->pgtable_size = resource_size(&res);
	} else {
		secsg_heap->pgtable_phys = 0;
		secsg_heap->pgtable_size = 0;
		pr_err("no memory-region phandle\n");
	}

	ret = of_property_read_u64(nd, "per-alloc-size", &per_alloc_sz);
	if (ret < 0) {
		pr_err("can't find prop:per-alloc-size\n");
		goto out;
	}
	secsg_heap->per_alloc_sz = PAGE_ALIGN(per_alloc_sz);

	ret = of_property_read_u64(nd, "per-bit-size", &per_bit_sz);
	if (ret < 0) {
		pr_err("can't find prop:per-bit-size\n");
		goto out;
	}
	secsg_heap->per_bit_sz = PAGE_ALIGN(per_bit_sz);

	ret = of_property_read_u64(nd, "water-mark", &water_mark);
	if (ret < 0) {
		pr_err("can't find prop:water-mark\n");
		water_mark = 0;
	}
	secsg_heap->water_mark = PAGE_ALIGN(water_mark);

	ret = of_property_read_u32(nd, "pool-shift", &pool_shift);
	if (ret < 0) {
		pr_err("can not find pool-shift.\n");
		pool_shift = ION_PBL_SHIFT;
	}
	secsg_heap->pool_shift = pool_shift;

	ret = of_property_read_u32(nd, "heap-attr", &heap_attr);
	if (ret < 0) {
		pr_err("can not find heap-arrt.\n");
		heap_attr = HEAP_NORMAL;
	}
	if (heap_attr >= HEAP_MAX)
		heap_attr = HEAP_NORMAL;
	secsg_heap->heap_attr = heap_attr;

out:
	return ret;
}

struct ion_heap *ion_secsg_heap_create(struct ion_platform_heap *heap_data)
{
	int ret;
	struct device *dev;
	struct ion_secsg_heap *secsg_heap;

	secsg_heap = kzalloc(sizeof(*secsg_heap), GFP_KERNEL);
	if (!secsg_heap)
		return ERR_PTR(-ENOMEM);/*lint !e747*/

	mutex_init(&secsg_heap->mutex);

	secsg_heap->pool = NULL;
	secsg_heap->heap.ops = &secsg_heap_ops;
	secsg_heap->heap.type = ION_HEAP_TYPE_SECSG;
	secsg_heap->heap_size = heap_data->size;
	secsg_heap->alloc_size = 0;
	dev = heap_data->priv;
	INIT_LIST_HEAD(&secsg_heap->allocate_head);

	if (!hisi_secsg_cma) {
		pr_err("hisi_secsg_cma failed.\n");
		goto free_heap;
	}
	secsg_heap->cma = hisi_secsg_cma;

	ret = __secsg_parse_dt(dev, heap_data, secsg_heap);
	if (ret)
		goto free_heap;

	if ((secsg_heap->heap_attr == HEAP_SECURE) ||
	    (secsg_heap->heap_attr == HEAP_PROTECT)) {
		secsg_heap->context = kzalloc(sizeof(TEEC_Context), GFP_KERNEL);
		if (!secsg_heap->context)
			goto free_heap;
		secsg_heap->session = kzalloc(sizeof(TEEC_Session), GFP_KERNEL);
		if (!secsg_heap->session)
			goto free_context;
	} else {
		secsg_heap->context = NULL;
		secsg_heap->session = NULL;
	}
	secsg_heap->TA_init = 0;
	secsg_heap->iova_init = 0;

	ret = __secsg_create_pool(secsg_heap);
	if (ret) {
		pr_err("[%s] pool create failed.\n", __func__);
		goto free_session;
	}

	if (secsg_heap->water_mark &&
	    __secsg_fill_watermark(secsg_heap))
		pr_err("__secsg_fill_watermark failed!\n");

	pr_err("secsg heap info %s:\n"
		  "\t\t\t\t heap attr : %u\n"
		  "\t\t\t\t pool shift : %u\n"
		  "\t\t\t\t heap size : %lu MB\n"
		  "\t\t\t\t per alloc size :  %llu MB\n"
		  "\t\t\t\t per bit size : %llu KB\n"
		  "\t\t\t\t water_mark size : %llu MB\n"
		  "\t\t\t\t cma base : 0x%llx\n"
		  "\t\t\t\t cma size : 0x%lx\n",
		  heap_data->name,
		  secsg_heap->heap_attr,
		  secsg_heap->pool_shift,
		  secsg_heap->heap_size / SZ_1M,
		  secsg_heap->per_alloc_sz / SZ_1M,
		  secsg_heap->per_bit_sz / SZ_1K,
		  secsg_heap->water_mark / SZ_1M,
		  cma_get_base(secsg_heap->cma),
		  cma_get_size(secsg_heap->cma));

	return &secsg_heap->heap;/*lint !e429*/
free_session:
	if (secsg_heap->session)
		kfree(secsg_heap->session);
free_context:
	if (secsg_heap->context)
		kfree(secsg_heap->context);
free_heap:
	kfree(secsg_heap);
	return ERR_PTR(-ENOMEM);/*lint !e747*/
}

void ion_secsg_heap_destroy(struct ion_heap *heap)
{
	struct ion_secsg_heap *secsg_heap =
		container_of(heap, struct ion_secsg_heap, heap);/*lint !e826*/

	secsg_tee_destory(secsg_heap);

	if (secsg_heap->context)
		kfree(secsg_heap->context);
	if (secsg_heap->session)
		kfree(secsg_heap->session);
	kfree(secsg_heap);
}
