#ifndef __BSP_MODULE_H
#define __BSP_MODULE_H

#if defined(__OS_RTOSCK_SMP__)|| defined(__OS_RTOSCK__) || defined(__OS_RTOSCK_TSP__)

#ifdef __OS_NRCCPU__
#include <bsp_module_nrccpu.h>
#elif defined(__OS_LRCCPU__) || defined(__OS_RTOSCK_TSP__)
#include <bsp_module_lrccpu.h>
#elif defined(__OS_HAC__)

#else
#error another cpu should use a_new bsp_module
#endif


#endif

#endif
