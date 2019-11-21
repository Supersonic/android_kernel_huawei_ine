/*
* mntn_l3cache_ecc.h
*/

#ifndef _MNTN_L3CACHE_ECC_H
#define _MNTN_L3CACHE_ECC_H
#ifdef CONFIG_HISI_L3CACHE_ECC
extern void l3cache_ecc_get_status(void);
#else
extern inline void l3cache_ecc_get_status(void) {};
#endif
#endif /* _MNTN_H_ */
