#ifndef __ASM_GENERIC_PLATFORM_DEVICE_H_
#define __ASM_GENERIC_PLATFORM_DEVICE_H_

/*
 * an architecture can override to define arch_setup_pdev_archdata
 */
#ifndef arch_setup_pdev_archdata
static inline void arch_setup_pdev_archdata(struct platform_device *pdev) { }
#endif

#endif /* __ASM_GENERIC_PLATFORM_DEVICE_H_ */
