#ifndef PV_HW_IRQ_H
#define PV_HW_IRQ_H

/* set default definiation to native implemenation */
#define raw_local_irq_restore(flags)   native_raw_local_irq_restore(flags)
#define raw_local_irq_disable()        native_raw_local_irq_disable()
#define raw_local_irq_enable()         native_raw_local_irq_enable()
#define raw_local_save_flags(flags)    native_raw_local_save_flags(flags)
#define raw_local_irq_save(flags)      native_raw_local_irq_save(flags)
#define raw_irqs_disabled()            native_raw_irqs_disabled()
#define raw_irqs_disabled_flags(flags) native_raw_irqs_disabled_flags(flags)

#define hard_irq_disable()             native_hard_irq_disable()

/* Hypervisor specific irq implementation */
#ifdef CONFIG_WRHV
#include <vbi/interface.h>
extern void wrhv_int_lock(void);
extern void wrhv_int_unlock(int lvl);
extern int wrhv_int_lvl_get (void);

#ifdef CONFIG_WRHV_E500
#define IRQS_ENABLED 0
#define IRQS_DISABLED -1
#elif defined(CONFIG_WRHV_P4080DS)
#define IRQS_ENABLED 1
#define IRQS_DISABLED 0
#endif

/* undefine native implementation */
#undef raw_local_irq_restore
#undef raw_local_irq_disable
#undef raw_local_irq_enable
#undef raw_local_save_flags
#undef raw_local_irq_save
#undef raw_irqs_disabled
#undef raw_irqs_disabled_flags
#undef hard_irq_disable
#undef irqs_disabled_flags

/* WRHV specific static inline implementation */
static inline void pv_local_irq_disable(void)
{
        wrhv_int_lock();
}

static inline void pv_local_irq_enable(void)
{
        wrhv_int_unlock(0);
}

static inline void pv_local_irq_save_ptr(unsigned long *flags)
{
        *flags = wrhv_int_lvl_get();
        wrhv_int_lock();
}

#define raw_local_irq_disable()        pv_local_irq_disable()
#define raw_local_irq_enable()         pv_local_irq_enable()

#define raw_local_save_flags(flags)    ((flags) = wrhv_int_lvl_get())
#define raw_local_irq_restore(flags)   (flags == IRQS_ENABLED  \
			 ? pv_local_irq_enable() : pv_local_irq_disable ())
#define raw_irqs_disabled_flags(flags) (flags == IRQS_DISABLED)
#define raw_irqs_disabled()		(wrhv_int_lvl_get() == IRQS_DISABLED)
#ifdef CONFIG_WRHV_P4080DS
#define raw_local_irq_save(flags)      do {    \
		flags = wrhv_int_lvl_get();     \
		pv_local_irq_disable();         \
	} while (IRQS_DISABLED)

#elif defined(CONFIG_WRHV_E500)
#define raw_local_irq_save(flags)      pv_local_irq_save_ptr(&flags)
#endif

#define hard_irq_disable()             raw_local_irq_disable()

#endif /* CONFIG_WRHV */
#endif /* PV_HW_IRQ_H */

