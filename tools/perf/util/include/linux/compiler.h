#ifndef _PERF_LINUX_COMPILER_H_
#define _PERF_LINUX_COMPILER_H_

#ifndef __always_inline
#define __always_inline	inline
#endif
#undef __user
#define __user
#ifndef __attribute_const__
#define __attribute_const__
#endif

#undef __used
#define __used		__attribute__((__unused__))

#endif
