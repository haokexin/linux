/*
 * version.c - creation version/date/time module
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

/*
This module is always built with each executable image.  It provides
the VBI version id, and the time and date it was built.

The date stamp may be overriden by defining VBI_RUNTIME_CREATION_DATE. This
will be primarily used by guest OS's that use VBI.

The ANSI predefined macros __DATE__ and __TIME__ are used to provide
the date/time information.  ANSI compliant compilers are required for
building all hypervisor executables.
*/

#include <linux/types.h>
#include <vbi/vbi.h>
#include <vbi/version.h>

/* numerical values for vbi version */
const uint32_t vbi_version_major = VBI_VERSION_MAJOR;
const uint32_t vbi_version_minor = VBI_VERSION_MINOR;
const uint32_t vbi_version_maint = VBI_VERSION_MAINT;

/* string identifiers for vbi version */
const char *vbi_runtime_name = VBI_RUNTIME_NAME;
const char *vbi_runtime_version = VBI_RUNTIME_VERSION;
const char *vbi_version = VBI_VERSION;

#ifdef VBI_RUNTIME_CREATION_DATE
const char *vbi_creation_date = VBI_RUNTIME_CREATION_DATE;
#else
const char *vbi_creation_date = __DATE__ ", " __TIME__;
#endif
