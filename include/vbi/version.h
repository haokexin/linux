/*
 * version.h - vbi version information
 *
 * Copyright (c) 2009-2011 Wind River Systems, Inc.
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

#ifndef _VBI_VERSION_H
#define _VBI_VERSION_H

/*
 * Release identification, major, minor, and maintenance numbers.
 *
 * Current vbi release number
 */

#define VBI_RUNTIME_NAME     "Wind River Hypervisor Virtual board interface"
#define VBI_RUNTIME_VERSION  "2.2.1"

/* textual description of this product version */

#define VBI_VERSION VBI_RUNTIME_NAME " " VBI_RUNTIME_VERSION

#ifndef _ASMLANGUAGE
extern const char *vbi_creation_date;
extern const char *vbi_runtime_version;
extern const char *vbi_runtime_name;
extern const char *vbi_version;

extern const uint32_t vbi_version_major;
extern const uint32_t vbi_version_minor;
extern const uint32_t vbi_version_maint;
#endif	/* __ASMLANGUAGE */

#endif	/* _VBI_VERSION_H */
