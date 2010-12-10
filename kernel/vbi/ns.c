/*
 * ns.c - hypervisor naming service, client side interface
 *
 * Copyright (c) 2008 Wind River Systems, Inc.
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
DESCRIPTION

This module implements a the client side of a simple naming service for the
internal thread managers of the hypervisor.

The interfaces formulate a message request and send it to the naming service
for processing.

*/

#include <linux/string.h>
#include <linux/kernel.h>
#include <vbi/vbi.h>
#include <vbi/private.h>

#ifdef DEBUG
#define DEBUGM(fmt, args...)    printk(fmt, ##args)
#else
#define DEBUGM(fmt, args...)
#endif

/*
 * vbi_ns_register - register a service with the naming system
 *
 * This routine registers us as the provider of the specified service.
 * A message for the request is formulated and sent off to the name service
 * manager for processing.
 *
 */
int32_t vbi_ns_register(char *name, uint32_t revision)
{
	/* Certifiable hypervisor does not support this function */
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_ns_register);
		return -1;
	}

	if (name == NULL)
		return -1;

	return vbi_ns_op(VBI_NS_REGISTER, name, revision, NULL,
			VBI_NS_NO_TIMEOUT, VBI_NS_OPTION_NONE);
}

/*
 * vbi_ns_unregister - un-register a service with the naming system
 *
 * This routine removes us as the provider of the specified service.
 * A message for the request is formulated and sent off to the name service
 * manager for processing.
 *
 */
int32_t vbi_ns_unregister(char *name, uint32_t revision)
{
	/* Certifiable hypervisor does not support this function */
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_ns_unregister);
		return -1;
	}

	if (name == NULL)
		return -1;

	return vbi_ns_op(VBI_NS_UNREGISTER, name, revision, NULL,
			VBI_NS_NO_TIMEOUT, VBI_NS_OPTION_NONE);
}

/*
 * vbi_ns_lookup_old - look up a service provider using the naming system
 *
 * This routine uses the naming system to look up the context id of the
 * provider of the specified service.  A service is denoted by both an ASCII
 * string name and a numerical version number.  When performing a search for
 * a matching service, both the name and the version number must match.
 *
 * If a specific version of a service is not required, the version can
 * be specified as SYS_NS_ANYVERSION.  In this case the highest version
 * of the service name will be returned.
 *
 * NOTE: this interface is the older interface which does not allow specifying
 * a timeout value or options.  It is only provided for backwards compatibility
 * purposes, vbiNsLookup() should be used instead.
 *
 */
int32_t vbi_ns_lookup_old(char *name, uint32_t revision, VBI_NS_HANDLE *handle)
{
	/* Optional VBI, Certifiable hypervisor does not
		support this function */
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_ns_lookup);
		return -1;
	}

	if (name == NULL || handle == NULL)
		return -1;

	return vbi_ns_op(VBI_NS_LOOKUP_OLD, name, revision, handle,
			VBI_NS_NO_TIMEOUT, VBI_NS_OPTION_NONE);
}

/*
 * vbi_ns_lookup - look up a service provider using the naming system
 *
 * This routine uses the naming system to look up the context id of the
 * provider of the specified service.  A service is denoted by both an ASCII
 * string name and a numerical version number.  When performing a search for
 * a matching service, both the name and the version number must match.
 *
 * If a specific version of a service is not required, the version can
 * be specified as SYS_NS_ANYVERSION.  In this case the highest version
 * of the service name will be returned.
 *
 * The following options may be specified:
 *
 * SYS_NS_OPTION_NONE - no additional options
 * SYS_NS_OPTION_LOOKUP_WAIT - wait for the service to be registered
 *
 * The timeout value can be used to limit the amount of time to wait when
 * the SYS_NS_OPTION_LOOKUP_WAIT option is specified.
 *
 */
int32_t vbi_ns_lookup(char *name, uint32_t revision, VBI_NS_HANDLE *handle,
			uint32_t timeout, uint32_t options)
{
	/* Optional VBI, Certifiable hypervisor does not
		support this function */
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_ns_lookup);
		return -1;
	}

	if (name == NULL || handle == NULL)
		return -1;

	return vbi_ns_op(VBI_NS_LOOKUP , name, revision, handle, timeout,
			options);
}
