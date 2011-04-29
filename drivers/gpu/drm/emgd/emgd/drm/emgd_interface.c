/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: emgd_interface.c
 * $Revision: 1.168 $
 *-----------------------------------------------------------------------------
 * Copyright (c) 2002-2010, Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *-----------------------------------------------------------------------------
 * Description:
 *  This file implements the kernel-space bridge between the IAL and the HAL.
 *  Each of the igd_dispatch_t functions are called by an ioctl's handling
 *  function, that's implemented in this file.  All parameters coming from user
 *  space are copied and sent to the HAL.  All return parameters and the
 *  overall return value is copied back to the user-space bridge code.
 *  See the description in the file "emgd_hal2drm.c" for more details.
 *-----------------------------------------------------------------------------
 */

#define MODULE_NAME hal.oal

#include "drmP.h"
#include "drm.h"

#include "drm_emgd_private.h"
#include "emgd_drm.h"
#include "emgd_drv.h"
#include "memory.h"

#include "module_init.h"
#include "mode_dispatch.h"
#include "ovl_dispatch.h"
#include "ovl_virt.h"
#include "msvdx.h"
#include "topaz.h"

#include "memmap.h"
#include "sched.h"

#include "services.h"
#include "perproc.h"
#include "pvr_bridge_km.h"
#include "syscommon.h"
#include "pvr_drm.h"

/* Turn on tracing for this file only */
/*
#undef EMGD_TRACE_ENTER
#define EMGD_TRACE_ENTER printk( KERN_ERR "%s Entry. ",__FUNCTION__)
*/

/* The compile-time configuration found in "user_config.c": */
extern emgd_drm_config_t config_drm;
/* Module parameters from "emgd_drv.c": */
extern int drm_emgd_init;
extern int drm_emgd_dc;
extern unsigned x_started;

/**
 * The driver handle for talking with the HAL, within the DRM/kernel code.
 * This is a "real handle" as opposed to the "fake handle" in user-space.
 * Notice that there's only one handle, as the secondary device shares this
 * handle, and so it is cached here to simplify the ioctl-handling procedures.
 */
static igd_driver_h handle = NULL;
/** This is the dispatch table for the HAL.  It is cached for quick access. */
static igd_dispatch_t *dispatch = NULL;


/*!
 * This is called by emgd_driver_load() once it has a "real" driver handle, to
 * allow the values to be cached in this file.
 *
 * @param drm_handle (IN) the kernel-space HAL's "real" driver handle
 */
void emgd_set_real_handle(igd_driver_h drm_handle)
{
	handle = drm_handle;
} /* emgd_set_real_handle() */

/*!
 * This is called by emgd_driver_load() once it has a "real" dispatch table, to
 * allow the values to be cached in this file.
 *
 * @param drm_dispatch (OUT) the kernel-space HAL's dispatch table
 */
void emgd_set_real_dispatch(igd_dispatch_t *drm_dispatch)
{
	dispatch = drm_dispatch;
} /* emgd_set_real_dispatch() */


/*
 * Externally-accessible global variables and functions, for PreInit()-time
 * configuration:
 */
extern mode_context_t mode_context[1];
extern void dsp_shutdown(igd_context_t *context);
extern int pi_pd_init(igd_display_port_t *port, unsigned long port_feature,
	unsigned long second_port_feature, int drm_load_time);
#ifdef DEBUG_BUILD_TYPE
extern void emgd_print_params(igd_param_t *params);
#endif



/*
 * NOTE: The rest of this file contains implementations of the HAL-to-DRM
 * ioctl-handling procedures.
 *
 * The naming convention is:  emgd_<HAL-procedure-pointer-name>()
 */


/*!
 * IOCTL to get chipset information from DRM.
 */
int emgd_get_chipset_info(struct drm_device *dev, void *arg,
		struct drm_file *file_priv)
{
	emgd_drm_driver_get_chipset_info_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;

	drm_data->device_id = priv->init_info->device_id;
	drm_data->revision_id = priv->init_info->vendor_id;

	/*
	 * Copy over the name and description fields.  Currently these
	 * are hard coded to a maximum of 40 and 20 characters respectively.
	 */
	if (strlen(priv->init_info->name) > 39) {
		strncpy(drm_data->name, priv->init_info->name, 39);
		drm_data->name[39] = '\0';
	} else {
		strcpy(drm_data->name, priv->init_info->name);
	}

	if (strlen(priv->init_info->chipset) > 19) {
		strncpy(drm_data->description, priv->init_info->chipset, 19);
		drm_data->description[19] = '\0';
	} else {
		strcpy(drm_data->description, priv->init_info->chipset);
	}

	return 0;
}


/*!
 * IOCTL to bridge the IAL to the HAL's alter_cursor() procedure.
 */
int emgd_alter_cursor(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_alter_cursor_t *drm_data = arg;

	/*EMGD_TRACE_ENTER;*/


	/* Call the HAL: */
	drm_data->rtn = dispatch->alter_cursor(drm_data->display_handle,
		&(drm_data->cursor_info),
		drm_data->image);


	/*EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);*/
	/*EMGD_DEBUG("Returning 0");*/
	/*EMGD_TRACE_EXIT;*/
	return 0;
} /* emgd_alter_cursor() */


/*!
 * IOCTL to bridge the IAL to the HAL's alter_cursor_pos() procedure.
 */
int emgd_alter_cursor_pos(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_alter_cursor_pos_t *drm_data = arg;

	/*EMGD_TRACE_ENTER;*/


	/* Call the HAL: */
	drm_data->rtn = dispatch->alter_cursor_pos(drm_data->display_handle,
		&(drm_data->cursor_info));


	/*EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);*/
	/*EMGD_DEBUG("Returning 0");*/
	/*EMGD_TRACE_EXIT;*/
	return 0;
} /* emgd_alter_cursor_pos() */


/*!
 * IOCTL to bridge the IAL to the HAL's alter_displays() procedure.
 */
int emgd_alter_displays(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_alter_displays_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->alter_displays(handle,
		/* Note: Since a pointer is passed to drm_data->primary/secondary,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->primary),
		&(drm_data->primary_pt_info),
		&(drm_data->primary_fb_info),
		&(drm_data->secondary),
		&(drm_data->secondary_pt_info),
		&(drm_data->secondary_fb_info),
		drm_data->dc,
		drm_data->flags);

	if (!drm_data->rtn) {
		/* Communicate the new info to the IMG 3rd-party display driver: */
		priv->dc = drm_data->dc;
		priv->primary = drm_data->primary;
		priv->secondary = drm_data->secondary;
		priv->primary_port_number = (drm_data->dc & 0xf0) >> 4;
		priv->secondary_port_number = (drm_data->dc & 0xf00000) >> 20;

		if (priv->must_power_on_ports) {
			/* The HAL was re-initialized during emgd_driver_pre_init()
			 * (i.e. because the X server was restarted).  At that time, it
			 * turned off the port drivers' hardware (so that it could poke new
			 * values into the port drivers).  It couldn't turn the hardware
			 * back on then.  Do so now.
			 */
			igd_context_t *context = (igd_context_t *) handle;
			igd_display_port_t *port = NULL;

			while ((port = context->mod_dispatch.dsp_get_next_port(context,
						port, 0)) != NULL) {
				/* only LVDS was turned off, so turn it back on */
				if (port->pd_driver &&  (port->port_number == IGD_PORT_TYPE_LVDS)) {
					EMGD_DEBUG("Turning on power for port %lu\n", port->port_number);

					dispatch->power_display(context, port->port_number,
						IGD_POWERSTATE_D0);
				}
			}
			priv->must_power_on_ports = 0;
		}

		if (priv->pvrsrv_started && priv->reinit_3dd) {
			priv->reinit_3dd(dev);
		}
	}

	EMGD_DEBUG("drm_data->primary = 0x%lx", (unsigned long) drm_data->primary);
	EMGD_DEBUG("drm_data->secondary = 0x%lx",(unsigned long) drm_data->secondary);
	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_alter_displays() */


/*!
 * IOCTL to bridge the IAL to the HAL's alter_ovl() procedure.
 */
int emgd_alter_ovl(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_alter_ovl_t *drm_data = arg;

	EMGD_TRACE_ENTER;
	EMGD_DEBUG("Deprecated version of alter_ovl. FIX ME.");

	/* Call the HAL: */
	drm_data->rtn = dispatch->alter_ovl2(drm_data->display_handle,
		(igd_surface_t *) &(drm_data->src_surf),
		(igd_rect_t *) &(drm_data->src_rect),
		(igd_rect_t *) &(drm_data->dst_rect),
		(igd_ovl_info_t *) &(drm_data->ovl_info),
		drm_data->flags);

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_alter_ovl() */


/*!
 * IOCTL to bridge the IAL to the HAL's alter_ovl() procedure.
 */
int emgd_alter_ovl2(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_alter_ovl2_t *drm_data = arg;

	EMGD_TRACE_ENTER;
	/* Dump the overlay parameters for debugging */
	/*
	printk (KERN_ERR "emgd_alter_ovl2 Entry."
		"offset=0x%X "
		"pitch=0x%X "
		"width=0x%X "
		"height=0x%X "
		"pixel_format=0x%X "
		"flags=0x%X "
		"virt_addr=0x%X "
		"pvr2d_mem_info=0x%X "
		"pvr2d_context_h=0x%X "
		"hPVR2DFlipChain=0x%X "
		"src_x1=0x%X "
		"src_x2=0x%X "
		"src_y1=0x%X "
		"src_y2=0x%X "
		"dest_x1=0x%X "
		"dest_x2=0x%X "
		"dest_y1=0x%X "
		"dest_y2=0x%X "
		"color_key.src_lo=0x%X "
		"color_key.src_hi=0x%X "
		"color_key.dest=0x%X "
		"color_key.flags=0x%X "
		"flags=0x%X "
		,
		(unsigned int)drm_data->src_surf.offset ,
		(unsigned int)drm_data->src_surf.pitch ,
		(unsigned int)drm_data->src_surf.width ,
		(unsigned int)drm_data->src_surf.height ,
		(unsigned int)drm_data->src_surf.pixel_format ,
		(unsigned int)drm_data->src_surf.flags ,
		(unsigned int)drm_data->src_surf.virt_addr ,
		(unsigned int)drm_data->src_surf.pvr2d_mem_info ,
		(unsigned int)drm_data->src_surf.pvr2d_context_h ,
		(unsigned int)drm_data->src_surf.hPVR2DFlipChain ,
		(unsigned int)drm_data->src_rect.x1,
		(unsigned int)drm_data->src_rect.x2,
		(unsigned int)drm_data->src_rect.y1,
		(unsigned int)drm_data->src_rect.y2,
		(unsigned int)drm_data->dst_rect.x1,
		(unsigned int)drm_data->dst_rect.x2,
		(unsigned int)drm_data->dst_rect.y1,
		(unsigned int)drm_data->dst_rect.y2,
		(unsigned int)drm_data->ovl_info.color_key.src_lo,
		(unsigned int)drm_data->ovl_info.color_key.src_hi,
		(unsigned int)drm_data->ovl_info.color_key.dest,
		(unsigned int)drm_data->ovl_info.color_key.flags,
		(unsigned int)drm_data->flags
		);
	*/
	drm_data->rtn = dispatch->alter_ovl2(drm_data->display_handle,
                                         &(drm_data->src_surf),
                                         &(drm_data->src_rect),
                                         &(drm_data->dst_rect),
                                         &(drm_data->ovl_info),
                                         drm_data->flags);

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_TRACE_EXIT;

	return 0;
} /* emgd_alter_ovl2() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_ovl_init_params() procedure.
 */
int emgd_get_ovl_init_params(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_ovl_init_params_t *drm_data = arg;
	ovl_um_context_t *ovl_um_context;

	EMGD_TRACE_ENTER;

	/* Call the HAL: */
	/* there may not be a need to call the hal for this, since we have
	 * access to the overlay context.  We can probably just copy what we need
	 * from the context to the drm_data
	 */

	if (!drm_data) {
		EMGD_DEBUG("Invalid drm_data in emgd_get_ovl_init_params.\n");
		return 0;
	}

	ovl_um_context = drm_data->ovl_um_context;

	if (!ovl_um_context) {
		drm_data->rtn = -1;
		return 0;
	}

	drm_data->rtn = dispatch->get_ovl_init_params(handle, ovl_um_context);

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_ovl_init_params() */


/*!
 * IOCTL to bridge the IAL to the HAL's appcontext_alloc() procedure.
 */
int emgd_appcontext_alloc(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_appcontext_alloc_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	/* NOTE: the return value is different than normal: */
	drm_data->appcontext_h =
		dispatch->appcontext_alloc(drm_data->display_handle,
			drm_data->priority,
			drm_data->flags);


	EMGD_DEBUG("drm_data->appcontext_h = %p", drm_data->appcontext_h);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_appcontext_alloc() */


/*!
 * IOCTL to bridge the IAL to the HAL's appcontext_free() procedure.
 */
int emgd_appcontext_free(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_appcontext_free_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	/* NOTE: no return value is desired: */
	dispatch->appcontext_free(drm_data->display_handle,
		drm_data->priority,
		drm_data->appcontext_h);


	EMGD_DEBUG("This function has no drm_data->rtn value");
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_appcontext_free() */


/*!
 * IOCTL to bridge the IAL to the HAL's driver_save_restore() procedure.
 */
int emgd_driver_save_restore(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	drm_emgd_private *priv = dev->dev_private;
	emgd_drm_driver_save_restore_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->driver_save_restore(handle);

	/* Change the state of what's saved: */
	if (priv->saved_registers == CONSOLE_STATE_SAVED) {
		priv->saved_registers = X_SERVER_STATE_SAVED;
		EMGD_DEBUG("State of saved registers is X_SERVER_STATE_SAVED");
	} else {
		priv->saved_registers = CONSOLE_STATE_SAVED;
		EMGD_DEBUG("State of saved registers is CONSOLE_STATE_SAVED");
	}


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_driver_save_restore() */


/*!
 * IOCTL to bridge the IAL to the HAL's enable_port() procedure.
 */
int emgd_enable_port(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_enable_port_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->enable_port(drm_data->display_handle,
		drm_data->port_number,
		drm_data->flag,
		drm_data->test);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_enable_port() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_attrs() procedure.
 */
int emgd_get_attrs(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_attrs_t *drm_data = arg;
	igd_attr_t *attr_list = NULL;
	igd_attr_t *ext_list = NULL;
	igd_extension_attr_t *tmp = NULL;
	int i = 0;
	int extended = 0;
	int core_cnt = 0;
	int ext_cnt = 0;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->get_attrs(handle,
		drm_data->port_number,
		&(drm_data->list_size),
		/* Note: get_attrs() returns a pointer to live data, that the caller
		 * (i.e. this procedure) is not supposed to free/alter.  Therefore,
		 * this data must be copied into the space pointed to by
		 * drm_data->attr_list (assuming enough memory was allocated for it--if
		 * not, the hal2drm code will allocate drm_data->list_size and call
		 * this ioctl again).
		 */
		&(attr_list));

	/*
	 * The attribute list may have an extension list in addition to
	 * the core attributes.  The caller may want the core list or
	 * the extenstion list. Use the extended flag in drm_data to
	 * determine this.  If the caller is looking for the count, then
	 * return either core or extended size.
	 */

	while ((i < drm_data->list_size) && (attr_list[i].id != PD_ATTR_LIST_END)){
		if (attr_list[i].id == PD_ATTR_ID_EXTENSION) {
			extended = 1;
			tmp = (igd_extension_attr_t *)&attr_list[i];
			ext_list = tmp->extension;

			/* Count extension attributes */
			while ((ext_cnt < drm_data->list_size) &&
					(ext_list[ext_cnt].id != PD_ATTR_LIST_END)){
				ext_cnt++;
			}
		}
		core_cnt++;
		i++;
	}

	if (drm_data->extended) {
		drm_data->list_size = ext_cnt; /* size of extension list */

		/* Copy the attribute array back to user-space, if enough room: */
		if ((drm_data->allocated_size > 0) &&
			(drm_data->list_size <= drm_data->allocated_size)) {
			OS_MEMCPY(drm_data->attr_list, ext_list,
				(drm_data->list_size * sizeof(igd_attr_t)));
		}
	} else {
		drm_data->list_size = core_cnt;

		/* Copy the attribute array back to user-space, if enough room: */
		if ((drm_data->allocated_size > 0) &&
			(drm_data->list_size <= drm_data->allocated_size)) {
			OS_MEMCPY(drm_data->attr_list, attr_list,
				(drm_data->list_size * sizeof(igd_attr_t)));
		}
	}


	drm_data->extended = extended;

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_attrs() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_display() procedure.
 */
int emgd_get_display(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	igd_context_t *context = (igd_context_t *) handle;
	emgd_drm_get_display_t *drm_data = arg;
	igd_display_context_t *display =
		(igd_display_context_t *) drm_data->display_handle;
	drm_emgd_private *priv = dev->dev_private;

	EMGD_TRACE_ENTER;

	/* Can we overload this to get the display handle if it's null? */
	if (display == NULL) {
		display = context->mod_dispatch.dsp_display_list[drm_data->port_number];
		drm_data->display_handle = display;
	}

	/* To prevent a kernel OOPS, ensure the following value is non-NULL: */
	if ((display == NULL) || (PORT(display, drm_data->port_number) == NULL)) {
		EMGD_ERROR_EXIT("emgd_get_display() given invalid display "
			"handle (0x%x) or port number (%d)", (unsigned int) display,
			drm_data->port_number);
		return -IGD_ERROR_INVAL;
	}


	/* Call the HAL: */
	drm_data->rtn = dispatch->get_display(drm_data->display_handle,
		drm_data->port_number,
		/* Note: Since a pointer is passed to drm_data->fb_info/pt_info,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->fb_info),
		&(drm_data->pt_info),
		drm_data->flags);

	/* In seamless mode this gets called instead of alter_displays */
	if(mode_context->seamless) {
		if (priv->pvrsrv_started && priv->reinit_3dd) {
			priv->dc = *(context->mod_dispatch.dsp_current_dc);
			priv->primary = drm_data->display_handle;
			priv->secondary = NULL;
			priv->primary_port_number = (priv->dc & 0xf0) >> 4;
			priv->secondary_port_number = (priv->dc & 0xf00000) >> 20;
			priv->reinit_3dd(dev);
		}
	}


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_display() */


/*!
 * IOCTL to support emgd_hal2drm_drm_config(), which returns to the X driver
 * the igd_param_t that was used at load time, as well as the "config ID" that
 * was either specified by the "configid" module parameter of stored in the
 * VBOIS (by calling igd_get_param(...,IGD_PARAM_PANEL_ID,...)).
 */
int emgd_get_drm_config(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_drm_config_t *drm_data = arg;
	igd_param_t *params;
	int i;


	EMGD_TRACE_ENTER;


	/*
	 * Return the igd_param_t parameter values used at module load time:
	 */
	if (drm_emgd_configid < 0) {
		params = config_drm.hal_params[0];
	} else {
		params = config_drm.hal_params[drm_emgd_configid-1];
	}
	if (drm_emgd_init) {
		drm_data->display_config = drm_emgd_dc;
	} else {
		drm_data->display_config = config_drm.dc;
	}
	drm_data->params.page_request = params->page_request;
	drm_data->params.max_fb_size = params->max_fb_size;
	drm_data->params.preserve_regs = params->preserve_regs;
	drm_data->params.display_flags = params->display_flags;
	drm_data->params.display_color = params->display_color;
	drm_data->params.quickboot = params->quickboot;
	drm_data->params.qb_seamless = params->qb_seamless;
	drm_data->params.qb_video_input = params->qb_video_input;
	drm_data->params.qb_splash = params->qb_splash;
	drm_data->params.polling = params->polling;
	drm_data->params.ref_freq = params->ref_freq;
	drm_data->params.tuning_wa = params->tuning_wa;

	for (i = 0 ; i < IGD_MAX_PORTS ; i++) {
		drm_data->params.port_order[i] = params->port_order[i];
		drm_data->params.display_params[i].port_number =
			params->display_params[i].port_number;
		drm_data->params.display_params[i].present_params =
			(params->display_params[i].present_params &
				~(IGD_PARAM_DTD_LIST | IGD_PARAM_ATTR_LIST));
		drm_data->params.display_params[i].flags =
			params->display_params[i].flags;
		drm_data->params.display_params[i].edid_avail =
			params->display_params[i].edid_avail;
		drm_data->params.display_params[i].edid_not_avail =
			params->display_params[i].edid_not_avail;
		drm_data->params.display_params[i].ddc_gpio =
			params->display_params[i].ddc_gpio;
		drm_data->params.display_params[i].ddc_speed =
			params->display_params[i].ddc_speed;
		drm_data->params.display_params[i].ddc_dab =
			params->display_params[i].ddc_dab;
		drm_data->params.display_params[i].i2c_gpio =
			params->display_params[i].i2c_gpio;
		drm_data->params.display_params[i].i2c_speed =
			params->display_params[i].i2c_speed;
		drm_data->params.display_params[i].i2c_dab =
			params->display_params[i].i2c_dab;
		drm_data->params.display_params[i].fp_info =
			params->display_params[i].fp_info;
		/* No DTDs nor attrs are sent up.  If "xorg.conf" lists none, the
		 * defaults (in "user_config.c") are still in force.  If anything
		 * is listed in "xorg.conf", it overrides the defaults.  Thus,
		 * there's no need to send them up.
		 */
	}

	/*
	 * Return the "ConfigID" to return (either the module parameter or the
	 * value stored in VBIOS:
	 */
	if (drm_emgd_configid > 0) {
		drm_data->config_id = drm_emgd_configid;
	} else {
		/* Call the HAL: */
		int err = igd_get_param(handle, IGD_PARAM_PANEL_ID,
			/* Note: Since a pointer is passed to drm_data->value,
			 * there's no need to copy anything back into drm_data, except
			 * for the return value.
			 */
			&drm_data->config_id);
		if (err) {
			/* Either the vBIOS doesn't exist, or there was a problem reading
			 * it.  Either way, don't error; just set the config_id to an
			 * invalid value:
			 */
			drm_data->config_id = -1;
		}
	}


	drm_data->rtn = 0;
	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_drm_config() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_EDID_block() procedure.
 */
int emgd_get_EDID_block(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_EDID_block_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->get_EDID_block(handle,
		drm_data->port_number,
		/* Note: Since a pointer is passed to drm_data->edid_block,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		drm_data->edid_block,
		drm_data->block_number);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_EDID_block() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_EDID_info() procedure.
 */
int emgd_get_EDID_info(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_EDID_info_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->get_EDID_info(handle,
		drm_data->port_number,
		/* Note: Since a pointer is passed to drm_data->edid_*,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->edid_version),
		&(drm_data->edid_revision),
		&(drm_data->edid_size));


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_EDID_info() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_pixelformats() procedure.
 */
int emgd_get_pixelformats(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_pixelformats_t *drm_data = arg;
	unsigned long *format_list = NULL;
	unsigned long *fb_list_pfs = NULL;
	unsigned long *cu_list_pfs = NULL;
	unsigned long *overlay_pfs = NULL;
	unsigned long *render_pfs = NULL;
	unsigned long *texture_pfs = NULL;
	unsigned long *pf;
	int count = 1;

	EMGD_TRACE_ENTER;


	/* The HAL's get_pixelformats() procedure determines which formats are
	 * wanted based on whether a pointer is NULL or not, and so we need to make
	 * the desired-format's pointer non-NULL (setting it to 1 works, though it
	 * is kinda cheating:-):
	 */
	if (drm_data->format == PIXEL_FORMAT_FRAMEBUFFER) {
		fb_list_pfs = (unsigned long *) 1;
	} else if (drm_data->format == PIXEL_FORMAT_CURSOR) {
		cu_list_pfs = (unsigned long *) 1;
	} else if (drm_data->format == PIXEL_FORMAT_OVERLAY) {
		overlay_pfs = (unsigned long *) 1;
	} else if (drm_data->format == PIXEL_FORMAT_RENDER) {
		render_pfs = (unsigned long *) 1;
	} else if (drm_data->format == PIXEL_FORMAT_TEXTURE) {
		texture_pfs = (unsigned long *) 1;
	}


	/* Call the HAL: */
	drm_data->rtn = dispatch->get_pixelformats(drm_data->display_handle,
		&fb_list_pfs, &cu_list_pfs, &overlay_pfs, &render_pfs, &texture_pfs);


	/* Next, point format_list at the correct list: */
	if (drm_data->format == PIXEL_FORMAT_FRAMEBUFFER) {
		format_list = fb_list_pfs;
	} else if (drm_data->format == PIXEL_FORMAT_CURSOR) {
		format_list = cu_list_pfs;
	} else if (drm_data->format == PIXEL_FORMAT_OVERLAY) {
		format_list = overlay_pfs;
	} else if (drm_data->format == PIXEL_FORMAT_RENDER) {
		format_list = render_pfs;
	} else if (drm_data->format == PIXEL_FORMAT_TEXTURE) {
		format_list = texture_pfs;
	}


	/* Count how many pixelformats were returned: */
	pf = format_list;
	while (*pf) {
		pf++;
		count++;
	}
	drm_data->list_size = count;


	/* Copy the pixel format list/array back to user-space, if enough room.  If
	 * there isn't enough memory allocated, the hal2drm code will allocate
	 * drm_data->list_size and call this ioctl again).
	 */
	if ((drm_data->allocated_size > 0) && (count <= drm_data->allocated_size)) {
		OS_MEMCPY(drm_data->format_list, format_list,
			(count * sizeof(unsigned long)));
	}

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_pixelformats() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_port_info() procedure.
 */
int emgd_get_port_info(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_port_info_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->get_port_info(handle,
		drm_data->port_number,
		/* Note: Since a pointer is passed to drm_data->port_info,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->port_info));


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_get_port_info() */


/*!
 * IOCTL to bridge the IAL to the HAL's gmm_alloc_region() procedure.
 */
int emgd_gmm_alloc_region(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_gmm_alloc_region_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->gmm_alloc_region(
		/* Note: Since a pointer is passed to drm_data->offset/size,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->offset),
		&(drm_data->size),
		drm_data->type,
		drm_data->flags);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_gmm_alloc_region() */


/*!
 * IOCTL to bridge the IAL to the HAL's gmm_alloc_surface() procedure.
 */
int emgd_gmm_alloc_surface(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_gmm_alloc_surface_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->gmm_alloc_surface(
		/* Note: Since a pointer is passed to drm_data->*,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->offset),
		drm_data->pixel_format,
		&(drm_data->width),
		&(drm_data->height),
		&(drm_data->pitch),
		&(drm_data->size),
		drm_data->type,
		&(drm_data->flags));


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_gmm_alloc_surface() */

int emgd_gmm_get_num_surface(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_gmm_get_num_surface_t *drm_data = arg;
	EMGD_TRACE_ENTER;

	drm_data->rtn = dispatch->gmm_get_num_surface(
		&(drm_data->count));

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
}

int emgd_gmm_get_surface_list(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_gmm_get_surface_list_t *drm_data = arg;
	igd_surface_list_t *surface_list = NULL;
	EMGD_TRACE_ENTER;

	drm_data->rtn = dispatch->gmm_get_surface_list(
		drm_data->allocated_size,
		&(drm_data->list_size),
		&(surface_list));

	/* Copy the surface array back to user-space, if enough room */
	if ((drm_data->allocated_size > 0) &&
		(drm_data->list_size <= drm_data->allocated_size)) {
		OS_MEMCPY(*(drm_data->surface_list), surface_list,
			(drm_data->list_size * sizeof(igd_surface_list_t)));
	}

	vfree(surface_list);

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
}

/*!
 * IOCTL to bridge the IAL to the HAL's gmm_free() procedure.
 */
int emgd_gmm_free(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_gmm_free_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	/* NOTE: no return value is desired: */
	dispatch->gmm_free(drm_data->offset);


	EMGD_DEBUG("This function has no drm_data->rtn value");
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_gmm_free() */


/*!
 * IOCTL to bridge the IAL to the HAL's gmm_flush_cache() procedure.
 */
int emgd_gmm_flush_cache(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_gmm_flush_cache_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->gmm_flush_cache();


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_gmm_flush_cache() */


/*!
 * IOCTL to bridge the IAL to the HAL's pan_display() procedure.
 */
int emgd_pan_display(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_pan_display_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	/* NOTE: the return value is different than normal: */
	drm_data->rtn = dispatch->pan_display(drm_data->display_handle,
		drm_data->x_offset,
		drm_data->y_offset);


	EMGD_DEBUG("drm_data->rtn = %ld", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_pan_display() */


/*!
 * IOCTL to bridge the IAL to the HAL's power_display() procedure.
 */
int emgd_power_display(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_power_display_t *drm_data = arg;
	igd_context_t *context = (igd_context_t *) handle;
	igd_display_context_t *display;

	EMGD_TRACE_ENTER;


	/* To prevent a kernel OOPS, ensure the following value is non-NULL: */
	display = context->mod_dispatch.dsp_display_list[drm_data->port_number];
	if ((display == NULL) || (PORT(display, drm_data->port_number) == NULL)) {
		EMGD_ERROR_EXIT("emgd_power_display() given an invalid port "
			"number (%d)\n", drm_data->port_number);
		return -IGD_ERROR_INVAL;
	}


	/* Call the HAL: */
	drm_data->rtn = dispatch->power_display(handle,
		drm_data->port_number,
		drm_data->power_state);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_power_display() */


/*!
 * IOCTL to bridge the IAL to the HAL's pwr_alter() procedure.
 */
int emgd_pwr_alter(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_pwr_alter_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->pwr_alter(handle, drm_data->power_state);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_pwr_alter() */


/*!
 * IOCTL to bridge the IAL to the HAL's query_dc() procedure.
 */
int emgd_query_dc(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_query_dc_t *drm_data = arg;
	unsigned long *dc_list = NULL;
	unsigned long *dc;
	int count = 1;

	EMGD_TRACE_ENTER;


	/* Next, We don't need to copy the DC list from user-space, since
	 * the HAL is going to point us to its internal list.
	 */


	/* Call the HAL: */
	drm_data->rtn = dispatch->query_dc(handle,
		drm_data->request,
		/* Note: query_dc() returns a pointer to its live internal list, that
		 * the caller (i.e. this procedure) is not supposed to free/alter.
		 * Therefore, this data must be copied into the space pointed to by
		 * drm_data->dc_list (assuming enough memory was allocated for it--if
		 * not, the hal2drm code will allocate drm_data->list_size and call
		 * this ioctl again).
		 */
		&dc_list,
		drm_data->flags);


	/* Count how many DCs were returned: */
	if(dc_list == NULL){
		EMGD_ERROR("query_dc() returned NULL !");
		return 1;
	}
	dc = dc_list;
	while (*dc) {
		dc++;
		count++;
	}

	drm_data->list_size = count;
	EMGD_DEBUG("drm_data->list_size = %d", drm_data->list_size);


	/* Copy the DC list back to user-space, if enough room: */
	if ((drm_data->allocated_size > 0) && (count <= drm_data->allocated_size)) {
		EMGD_DEBUG("Copying list");
		OS_MEMCPY(drm_data->dc_list, dc_list, (count * sizeof(unsigned long)));
		EMGD_DEBUG("Copied list");
	}


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_query_dc() */


/*!
 * IOCTL to bridge the IAL to the HAL's query_max_size_ovl() procedure.
 */
int emgd_query_max_size_ovl(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_query_max_size_ovl_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->query_max_size_ovl(drm_data->display_handle,
		drm_data->pf,
		/* Note: Since a pointer is passed to drm_data->*,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->max_width),
		&(drm_data->max_height));


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_query_max_size_ovl() */


/*!
 * IOCTL to bridge the IAL to the HAL's query_ovl() procedure.
 */
int emgd_query_ovl(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_query_ovl_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->query_ovl(drm_data->display_handle,
		drm_data->flags);

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_query_ovl() */


/*!
 * IOCTL to bridge the IAL to the HAL's query_mode_list() procedure.
 */
int emgd_query_mode_list(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_query_mode_list_t *drm_data = arg;
	igd_display_info_t *mode_list = NULL;
	igd_display_info_t *mode;
	int count = 1;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->query_mode_list(handle,
		drm_data->dc,
		/* Note: query_mode_list() **MAY** return a pointer to live data, that
		 * the caller (i.e. this procedure) is not supposed to free/alter.
		 * Therefore, this data is copied into the space pointed to by
		 * drm_data->mode_list (assuming enough memory was allocated for it--if
		 * not, the hal2drm code will allocate drm_data->list_size and call
		 * this ioctl again).
		 */
		&mode_list,
		drm_data->flags);


	/* Count how many modes were returned: */
	mode = mode_list;
	while (mode && (mode->width != IGD_TIMING_TABLE_END)) {
		mode++;
		count++;
	}
	drm_data->list_size = count;
	EMGD_DEBUG("drm_data->list_size = %d", drm_data->list_size);


	/* Next, copy the mode list back to user-space: */
	if ((drm_data->allocated_size > 0) && (count <= drm_data->allocated_size)) {
		EMGD_DEBUG("Copying list");
		OS_MEMCPY(drm_data->mode_list, mode_list,
			(count * sizeof(igd_display_info_t)));
		EMGD_DEBUG("Copied list");
	}

	/* Finally, if a non-live list was requested, free the kernel memory: */
	if (!(drm_data->flags & IGD_QUERY_LIVE_MODES)) {
		dispatch->free_mode_list(mode_list);
	}

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_query_mode_list() */


/*!
 * IOCTL to bridge the IAL to the HAL's set_attrs() procedure.
 */
int emgd_set_attrs(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_set_attrs_t *drm_data = arg;
	igd_attr_t *attr_list = drm_data->attr_list;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->set_attrs(handle,
		drm_data->port_number,
		/* Note: Since a pointer is passed to drm_data->num_attrs and
		 * drm_data->attr_list, there's no need to copy anything back into
		 * drm_data, except for the return value.
		 */
		drm_data->list_size,
		attr_list);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_set_attrs() */


/*!
 * IOCTL to bridge the IAL to the HAL's set_palette_entry() procedure.
 */
int emgd_set_palette_entry(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_set_palette_entry_t *drm_data = arg;

	/*EMGD_TRACE_ENTER;*/


	/* Call the HAL: */
	drm_data->rtn = dispatch->set_palette_entry(drm_data->display_handle,
		drm_data->palette_entry,
		drm_data->palette_color);


	/*EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);*/
	/*EMGD_DEBUG("Returning 0");*/
	/*EMGD_TRACE_EXIT;*/
	return 0;
} /* emgd_set_palette_entry() */


/*!
 * IOCTL to bridge the IAL to the HAL's set_surface() procedure.
 */
int emgd_set_surface(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_set_surface_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->set_surface(drm_data->display_handle,
		drm_data->priority,
		drm_data->type,
		&(drm_data->surface),
		drm_data->appcontext,
		drm_data->flags);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_set_surface() */


/*!
 * IOCTL to bridge the IAL to the HAL's sync() procedure.
 */
int emgd_sync(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_sync_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = dispatch->sync(drm_data->display_handle,
		drm_data->priority,
		/* Note: Since a pointer is passed to drm_data->in_sync,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		&(drm_data->in_sync),
		drm_data->flags);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_sync() */


/*!
 * IOCTL to bridge the IAL to the HAL's driver_pre_init() procedure.
 */
int emgd_driver_pre_init(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	drm_emgd_private *priv = dev->dev_private;
	igd_context_t *context = NULL;
	emgd_drm_driver_pre_init_t *drm_data = (emgd_drm_driver_pre_init_t *) arg;
	igd_param_t *x_params = NULL;
	igd_param_t *uc_params = NULL;
	igd_fb_caps_t *pf_caps;
	igd_display_port_t *port = NULL;
	int i, err = 0, need_to_startup_hal = (!priv->hal_running) ? 1 : 0;

	EMGD_TRACE_ENTER;

	/* This flag will cause a call to emgd_init_display() in
	 * emgd_driver_lastclose() if drm_emgd_init is true */
	x_started = true;

	/*
	 * Need to apply the "x_params" (e.g. "xorg.conf" values, from user-space)
	 * in the same order as igd_module_init():
	 * - Nothing to do for REG_INIT(), gmm_init(), nor CMD_INIT().
	 * - mode_init() has several steps (see below).
	 * - Nothing to do for APPCONTEXT_INIT().
	 * - OVERLAY_INIT() is simple (see below).
	 * - Nothing to do for PWR_INIT(), RESET_INIT(), OS_INIT_INTERRUPT(),
	 *   BLEND_INIT(), nor INIT_2D().
	 */
	x_params = &(drm_data->params);


	/* Before applying them, first see if the X driver sent any DTD or attr
	 * list(s) for the given ports (i.e. from "xorg.conf").  If not, the values
	 * from "user_config.c" are supposed to be used (and so they need to be
	 * looked up and copied).
	 */
	if (drm_emgd_configid < 0) {
		uc_params = config_drm.hal_params[0];
	} else {
		uc_params = config_drm.hal_params[drm_emgd_configid-1];
	}
	for (i = 0 ; i < IGD_MAX_PORTS ; i++) {
		igd_display_params_t *x_dp = &x_params->display_params[i];
		igd_display_params_t *uc_dp = NULL;
		int j;

		if ((x_dp->port_number > 0) && (x_dp->dtd_list.num_dtds == 0)) {
			/* X didn't have any DTDs for this port.  Look up the corresponding
			 * display params from "user_config.c"
			 */
			for (j = 0 ; j < IGD_MAX_PORTS ; j++) {
				uc_dp = &uc_params->display_params[j];
				if (uc_dp->port_number == x_dp->port_number) {
					break;
				}
			}
			if ((x_dp->port_number == uc_dp->port_number) ||
				(uc_dp->dtd_list.num_dtds > 0)) {
				/* Have X's DTD list point to "user_config.c"'s DTD list */
				x_dp->dtd_list.num_dtds = uc_dp->dtd_list.num_dtds;
				x_dp->dtd_list.dtd = uc_dp->dtd_list.dtd;
				x_dp->present_params |= IGD_PARAM_DTD_LIST;
			}
		}

		if ((x_dp->port_number > 0) && (x_dp->attr_list.num_attrs == 0)) {
			/* X didn't have any ATTRs for this port.  Look up the corresponding
			 * display params from "user_config.c"
			 */
			for (j = 0 ; j < IGD_MAX_PORTS ; j++) {
				uc_dp = &uc_params->display_params[j];
				if (uc_dp->port_number == x_dp->port_number) {
					break;
				}
			}
			if ((x_dp->port_number == uc_dp->port_number) ||
				(uc_dp->attr_list.num_attrs > 0)) {
				/* Have X's ATTR list point to "user_config.c"'s attr list */
				x_dp->attr_list.num_attrs = uc_dp->attr_list.num_attrs;
				x_dp->attr_list.attr = uc_dp->attr_list.attr;
				x_dp->present_params |= IGD_PARAM_ATTR_LIST;
			}
		}
	}
#ifdef DEBUG_BUILD_TYPE
	if(emgd_debug && emgd_debug->MODULE_NAME)
		emgd_print_params(x_params);
#endif


	if (need_to_startup_hal) {
		/* emgd_driver_load() deferred driver initialization and configuration.
		 * Therefore, do it now.
		 *
		 * Before doing so, ensure both ports are turned on, even if the user
		 * only requested one port.
		 *
		 * TODO/FIXME - This assumes only two ports; is that wise long-term?
		 */
		if (x_params->port_order[1] == 0) {
			if (x_params->port_order[0] == 2) {
				x_params->port_order[1] = 4;
			} else {
				x_params->port_order[1] = 2;
			}
		}

		EMGD_DEBUG("Starting the HAL");
		err = emgd_startup_hal(dev, x_params);
		if (err != 0) {
			mutex_unlock(&dev->struct_mutex);
			return err;
		} else {
			priv = dev->dev_private;
			priv->hal_running = 1;
		}
	}
	/* Now that we know the driver is initialized, we can do the following: */
	context = (igd_context_t *) handle;
	context->mod_dispatch.init_params = x_params;

	if (!need_to_startup_hal) {
		/* emgd_driver_load() initialized and configured the driver.
		 * Therefore, we must now re-initialize and poke x_params values into
		 * various parts of the driver.
		 */
		EMGD_DEBUG("Re-initializing the HAL with X driver values");

		/* mode_init() (in "micro_mode.c") only sets the following value, which
		 * affects future alter_displays() calls.  Note that no code seems to
		 * set x_params->display_color in the X driver.
		 */
		mode_context->display_color = x_params->display_color;

		/* mode_init() calls full_mode_init() (in "igd_mode.c"), which set the
		 * following values, of which, only seamless is used in the HAL.
		 *
		 * If seamless is TRUE, full_mode_init() will call the static procedure
		 * get_fw_info(), which will get firmware-programmed info before the
		 * driver starts re-programming it.  get_fw_info will set seamless to
		 * FALSE if there's any errors.  The get_fw_info() procedure can't be
		 * called here because it's static.  It may also be too late to call it
		 * at this point (i.e. if the HAL has already touched anything, and
		 * that may have something to do with how many times the X server
		 * process starts).
		 */
		mode_context->quickboot = x_params->quickboot;
		mode_context->seamless = x_params->qb_seamless;
		mode_context->video_input = x_params->qb_video_input;
		mode_context->splash = x_params->qb_splash;
		mode_context->first_alter = TRUE;

		/* In case the X server ran, exited, and is starting again, we may need
		 * to put the X server's state back:
		 */
		if (priv->saved_registers == X_SERVER_STATE_SAVED) {
			EMGD_DEBUG("Need to restore the X server's saved register state");
			err = dispatch->driver_save_restore(handle);
			EMGD_DEBUG("State of saved registers is CONSOLE_STATE_SAVED");
			priv->saved_registers = CONSOLE_STATE_SAVED;
		}

		if(!mode_context->seamless) {
		/* NOTE: In order for some new values to be poked into the port
		 * drivers' hardware (e.g. the LVDS panel depth), the power must be
		 * turned off on those devices.  This used to be done during the
		 * emgd_driver_lastclose() function, but that prevents the console from
		 * being seen after X quits.
		 */
		while ((port = context->mod_dispatch.dsp_get_next_port(context,
					port, 0)) != NULL) {
			/* power off LVDS only */
			if (port->pd_driver &&  (port->port_number == IGD_PORT_TYPE_LVDS) &&
				!mode_context->seamless) {
				EMGD_DEBUG("Turning off power for port %lu\n", port->port_number);
				dispatch->power_display(context, port->port_number,
					IGD_POWERSTATE_D3);
			}
		}

		/* mode_init() calls dsp_init() (in "dsp.c"), which uses the
		 * display_flags, display_params & port_order params.  In the case of
		 * display_params, each element of the array contains dtd_list and
		 * attr_list, both of which point to memory that must be separately
		 * copied to kernel-space.  The display_flags, display_params &
		 * port_order params affect the initialization of the DSP module, and
		 * as such, it may not be easy to simply poke values here.
		 *
		 * Thus, it appears that the DSP module must somehow be re-initialized.
		 *
		 * Note: dsp_init() also calls full_dsp_init() and the device-specific
		 * init procedures, none of which uses any params.
		 */
		EMGD_DEBUG("Calling dsp_shutdown()");
		dsp_shutdown(handle);

		EMGD_DEBUG("Calling dsp_init()");
		dsp_init(handle);

		/* Poke any new port attributes & DTDs into the port drivers: */
		while ((port = context->mod_dispatch.dsp_get_next_port(context,
					port, 0)) != NULL) {
				if (port->pd_driver && !mode_context->seamless) {
					EMGD_DEBUG("Insert new port attrs/DTDs for port %lu",
					port->port_number);
				pi_pd_init(port, 0, 0, FALSE);
			}
		}
		}


		/* Set a flag so that emgd_alter_displays() will turn back on the port
		 * drivers' hardware:
		 */
		if(!mode_context->seamless) {
			priv->must_power_on_ports = 1;
		}

		/* OVERLAY_INIT() translates to the _overlay_init() procedure (in
		 * "micro_ovl.c").  It calls ovl_full_init() (in "igd_ovl.c").  If the
		 * IGD_DISPLAY_FB_BLEND_OVL bit in "display_flags" is set, it sets
		 * ovl_context->fb_blend_ovl to 1, which affects future atler_ovl()
		 * calls.
		 */
		if (x_params->display_flags & IGD_DISPLAY_FB_BLEND_OVL) {
			ovl_context->fb_blend_ovl = 1;
		}
	} /* if (!need_to_startup_hal) */

/* NOTE -- Below is Ian Elliott's original analysis of what values/modules need
 * to be dealt with (above):
 *
 * - mode_init() (in "micro_mode.c") uses display_color.
 *
 *   - Note: the value is stored in a global variable:
 *     mode_context->display_color.  This value is used only once--in
 *     full_clear_fb() (igd_mode.c), which is only called by
 *     configure_display() (micro_mode.c), which is only called by
 *     igd_alter_displays() (micro_mode.c).
 *
 *   [ ] Implemented proposal: set the value.  Correct approach?
 *
 * - full_mode_init() (in "igd_mode.c", called by mode_init(); lives in
 *   "micro_mode.c", which is called by init_modules()) uses the quickboot &
 *   qb_* params.  The values are stored in a global variable: mode_context->*.
 *   Here's what I learned in terms of what is used (same is true for IEGD
 *   source):
 *
 *     - The mode_context->quickboot value is only set (here) and is never used.
 *
 *     - mode_context->seamless (corresponding to params->qb_seamless) is
 *       used in full_mode_init() (igd_mode.c) to decide whether (if TRUE) to
 *       call get_fw_info(), which may set it to FALSE depending on whether
 *       there is firmware info that should affect it.  It is also read by
 *       configure_display() (micro_mode.c), which is only called by
 *       igd_alter_displays() (micro_mode.c).  It is also read & used to decide
 *       something by igd_alter_displays() (micro_mode.c).
 *
 *     - mode_context->video_input (corresponding to params->qb_video_input) is
 *       only set (here) and is never used.
 *
 *     - mode_context->splash (corresponding to params->qb_splash) is only set
 *       (here) and is never used.
 *
 *   [ ] New Proposal: potentially set the values and do nothing else.  Is this
 *       the correct approach?
 *
 *   [-] Old Proposal: set the values, and if seamless is TRUE, call
 *       get_fw_info().  Can't actually do so, as get_fw_info() is a static
 *       procedure.
 *
 * - dsp_init() (in "dsp.c", called by mode_init(); lives in "micro_mode.c",
 *   which is called by init_modules()) uses the display_flags, display_params
 *   & port_order params.  It also calls full_dsp_init() and the
 *   device-specific init procedures, neither of which uses any params.  Within
 *   display_params, there are several params looked at: port_number,
 *   present_params, ddc_gpio, ddc_speed, ddc_dab, i2c_gpio, i2c_speed,
 *   i2c_dab, fp_info, dtd_list, and attr_list.  Right now, none of these are
 *   set.
 *
 * - Notes: Here's what I learned in terms of what is used:
 *
 *     - params->display_flags is stored in dsp_context->display_flags.  The
 *       value is used by dsp_dc_init() (dsp.c, called by igd_query_dc() and
 *       mode_init()--called after dsp_init()) to determine if
 *       IGD_DISPLAY_DETECT is set (used frequently in that procedure).  The
 *       value is used by igd_query_dc() (dsp.c, called by igd_query_dc() and
 *       mode_init()--called after dsp_init())--after calling dsp_dc_init()
 *       once, if there's only 1 DC, it uses this value to determine if
 *       IGD_DISPLAY_DETECT is set, and if so the bit is immediately turned off
 *       and dsp_dc_init() is called again.
 *
 *     - params->port_order is used as a parameter to do_port_order() (in
 *       "dsp.c"), which is only called this once via dsp_init().  Based on
 *       this, it seems that we should try to figure out how to re-init this
 *       code.  Correct approach?
 *
 *     - params->display_params is iterated over ... OUCH!  display_params IS A
 *       STRUCT OF TYPE igd_param_attr_list_t, AND SOME OF ITS MEMBERS HAVE
 *       POINTERS WITHIN THEM (i.e. HARDER TO GET ACROSS THE IOCTL BOUNDARY).
 *
 *       Based on the above, it seems that Bob's original idea of
 *       re-initializing the DSP module seems like the correct approach.  I'm
 *       not quite sure how to do that, but it seems like the right direction
 *       to both keep the HAL code unmodified, and do the user/kernel-space
 *       split.
 *
 *       [X] Implemented proposal: copy all user-space memory to kernel-space,
 *           and later free any temporarily allocated memory.
 *       [ ] Proposal: re-initialize the DSP module.  Correct approach?
 *
 * - OVERLAY_INIT() translates to _overlay_init() (in "micro_ovl.c") takes
 *   params as a parameter, which it passes to ovl_full_init() (in
 *   "igd_ovl.c").  If the IGD_DISPLAY_FB_BLEND_OVL bit in "display_flags" is
 *   set, it sets ovl_context->fb_blend_ovl to 1.  This flag is used in the
 *   following procedures:
 *
 *   - ovl_send_instr_plb() (in "emgd_drm/video/overlay/plb/ovl_plb.c") is
 *     called by alter_ovl_plb(), which is called through the ovl_dispatch_plb
 *     table (of type ovl_dispatch_t).
 *
 *   - ovl2_send_instr_plb() (in "emgd_drm/video/overlay/plb/ovl2_plb.c") is
 *     called by alter_ovl2_plb(), which is called through the ovl_dispatch_plb
 *     table (of type ovl_dispatch_t).
 *
 *   - ovl_send_instr_tnc() (in "emgd_drm/video/overlay/tnc/ovl_tnc.c") is
 *     called by alter_ovl_tnc(), which is called through the ovl_dispatch_tnc
 *     table (of type ovl_dispatch_t).
 *
 *   [ ] Implemented proposal: If the IGD_DISPLAY_FB_BLEND_OVL bit in
 *   "display_flags" is set, set ovl_context->fb_blend_ovl to 1.  Correct
 *   approach?
 *
 * - pi_pd_init() (in the file "emgd_drm/display/pi/cmd/pi.c") accesses
 *   params->display_params.  It is called by pi_pd_register() ("pi.c"), which
 *   is called by both igd_pd_register() (in "igd_pi.c") and pd_register() (in
 *   "emgd_drm/display/pd/pd.c").  igd_pd_register() is called by
 *   load_port_drivers() (in "emgd_drv/emgd_dpdloader.c") which is no longer
 *   called (was called during PreInit(), but Ian just took it out).
 *   pd_register() is called by lvds_init() and sdvo_init(), both of which are
 *   called by pi_init_all(), which is not being called.
 *
 *   [ ] TODO - LEARN ABOUT HOW THE STATICALLY-LOADED PORT DRIVERS WILL GET
 *       THEIR params INFO, AND WHEN THEY WILL WANT IT (i.e. IN RELATION TO X
 *       DRIVER PreInit())
 *
 * None of the other modules use and/or need any params at this point:
 *
 * - REG_INIT translates to _reg_init() and uses preserve_reg.  It's too late
 *   to undo what we chose at kernel init time.
 * - gmm_init() ignores page_request and max_fb_size.
 * - CMD_INIT() translates to 0 (i.e. Koheo doesn't use this module).
 * - pi_init() (in "pi.c", called by mode_init(); lives in "micro_mode.c",
 *   which is called by init_modules()) uses no params.
 * - APPCONTEXT_INIT() translates to appcontext_init() doesn't use any params
 * - PWR_INIT() translates to _pwr_init() (in "igd_pwr.c") doesn't look a any
 *   params in context, nor does pwr_init_plb() (in "pwr_plb.c").
 * - RESET_INIT() translates to _reset_init() IS MISSING.  IS IT NEEDED???
 *   Just in case, I looked at the IEGD version (in "igd_reset.c"), which
 *   doesn't look at any params.
 * - OS_INIT_INTERRUPT() doesn't translate into anything real (i.e. IT IS
 *   MISSING).  IS IT NEEDED???  Just in case, I looked at the IEGD version (in
 *   "oal/xfree86/src/interrupt/interrupt.c"), and it is a no-op.
 * - BLEND_INIT() translates to 0 (i.e. Koheo doesn't use this module).
 * - INIT_2D() translates to 0 (i.e. Koheo doesn't use this module).
 */


	/* Call the HAL's igd_get_config_info() procedure: */
	drm_data->rtn = igd_get_config_info(handle,
		/* Note: Since a pointer is passed to drm_data->config_info, there's no
		 * need to copy anything back into drm_data, except for the return
		 * value.
		 */
		&(drm_data->config_info));

	/* Check for a bit depth we support */
	pf_caps = drm_data->config_info.fb_caps;

	while (pf_caps && pf_caps->pixel_format != 0) {
		if (drm_data->bpp == IGD_PF_DEPTH(pf_caps->pixel_format)) {
			EMGD_DEBUG("Found matching PF: %lx for depth %lx bpp.",
				pf_caps->pixel_format, drm_data->bpp);
			drm_data->config_info.pixel_format = pf_caps->pixel_format;
			break;
		}
		pf_caps++;
	}
	if (!drm_data->config_info.pixel_format) {
		EMGD_DEBUG("Given depth (%lx) is not supported.", drm_data->bpp);
	}


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_driver_pre_init() */


/*!
 * IOCTL to support emgd_hal2drm_get_ports(), which replaces the X driver's
 * usage of igd_get_param(...,IGD_PARAM_PORT_LIST,...)
 */
int emgd_driver_get_ports(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_driver_get_ports_t *drm_data = arg;

	EMGD_TRACE_ENTER;


	/* Call the HAL: */
	drm_data->rtn = igd_get_param(handle, IGD_PARAM_PORT_LIST,
		/* Note: Since a pointer is passed to drm_data->ports,
		 * there's no need to copy anything back into drm_data, except
		 * for the return value.
		 */
		drm_data->ports);


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_driver_get_ports() */

/*!
 * IOCTL to support emgd_hal2drm_get_page_list(), which get's the list
 * of page address that make up a memory allocation.
 */
int emgd_get_page_list(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_get_page_list_t *drm_data = arg;
	unsigned long cnt;
	unsigned long *list;
	unsigned long *new_list;
	int i;

	EMGD_TRACE_ENTER;

	dispatch->gmm_get_page_list(drm_data->offset, &list, &cnt);
	if (cnt > drm_data->addr_count) {
		/*
		 * The user didn't allocate enough space for the
		 * page list, this is an error.
		 */
		drm_data->addr_count = cnt;
		drm_data->rtn = -IGD_NO_MEM;
	} else {
		/* Copy the address list to the caller */
		new_list = (unsigned long *)(arg + sizeof(emgd_drm_get_page_list_t));
		for (i = 0; i < cnt; i++) {
			new_list[i] = list[i];
		}
	}

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_driver_get_page_list() */

/*!
 * IOCTL to allow the X driver (in "egd_driver.c") to start the Imagination
 * Technologies PVR services DRM/kernel module, which will start our 3rd-party
 * display driver (3DD).
 */
int emgd_start_pvrsrv(struct drm_device *dev, void *arg,
	struct drm_file *file_priv)
{
	emgd_drm_start_pvrsrv_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;

	EMGD_TRACE_ENTER;


	/* Tell the 3DD the status of whether the X server is running: */
	if (!priv->xserver_running && drm_data->xserver) {
		priv->xserver_running = 1;
		if (priv->pvrsrv_started && priv->reinit_3dd) {
			priv->reinit_3dd(dev);
		}
	}


	if (0 == priv->pvrsrv_started) {
		/* Start the PVR services DRM code: */
		EMGD_DEBUG("Calling PVRSRVDrmLoad()");
		drm_data->rtn = (int) PVRSRVDrmLoad(dev, 0);
		EMGD_DEBUG("PVRSRVDrmLoad() returned %d", drm_data->rtn);

		if (0 == drm_data->rtn) {
			/* Must also call PVRSRVOpen() on behalf of this process: */
			EMGD_DEBUG("Calling PVRSRVOpen()");
			drm_data->rtn = (int) PVRSRVOpen(dev, file_priv);
			EMGD_DEBUG("PVRSRVOpen() returned %d", drm_data->rtn);
		}

		if (0 == drm_data->rtn) {
			priv->pvrsrv_started = 1;
		}
	} else {
		drm_data->rtn = 0;
		EMGD_DEBUG("Calling PVRSRVOpen()");
		drm_data->rtn = (int) PVRSRVOpen(dev, file_priv);
		EMGD_DEBUG("PVRSRVOpen() returned %d", drm_data->rtn);
	}


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_start_pvrsrv() */


/*!
 * IOCTL to bridge the IAL to the HAL's video_cmd_buf() procedure.
 */
int emgd_video_cmd_buf(struct drm_device *dev, void *arg,
		struct drm_file *file_priv)
{
	emgd_drm_video_cmd_buf_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;
	igd_context_t *context = priv->context;


	EMGD_TRACE_ENTER;

	/* Call the HAL: */
	switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = process_video_decode_plb(context, drm_data->offset,
								drm_data->kernel_virt_addr,
								&(drm_data->fence_id));
			break;
		case TNC_ENGINE_ENCODE:
			drm_data->rtn = process_video_encode_tnc(context, drm_data->offset,
								drm_data->kernel_virt_addr,
								&(drm_data->fence_id));
			break;
		default:
			break;
	}


	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_video_cmd_buf() */


/*!
 * IOCTL to bridge the IAL to the HAL's get_device_info() procedure.
 */
int emgd_get_device_info(struct drm_device *dev, void *arg,
    struct drm_file *file_priv)
{
    emgd_drm_device_info_t *drm_data = arg;
    drm_emgd_private *priv = dev->dev_private;
    igd_context_t *context = priv->context;

    EMGD_TRACE_ENTER;

    drm_data->display_memory_offset = context->device_context.fb_adr;
    drm_data->display_memory_size = context->device_context.mem_size;
    drm_data->device_id = context->device_context.did;
    drm_data->revision_id = context->device_context.rid;
    drm_data->bridge_id = context->device_context.bid;

    EMGD_TRACE_EXIT;
    return 0;
} /* emgd_get_device_info() */


int emgd_init_video(struct drm_device *dev, void *arg,
		struct drm_file *file_priv)
{
	emgd_drm_init_video_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;
	igd_context_t *context = priv->context;

	EMGD_TRACE_ENTER;

	switch (drm_data->cmd) {
	case CMD_VIDEO_STATE :
		switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_query_plb(context, &drm_data->status);
			break;
		default:
			drm_data->rtn = 1;
			break;
		}
		break;
	case CMD_VIDEO_INITIALIZE :
		/* Call the HAL: */
		switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_init_plb(drm_data->base0, drm_data->base1,
						       drm_data->fw_priv, drm_data->fw_size);
			break;
		case TNC_ENGINE_ENCODE:
			drm_data->rtn = topaz_init_tnc(drm_data->wb_offset, drm_data->wb_addr, drm_data->firm_addr);
			break;
		default:
			break;
		}
		break;
	case CMD_VIDEO_UNINITIALIZE :
		switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_uninit_plb(context);
			break;
		default:
			break;
		}
		break;
	case CMD_VIDEO_CLOSE_CONTEXT :
		switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_close_context(context);
			break;
		default:
			break;
		}
		break;
	case CMD_VIDEO_CREATE_CONTEXT:
		switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_create_context(context);
			break;
		default:
			break;
		}
		break;
	case CMD_VIDEO_SHUTDOWN:
		switch (drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_shutdown_plb(context);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
} /* emgd_init_video() */

int emgd_video_get_info(struct drm_device *dev, void *arg,
			struct drm_file *file_priv)
{
	emgd_drm_video_get_info_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;
	igd_context_t *context = priv->context;

	EMGD_TRACE_ENTER;

	switch(drm_data->cmd) {
		case CMD_VIDEO_GET_FENCE_ID:
			switch(drm_data->engine){
				case PSB_ENGINE_VIDEO:
					drm_data->rtn = msvdx_get_fence_id(context, &(drm_data->fence_id));
					break;
				case TNC_ENGINE_ENCODE:
					drm_data->rtn = topaz_sync_surface(context, &(drm_data->sync_done),
								&(drm_data->last_frame));
					break;
				default:
					break;
			}
			break;
		case CMD_VIDOE_GET_FRAME_SKIP:
			switch(drm_data->engine){
				case PSB_ENGINE_VIDEO:
					break;
				case TNC_ENGINE_ENCODE:
					drm_data->rtn = topaz_get_frame_skip(context, &(drm_data->frame_skip));
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
}


int emgd_video_flush_tlb(struct drm_device *dev, void *arg,
			struct drm_file *file_priv)
{
	emgd_drm_video_flush_tlb_t *drm_data = arg;
	drm_emgd_private *priv = dev->dev_private;
	igd_context_t *context = priv->context;

	EMGD_TRACE_ENTER;

	switch(drm_data->engine) {
		case PSB_ENGINE_VIDEO:
			drm_data->rtn = msvdx_flush_tlb(context);
			break;
		case TNC_ENGINE_ENCODE:
			drm_data->rtn = topaz_flush_tnc(context);
			break;
		default:
			break;
	}

	EMGD_DEBUG("drm_data->rtn = %d", drm_data->rtn);
	EMGD_DEBUG("Returning 0");
	EMGD_TRACE_EXIT;
	return 0;
}

int emgd_get_golden_htotal(struct drm_device *dev, void *arg,
   	struct drm_file *file_priv){


	emgd_drm_get_golden_htotal_t *drm_data = arg;
	igd_context_t *context = (igd_context_t *) handle;
    igd_display_context_t *display;
    pd_timing_t igd_mode_table_in[2];
    pd_timing_t igd_mode_table_out;

	igd_display_info_t *out_mode = (igd_display_info_t *) drm_data->out_mode;
	igd_display_info_t *in_mode = (igd_display_info_t *) drm_data->in_mode;


   	EMGD_DEBUG("emgd_get_golden_htotal : Entry");

    /* parameters sanity check */
   	if (out_mode == NULL) {
          EMGD_ERROR("emgd_get_golden_htotal : NO Output Buffer");
        return -IGD_ERROR_INVAL;
    }

    if (in_mode == NULL) {
        EMGD_ERROR("emgD_get_golden_htotal : NO Input Buffer");
		return -IGD_ERROR_INVAL;

    }

    /* Zero out the data structures so that we can check for error later */
    memset(igd_mode_table_in, 0, 2 * sizeof(pd_timing_t));
    memset(&igd_mode_table_out, 0, sizeof(pd_timing_t));

	/* To prevent a kernel OOPS, ensure the following value is non-NULL: */
	display = context->mod_dispatch.dsp_display_list[2];
	if ((display == NULL) || (PORT(display, 2) == NULL)) {
		EMGD_ERROR_EXIT("emgd_get_golden_htotal() given an invalid port "
			"number (%d)\n", 2);
		return -IGD_ERROR_INVAL;
	}

    /* convert the data from escape structure to pd_timing */
    igd_mode_table_in[0].width = in_mode->width;
    igd_mode_table_in[0].height = in_mode->height;
    igd_mode_table_in[0].refresh = in_mode->refresh;

    /* Set the end of list */
    igd_mode_table_in[1].width = PD_TIMING_LIST_END;
    igd_mode_table_in[1].extn_ptr = NULL;

    /************************************************************/
    /* call dispatch function that will generate the golden htotal
     * for all the modes. The pd_timing_t has the same structure as
     * igd_display_t so we can typecast without issue.
     */
    /************************************************************/
    context->mod_dispatch.get_dd_timing(display,
                            (pd_timing_t*)&igd_mode_table_in);

    /************************************************************/
    /* Send back the modified mode list that contains the delta */
    /************************************************************/

    memcpy (out_mode, in_mode, sizeof(igd_display_info_t));
    out_mode->reserved_dd = igd_mode_table_in[0].reserved_dd;

	out_mode->hblank_end = igd_mode_table_in[0].hblank_end;
	out_mode->vblank_end = igd_mode_table_in[0].vblank_end;


    return 0;
}
