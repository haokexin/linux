/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: reg.c
 * $Revision: 1.13 $
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
 *  This is the implementation file for the register module's main API's called
 *  by external devices.
 *-----------------------------------------------------------------------------
 */

#define MODULE_NAME hal.state

#include <io.h>

#include <igd_init.h>
#include <igd_mode.h>
#include <igd_errno.h>

#include <context.h>
#include <utils.h>
#include <dsp.h>
#include <mode.h>
#include <memory.h>
#include <utils.h>

#include <intelpci.h>
#include <dispatch.h>

#include "reg_dispatch.h"

/*!
 * @addtogroup state_group
 * @{
 */

typedef struct _reg_context {
	unsigned long flags;     /* IGD_DRIVER_SAVE_RESTORE */
	reg_dispatch_t *dispatch;
	reg_buffer_t *device_state;
	void *gmm_state;
} reg_context_t;

static reg_context_t reg_ctx[1];


static dispatch_table_t reg_dispatch_list[] = {

#ifdef CONFIG_PLB
	{PCI_DEVICE_ID_VGA_PLB, &reg_dispatch_plb},
#endif
#ifdef CONFIG_TNC
	{PCI_DEVICE_ID_VGA_TNC, &reg_dispatch_tnc},
#endif
	{0, NULL}
};

void *reg_alloc(igd_context_t *context,
	unsigned long flags)
{
	return (void *)reg_ctx->dispatch->reg_alloc(context, flags,
		reg_ctx->dispatch->platform_context);
}

void reg_free(igd_context_t *context,
	void *reg_set)
{
	reg_ctx->dispatch->reg_free(context, (reg_buffer_t *)reg_set,
		reg_ctx->dispatch->platform_context);
}

int reg_save(igd_context_t *context,
	void *reg_set)
{
	return reg_ctx->dispatch->reg_save(context, (reg_buffer_t *)reg_set,
		reg_ctx->dispatch->platform_context);
}

int reg_restore(igd_context_t *context,
	void *reg_set)
{
	return reg_ctx->dispatch->reg_restore(context, (reg_buffer_t *)reg_set,
		reg_ctx->dispatch->platform_context);
}

/*!
 * This function calls reg_save() to save the state of the graphics engine
 * and then reg_restore to restore the previous state.
 *
 * @param driver_handle  our driver context
 *
 * @return 0 on success
 * @return -IGD_INVAL on failure
 */
static int igd_driver_save_restore(igd_driver_h driver_handle)
{
	igd_context_t *context = (igd_context_t *)driver_handle;
	reg_buffer_t *reg_state;
	short restored = 0;
	unsigned long save_flags;

	EMGD_ASSERT(context, "Null driver_handle!\n", -IGD_ERROR_INVAL);

	/*
	 * If the gmm_state exists, the registers are not setup, so we should not
	 * try to save the GTT, the ring buffer and appcontext, or it will cause a
	 * sync which will not be fulfilled, so it will wait forever.  We can
	 * however save everything else.  If the gmm_state is null, then everything
	 * is still setup and we can save everything including the GTT, ring buffer
	 * and appcontext.
	 */
	if (reg_ctx->gmm_state) {
		save_flags = IGD_REG_SAVE_ALL & ~IGD_REG_SAVE_GTT & ~IGD_REG_SAVE_RB;
	} else {
		save_flags = IGD_REG_SAVE_ALL;
	}

	/* Save current state. We have to save the current state first before
	 * restoring the GART bindings. Otherwise we will end up restoring wrong
	 * pages when return back to the graphic mode console */
	reg_state = reg_ctx->dispatch->reg_alloc(context, save_flags,
		reg_ctx->dispatch->platform_context);
	reg_ctx->dispatch->reg_save(context, reg_state,
		reg_ctx->dispatch->platform_context);

	/* Restore GART bindings, if saved */
	if (reg_ctx->gmm_state && context->mod_dispatch.gmm_restore) {
		context->mod_dispatch.gmm_restore(context, reg_ctx->gmm_state);
		reg_ctx->gmm_state = NULL;
		restored = 1;
	}

	/* restore previously saved state */

	/*
	 * NAPA class seems to work much better all the display stuff is
	 * turned off prior to restoring the registers.
	 */
	context->mod_dispatch.mode_reset_plane_pipe_ports(context);


	reg_ctx->dispatch->reg_restore(context, reg_ctx->device_state,
		reg_ctx->dispatch->platform_context);
	reg_ctx->dispatch->reg_free(context, reg_ctx->device_state,
		reg_ctx->dispatch->platform_context);

	reg_ctx->device_state = reg_state;

	/* Free GART bindings, if not already saved */
	if (!restored && !reg_ctx->gmm_state &&
		context->mod_dispatch.gmm_save) {
		context->mod_dispatch.gmm_save(context, &(reg_ctx->gmm_state));
	}

	return 0;
}

/*!
 * This function calls reg_save() to save the state of the register
 *
 * @param driver_handle our driver context
 * @param flags specifies which states to save
 *
 * @return 0 on success
 * @return -IGD_INVAL on failure
 */
static int igd_driver_save(igd_driver_h driver_handle,
	const unsigned long flags)
{
	igd_context_t *context = (igd_context_t *)driver_handle;
	reg_buffer_t *reg_state;

	EMGD_ASSERT(context, "Null driver_handle!\n", -IGD_ERROR_INVAL);

	/* Save current state */
	reg_state = reg_ctx->dispatch->reg_alloc(context, flags,
		reg_ctx->dispatch->platform_context);
	reg_ctx->dispatch->reg_save(context, reg_state,
		reg_ctx->dispatch->platform_context);

	reg_ctx->device_state = reg_state;

	return 0;
}
/* igd_driver_save */

/*!
 * This function calls reg_restore() to restore previously saved
 *
 * @param driver_handle our driver context
 *
 * @return 0 on success
 * @return -IGD_INVAL on failure
 */
static int igd_driver_restore(igd_driver_h driver_handle)
{
	igd_context_t *context;

	EMGD_ASSERT(driver_handle, "Null driver_handle!\n", -IGD_ERROR_INVAL);

	context = (igd_context_t *)driver_handle;

	/*
	 * NAPA class seems to work much better all the display stuff is
	 * turned off prior to restoring the registers.
	 */
	context->mod_dispatch.mode_reset_plane_pipe_ports(context);

	/* restore previously saved state */
	reg_ctx->dispatch->reg_restore(context, reg_ctx->device_state,
		reg_ctx->dispatch->platform_context);
	reg_ctx->dispatch->reg_free(context, reg_ctx->device_state,
		reg_ctx->dispatch->platform_context);

	return 0;
}

/*!
 *
 * @param id
 * @param state
 * @param flags
 *
 * @return 0 on success
 * @return -IGD_ERROR_INVAL on failure
 */
static int reg_get_mod_state(reg_state_id_t id, module_state_h **state,
	unsigned long **flags)
{
	if(!reg_ctx->device_state) {
		return 0;
	}

	*flags = &reg_ctx->device_state->flags;
	switch(id) {
	case REG_MODE_STATE:
		if((**flags & IGD_REG_SAVE_MODE)){
			*state = &reg_ctx->device_state->mode_buffer;
		} else {
			state = NULL;
		}
		break;
	default:
		state = NULL;
		return -IGD_ERROR_INVAL;
	}

	return 0;
}

/*!
 * Free any memory allocated for registry state info.
 *
 * @param context
 *
 * @return void
 */
void _reg_shutdown(igd_context_t *context)
{

	EMGD_TRACE_ENTER;


	if(reg_ctx->device_state) {
		EMGD_DEBUG("Restoring register values prior to exit...");
		reg_ctx->dispatch->reg_restore(context, reg_ctx->device_state,
			reg_ctx->dispatch->platform_context);
		reg_ctx->dispatch->reg_free(context, reg_ctx->device_state,
			reg_ctx->dispatch->platform_context);

	}
	OS_MEMSET(reg_ctx, 0, sizeof(reg_context_t));

	EMGD_TRACE_EXIT;
}

/*!
 * Initialize the registry state context for registry save
 * /restore operations.
 *
 * @param context
 * @param flags
 *
 * @return 0 on success
 * @return 1 on failure
 */
int _reg_init(igd_context_t *context, unsigned long flags)
{
	reg_dispatch_t *dispatch;

	EMGD_TRACE_ENTER;

	dispatch = (reg_dispatch_t *)dispatch_acquire(context, reg_dispatch_list);
	if(!dispatch) {
		return -IGD_ERROR_NODEV;
	}

	OS_MEMSET(reg_ctx, 0, sizeof(reg_context_t));

	reg_ctx->dispatch = dispatch;

	/*
	 * Hook up functions in dispatch table.
	 */
	context->dispatch.driver_save_restore = igd_driver_save_restore;
	context->dispatch.driver_save         = igd_driver_save;
	context->dispatch.driver_restore      = igd_driver_restore;
	context->mod_dispatch.reg_get_mod_state = reg_get_mod_state;
	context->mod_dispatch.reg_alloc   = reg_alloc;
	context->mod_dispatch.reg_free    = reg_free;
	context->mod_dispatch.reg_save    = reg_save;
	context->mod_dispatch.reg_restore = reg_restore;
	context->mod_dispatch.reg_shutdown = _reg_shutdown;

	if(flags & IGD_DRIVER_SAVE_RESTORE) {
		reg_ctx->flags |= IGD_DRIVER_SAVE_RESTORE;
		EMGD_DEBUG("Saving Device State");

		reg_ctx->device_state = reg_ctx->dispatch->reg_alloc(context,
			IGD_REG_SAVE_STATE|IGD_REG_SAVE_MODE,
			reg_ctx->dispatch->platform_context);
		reg_ctx->dispatch->reg_save(context, reg_ctx->device_state,
			reg_ctx->dispatch->platform_context);
	}

	EMGD_TRACE_EXIT;

	return 0;
}

