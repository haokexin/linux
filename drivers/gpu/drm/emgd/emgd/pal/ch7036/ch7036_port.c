/*-----------------------------------------------------------------------------
* Copyright (c) Chrontel Inc.
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
* @file  ch7036_port.c
* @version 1.1.4
*-----------------------------------------------------------------------------
*/




#include "ch7036_port.h"
#include "ch7036_fw.h"
#include "lvds/lvds.h"
#include <linux/kernel.h>




static pd_version_t  g_ch7036_version = {1, 0, 0, 0};
static unsigned long g_ch7036_dab_list[] = {0xEC,PD_DAB_LIST_END};


void ch7036_update_position(ch7036_device_context_t *p_ctx, OUTPUT_INFO* pOutput_Info);
int ch7036_initialize_device(ch7036_device_context_t *p_ctx);


static pd_driver_t	 g_ch7036_drv = {
	PD_SDK_VERSION,
	"Chrontel CH7036 Port Driver",
	0,
	&g_ch7036_version,
	PD_DISPLAY_LVDS_INT,
	PD_FLAG_UP_SCALING,
	g_ch7036_dab_list,
	100,
	ch7036_validate,
	ch7036_open,
	ch7036_init_device,
	ch7036_close,
	ch7036_set_mode,
	ch7036_post_set_mode,
	ch7036_set_attributes,
	ch7036_get_attributes,
	ch7036_get_timing_list,
	ch7036_set_power,
	ch7036_get_power,
	ch7036_save,
	ch7036_restore,
	ch7036_get_port_status
};



ch7036_edid_blk_t crt_edid;
ch7036_edid_blk_t hdvi_edid;





int PD_MODULE_INIT(ch7036_init, (void *handle))
{
	int status;

	PD_DEBUG("ch7036: ch7036_init()\n");

	status = pd_register(handle, &g_ch7036_drv);
	if (status != PD_SUCCESS) {
		PD_DEBUG("ch7036: Error ! ch7036_init: pd_register() failed with "
				  "status=%#x\n", status);
	}
	return status;
}



int PD_MODULE_EXIT(ch7036_exit, (void))
{
	PD_DEBUG("ch7036: ch7036_exit()\n");

	return (PD_SUCCESS);
}


unsigned long ch7036_validate(unsigned long cookie)
{
	PD_DEBUG("ch7036: ch7036_validate()\n");
	return cookie;
}


int ch7036_open(pd_callback_t *p_callback, void **pp_context)
{
	uint8 device_ID;
	/*uint8 reg;*/
	ch7036_device_context_t* p_ctx;
	DEV_CONTEXT* p_ch7xxx_context;

	/*uint8 page, n;*/

	int ret;


	PD_DEBUG("ch7036: ch7036_open()\n");

	ret = PD_INTERNAL_LVDS_MODULE_OPEN(ch7036_internal_lvds_open,(p_callback, pp_context));
	if ( ret != PD_SUCCESS)
	{
		pd_free(p_ctx->fw);
		pd_free(p_ctx);
		return ret;
	}


	p_ctx = pd_malloc(sizeof(ch7036_device_context_t));
	if (p_ctx == NULL) {
		goto exit6;
	}

	pd_memset(p_ctx, 0, sizeof(ch7036_device_context_t));

	p_ctx->fw = (FW7036_CFG *)(pd_malloc(sizeof(FW7036_CFG)));

	if (p_ctx == NULL) {
		PD_ERROR("ch7036: Error ! ch7036_open: pd_malloc() failed allocating FW7036_CFG struct\n");
		goto exit5;
	}


	if (p_ctx == NULL || *pp_context == NULL || p_callback == NULL) {
		goto exit5;
	}

	p_ctx->internal_lvds = *pp_context;

	p_ctx->p_callback = p_callback;


	p_ctx->p_ch7xxx_context = pd_malloc(sizeof(DEV_CONTEXT));
	if (p_ctx->p_ch7xxx_context == NULL) {
		PD_ERROR("ch7036: Error ! ch7036_open: pd_malloc() failed allocating DEV_CONTEXT struct");
		goto exit4;
	}

	p_ch7xxx_context = p_ctx->p_ch7xxx_context;

	p_ch7xxx_context->pd_context = (void *)p_ctx;


	I2CWrite(p_ch7xxx_context,0x03, 0x04);

	device_ID = I2CRead(p_ch7xxx_context,0x50);

	PD_DEBUG("ch7036: ch7036_open()- read device ID= 0x%.2X\n", device_ID);



	if(device_ID != 0x56)
	{
		PD_DEBUG("ch7036: ch7036_open()- device is NOT found...\n");

		if(p_ch7xxx_context->pd_context)
			p_ch7xxx_context->pd_context=NULL;
		pd_free(p_ch7xxx_context);
		if(p_ctx->internal_lvds) {
			p_ctx->internal_lvds=NULL;

		}
		pd_free(p_ctx->fw);
		pd_free(p_ctx);
		return PD_ERR_NODEV;

	}
	else
	{
		PD_DEBUG("ch7036: ch7036_open()- ch7036 device is found...\n");


	}


	p_ch7xxx_context->DeviceID = device_ID;


	p_ctx->hpd = 0;

	if (ch7036_load_firmware(p_ctx) != SS_SUCCESS)   {
		p_ctx->use_firmware =0;
		p_ctx->cedid = NULL;
		p_ctx->hedid = NULL;


	}
	else {
		p_ctx->use_firmware =1;
		p_ctx->cedid = (ch7036_edid_blk_t *)&crt_edid;
		p_ctx->hedid = (ch7036_edid_blk_t *)&hdvi_edid;

	}



	p_ch7xxx_context->pInput_Info = pd_malloc(sizeof(INPUT_INFO));
	if (p_ch7xxx_context->pInput_Info == NULL) {
		PD_ERROR("ch7036: Error ! ch7036_open: pd_malloc() failed allocating INPUT_INFO struct");

		goto exit3;
	}

	p_ch7xxx_context->pOutput_Info = pd_malloc(sizeof(OUTPUT_INFO));
	if (p_ch7xxx_context->pOutput_Info == NULL) {
		PD_ERROR("ch7036: Error ! ch7036_open: pd_malloc() failed allocating OUTPUT_INFO struct");

		goto exit2;
	}

	p_ch7xxx_context->pPrefer_Info = pd_malloc(sizeof(PREFER_INFO));
	if (p_ch7xxx_context->pPrefer_Info == NULL) {
		PD_ERROR("ch7036: Error ! ch7036_open: pd_malloc() failed allocating PREFER_INFO struct");

		goto exit1;
	}




	p_ctx->p_ch7036_attr_table = NULL;

	if( ch7036_init_attribute_table(p_ctx, NULL) == SS_MEM_ALLOC_ERR)
	{
		pd_free(p_ch7xxx_context->pPrefer_Info);
		goto exit1;
	}

	ch7036_initialize_device(p_ctx);


	g_ch7036_drv.type = PD_DISPLAY_LVDS_INT;


	*pp_context = (void *)p_ctx;

	return (PD_SUCCESS);

exit1:
	pd_free(p_ch7xxx_context->pOutput_Info);
exit2:
	pd_free(p_ch7xxx_context->pInput_Info);
exit3:
	p_ch7xxx_context->pd_context=NULL;
	pd_free(p_ch7xxx_context);
exit4:
	pd_free(p_ctx->internal_lvds);
	pd_free(p_ctx->fw);
exit5:
	pd_free(p_ctx);
exit6:
	return PD_ERR_NOMEM;
}


int ch7036_init_device(void *p_context)
{
	ch7036_device_context_t* p_ctx  = (ch7036_device_context_t*)p_context;

	PD_DEBUG("ch7036: ch7036_init_device()-enter\n");

	p_ctx->init_done = 1;

	return (PD_INTERNAL_LVDS_MODULE_INIT_DEVICE(ch7036_init_device, (p_ctx->internal_lvds)));

}



int ch7036_set_mode(void *p_context, pd_timing_t *p_mode, unsigned long flags)
{

	ch7036_device_context_t *p_ctx = (ch7036_device_context_t*)p_context;

	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	INPUT_INFO* pInput_Info = p_ch7xxx_context->pInput_Info;
	OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;
	PREFER_INFO* pPrefer_Info = p_ch7xxx_context->pPrefer_Info;
	int ret, channel_on=0;


	PD_DEBUG("ch7036: ch7036_set_mode()-enter\n");



	if (!p_ctx || !p_mode) {
		return (PD_ERR_NULL_PTR);
	}

	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}
	PD_DEBUG("ch7036_set_mode: requested width = %u height = %u\n",
		p_mode->width, p_mode->height);


	if (
		(p_ctx->fp_width && (p_mode->width > p_ctx->fp_width)) ||
		(p_ctx->fp_height && (p_mode->height > p_ctx->fp_height))
		) {
		return PD_ERR_MODE_NOTSUPP;
	}


	p_ctx->p_cur_mode = p_mode;


	if(pOutput_Info->channel  == CHANNEL_LVDS_HDMI_VGA_OFF) {

		pOutput_Info->channel = ch7036_get_output_channel(p_ctx);
		channel_on =1;
	}




	if(pOutput_Info->channel == CHANNEL_LVDS && p_ctx->use_firmware ) {
		pOutput_Info->channel = CHANNEL_LVDS_HDMI;
		p_ctx->req_ddc =1;
	}
	if (pOutput_Info->channel == CHANNEL_LVDS) {
		pOutput_Info->channel = CHANNEL_LVDS_HDMI;
		p_ctx->lvds_only =1;
	}


	ch7036_set_input_timing_info(p_ctx,pInput_Info);
	ch7036_set_output_timing_info(p_ctx, pOutput_Info);
	ch7036_set_prefer_timing_info(p_ctx,pPrefer_Info);


	if(ch7036_device_prepare(p_ctx)== SS_UNSUCCESSFUL)
	{
		PD_DEBUG("ch7036_set_mode: ch7036_device_prepare()- NOT SUCCESS\n");
		return PD_ERR_UNSUCCESSFUL;
	}


	ch7036_update_position(p_ctx, pOutput_Info);

	if(ch7036_device_config(p_ctx) == SS_UNSUCCESSFUL)
	{
		PD_DEBUG("ch7036_set_mode: ch7036_device_config()- NOT SUCCESS\n");
		return PD_ERR_UNSUCCESSFUL;
	}

	if(channel_on)
		pOutput_Info->channel = CHANNEL_LVDS_HDMI_VGA_OFF;

	ret = PD_INTERNAL_LVDS_MODULE_SET_MODE(ch7036_internal_lvds_set_mode,(p_ctx->internal_lvds,p_mode,flags));
	if(ret != PD_SUCCESS)
		return ret;


	return PD_SUCCESS;
}
//thua- 10/24/11- when user switches back and forth from VGA to HDMI or vice versa
//need to set h/v pos. accordingly
void ch7036_update_position(ch7036_device_context_t *p_ctx, OUTPUT_INFO* pOutput_Info)
{

	PD_DEBUG("ch7036: ch7036_update_position()- enter\n");


	if(pOutput_Info->channel & CHANNEL_HDMI) {

		pOutput_Info->h_position = DEFAULT_POSITION;
		pOutput_Info->v_position = DEFAULT_POSITION;

	}
	else {//pOutput_Info->channel & CHANNEL_VGA; CHANNEL_LVDS would never come here

			pOutput_Info->h_position =  pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_HPOSITION,
					PD_GET_ATTR_LIST)->current_value;
			pOutput_Info->v_position = pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_VPOSITION,
					PD_GET_ATTR_LIST)->current_value;

	}

	PD_DEBUG("ch7036: ch7036_update_position()- h pos [%d]\n",pOutput_Info->h_position);
	PD_DEBUG("ch7036: ch7036_update_position()- v pos [%d]\n",pOutput_Info->v_position);

	return;
}


int ch7036_post_set_mode(void *p_context, pd_timing_t *p_mode,
						  unsigned long flags)
{


	ch7036_device_context_t* p_ctx  = (ch7036_device_context_t*)p_context;
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;

	int ret;
	/*uint8 page, n;*/
	pd_list_entry_attr_t  *list_entry;



	if (!p_ctx || !p_mode ) {
		return (PD_ERR_NULL_PTR);
	}

	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}

	list_entry = (pd_list_entry_attr_t *)pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs, PD_ATTR_ID_DISPLAY, PD_GET_ATTR_LIST_ENTRY);
	ch7036_set_output_channel(p_ctx,(uint32)(list_entry->value));


	PD_DEBUG("ch7036_post_set_mode- current output channel is [%x]\n",pOutput_Info->channel);

	if(ch7036_device_start(p_ctx) == SS_UNSUCCESSFUL)
	{
		PD_DEBUG("ch7036_post_set_mode: ch7036_device_start()- NOT SUCCESS\n");
		return PD_ERR_UNSUCCESSFUL;
	}



	p_ctx->req_ddc = 0;
	p_ctx->lvds_only = 0;

#if 0
	if(p_ctx->req_ddc && pOutput_Info->channel == CHANNEL_LVDS_HDMI && p_ctx->use_firmware) {

		pOutput_Info->channel = CHANNEL_LVDS;
		p_ctx->req_ddc = 0;
	}

	if (p_ctx->lvds_only  && pOutput_Info->channel == CHANNEL_LVDS_HDMI) {
		pOutput_Info->channel = CHANNEL_LVDS;
		p_ctx->lvds_only = 0;
	}

#endif

	ret = PD_INTERNAL_LVDS_MODULE_POST_SET_MODE(ch7036_post_set_mode,(p_ctx->internal_lvds,p_mode,flags));

	if(ret != PD_SUCCESS)
		return ret;
	else
	{
		ch7036_reset_datapath(p_ch7xxx_context);
		pd_usleep(50);
		ch7036_device_set_power(p_ctx, pOutput_Info->channel);
	}


	return PD_SUCCESS;
}





int ch7036_close(void *p_context)
{

	ch7036_device_context_t* p_ctx  = (ch7036_device_context_t*)p_context;

	PD_DEBUG("ch7036: ch7036_close()\n");

	PD_INTERNAL_LVDS_MODULE_CLOSE(ch7036_internal_lvds_close, (p_ctx->internal_lvds));

	if (p_ctx!= NULL)
	{

		pd_free(p_ctx->p_ch7xxx_context->pInput_Info);
		pd_free(p_ctx->p_ch7xxx_context->pOutput_Info);
		pd_free(p_ctx->p_ch7xxx_context->pPrefer_Info);
		pd_free(p_ctx->p_ch7xxx_context);
		pd_free(p_ctx->p_ch7036_attr_table);
		pd_free(p_ctx);
	}

	return PD_SUCCESS;
}


int ch7036_get_timing_list(void *p_context, pd_timing_t *p_in_list,
	pd_timing_t **pp_out_list)
{
	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	/*OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;*/

	int ret;
//	int i;
//	pd_timing_t * p_table;
//	internal_lvds_context_t * p_lvds = (internal_lvds_context_t *)(p_ctx->internal_lvds);

	PD_DEBUG("ch7036: ch7036_get_timing_list()-enter\n");

	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}


	if ( (g_ch7036_drv.type & PD_DISPLAY_LVDS_INT) || (g_ch7036_drv.type & PD_DISPLAY_LVDS_LHDV) )
	{
		ret = PD_INTERNAL_LVDS_MODULE_GET_TIMING_LIST(ch7036_internal_lvds_get_timing_list, (p_ctx->internal_lvds,p_in_list,pp_out_list));
		PD_ERROR("ch7036: ch7036_get_timing_list()- ret = %X\n", ret);



		p_ctx->p_lvds_table = *pp_out_list;

#ifdef T_PANEL_NATIVE_DTD



		p_ctx->native_dtd = ((internal_lvds_context_t *)(p_ctx->internal_lvds))->native_dtd;
		p_ctx->fp_width = ((internal_lvds_context_t *)(p_ctx->internal_lvds))->fp_width;
		p_ctx->fp_height = ((internal_lvds_context_t *)(p_ctx->internal_lvds))->fp_height;

		if( (p_ctx->fp_width == 0) || (p_ctx->fp_height == 0) )
		{	PD_ERROR("ch7036: ch7036_get_timing_list()- ret = PD_ERR_NO_TIMINGS\n" );
			return PD_ERR_NO_TIMINGS ;
		}
#else

		for(i=0,p_table = *pp_out_list;i< 30;i++)
			{

				if((p_table->width == 1024) && (p_table->height == 768 )&&
					(p_table->refresh == 60) )
				{

					p_ctx->native_dtd =  p_table;
					p_ctx->fp_width = p_table->width;
					p_ctx->fp_height = p_table->height;

					break;
				}

				p_table= (pd_timing_t*)((uint8*)p_table + (sizeof(pd_timing_t)+4));

			}

			p_lvds->native_dtd = p_ctx->native_dtd;
			p_lvds->fp_width = p_ctx->fp_width;
			p_lvds->fp_height = p_ctx->fp_height;

#endif

	}

	PD_ERROR("ch7036: ch7036_get_timing_list()- return ret=%X;\n", ret);
	return ret;

}



int ch7036_get_attributes(void *p_context, unsigned long *p_num_attr,
	pd_attr_t **pp_list)
{
	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	/*int ret;*/

	PD_DEBUG("ch7036: ch7036_get_attributes()-enter\n");

	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}


	if (!p_ctx || !p_num_attr || !pp_list) {
		return PD_ERR_NULL_PTR;
	}

	*pp_list = p_ctx->p_ch7036_attr_table;


	*p_num_attr  = p_ctx->ch7036_num_attrs;

	PD_DEBUG("ch7036: ch7036_get_attributes()- total num_attrs = [%u]\n",*p_num_attr);

	return PD_SUCCESS;
}


int ch7036_set_attributes(void *p_context, unsigned long num_attrs,
	pd_attr_t *p_list)
{
	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;
	pd_list_entry_attr_t* list_item;

	pd_port_status_t port_status;


	pd_attr_t        *p_curr, *p_attr;
	int ret;
	unsigned long i;

	PD_DEBUG("ch7036: ch7036_set_attributes()-enter: num_attrs=%u\n", num_attrs);

	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}


	ret = PD_INTERNAL_LVDS_MODULE_SET_ATTRIBUTES(ch7036_set_attrs, (p_ctx->internal_lvds,num_attrs,p_list));



	if(ret != PD_SUCCESS)
		return ret;



	#ifdef LVDS_ONLY
		return PD_SUCCESS;
	#endif


	if (!p_ctx->init_done) {

		PD_DEBUG("ch7036: ch7036_set_attributes()- at bootup...\n");

		PD_DEBUG("ch7036: ch7036_set_attributes()- p_ctx->hpd [%x}\n", p_ctx->hpd);

		p_attr = pd_get_attr(p_list, num_attrs, PD_ATTR_ID_DISPLAY, 0);
		if (p_attr && (p_attr->flags & PD_ATTR_FLAG_VALUE_CHANGED) ) {


			pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_DISPLAY,
				PD_GET_ATTR_LIST)->current_value
			= p_attr->current_value;

		}



		p_attr = pd_get_attr(p_list, num_attrs, PD_ATTR_ID_HDMI_OUT_MODE, 0);
		p_curr = pd_get_attr(p_ctx->p_ch7036_attr_table, num_attrs, PD_ATTR_ID_HDMI_OUT_MODE, 0);

		if (p_attr && (p_attr->flags & PD_ATTR_FLAG_VALUE_CHANGED) ) {

			if(p_curr) {
				pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_HDMI_OUT_MODE,
					PD_GET_ATTR_LIST)->current_value
				= p_attr->current_value;
				p_curr->flags &= ~PD_ATTR_FLAG_USER_INVISIBLE;
			}


		}


		p_attr = pd_get_attr(p_list, num_attrs, PD_ATTR_ID_DVI_OUT_MODE, 0);
		p_curr = pd_get_attr(p_ctx->p_ch7036_attr_table, num_attrs, PD_ATTR_ID_DVI_OUT_MODE, 0);

		if (p_attr && (p_attr->flags & PD_ATTR_FLAG_VALUE_CHANGED) ) {



			if(p_curr) {
				pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_DVI_OUT_MODE,
					PD_GET_ATTR_LIST)->current_value
				= p_attr->current_value;
				p_curr->flags &= ~PD_ATTR_FLAG_USER_INVISIBLE;
			}


		}


		p_attr = pd_get_attr(p_list, num_attrs, PD_ATTR_ID_CRT_OUT_MODE, 0);
		p_curr = pd_get_attr(p_ctx->p_ch7036_attr_table, num_attrs, PD_ATTR_ID_CRT_OUT_MODE, 0);

		if (p_attr && (p_attr->flags & PD_ATTR_FLAG_VALUE_CHANGED) ) {

			if(p_curr) {
				pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_CRT_OUT_MODE,
						PD_GET_ATTR_LIST)->current_value
					= p_attr->current_value;

				p_curr->flags &= ~PD_ATTR_FLAG_USER_INVISIBLE;
			}


		}



	}


	for (i = 0, p_attr = p_list; i < num_attrs; i++,p_attr++)
	{


		if (!(p_attr->flags & PD_ATTR_FLAG_VALUE_CHANGED)) {

			continue;
		}


		p_attr->flags &= ~PD_ATTR_FLAG_VALUE_CHANGED;

		if (p_attr->flags & PD_ATTR_FLAG_USER_INVISIBLE)
			continue;



		if (p_attr->name == NULL)
		{
			continue;
		}
		if ((p_attr->id != PD_ATTR_ID_DISPLAY)
			&& (p_attr->id != PD_ATTR_ID_HDMI_OUT_MODE)
			&& (p_attr->id != PD_ATTR_ID_DVI_OUT_MODE)
			&& (p_attr->id != PD_ATTR_ID_CRT_OUT_MODE)
			&& (p_attr->id != PD_ATTR_ID_HPOSITION)
			&& (p_attr->id != PD_ATTR_ID_VPOSITION)

			&& (p_attr->id != PD_ATTR_ID_HSCALE)
			&& (p_attr->id !=  PD_ATTR_ID_VSCALE)
			&& (p_attr->id !=  PD_ATTR_ID_HSCALE_CRT)
			&& (p_attr->id !=  PD_ATTR_ID_VSCALE_CRT)
			&& (p_attr->id !=  PD_ATTR_ID_DITHER_BYPASS)
			&& (p_attr->id !=  PD_ATTR_ID_LOAD_FIRMWARE)
			&& (p_attr->id !=  PD_ATTR_ID_REFRESH))
		{
			continue;
		}

		p_curr = pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,p_attr->id,
				PD_GET_ATTR_LIST);


		p_curr->current_value = p_attr->current_value;
		switch (p_attr->id) {
			case PD_ATTR_ID_DISPLAY:

				list_item = (pd_list_entry_attr_t *)pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_DISPLAY,
							PD_GET_ATTR_LIST_ENTRY);

				if(list_item->value & CHANNEL_DVI) {

					pOutput_Info->hdmi_fmt.is_dvi_mode=1;
					list_item->value = (list_item->value & 0xF7) | CHANNEL_HDMI;
				}


				ch7036_set_output_channel(p_ctx,(uint32)list_item->value);

				if (pOutput_Info->channel == CHANNEL_LVDS)


					ch7036_set_power((void *)p_ctx,PD_POWER_MODE_D0);
				else
					p_attr->flags |= PD_ATTR_FLAG_SETMODE;



				break;

			case PD_ATTR_ID_HDMI_OUT_MODE:
				list_item = (pd_list_entry_attr_t *)pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_HDMI_OUT_MODE,
							PD_GET_ATTR_LIST_ENTRY);
				p_ctx->hdmi_mode_index = list_item->value;
				PD_DEBUG("ch7036_set_attributes(): updated hdmi_mode_index is: value [%ld]\n",p_ctx->hdmi_mode_index);
				p_attr->flags |= PD_ATTR_FLAG_SETMODE;
				break;

			case PD_ATTR_ID_DVI_OUT_MODE:
				list_item = (pd_list_entry_attr_t *)pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_DVI_OUT_MODE,
							PD_GET_ATTR_LIST_ENTRY);
				p_ctx->dvi_mode_index = list_item->value;
				PD_DEBUG("ch7036_set_attributes(): updated dvi_mode_index is: value [%ld]\n",p_ctx->dvi_mode_index);
				p_attr->flags |= PD_ATTR_FLAG_SETMODE;
				break;

			case PD_ATTR_ID_CRT_OUT_MODE:
				list_item = (pd_list_entry_attr_t *)pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs,PD_ATTR_ID_CRT_OUT_MODE,
							PD_GET_ATTR_LIST_ENTRY);
				p_ctx->crt_mode_index = list_item->value;
				PD_DEBUG("ch7036_set_attributes(): updated crt_mode_index is: value [%ld]\n",p_ctx->crt_mode_index);
				p_attr->flags |= PD_ATTR_FLAG_SETMODE;
				break;

			case PD_ATTR_ID_HPOSITION:
			case PD_ATTR_ID_VPOSITION:
				if(pOutput_Info->channel & CHANNEL_HDMI)
					p_curr->current_value = DEFAULT_POSITION;
				else {
					if(p_curr->current_value > ((pd_range_attr_t *)p_curr)->max)
						p_curr->current_value = ((pd_range_attr_t *)p_curr)->max;
					else if (p_curr->current_value < ((pd_range_attr_t *)p_curr)->min)
						p_curr->current_value = ((pd_range_attr_t *)p_curr)->min;
				}

				if (p_attr->id == PD_ATTR_ID_HPOSITION)
					pOutput_Info->h_position = (uint16)(p_curr->current_value);
				else
					pOutput_Info->v_position = (uint16)(p_curr->current_value);
				PD_DEBUG("ch7036_set_attributes(): updated position is: value [%d]\n",p_curr->current_value);
				ch7036_set_position(p_ctx, (uint8)p_attr->id, (uint16)(p_curr->current_value));
				break;
			case PD_ATTR_ID_HSCALE:
			case PD_ATTR_ID_VSCALE:
			case PD_ATTR_ID_HSCALE_CRT:
			case PD_ATTR_ID_VSCALE_CRT:

				if(p_curr->current_value > ((pd_range_attr_t *)p_curr)->max)
					p_curr->current_value = ((pd_range_attr_t *)p_curr)->max;
				else if (p_curr->current_value < ((pd_range_attr_t *)p_curr)->min)
					p_curr->current_value = ((pd_range_attr_t *)p_curr)->min;


				if(p_attr->id== PD_ATTR_ID_HSCALE || p_attr->id== PD_ATTR_ID_HSCALE_CRT)
					ch7036_set_scaling (pOutput_Info,PD_ATTR_ID_HSCALE, (uint8)p_curr->current_value);
				else
					ch7036_set_scaling (pOutput_Info,PD_ATTR_ID_VSCALE, (uint8)p_curr->current_value);
				PD_DEBUG("ch7036_set_attributes(): updated scale value is: value [%ld]\n",p_curr->current_value);

				p_attr->flags |= PD_ATTR_FLAG_SETMODE;

				break;
			case PD_ATTR_ID_DITHER_BYPASS:


				PD_DEBUG("ch7036_set_attributes(): updated quality enhance value is: value [%ld]\n",p_curr->current_value);
				ch7036_set_quality_enhancement(p_ctx,(uint8)p_curr->current_value);
				break;

			case PD_ATTR_ID_LOAD_FIRMWARE:
				PD_DEBUG("ch7036_set_attributes(): updated [load-firmware] value is: value [%ld]\n",p_curr->current_value);
#if 0
				if(p_curr->current_value)
					p_ctx->use_firmware =1;
				else
					p_ctx->use_firmware =0;
#endif
				break;

			case PD_ATTR_ID_REFRESH:
				PD_DEBUG("ch7036_set_attributes(): refresh value is: value [%ld]\n",p_curr->current_value);

				ch7036_get_port_status((void *)p_ctx, &port_status);
				p_attr->flags |= PD_ATTR_FLAG_SETMODE;
				if(p_curr->current_value ) {
					p_curr->current_value= 0;

				}
				break;

			default:

				PD_DEBUG("ch7036_set_attr(): unhandled attr name[%s]id[%ld]curr_index[%ld]\n",p_attr->name, p_attr->id,p_attr->current_value);
				break;

		}


	}



	PD_DEBUG("ch7036: ch7036_set_attributes()-exit\n");

	return ret;
}




int ch7036_set_power(void *p_context, unsigned long state)
{
	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	OUTPUT_INFO* pOutput_Info = (p_ctx->p_ch7xxx_context)->pOutput_Info;


	pd_list_entry_attr_t  *list_entry;
	int ret;


	PD_DEBUG("ch7036: ch7036_set_power()-enter: requested state=%x\n", state);

	if (!p_ctx)
		return PD_ERR_NULL_PTR;

	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}

	if (state > PD_POWER_MODE_D3)
		return PD_ERR_INVALID_POWER;

	if (state != PD_POWER_MODE_D0) {


		pOutput_Info->channel = CHANNEL_LVDS_HDMI_VGA_OFF;

		#ifndef LVDS_ONLY
		ch7036_device_set_power(p_ctx,CHANNEL_LVDS_HDMI_VGA_OFF);
		#endif

		ret = PD_INTERNAL_LVDS_MODULE_SET_POWER(ch7036_internal_lvds_set_power, (p_ctx->internal_lvds,state));


	}
	else {


		list_entry = (pd_list_entry_attr_t *)pd_get_attr(p_ctx->p_ch7036_attr_table, p_ctx->ch7036_num_attrs, PD_ATTR_ID_DISPLAY, PD_GET_ATTR_LIST_ENTRY);
		ch7036_set_output_channel(p_ctx,(uint32)(list_entry->value));

		PD_DEBUG("ch7036: ch7036_set_power()- p->ctx-hpd [0x%x]\n",p_ctx->hpd);
		PD_DEBUG("ch7036: ch7036_set_power()- requested output channel- [%x]\n", pOutput_Info->channel);


		PD_INTERNAL_LVDS_MODULE_SET_POWER(ch7036_internal_lvds_set_power, (p_ctx->internal_lvds,state));

		#ifndef LVDS_ONLY
		ch7036_device_set_power(p_ctx,pOutput_Info->channel);
		#endif
	}


	p_ctx->pwr_state = state;

	return PD_SUCCESS;
}


int ch7036_get_power(void *p_context, unsigned long *p_state)
{
	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;


	PD_DEBUG("ch7036: ch7036_get_power()\n");


	*p_state = p_ctx->pwr_state;

	return PD_SUCCESS;
}

int ch7036_save(void *p_context, void **state, unsigned long flags)
{

	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;


	PD_DEBUG("ch7036: ch7036_save()\n");


	p_ctx->prev_outchannel = ch7036_get_output_channel(p_context);



	*state = NULL;

	return PD_SUCCESS;

}

int ch7036_restore(void *p_context, void *state, unsigned long flags)
{

	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)p_context;
	unsigned long i;


	PD_DEBUG("ch7036: ch7036_restore()\n");


	if (ch7036_load_firmware(p_ctx) != SS_SUCCESS)   {
		PD_DEBUG("ch7036: ch7036_restore()-  load fw is NOT a SUCCESS\n");
		return PD_ERR_UNSUCCESSFUL;

	}
	else {
		PD_DEBUG("ch7036: ch7036_restore()-  load fw is a SUCCESS\n");
	}


	if(p_ctx->prev_outchannel == CHANNEL_LVDS_HDMI) {

		ch7036_set_output_channel(p_context, p_ctx->prev_outchannel);
		ch7036_set_mode(p_context, &(p_ctx->native_dtd), 0);
		ch7036_post_set_mode(p_context, &(p_ctx->native_dtd), 0);
	}



	for(i=0;i<p_ctx->ch7036_num_attrs;i++) {

		if( p_ctx->p_ch7036_attr_table[i].id == PD_ATTR_ID_REFRESH ) {


			p_ctx->p_ch7036_attr_table[i].flags |= PD_ATTR_FLAG_VALUE_CHANGED;
			p_ctx->p_ch7036_attr_table[i].current_value = 1;

			ch7036_set_attributes(p_context, 1, &p_ctx->p_ch7036_attr_table[i]);
			break;
		}

	}


	return PD_SUCCESS;

}


int ch7036_get_port_status(void *context, pd_port_status_t *port_status)
{
	ch7036_device_context_t *p_ctx = (ch7036_device_context_t *)context;
//	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
//	OUTPUT_INFO* pOutput_Info = (p_ctx->p_ch7xxx_context)->pOutput_Info;



	port_status->display_type = PD_DISPLAY_LVDS_INT;
	port_status->connected    = PD_DISP_STATUS_UNKNOWN;

	PD_DEBUG("ch7036: ch7036_get_port_status()-enter..\n");


	if(p_ctx->use_firmware) {

		ch7036_get_attached_device(p_ctx);

		if(p_ctx ->hpd & 0x22)
			port_status->connected = PD_DISP_STATUS_ATTACHED;


		ch7036_alter_display_list(p_ctx);

	}


	PD_DEBUG("ch7036: ch7036_get_port_status()-exit. p_ctx->hpd [%x}\n", p_ctx->hpd);


	return PD_SUCCESS;

}


int ch7036_initialize_device(ch7036_device_context_t *p_ctx)
{
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	OUTPUT_INFO* pOutput_Info = (p_ctx->p_ch7xxx_context)->pOutput_Info;
	PREFER_INFO* pPrefer_Info = p_ch7xxx_context->pPrefer_Info;
	//pd_attr_t *p_attr;
	uint8 reg;

	ch7036_edid_blk_t* p_hedid;
	ch7036_edid_blk_t* p_cedid ;


	PD_DEBUG("ch7036: ch7036_initialize_device()- ENTER...\n");


	if (p_ch7xxx_context->DeviceID != 0x56)
	{
		return (PD_ERR_NULL_PTR);
	}
	p_ctx->init_done = 0;


	if(p_ctx->hedid) {
		p_hedid = (ch7036_edid_blk_t *)p_ctx->hedid;
		p_cedid = (ch7036_edid_blk_t *)p_ctx->cedid;

		p_cedid->is_edid = 0;
		p_hedid->is_edid = 0;
		p_cedid->ebn = 0;
		p_hedid->ebn = 0;

	}



	pOutput_Info->hdmi_fmt.is_dvi_mode = 0;

	pOutput_Info->channel = CHANNEL_LVDS | CHANNEL_HDMI;
	p_ctx->req_ddc = 0;
	p_ctx->lvds_only = 0;

	PD_DEBUG("ch7036: ch7036_initialize_device()- set output channel to [%ld]\n",pOutput_Info->channel);



	pOutput_Info->h_position = DEFAULT_POSITION;
	pOutput_Info->v_position = DEFAULT_POSITION;


	p_ctx->dither_select = DITHER_18_TO_18;

	ch7036_set_scaling (pOutput_Info,PD_ATTR_ID_HSCALE, HDMI_DEFAULT_UNDERSCAN);
	ch7036_set_scaling (pOutput_Info,PD_ATTR_ID_VSCALE, HDMI_DEFAULT_UNDERSCAN);

	ch7036_set_prefer_timing_info(p_ctx,pPrefer_Info);


	if(p_ctx->use_firmware) {


		I2CWrite(p_ch7xxx_context,0x03, 0x04);
		reg = I2CRead(p_ch7xxx_context,0x52);
		reg = reg & 0xEF;
		I2CWrite(p_ch7xxx_context,0x52, reg);


		I2CWrite(p_ch7xxx_context,0x03, 0x01);
		reg = I2CRead(p_ch7xxx_context,0x0F);
		reg = reg & 0x7F;
		I2CWrite(p_ch7xxx_context,0x0F, reg);


		ch7036_get_attached_device(p_ctx);

	}


	return PD_SUCCESS;
}

