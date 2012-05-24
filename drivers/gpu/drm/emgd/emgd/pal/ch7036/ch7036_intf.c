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
* @file  ch7036_intf.c
* @version 1.1.4
*-----------------------------------------------------------------------------
*/

#include "ch7036_intf.h"
#include "ch7036_attr.h"
#include "ch7036_fw.h"





OUT_FMT hdmi_timing_table[OUT_HDMI_END] = {


	{1,     25175,  1, {800,   640, 16,  96, 525,  480, 10,  2, 59, SCANTYPE_PROGRESSIVE} },
    {1,     25250,  1, {800,   640, 16,  96, 525,  480, 10,  2, 60, SCANTYPE_PROGRESSIVE} },
    {2,     27000,  1, {858,   720, 16,  62, 525,  480,  9,  6, 59, SCANTYPE_PROGRESSIVE} },
    {2,     27027,  1, {858,   720, 16,  62, 525,  480,  9,  6, 60, SCANTYPE_PROGRESSIVE} },

    {4,     74176,  2, {1650, 1280, 110, 40, 750,  720,  5,  5, 59, SCANTYPE_PROGRESSIVE} },
    {4,     74250,  2, {1650, 1280, 110, 40, 750,  720,  5,  5, 60, SCANTYPE_PROGRESSIVE} },
    {5,     74176,  2, {2200, 1920, 88,  44, 1125, 1080, 2,  5, 59, SCANTYPE_INTERLACED} },
    {5,     74250,  2, {2200, 1920, 88,  44, 1125, 1080, 2,  5, 60, SCANTYPE_INTERLACED} },


   {16,    148350,  2, {2200, 1920, 88,  44, 1125, 1080, 4,  5, 59, SCANTYPE_PROGRESSIVE} },
   {16,    148500,  2, {2200, 1920, 88,  44, 1125, 1080, 4,  5, 60, SCANTYPE_PROGRESSIVE} },

   {17,     27000,  1, {864,  720,  12,  64, 625,  576,  5,  5, 50, SCANTYPE_PROGRESSIVE} },
   {19,     74250,  2, {1980, 1280, 440, 40, 750,  720,  5,  5, 50, SCANTYPE_PROGRESSIVE} },

   {20,     74250,  2, {2640, 1920, 528, 44, 1125, 1080, 2, 5, 50, SCANTYPE_INTERLACED} },


   {31,    148500,  2, {2640, 1920, 528, 44, 1125, 1080, 4,  5, 50, SCANTYPE_PROGRESSIVE} },
   {32,     74175,  2, {2750, 1920, 638, 44, 1125, 1080, 4,  5, 23, SCANTYPE_PROGRESSIVE} },
   {32,     74250,  2, {2750, 1920, 638, 44, 1125, 1080, 4,  5, 24, SCANTYPE_PROGRESSIVE} },
   {33,     74250,  2, {2640, 1920, 528, 44, 1125, 1080, 4,  5, 25, SCANTYPE_PROGRESSIVE} },
   {34,     74175,  2, {2200, 1920, 88,  44, 1125, 1080, 4,  5, 29, SCANTYPE_PROGRESSIVE} },
   {34,     74250,  2, {2200, 1920, 88,  44, 1125, 1080, 4,  5, 30, SCANTYPE_PROGRESSIVE} },

   {40,    148500,  2, {2640, 1920, 528, 44, 1124, 1080, 4, 10, 100, SCANTYPE_INTERLACED} },
   {41,    148500,  2, {1980, 1280, 440, 40, 750,  720,  5,  5, 100, SCANTYPE_PROGRESSIVE} },
   {42,     54000,  1, {864,   720, 12,  64, 625,  576,  5,  5, 100, SCANTYPE_PROGRESSIVE} },


   {46,    148352,  2, {2200, 1920, 88,  44, 1124, 1080, 4, 10, 119, SCANTYPE_INTERLACED} },
   {46,    148500,  2, {2200, 1920, 88,  44, 1124, 1080, 4, 10, 120, SCANTYPE_INTERLACED} },

   {47,    148352,  2, {1650, 1280, 110, 40, 750,  720,  5,  5, 119, SCANTYPE_PROGRESSIVE} },
   {47,    148500,  2, {1650, 1280, 110, 40, 750,  720,  5,  5, 120, SCANTYPE_PROGRESSIVE} },

   {48,     54000,  1, {858,   720, 16,  62, 525,  480,  9,  6, 119, SCANTYPE_PROGRESSIVE} },
   {48,     54054,  1, {858,   720, 16,  62, 525,  480,  9,  6, 120, SCANTYPE_PROGRESSIVE} },

   {52,    108000,  1, {864,   720, 12,  64, 625,  576,  5,  5, 200, SCANTYPE_PROGRESSIVE} },

   {56,    108000,  1, {858,   720, 16,  62, 525,  480,  9,  6, 239, SCANTYPE_PROGRESSIVE} },
   {56,    108108,  1, {858,   720, 16,  62, 525,  480,  9,  6, 240, SCANTYPE_PROGRESSIVE} },

};

OUT_FMT dvi_timing_table[OUT_DVI_END] = {



	{0,     25170,  0, { 800,  640, 16,  96, 525,  480, 10,  2,  60, SCANTYPE_PROGRESSIVE}	},
	{0,     31500,  0, { 832,  640, 24,  40, 520,  480,  9,  3,  72, SCANTYPE_PROGRESSIVE}	},
    {0,     31500,  0, { 800,  640, 16,  96, 525,  480, 11,  2,  75, SCANTYPE_PROGRESSIVE}	},

	{0,     28322,  0, { 900,  720, 15, 108, 449,  400, 11,  2,  70, SCANTYPE_PROGRESSIVE}	},

	{0,		38100,	0, {1088,  800,	32, 128, 619,  600,  1,  4,  56, SCANTYPE_PROGRESSIVE}	},


	{0,		40000,	0, {1056,  800,	40, 128, 628,  600,  1,  4,  60, SCANTYPE_PROGRESSIVE}	},
	{0,		50000,	0, {1040,  800,	56, 120, 666,  600, 37,  6,  72, SCANTYPE_PROGRESSIVE}	},
	{0,		49500,	0, {1056,  800,	16,  80, 624,  600,  1,  2,  75, SCANTYPE_PROGRESSIVE}	},

	{0,		65000,	0, {1344, 1024,	24,	136, 806,  768,	 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},
    {0,		75000,	0, {1328, 1024,	24,	136, 806,  768,	 3,	 6,  70, SCANTYPE_PROGRESSIVE}	},
	{0,		78750,	0, {1312, 1024,	16,	 96, 800,  768,	 1,	 3,  75, SCANTYPE_PROGRESSIVE}	},

	{0,		81517,	0, {1688, 1152,	48,112,1066,  864,	 1,	 3,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		83460,	0, {1680, 1280,	64,	136, 828,  720,  1,	 3,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		83865,	0, {1680, 1280,	64,	136, 828,  800,  1,	 3,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		100638,	0, {1688, 1280,	48,	112, 1066, 960,  1,	 3,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		108000,	0, {1688, 1280,	48,	112, 1066, 1024, 1,	 3,  60, SCANTYPE_PROGRESSIVE}	},
	{0,		135000,	0, {1688, 1280,	16,	144, 1066, 1024, 1,	 3,  75, SCANTYPE_PROGRESSIVE}	},

	{0,		85543,	0, {2256, 1360,104, 184, 1087, 768,  1,	 3,  60, SCANTYPE_PROGRESSIVE}	},
	{0,		85920,	0, {2256, 1366,104, 184, 1087, 768,  1,	 3,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		121750,	0, {1864, 1400,	88,144,1089,  1050,	 3,	 4,  60, SCANTYPE_PROGRESSIVE}	},
    {0,		156000,	0, {1896, 1400,104,144,1099,  1050,	 3,	 4,  75, SCANTYPE_PROGRESSIVE}	},


	{0,		88750,	0, {1600, 1440,	48,	32,	 926,  900,	 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},
	{0,		119000,	0, {1840, 1440,	48,	32,	 1080, 1050, 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		117936,	0, {2256, 1600,104,	184, 1250,  900, 1,	 3,  60, SCANTYPE_PROGRESSIVE}	},
	{0,		162000,	0, {2160, 1600,	64,	192, 1250, 1200, 1,	 3,  60, SCANTYPE_PROGRESSIVE}	},

	{0,		147130,	0, {2256, 1680,104,	184, 1087, 1050, 1,	 3,  60, SCANTYPE_PROGRESSIVE}	},
	{0,     148500, 0, {2200, 1920, 88,  44, 1125, 1080, 4,  5, 60, SCANTYPE_PROGRESSIVE} },
	{0,		154000,	0, {2080, 1920,	48,	 32, 1235, 1200, 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},

};


OUT_FMT ch_vga_timing_table[OUT_CRT_END] = {


   {100,    31500,  1, {832,   640, 32,  64,  445,  400, 1,  3,  85, SCANTYPE_PROGRESSIVE}  },

   {100,    25175,  1, {800,   640,  8,  96,  525,  480, 2,  2,  60, SCANTYPE_PROGRESSIVE}  },
   {100,    31500,  1, {832,   640, 16,  40,  520,  480, 1,  3,  72, SCANTYPE_PROGRESSIVE}  },
   {100,    31500,  1, {840,   640, 16,  64,  500,  480, 1,  3,  75, SCANTYPE_PROGRESSIVE}  },
   {100,    36000,  1, {832,   640, 56,  56,  509,  480, 1,  3,  85, SCANTYPE_PROGRESSIVE}  },

   {100,    35500,  1, {936,   720, 36,  72,  446,  400, 1,  3,  85, SCANTYPE_PROGRESSIVE}  },



   {100,    36000,  1, {1024,  800, 24,  72,  625,  600, 1,  2,  56, SCANTYPE_PROGRESSIVE}  },


   {100,    38250,  1, {1024,  800, 32,  80,  624,  600, 3,  4,  60, SCANTYPE_PROGRESSIVE}  },
   {100,    50000,  1, {1040,  800, 56, 120,  666,  600, 37, 6,  72, SCANTYPE_PROGRESSIVE}  },
   {100,    49000,  1, {1040,  800, 40,  80,  629,  600, 3,  4,  75, SCANTYPE_PROGRESSIVE}  },
   {100,    56750,  1, {1056,  800, 48,  80,  633,  600, 3,  4,  85, SCANTYPE_PROGRESSIVE}  },


   {100,    65000,  1, {1344, 1024, 24, 136,  806,  768, 3,  6,  60, SCANTYPE_PROGRESSIVE}  },
   {100,    75000,  1, {1328, 1024, 24, 136,  806,  768, 3,  6,  70, SCANTYPE_PROGRESSIVE}  },
   {100,    78750,  1, {1312, 1024, 16,  96,  800,  768, 1,  3,  75, SCANTYPE_PROGRESSIVE}  },

   {100,    94500,  1, {1376, 1024, 48,  96,  808,  768, 1,  3,  85, SCANTYPE_PROGRESSIVE}  },


   {100,   108000,  1, {1600, 1152, 64, 128,  900,  864, 1,  2,  75, SCANTYPE_PROGRESSIVE}  },


   {100,    79500,  1, {1664, 1280, 64, 128,  798,  768, 3,  7,  60, SCANTYPE_PROGRESSIVE}  },
   {100,   102250,  1, {1696, 1280, 80, 128,  805,  768, 3,  7,  75, SCANTYPE_PROGRESSIVE}  },
   {100,   117500,  1, {1712, 1280, 80, 136,  809,  768, 3,  7,  85, SCANTYPE_PROGRESSIVE}  },


   {100,   108000,  1, {1800, 1280, 96, 112, 1000,  960, 1,  3,  60, SCANTYPE_PROGRESSIVE}  },
   {100,   148500,  1, {1728, 1280, 64, 160, 1011,  960, 1,  3,  85, SCANTYPE_PROGRESSIVE}  },


   {100,   108000,  1, {1688, 1280, 48, 112, 1066, 1024, 1,  3,  60, SCANTYPE_PROGRESSIVE}  },
   {100,   135000,  1, {1688, 1280, 16, 144, 1066, 1024, 1,  3,  75, SCANTYPE_PROGRESSIVE}  },
   {100,   157500,  1, {1728, 1280, 64, 160, 1072, 1024, 1,  3,  85, SCANTYPE_PROGRESSIVE}  },


   {100,    85500,	1, {1792, 1360,	64, 112,  795,  768, 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},


   {100,   121750,	1, {1864, 1400,	88,144,1089,  1050,	 3,	 4,  60, SCANTYPE_PROGRESSIVE}	},
   {100,   156000,	1, {1896, 1400,104,144,1099,  1050,	 3,	 4,  75, SCANTYPE_PROGRESSIVE}	},


   {100,	88750,	1, {1600, 1440,	48,	32,	 926,  900,	 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},


   {100,   119000,	1, {1840, 1440,	48,	32,	 1080, 1050, 3,	 6,  60, SCANTYPE_PROGRESSIVE}	},


   {100,   117936,	1, {2256, 1600,104,	184, 1250,  900, 1,	 3,  60, SCANTYPE_PROGRESSIVE}	},


   {100,   162000,  1, {2160, 1600, 64, 192, 1250, 1200, 1,  3,  60, SCANTYPE_PROGRESSIVE}  },



   {100,   148500, 1, {2200, 1920, 88,  44, 1125, 1080, 4,  5, 60, SCANTYPE_PROGRESSIVE} },



};


uint8 I2CRead(DEV_CONTEXT* pDevContext,uint8 index)
{

	ch7036_device_context_t *p_ctx= pDevContext->pd_context;
	pd_reg_t reg_list[2];



	reg_list[0].reg = (i2c_reg_t)index;
	reg_list[1].reg = PD_REG_LIST_END;

	p_ctx->p_callback->read_regs(p_ctx->p_callback->callback_context, reg_list,PD_REG_DDC_FW);

	return (uint8)(reg_list[0].value);
}

void I2CWrite(DEV_CONTEXT* pDevContext,uint8 index, uint8 value)
{
	ch7036_device_context_t *p_ctx= pDevContext->pd_context;
	pd_reg_t reg_list[2];



	reg_list[0].reg = (i2c_reg_t)index;
	reg_list[0].value = (i2c_reg_t)value;

	reg_list[1].reg = PD_REG_LIST_END;

	p_ctx->p_callback->write_regs(p_ctx->p_callback->callback_context, reg_list,PD_REG_DDC_FW);

	return;
}


void I2CBlockWrite(DEV_CONTEXT* pDevContext,uint8 index, uint8* value, uint16 len)
{
	ch7036_device_context_t *p_ctx= pDevContext->pd_context;
	pd_reg_t reg_list[33];
	uint16 i=0;



	for(i=0;i<len;i++) {
		reg_list[i].reg = (i2c_reg_t)index;
		reg_list[i].value = (i2c_reg_t)value[i];
	}

	reg_list[len].reg = PD_REG_LIST_END;

	p_ctx->p_callback->write_regs(p_ctx->p_callback->callback_context, reg_list,PD_REG_DDC_FW);

	return;
}

ch7036_status_t ch7036_device_prepare(ch7036_device_context_t* p_ctx)
{
	DEV_CONTEXT* p_ch_ctx = p_ctx->p_ch7xxx_context;
	ch7036_status_t status = SS_SUCCESS;

	PD_DEBUG("ch7036_intf: ch7036_device_prepare()\n");


	if(!DevicePrepare(p_ch_ctx))
	{
		p_ctx->last_emsg = GetLastErrorMessage();
		status = SS_UNSUCCESSFUL;
	}

	return status;
}

ch7036_status_t ch7036_device_config(ch7036_device_context_t* p_ctx)
{
	DEV_CONTEXT* p_ch_ctx = p_ctx->p_ch7xxx_context;
	ch7036_status_t status = SS_SUCCESS;

	PD_DEBUG("ch7036_intf: ch7036_device_config()\n");
#if 0
	printk ("p_ch_ctx->DeviceID = 0x%X\n", p_ch_ctx->DeviceID);
	printk ("p_ch_ctx->pInput_Info->timing->ht = 0x%X\n", p_ch_ctx->pInput_Info->timing.ht);
	printk ("p_ch_ctx->pInput_Info->timing->ha = 0x%X\n", p_ch_ctx->pInput_Info->timing.ha);
	printk ("p_ch_ctx->pInput_Info->timing->ho = 0x%X\n", p_ch_ctx->pInput_Info->timing.ho);
	printk ("p_ch_ctx->pInput_Info->timing->hw = 0x%X\n", p_ch_ctx->pInput_Info->timing.hw);
	printk ("p_ch_ctx->pInput_Info->timing->vt = 0x%X\n", p_ch_ctx->pInput_Info->timing.vt);
	printk ("p_ch_ctx->pInput_Info->timing->va = 0x%X\n", p_ch_ctx->pInput_Info->timing.va);
	printk ("p_ch_ctx->pInput_Info->timing->vo = 0x%X\n", p_ch_ctx->pInput_Info->timing.vo);
	printk ("p_ch_ctx->pInput_Info->timing->vw = 0x%X\n", p_ch_ctx->pInput_Info->timing.vw);
	printk ("p_ch_ctx->pInput_Info->timing->hz = 0x%X\n", p_ch_ctx->pInput_Info->timing.hz);
	printk ("p_ch_ctx->pInput_Info->timing->stype = 0x%X\n", p_ch_ctx->pInput_Info->timing.stype);
	printk ("p_ch_ctx->pInput_Info->rx_clk_khz = 0x%X\n", p_ch_ctx->pInput_Info->rx_clk_khz);
	printk ("p_ch_ctx->pInput_Info->pixel_fmt = 0x%X\n", p_ch_ctx->pInput_Info->pixel_fmt);
	printk ("p_ch_ctx->pInput_Info->hs_pol = 0x%X\n", p_ch_ctx->pInput_Info->hs_pol);
	printk ("p_ch_ctx->pInput_Info->vs_pol = 0x%X\n", p_ch_ctx->pInput_Info->vs_pol);
	printk ("p_ch_ctx->pInput_Info->de_pol = 0x%X\n", p_ch_ctx->pInput_Info->de_pol);
	printk ("p_ch_ctx->pInput_Info->data_ch_pol = 0x%X\n", p_ch_ctx->pInput_Info->data_ch_pol);
	printk ("p_ch_ctx->pInput_Info->data_ch_invert = 0x%X\n", p_ch_ctx->pInput_Info->data_ch_invert);
	printk ("p_ch_ctx->pInput_Info->audio_type = 0x%X\n", p_ch_ctx->pInput_Info->audio_type);
	printk ("p_ch_ctx->pInput_Info->i2s_pol = 0x%X\n", p_ch_ctx->pInput_Info->i2s_pol);
	printk ("p_ch_ctx->pInput_Info->i2s_len = 0x%X\n", p_ch_ctx->pInput_Info->i2s_len);
	printk ("p_ch_ctx->pInput_Info->i2s_fmt = 0x%X\n", p_ch_ctx->pInput_Info->i2s_fmt);

	printk ("\n\n");

	printk ("p_ch_ctx->pOutput_Info->channel = 0x%X\n", p_ch_ctx->pOutput_Info->channel );
	printk ("p_ch_ctx->pOutput_Info->uclk_khz = 0x%X\n", p_ch_ctx->pOutput_Info->uclk_khz );
	printk ("p_ch_ctx->pOutput_Info->ds_percent_h = 0x%X\n", p_ch_ctx->pOutput_Info->ds_percent_h );
	printk ("p_ch_ctx->pOutput_Info->ds_percent_v = 0x%X\n", p_ch_ctx->pOutput_Info->ds_percent_v );
	printk ("p_ch_ctx->pOutput_Info->rotate = 0x%X\n", p_ch_ctx->pOutput_Info->rotate );
	printk ("p_ch_ctx->pOutput_Info->h_flip = 0x%X\n", p_ch_ctx->pOutput_Info->h_flip );
	printk ("p_ch_ctx->pOutput_Info->v_flip = 0x%X\n", p_ch_ctx->pOutput_Info->v_flip );
	printk ("p_ch_ctx->pOutput_Info->h_position = 0x%X\n", p_ch_ctx->pOutput_Info->h_position );
	printk ("p_ch_ctx->pOutput_Info->v_position = 0x%X\n", p_ch_ctx->pOutput_Info->v_position );
#endif
	if(!DeviceConfig(p_ch_ctx))
	{
		p_ctx->last_emsg = GetLastErrorMessage();
		status = SS_UNSUCCESSFUL;
	}

	return status;

}

ch7036_status_t ch7036_device_start(ch7036_device_context_t* p_ctx)
{

	DEV_CONTEXT* p_ch_ctx = p_ctx->p_ch7xxx_context;
	ch7036_status_t status = SS_SUCCESS;

	PD_DEBUG("ch7036_intf: ch7036_device_start()\n");

	if(!DeviceRunning(p_ch_ctx))
	{
		p_ctx->last_emsg = GetLastErrorMessage();
		status = SS_UNSUCCESSFUL;
	}

	return status;

}

ch7036_status_t ch7036_device_set_power(ch7036_device_context_t* p_ctx, unsigned long channel)
{
	DEV_CONTEXT* p_ch_ctx = p_ctx->p_ch7xxx_context;
	ch7036_status_t status = SS_SUCCESS;

	PD_DEBUG("ch7036_intf: ch7036_device_set_power()- channel [%ld]\n", channel);

	if(!DeviceSetPower(p_ch_ctx,channel))
	{
		p_ctx->last_emsg = GetLastErrorMessage();
		status = SS_UNSUCCESSFUL;
	}

	return status;
}



ch7036_status_t ch7036_load_firmware(ch7036_device_context_t* p_ctx)
{

	DEV_CONTEXT* p_ch_ctx = p_ctx->p_ch7xxx_context;

	/*unsigned fs1;*/
	/*uint8 ch;*/
	ch7036_status_t status = SS_UNSUCCESSFUL;

	PD_DEBUG("ch7036: ch7036_load_firmware()\n");


	if(LHFM_load_firmware(p_ch_ctx) == -1) {

		PD_DEBUG("ch7036_load_firmware: LHFM_load_firmware()- firmware loading FAILED\n");
		p_ctx->last_emsg = GetLastErrorMessage();

	}
	else  {
		PD_DEBUG("ch7036_load_firmware: LHFM_load_firmware()- firmware loading is a SUCCESS\n");
		status = SS_SUCCESS;
	}


	return status;


}


ch7036_status_t ch7036_get_hdvi_display_modes_supported(ch7036_device_context_t* p_ctx)
{
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	ch7036_status_t status;
	ch7036_edid_blk_t* p_hedid = (ch7036_edid_blk_t *)p_ctx->hedid;


	PD_DEBUG("ch7036_get_hdvi_display_modes_supported()- enter\n");

	status = LHFM_get_hdmi_modeinfo(p_ch7xxx_context,p_hedid->supported_modes);

	if (status == SS_SUCCESS) {

		PD_DEBUG("HDMI_Modes=%02X. Vesa_Modes=%02x\r\n", p_hedid->supported_modes[13], p_hedid->supported_modes[14]);
		ch7036_dump("Prefered Mode Timing", 13, p_hedid->supported_modes);


	}
	else {
		PD_DEBUG("ch7036_get_hdvi_display_modes_supported()-- failed!\r\n");
		PD_DEBUG("status: [%s]\n",status == SS_FIRMWARE_TIMEOUT?"timeout!":"firmware_error!");


	}

	return status;
}


ch7036_status_t ch7036_read_edid(ch7036_device_context_t* p_ctx)
{

	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;
	ch7036_status_t status = SS_UNSUCCESSFUL;
	/*uint8 ebn;*/

	/*int i;*/

	ch7036_edid_blk_t* p_hedid = (ch7036_edid_blk_t *)p_ctx->hedid;
	ch7036_edid_blk_t* p_cedid = (ch7036_edid_blk_t *)p_ctx->cedid;

	/*ch7036_attr_list_header_t* p_list_header;*/

	unsigned char* hedidblk = p_hedid->edidblk;
	unsigned char* cedidblk = p_cedid->edidblk;




	if((p_ctx->hpd & 0x22) == 0) {

		p_ctx->hpd &= 0xCC;
		p_hedid->is_edid =0;
		p_cedid->is_edid =0;
		p_hedid->ebn = 0;
		p_cedid->ebn = 0;


		return status;

	}


	if( (p_ctx->hpd & 0x20) == 0x20  ) {


		status = LHFM_get_edid(p_ch7xxx_context,hedidblk, &(p_hedid->ebn), CH7036_HDMI_DDC);

		if(status == SS_SUCCESS) {

			PD_DEBUG("ch7036_read_edid()- attached, hdmi-dvi edid read is a SUCCESS\n");
			PD_DEBUG("ch7036_read_edid()- number of blocks read [%x]\n", p_hedid->ebn);

			p_hedid->is_edid = 1;
			if(p_hedid->ebn == 1)
				pOutput_Info->hdmi_fmt.is_dvi_mode =1;
			else
				pOutput_Info->hdmi_fmt.is_dvi_mode =0;
		}


		else {
				p_hedid->is_edid = 0;
				PD_DEBUG("ch7036_read_edid()- attached, hdmi-dvi edid read is UNSUCCESSFUL\n");

		}

	}



	else  {
		PD_DEBUG("ch7036_read_edid()- hdmi HPD status has-or has NOT-  changed, and not attached- no edid needed...\n");
		p_hedid->is_edid =0;

	}





	if( (p_ctx->hpd & 0x02) == 0x02) {
		status = LHFM_get_edid(p_ch7xxx_context,cedidblk, &(p_cedid->ebn), CH7036_VGA_DDC);
		if (status== SS_SUCCESS ) {
			p_cedid->is_edid =1;
			PD_DEBUG("ch7036_read_edid()- attached, crt edid read is a SUCCESS\n");
			PD_DEBUG("ch7036_read_edid()- number of blocks read [%x]\n", p_cedid->ebn);

		}
		else {
			p_cedid->is_edid =0;
			PD_DEBUG("ch7036_read_edid()- crt edid read is UNSUCCESSFUL\n");
		}

	}
	else {
		PD_DEBUG("ch7036_read_edid()- crt HPD status has-or has NOT-  changed, and not attached- no edid needed...\n");
		p_cedid->is_edid =0;
	}

	if( (p_hedid->is_edid == 0) && (p_cedid->is_edid ==0) )
		return SS_UNSUCCESSFUL;
	else
		return SS_SUCCESS;
}


ch7036_status_t ch7036_get_attached_device(ch7036_device_context_t* p_ctx)
{
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	/*OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;*/
	uint8 reg;
	ch7036_status_t status;

	uint8 hpd;
	/*uint8 ebn;*/


	I2CWrite(p_ch7xxx_context,0x03, 0x04);
	reg = I2CRead(p_ch7xxx_context,0x52);
	reg = reg & 0xEF;
	I2CWrite(p_ch7xxx_context,0x52, reg);


	I2CWrite(p_ch7xxx_context,0x03, 0x01);
	reg = I2CRead(p_ch7xxx_context,0x0F);
	reg = reg & 0x7F;
	I2CWrite(p_ch7xxx_context,0x0F, reg);


	LHFM_enable_crt_hpd(p_ch7xxx_context);



	status = LHFM_get_hdmi_hpd(p_ch7xxx_context, &hpd);



	if(status == SS_SUCCESS) {


		if( hpd == 0x81 ) {
			if(p_ctx->hpd & CH7036HPD_HDVI_ATTACHED)  {
				p_ctx->hpd = (p_ctx->hpd  & 0x0F) & (~CH7036HPD_HDVI_STATUS_CHANGED) | CH7036HPD_HDVI_ATTACHED | CH7036HPD_HDVI_HPD_STATUS;
				PD_DEBUG("ch7036_get_attached_device()- hdvi HPD status has not changed since last query and it's HIGH\n");
			}
			else {

				p_ctx->hpd = (p_ctx->hpd  & 0x0F) | CH7036HPD_HDVI_STATUS_CHANGED | CH7036HPD_HDVI_ATTACHED | CH7036HPD_HDVI_HPD_STATUS;
				PD_DEBUG("ch7036_get_attached_device()- hdvi HPD status changed since last query and it's HIGH\n");
			}
		}
		else if (hpd ==0x80) {
			if( (p_ctx->hpd & CH7036HPD_HDVI_ATTACHED) ==0 )  {
				p_ctx->hpd = (p_ctx->hpd  & 0x0F) & (~ CH7036HPD_HDVI_STATUS_CHANGED) & (~CH7036HPD_HDVI_ATTACHED) & (~CH7036HPD_HDVI_HPD_STATUS);
			PD_DEBUG("ch7036_get_attached_device()- hdvi HPD status not changed since last query and it's LOW\n");
			}
			else {
				p_ctx->hpd = (p_ctx->hpd  & 0x0F) | CH7036HPD_HDVI_STATUS_CHANGED & (~CH7036HPD_HDVI_ATTACHED) & (~CH7036HPD_HDVI_HPD_STATUS);
				PD_DEBUG("ch7036_get_attached_device()- hdvi HPD status changed since last query and it's LOW\n");
			}

		}
		else if (hpd == 0x01) {
			p_ctx->hpd = (p_ctx->hpd  & 0x0F) & (~CH7036HPD_HDVI_STATUS_CHANGED) | CH7036HPD_HDVI_ATTACHED | CH7036HPD_HDVI_HPD_STATUS;
			PD_DEBUG("ch7036_get_attached_device()- hdvi HPD status has not changed since last query and it's HIGH\n");
		}
		else {
			p_ctx->hpd &= 0x0F;
			PD_DEBUG("ch7036_get_attached_device()- hdvi HPD status has not changed since last query and it's LOW\n");
		}



		PD_DEBUG("ch7036: ch7036_get_attached_device()- SUCCESS- hdmi hpd [0x%x]\n", hpd);



	}
	else {
		PD_DEBUG("ch7036: ch7036_get_attached_device()- NOT SUCCESS- hdmi hpd [0x%x]\n", hpd);
		PD_DEBUG("status: [%s]\n",status == SS_FIRMWARE_TIMEOUT?"timeout!":"firmware_error!");


		if ( (!p_ctx->init_done) &&  (hpd == 0x86) )  {

			p_ctx->hpd |= CH7036HPD_HDVI_ATTACHED;
			PD_DEBUG("ch7036_get_attached_device()- special case when status is unsuccessful, it's attached...\n");

		}

		else
			p_ctx->hpd &= ~CH7036HPD_HDVI_ATTACHED;



		PD_DEBUG("ch7036_get_attached_device()- not attached\n");
		PD_DEBUG("ch7036_get_attached_device()- p_ctx-hpd [0x%x]\n",p_ctx->hpd);
	}





	status = LHFM_get_crt_hpd(p_ch7xxx_context);


	if(status == SS_SUCCESS) {
		if( (p_ctx->hpd & CH7036HPD_CRT_ATTACHED )== CH7036HPD_CRT_ATTACHED) {

				p_ctx->hpd = ((p_ctx->hpd  & 0xF0) & (~CH7036HPD_CRT_STATUS_CHANGED) ) | CH7036HPD_CRT_ATTACHED | CH7036HPD_CRT_HPD_STATUS;
				PD_DEBUG("ch7036_get_attached_device()- crt HPD status has NOT changed since last query and it's attached\n");
				PD_DEBUG("ch7036_get_attached_device()- p_ctx-hpd [0x%x]\n",p_ctx->hpd);
		}
		else {

			p_ctx->hpd = (p_ctx->hpd  & 0xF0) | CH7036HPD_CRT_STATUS_CHANGED | CH7036HPD_CRT_ATTACHED | CH7036HPD_CRT_HPD_STATUS;

			PD_DEBUG("ch7036_get_attached_device()- crt HPD status has changed since last query and it's attached\n");
			PD_DEBUG("ch7036_get_attached_device()- p_ctx-hpd [0x%x]\n",p_ctx->hpd);

		}


	}
	else  {

		if( (p_ctx->hpd & CH7036HPD_CRT_ATTACHED ) == 0 ) {
			p_ctx->hpd = p_ctx->hpd & 0xF8;
			PD_DEBUG("ch7036_get_attached_device()- crt HPD not changed, connected to ground- not attached\n");
			PD_DEBUG("ch7036_get_attached_device()- p_ctx-hpd [0x%x]\n",p_ctx->hpd);
		}
		else {

			p_ctx->hpd = (p_ctx->hpd & 0xF8) |  CH7036HPD_CRT_STATUS_CHANGED;
			PD_DEBUG("ch7036_get_attached_device()- crt HPD status changed, connected to ground- not attached, LOW\n");
			PD_DEBUG("ch7036_get_attached_device()- p_ctx-hpd [0x%x]\n",p_ctx->hpd);

		}


	}


	return SS_SUCCESS;
}


void ch7036_reset(ch7036_device_context_t* p_ctx)
{
	DEV_CONTEXT* p_ch_ctx = p_ctx->p_ch7xxx_context;

	ch7036_reset_mcu(p_ch_ctx);
	ch7036_reset_datapath(p_ch_ctx);

	return;
}

void ch7036_reset_datapath(DEV_CONTEXT* p_ch_ctx)
{
	uint8 reg=0x00;
	PD_DEBUG("ch7036: ch7036_reset_datapath()- enter\n");

    I2CWrite(p_ch_ctx,0x03, 0x04);
	reg = I2CRead(p_ch_ctx,0x52);
	reg = reg & 0xFE;

	I2CWrite(p_ch_ctx,0x52, reg);


	I2CWrite(p_ch_ctx,0x52, 0x2F);

	return;
}

void ch7036_reset_mcu(DEV_CONTEXT* p_ch_ctx)
{
	uint8 reg=0x00;

    I2CWrite(p_ch_ctx,0x03, 0x04);
	reg = I2CRead(p_ch_ctx,0x52);
	reg = reg & 0xFB;
	I2CWrite(p_ch_ctx,0x52, reg);

	I2CWrite(p_ch_ctx,0x52, 0x2F);

	return;
}





void ch7036_set_input_timing_info(ch7036_device_context_t *p_ctx, INPUT_INFO* pInput_Info)
{
	DEV_CONTEXT* p_ch7xxx_context = p_ctx->p_ch7xxx_context;
	OUTPUT_INFO* pOutput_Info = p_ch7xxx_context->pOutput_Info;

	pd_timing_t * p_current_mode = p_ctx->native_dtd;
	uint8 audio_id = AUDIO_SPDIF;
	PD_DEBUG("ch7036_intf: ch7036_set_input_timing_info()-\n");


	if (p_ctx->init_done) {


		pInput_Info->timing.ht = p_current_mode->htotal+1;

		pInput_Info->timing.ha = p_current_mode->width;
		pInput_Info->timing.ho = p_current_mode->hsync_start - p_current_mode->hblank_start;
		pInput_Info->timing.hw = p_current_mode->hsync_end - p_current_mode->hsync_start;
		pInput_Info->timing.vt = p_current_mode->vtotal+1;

		pInput_Info->timing.va = p_current_mode->height;
		pInput_Info->timing.vo = p_current_mode->vsync_start - p_current_mode->vblank_start;
		pInput_Info->timing.vw = p_current_mode->vsync_end - p_current_mode->vsync_start;


		pInput_Info->rx_clk_khz = p_current_mode->dclk;


		pInput_Info->hs_pol = ((uint8)(((p_current_mode)->mode_info_flags & PD_HSYNC_HIGH) >> 24 ))?1:0;
		pInput_Info->vs_pol = ((uint8)(((p_current_mode)->mode_info_flags & PD_VSYNC_HIGH) >> 24))?1:0;

	}





	pInput_Info->pixel_fmt = PIXEL_FMT_18BIT;


	pInput_Info->data_ch_pol = POL_NO_INV;


	pInput_Info->data_ch_invert = POL_NO_INV;





	pInput_Info->de_pol = POL_HIGH;





	if(pOutput_Info->channel & CHANNEL_HDMI)
		ch7036_set_audio_type(pInput_Info, audio_id);

}

void ch7036_set_output_timing_info(ch7036_device_context_t *p_ctx, OUTPUT_INFO* pOutput_Info)
{


	PD_DEBUG("ch7036: ch7036_set_output_timing_info()\n");

	PD_DEBUG("ch7036_set_output_timing_info()- p_ctx = %X \n", p_ctx);


	pOutput_Info->lvds_fmt.channel_swap = LVDS_CHANNEL_SWAP_DEF;
	pOutput_Info->lvds_fmt.channel_pol = (POL_LOW << 4) | (POL_LOW << 3) | (POL_LOW << 2) | (POL_LOW << 1) | (POL_LOW << 0);
	pOutput_Info->lvds_fmt.pixel_fmt = p_ctx->dither_select;



	pOutput_Info->hdmi_fmt.channel_swap = 0;
	pOutput_Info->hdmi_fmt.data_pol_invert = POL_NO_INV;
	pOutput_Info->hdmi_fmt.protect_enable = 0;




	if (pOutput_Info->channel & CHANNEL_HDMI)
	{

		if(!(pOutput_Info->hdmi_fmt.is_dvi_mode))
		{


			PD_DEBUG("ch7036_set_output_timing_info- hdmi mode index is [%x]\n",p_ctx->hdmi_mode_index);
			pOutput_Info->hdmi_fmt.format_index = (uint8)hdmi_timing_table[p_ctx->hdmi_mode_index].fmt_index;
			pOutput_Info->hdmi_fmt.aspect_ratio = (uint8)hdmi_timing_table[p_ctx->hdmi_mode_index].aspect;

			pOutput_Info->timing.ht = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.ht;
			pOutput_Info->timing.ha = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.ha;
			pOutput_Info->timing.ho = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.ho;
			pOutput_Info->timing.hw = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.hw;
			pOutput_Info->timing.vt = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.vt;
			pOutput_Info->timing.va = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.va;
			pOutput_Info->timing.vo = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.vo;
			pOutput_Info->timing.vw = hdmi_timing_table[p_ctx->hdmi_mode_index].timing.vw;
			pOutput_Info->uclk_khz = hdmi_timing_table[p_ctx->hdmi_mode_index].clk_freq;
		}
		else
		{
			pOutput_Info->hdmi_fmt.format_index = (uint8)dvi_timing_table[p_ctx->dvi_mode_index].fmt_index;
			pOutput_Info->hdmi_fmt.aspect_ratio = (uint8)dvi_timing_table[p_ctx->dvi_mode_index].aspect;

			pOutput_Info->timing.ht = dvi_timing_table[p_ctx->dvi_mode_index].timing.ht;
			pOutput_Info->timing.ha = dvi_timing_table[p_ctx->dvi_mode_index].timing.ha;
			pOutput_Info->timing.ho = dvi_timing_table[p_ctx->dvi_mode_index].timing.ho;
			pOutput_Info->timing.hw = dvi_timing_table[p_ctx->dvi_mode_index].timing.hw;
			pOutput_Info->timing.vt = dvi_timing_table[p_ctx->dvi_mode_index].timing.vt;
			pOutput_Info->timing.va = dvi_timing_table[p_ctx->dvi_mode_index].timing.va;
			pOutput_Info->timing.vo = dvi_timing_table[p_ctx->dvi_mode_index].timing.vo;
			pOutput_Info->timing.vw = dvi_timing_table[p_ctx->dvi_mode_index].timing.vw;
			pOutput_Info->uclk_khz = dvi_timing_table[p_ctx->dvi_mode_index].clk_freq;

		}



	} else if((pOutput_Info->channel & CHANNEL_VGA) && ((pOutput_Info->channel & CHANNEL_HDMI)==0x00) )
	{



		PD_DEBUG("ch7036_set_output_timing_info- crt mode index is [%ld]\n",p_ctx->crt_mode_index);
		pOutput_Info->timing.ht = ch_vga_timing_table[p_ctx->crt_mode_index].timing.ht;
		pOutput_Info->timing.ha = ch_vga_timing_table[p_ctx->crt_mode_index].timing.ha;
		pOutput_Info->timing.ho = ch_vga_timing_table[p_ctx->crt_mode_index].timing.ho;
		pOutput_Info->timing.hw = ch_vga_timing_table[p_ctx->crt_mode_index].timing.hw;
		pOutput_Info->timing.vt = ch_vga_timing_table[p_ctx->crt_mode_index].timing.vt;
		pOutput_Info->timing.va = ch_vga_timing_table[p_ctx->crt_mode_index].timing.va;
		pOutput_Info->timing.vo = ch_vga_timing_table[p_ctx->crt_mode_index].timing.vo;
		pOutput_Info->timing.vw = ch_vga_timing_table[p_ctx->crt_mode_index].timing.vw;
		pOutput_Info->uclk_khz = ch_vga_timing_table[p_ctx->crt_mode_index].clk_freq;


	} else
		;


	if(pOutput_Info->channel & CHANNEL_HDMI)
		ch7036_set_hdmi_sync_polarity(pOutput_Info);

	if(pOutput_Info->channel & CHANNEL_HDMI || pOutput_Info->channel & CHANNEL_VGA) {

		ch7036_set_rotate (pOutput_Info);
		ch7036_set_hflip (pOutput_Info);
		ch7036_set_vflip (pOutput_Info);
	}
}

void ch7036_set_prefer_timing_info(ch7036_device_context_t *p_ctx, PREFER_INFO* pPrefer_Info)
{




	PD_DEBUG("ch7036_intf: ch7036_set_prefer_timing_info()\n");


	if (!p_ctx->init_done) {

	pPrefer_Info->mclk_khz = 166000;
	pPrefer_Info->uclkod_sel = 1;
	pPrefer_Info->dat16_32b = 0;
	pPrefer_Info->true24 = 0;
	pPrefer_Info->true_com = 0;
	pPrefer_Info->lvds_out_hs_tolerance = HS_TOLERANCE_LEVEL0;
	pPrefer_Info->lvds_out_reset_bit_sel = RST_BIT_VSYNC;
	pPrefer_Info->dither_filter_enable = DITHER_ENABLE;

	pPrefer_Info->hscale_ratio_gate = 130;
	pPrefer_Info->reset=0;
	pPrefer_Info->vga_enable=0;

	pPrefer_Info->text_enhancement = DEFAULT_TEXT_ENHANCE;
	pPrefer_Info->pll_ref_dly = DEF_PLL_REF_DLY;
	pPrefer_Info->pll_ref_fbdly = DEF_PLL_REF_FBDLY;
	pPrefer_Info->lvds_txdrv_ctrl = DEF_LVDS_TXDRV_CTRL;

	pPrefer_Info->eye_bgtrim=0;
	pPrefer_Info->eye_dacg=0;
	pPrefer_Info->eye_dri_demp=0;
	pPrefer_Info->eye_dri_pll_cp=0;
	pPrefer_Info->eye_dri_damp=0;
	pPrefer_Info->eye_dri_pll_rlf=0;
	pPrefer_Info->eye_rdac=0;

	}

	pPrefer_Info->scale_line_adjust = 0;



}









