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
* @file  ch7036_fw.c
* @version 1.1.4
*-----------------------------------------------------------------------------
*/



#include "ch7036_intf.h"
#include "ch7036_fw.h"

#include "hdcp7036.car"


#define EDID_RETRY_MAX_TIMES 5
#define HPD_LOOP_MAX 10



#define SLEEP_TIME 40


#define lhfm_size  sizeof(lhfm_array)


#define LHFM_TIMEOUT	0x1F

static unsigned char es_map[16] = {
	0x26,0x27,0x42,0x43,0x44,0x45,0x46,0x47,
	0x6A,0x51,0x52,0x53,0x57,0x58,0x59,0x5A
};



established_timings_t et_I[8] = {
	{0,"800x600_60"},
	{0,"800x600_56"},
	{0,"640x480_75"},
	{0,"640x480_72"},
	{0,"640x480_67"},
	{0,"640x480_60"},
	{0,"720x400_88"},
	{0,"720x400_70"}
};

established_timings_t et_II[8] ={
	{0, "1280x1024_75"},
	{0, "1024x768_75"},
	{0, "1024x768_70"},
	{0, "1024x768_60"},
	{0, "1024x768_87"},
	{0, "832x624_75"},
	{0, "800x600_75"},
	{0, "800x600_72"}
};

established_timings_t et_man = {
	0, "1152x870_75"
};

extern OUT_FMT hdmi_timing_table[OUT_HDMI_END];
extern OUT_FMT dvi_timing_table[OUT_DVI_END];
extern OUT_FMT ch_vga_timing_table[OUT_CRT_END];


int LHFM_get_version(DEV_CONTEXT* p_ch7xxx_context,struct _FW7036_CFG* cfg)
{

	unsigned char reg;
	unsigned wj;

		 I2CWrite(p_ch7xxx_context,0x03, 0x00);
		 reg = I2CRead(p_ch7xxx_context,0x4F);


		 if (0==(LHFM_REQUEST & reg)) {
			 I2CWrite(p_ch7xxx_context,0x4F, (LHFM_REQUEST+LHFM_GET_VERSION));
			 wj = 0;
			 while (wj++< LHFM_TIMEOUT) {


				 I2CWrite(p_ch7xxx_context,0x03, 0x00);
				 pd_usleep(SLEEP_TIME );
				 PD_DEBUG("LHFM_get_version [%x]\r\n", wj);

				 reg = I2CRead(p_ch7xxx_context,0x4F);
				 if(reg == LHFM_GET_VERSION) {
					 reg = I2CRead(p_ch7xxx_context,0x50);
					 if (!(LHFM_RET_ERROR & reg)) {
						    I2CWrite(p_ch7xxx_context,0x03, 0x01);
						    cfg->size = I2CRead(p_ch7xxx_context,es_map[0]);
							cfg->ver_major = I2CRead(p_ch7xxx_context,es_map[1]);
							cfg->ver_minor = I2CRead(p_ch7xxx_context,es_map[2]);
							cfg->did = I2CRead(p_ch7xxx_context,es_map[3]);
							cfg->rid = I2CRead(p_ch7xxx_context,es_map[4]);
							cfg->capbility = I2CRead(p_ch7xxx_context,es_map[5]);
							cfg->reserved = I2CRead(p_ch7xxx_context,es_map[6]);
						    I2CWrite(p_ch7xxx_context,0x03, 0x00);
							return 0;
					 } else return -2;

				 }




			 }

		 }

	return -1;
}



ch7036_status_t LHFM_get_hdmi_hpd(DEV_CONTEXT* p_ch7xxx_context,uint8 *hpd)
{

	unsigned char reg;
	unsigned wj, count=0;
	ch7036_status_t status = SS_FIRMWARE_TIMEOUT;


	do {


		 I2CWrite(p_ch7xxx_context,0x03, 0x00);
		 reg = I2CRead(p_ch7xxx_context,0x4F);


		 if (0==(LHFM_REQUEST & reg)) {

			 I2CWrite(p_ch7xxx_context,0x4F, (LHFM_REQUEST+ LHFM_GET_HPD));
			 wj = 0;

			 while (wj++< LHFM_TIMEOUT) {
				 I2CWrite(p_ch7xxx_context,0x03, 0x00);
				 reg = I2CRead(p_ch7xxx_context,0x4F);

				 if(reg == LHFM_GET_HPD) {

					 reg = I2CRead(p_ch7xxx_context,0x50);

					 if (!(LHFM_RET_ERROR & reg)) {

						    I2CWrite(p_ch7xxx_context,0x03, 0x01);
						    reg = I2CRead(p_ch7xxx_context,es_map[0]);

							if (hpd) *hpd = reg;

							PD_DEBUG("ch7036: LHFM_get_hdmi_hpd- SUCCESS- hpd [%x]\n", *hpd);
							return SS_SUCCESS;
					 } else {

						 status = SS_FIRMWARE_ERR;
					 }

				 }

				if(status == SS_FIRMWARE_ERR)  break;
				pd_usleep(SLEEP_TIME);

			 }

		 }

		pd_usleep(SLEEP_TIME);
		PD_DEBUG("ch7036: LHFM_get_hdmi_hpd- NOT SUCCESS-status = [%s]\n",status==SS_FIRMWARE_ERR?"firmware error":"timeout");

	} while ( ( (status== SS_FIRMWARE_ERR) || (status== SS_FIRMWARE_TIMEOUT) ) && ( (++count) < HPD_LOOP_MAX) );


	return status;
}


void LHFM_enable_crt_hpd(DEV_CONTEXT* p_ch7xxx_context)
{
	uint8 reg;


	I2CWrite(p_ch7xxx_context,0x03, 0x04);
	reg = I2CRead(p_ch7xxx_context,0x57);
	I2CWrite(p_ch7xxx_context,0x57, reg | 0x02);


	pd_usleep(SLEEP_TIME);


	I2CWrite(p_ch7xxx_context,0x03, 0x00);
	reg = I2CRead(p_ch7xxx_context,0x74);
	I2CWrite(p_ch7xxx_context,0x74, reg | 0x02);

	pd_usleep(SLEEP_TIME);


}


ch7036_status_t LHFM_get_crt_hpd(DEV_CONTEXT* p_ch7xxx_context)
{

	unsigned char reg, count =0;
	ch7036_status_t status = SS_CRT_HPD_NOTCONNECTED;


	do {
		I2CWrite(p_ch7xxx_context,0x03, 0x00);
		reg = I2CRead(p_ch7xxx_context,0x74);
		I2CWrite(p_ch7xxx_context,0x74, reg & 0xFD);

		pd_usleep(SLEEP_TIME);

		I2CWrite(p_ch7xxx_context,0x03, 0x00);
		reg = I2CRead(p_ch7xxx_context,0x74);
		I2CWrite(p_ch7xxx_context,0x74, reg | 0x02);

		pd_usleep(SLEEP_TIME);


		I2CWrite(p_ch7xxx_context,0x03, 0x01);
		reg = I2CRead(p_ch7xxx_context,0x7C);



		PD_DEBUG("ch7036: LHFM_get_crt_hpd- DAC sense- reg dump [%x]\n", reg);

		if( (reg & 0xFC) == 0x54 || (reg & 0xFC) == 0x50 ){

			status= SS_SUCCESS;

		}
		else
			LHFM_enable_crt_hpd(p_ch7xxx_context);

		pd_usleep(SLEEP_TIME+100000);


	} while ( (status == SS_CRT_HPD_NOTCONNECTED) && ( (++count) < HPD_LOOP_MAX ) );


	I2CWrite(p_ch7xxx_context,0x03, 0x00);
	reg = I2CRead(p_ch7xxx_context,0x74);
	I2CWrite(p_ch7xxx_context,0x74, reg & 0xFD);

	return status;

}


ch7036_status_t LHFM_get_hdmi_modeinfo(DEV_CONTEXT* p_ch7xxx_context,unsigned char *minfo)
{
	unsigned char reg;
	unsigned wj;
	unsigned i;

		 I2CWrite(p_ch7xxx_context,0x03, 0x00);
		 reg = I2CRead(p_ch7xxx_context,0x4F);

		 if (0==(LHFM_REQUEST & reg)) {
			 I2CWrite(p_ch7xxx_context,0x4F, (LHFM_REQUEST+ LHFM_GET_MODEINFO));
			 wj = 0;
			 while (wj++< LHFM_TIMEOUT) {
				 I2CWrite(p_ch7xxx_context,0x03, 0x00);
				 pd_usleep(SLEEP_TIME);

				 reg = I2CRead(p_ch7xxx_context,0x4F);
				 if(reg == LHFM_GET_MODEINFO) {
					 reg = I2CRead(p_ch7xxx_context,0x50);
					 if (!(LHFM_RET_ERROR & reg)) {
						 if (minfo) {
						    I2CWrite(p_ch7xxx_context,0x03, 0x01);
							for (i=0; i<15; i++) minfo[i] = I2CRead(p_ch7xxx_context,es_map[i]);
						}
						return SS_SUCCESS;
					 } else return SS_FIRMWARE_ERR;

				 }




			 }

		 }

	return SS_FIRMWARE_TIMEOUT;

}



ch7036_status_t LHFM_get_edid(DEV_CONTEXT* p_ch7xxx_context,unsigned char* edid, unsigned char* ebn, unsigned char flag)
{

	unsigned i,j, ie;
	unsigned k1, k2;
	unsigned char reg;
	unsigned wj;

	ch7036_status_t status = SS_SUCCESS;


	*ebn = 0;


	for (i=0; i<512; i++) edid[i]=0;

	 I2CWrite(p_ch7xxx_context,0x03, 0x00);
	 reg = I2CRead(p_ch7xxx_context,0x4F);


	 if (0==(LHFM_REQUEST & reg)) {


		 for(i=0,j=0; i < 8; i++,j+=16) {

		    I2CWrite(p_ch7xxx_context,0x03, 0x00);
			I2CWrite(p_ch7xxx_context,0x50, i + flag);
			I2CWrite(p_ch7xxx_context,0x4F, (LHFM_REQUEST+LHFM_GET_EDID));
			wj = 0;

			status = SS_FIRMWARE_TIMEOUT;

			while (wj++< (2*LHFM_TIMEOUT) ) {
				 I2CWrite(p_ch7xxx_context,0x03, 0x00);
				 pd_usleep(SLEEP_TIME);

				 reg = I2CRead(p_ch7xxx_context,0x4F);

				 if(reg == LHFM_GET_EDID) {

					 reg = I2CRead(p_ch7xxx_context,0x50);

					 if (!(LHFM_RET_ERROR & reg)) {
							I2CWrite(p_ch7xxx_context,0x03, 0x01);
							for (ie=0; ie<16; ie++) edid[j+ie] = I2CRead(p_ch7xxx_context,es_map[ie] );


							break;
					 }
					 else {




						return SS_FIRMWARE_ERR;

					 }

				 }


			}

			status = SS_SUCCESS;


		}


		(*ebn)++;


		k2 = edid[0x7E];

		k2 = (k2 > 3)? 3 : k2;
		if (k2>0) {
		  for (k1=1; k1<=k2; k1++) {



		    for(i=k1*8; i < 8+k1*8; i++,j+=16){
				I2CWrite(p_ch7xxx_context,0x03, 0x00);
			    I2CWrite(p_ch7xxx_context,0x50, i + flag);
				I2CWrite(p_ch7xxx_context,0x4F, (LHFM_REQUEST+LHFM_GET_EDID));
				wj = 0;

				status = SS_FIRMWARE_TIMEOUT;

				while (wj++< (2*LHFM_TIMEOUT)) {
					 I2CWrite(p_ch7xxx_context,0x03, 0x00);
					 pd_usleep(SLEEP_TIME);

					 reg = I2CRead(p_ch7xxx_context,0x4F);

					 if(reg == LHFM_GET_EDID) {
						 reg = I2CRead(p_ch7xxx_context,0x50);
						 if (!(LHFM_RET_ERROR & reg)) {
								I2CWrite(p_ch7xxx_context,0x03, 0x01);
								for (ie=0; ie<16; ie++) edid[j+ie] = I2CRead(p_ch7xxx_context,es_map[ie] );


								break;
						 }
						 else {

							return SS_FIRMWARE_ERR;
						 }
					 }




				 }

				status = SS_SUCCESS;
			}

			(*ebn)++;
		  }

		}



	 }

	return status;

}


int LHFM_load_firmware(DEV_CONTEXT* p_ch7xxx_context)
{


	unsigned fs1;
	unsigned char ch;


	I2CWrite(p_ch7xxx_context,0x03, 0x04);

	ch = 0x29 | I2CRead(p_ch7xxx_context,0x52);
	I2CWrite(p_ch7xxx_context,0x52, ch & 0xFB);



	I2CWrite(p_ch7xxx_context,0x5B, 0x9E);

	I2CWrite(p_ch7xxx_context,0x5B, 0xB3);


	I2CWrite(p_ch7xxx_context,0x03, 0x04);

	I2CWrite(p_ch7xxx_context,0x03, 0x07);


	for (fs1=0; fs1<lhfm_size; fs1++) {

		I2CWrite(p_ch7xxx_context, 0x07, lhfm_array[fs1]);
	}

	I2CWrite(p_ch7xxx_context, 0x03, 0x03);
	ch = I2CRead(p_ch7xxx_context,0x74);
	if (ch & 0x1) {

		return -1;
	}
	else {

		I2CWrite( p_ch7xxx_context,0x03, 0x04);
		ch = I2CRead(p_ch7xxx_context,0x52);
		I2CWrite(p_ch7xxx_context, 0x52, ch | 0x24);
	}


	I2CWrite(p_ch7xxx_context, 0x03, 0x00);


	return 0;
}

void ch7036_dump( char *s, int size, unsigned char *regdata)
{

    return;
}



