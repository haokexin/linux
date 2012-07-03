/*
* linux/spr_adc_st10_usr.h
* ST ADC driver - user header file
*
* Copyright (ST) 2009 Viresh Kumar (viresh.kumar@st.com)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/

#ifndef SPEAR_ADC_USR_H
#define SPEAR_ADC_USR_H

#define ADC_NAME 'A'
#define ADCIOC_CONFIG _IOW(ADC_NAME, 1, struct adc_config)
#define ADCIOC_CHAN_CONFIG _IOW(ADC_NAME, 2, struct adc_chan_config)
#define ADCIOC_GET_CONFIG _IOR(ADC_NAME, 3, struct adc_config)
#define ADCIOC_GET_CHAN_CONFIG _IOR(ADC_NAME, 4, struct adc_chan_config)

/* channel list */
enum adc_chan_id {
	ADC_CHANNEL0,
	ADC_CHANNEL1,
	ADC_CHANNEL2,
	ADC_CHANNEL3,
	ADC_CHANNEL4,
	ADC_CHANNEL5,
	ADC_CHANNEL6,
	ADC_CHANNEL7,
	ADC_CHANNEL_NONE
};

/* sample rates */
enum adc_avg_samples {
	SAMPLE1,
	SAMPLE2,
	SAMPLE4,
	SAMPLE8,
	SAMPLE16,
	SAMPLE32,
	SAMPLE64,
	SAMPLE128
};

/* adc conversion modes */
enum adc_conv_mode {
	SINGLE_CONVERSION,
	CONTINUOUS_CONVERSION,
	CONVERSION_NONE
};

#ifndef CONFIG_ARCH_SPEAR6XX
/* adc resolution */
enum adc_resolution {
	NORMAL_RESOLUTION,
	HIGH_RESOLUTION,
	RESOLUTION_NONE
};
#else
/**
* struct adc_output: structure representing data output from adc
* @integral: integral part of data
* @fractional: fractional part of data
*/
struct adc_output {
	short int integral;
	short int fractional;
};
#endif

/* scan rate reference */
enum adc_scan_ref {
	INTERNAL_SCAN,
	EXTERNAL_SCAN
};

/* reference voltage */
enum adc_volt_ref {
	EXTERNAL_VOLT,
	INTERNAL_VOLT
};

/**
* struct adc_config: adc configuration structure
* @mode: mode to be configured, can be single or continuous
* @volt_ref: reference of voltage, can be internal or external
* @mvolt: reference voltage in millivolts
* @scan_ref: reference of scan rate, can be internal or external
* @resolution: resolution of output data, can be normal or high, for SPEAR300
* @req_clk: adc clk requested, in Hz
* @avail_clk: closest clk less than equal to req_clk possible, in Hz
*/
struct adc_config {
	enum adc_conv_mode mode;
	enum adc_volt_ref volt_ref;
	unsigned int mvolt;
	enum adc_scan_ref scan_ref;
#ifndef CONFIG_ARCH_SPEAR6XX
	enum adc_resolution resolution;
#endif
	unsigned int req_clk;
	unsigned int avail_clk;
};

/**
* struct adc_chan_config: adc channel configuration structure
* @chan_id: channel to be configured
* @avg_samples: number of average samples
* @scan_rate: rate at which adc converts data, in microseconds
* @scan_rate_fixed: if 1 configured scan rate should be equal to requested
* scan rate else if FALSE configured scan rate can be less than equal to
* requsted scan rate
*/
struct adc_chan_config {
	enum adc_chan_id chan_id;
	enum adc_avg_samples avg_samples;
	unsigned int scan_rate;
	int scan_rate_fixed;
};
#endif /* SPEAR_ADC_USR_H */
