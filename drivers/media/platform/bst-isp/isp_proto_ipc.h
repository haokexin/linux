/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_PROTO_ISP_IPC_H__
#define __BST_PROTO_ISP_IPC_H__

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

typedef enum _sensorIndex_e { // [7:2]isp core index(0/1/2) [1:0] sensor index(0/1/2/3) per core
	S00 = 0,
	S01,
	S02,
	S03,
	S04,
	S05,
	S06,
	S07,
	S08,
	S09,
	S10,
	S11,
	S12,
	S13,
	S14,
	S15
} sensorIndex_e;

typedef enum __viewMode_e {
	noView = 0,	   // 0b000
	View0,		   // 0b001
	View1,		   // 0b010
	View0_View1,	   // 0b011
	View2,		   // 0b100
	View0_View2,	   // 0b101
	View1_View2,	   // 0b110
	View0_View1_View2, // 0b111
	ViewRaw = 32	   // 0b10000
} viewMode_e;

typedef enum _srcSel_e {
	SRC_MIPI,
	SRC_HDMI,
	SRC_FILE2FILE_HDR,
	SRC_FILE2FILE_SINGLE
} srcSel_e;

typedef enum _sensorRdWrMode_e {
	Addr16bit_Data16bit = 0,
	Addr16bit_Data8bit,
	Addr8bit_Data8bit
} sensorRdWrMode_e;

typedef enum _view0Fmt_e {
	View0_YUVSep_Fmt,
	View0_NV12_Fmt,
	View0_NV21_Fmt,
	View0_YUV422_Fmt,
	View0_RGB888_Fmt,
	View0_Dis
} view0Fmt_e;

typedef enum _view1Fmt_e {
	View1_YUVSep_Fmt,
	View1_NV12_Fmt,
	View1_NV21_Fmt,
	View1_YUV422_Fmt,
	View1_Raw_Fmt,
	View1_Dis
} view1Fmt_e;

typedef enum _view2Fmt_e {
	View2_YUVSep_Fmt,
	View2_NV12_Fmt,
	View2_NV21_Fmt,
	View2_YUV422_Fmt,
	View2_Raw_Fmt,
	View2_Dis
} view2Fmt_e;

typedef enum _abnormalItem_e {
	ABN_INVALID,

	ABN_HW_QUEUE_IRQ, // DDR NOC queue abnormal interrupt for low queue(offline mode) or high queue(online mode)
	ABN_HW_RAW2DDR_WR_OVERFLOW,	     // RAW to DDR write overflow
	ABN_HW_ISP2DDR_PDNS2DDR_WR_OVERFLOW, // ISP/PDNS to DDR write overflow

	ABN_SW_RAWRDYLST_OVERFLOW = 0x80,
	ABN_SW_NO_SYNC_FRAME_DATA,
	ABN_SW_GRP_RAWRDYLST_OVERFLOW,
	ABN_SW_PDNS_P0RDYLST_OVERFLOW,
	ABN_SW_PDNS_P1RDYLST_OVERFLOW,
	ABN_SW_DVP_FRAMEBUF_OVERFLOW,
	ABN_SW_SRAM_ALLOC_OVERFLOW,
	ABN_SW_RAW_BADFRAME,
	ABN_SW_CAMERA_PLUG_OUT,
} abnormalItem_e;

// to arm
typedef volatile struct _MINOR_BOOT_DONE {
	uint8_t	 chipType;
	uint8_t	 ispValidCore;
	uint8_t	 rsv0[2];
	uint32_t rsv[3];
} boot_done_t;

typedef volatile struct _MINOR_ABNORMAL {
	uint8_t	 abnormalId; // abnormalItem_e
	uint8_t	 abnormalType;
	uint8_t	 rsv0[2];
	uint32_t rsv[3];
} abnormal_t;

typedef volatile struct _MINOR_RW_ADDR {
	uint32_t	 addr;
	uint32_t	 value;
	uint8_t		 flag; // 0: read 1: write
	uint8_t		 rsv0[3];
	uint32_t	 rsv;
} rw_addr_t;

typedef volatile struct _MINOR_SYNC_ISP_VIEW_FRAME_DONE {
	struct _viewReply {
		uint32_t vsyncCnt;
	} viewReply;

	uint32_t viewBuf[3]; // DDR Addr
} new_frame_done_t;

// from/to arm
typedef volatile struct _MINOR_GET_VERSION {
	union {
		uint32_t rsv0;	  // from arm
		uint32_t version; // to arm, reply val
	};

	uint32_t rsv[3];
} get_version_t;

typedef enum _txMsgMode_e {
	DropMsg_TimeOut	  = 0,
	AttachMsg_TimeOut = 1
} txMsgMode_e;

typedef volatile struct _MINOR_ISP_START {
	uint32_t reconfDDRBase;
	uint32_t reconfDDRSize;

	union {
		uint32_t psm;
		uint32_t status; // to arm, reply status
	};

	uint8_t	 txMsgMode; // 0: drop msg when timeout, 1:will attach this msg to new one when timeout
	uint8_t	 rsv2;
	uint16_t txTimeOut; // 0.5us*txTimeOut, 0: use default timeout value (10*2000), 10ms
} isp_start_t;

// from/to arm, set payload bit
typedef volatile struct _MINOR_ISP_BOOTLD_RECONF {
	uint8_t	 mipiSensorIndex;
	uint8_t	 rsv0[3];
	uint32_t payloadAddr; // DDR Addr for ipc_reconf_t

	union {
		uint32_t rsv1;
		uint32_t status; // to arm, reply status
	};

	union {
		uint32_t rsv2;
		uint32_t payloadSize; // 0:auto, xxx
	};
} isp_ld_reconf_t;

typedef volatile struct _MINOR_ISP_BOOTLD_ALGO_BIN {
	uint8_t	 mipiSensorIndex;
	uint8_t	 rsv0[3];
	uint32_t payloadAddr; // DDR Addr for algo bin file

	union {
		uint32_t rsv1;
		uint32_t status; // to arm, reply status
	};

	union {
		uint32_t rsv2;
		uint32_t payloadSize; // 0:auto, xxx
	};
} isp_ld_algo_bin_t;

typedef volatile struct _MINOR_ISP_BOOTLD_IQ_BIN {
	uint8_t	 mipiSensorIndex;
	uint8_t	 rsv0[3];
	uint32_t payloadAddr; // DDR Addr for iq bin file

	union {
		uint32_t rsv1;
		uint32_t status; // to arm, reply status
	};

	union {
		uint32_t rsv2;
		uint32_t payloadSize; // 0:auto, xxx
	};
} isp_ld_iq_bin_t;

typedef volatile struct _MINOR_ISP_NEW_VIEW_FRAME_BUF {
	uint8_t	 mipiSensorIndex;
	uint8_t	 viewMode;
	uint8_t	 bufFlag; // used in raw viewMode
	uint8_t rsv[1];
	uint32_t viewbuf[3];
} isp_new_frame_buf_t;

typedef volatile struct _MINOR_ISP_CAM_OPEN {
	uint8_t	 mipiSensorIndex;
	uint8_t	 viewMode;
	uint8_t	 rsv[1];
	uint8_t	 embedded_view;
	uint32_t embedded_offset[2];
	uint32_t isp_meta_offset;
} isp_cam_open_t;

typedef volatile struct _MINOR_ISP_CAM_CLOSE {
	uint8_t	 mipiSensorIndex;
	uint8_t	 viewMode;
	uint8_t	 rsv0[2];
	uint32_t rsv[3];
} isp_cam_close_t;

typedef volatile struct _MINOR_ISP_CAM_PLUGINOUT {
	uint8_t	 mipiSensorIndex;
	uint8_t	 rsv0[3];
	uint32_t rsv[3];
} isp_cam_pluginout_t;

typedef volatile struct _ispusrdata {
	union {
		// to arm
		abnormal_t	    abnormal;
		new_frame_done_t    newFrameDone;
		boot_done_t	    bootDone;

		// from/to arm
		get_version_t	    getVersion;
		isp_start_t	    ispStart;
		rw_addr_t	    rwAddr;
		// from/to arm,  set payload bit
		isp_ld_reconf_t	    ispLdReconf;
		isp_ld_algo_bin_t   ispLdAlgoBin;
		isp_ld_iq_bin_t	    ispLdIqBin;

		// from arm
		isp_new_frame_buf_t newFrameBuf;
		isp_cam_open_t	    camOpen;
		isp_cam_close_t	    camClose;
		isp_cam_pluginout_t camPlugInOut;
		uint32_t	    data[4];
	};
} ispusrdata;

// payload info
typedef volatile struct _raw_cfg_t {
	uint8_t srcSel;	  // srcSel_e
	uint8_t dataType; // sensor input data type, dataType_e

	union {
		uint8_t hdrMode; // [1:0] 0-HDR WxN/1-HDR Stagger, [7] 1-force offlineMode [6] VS Mode [5] DSP IPC

		struct _hdrModeBits {
			uint8_t hdrFmt		 : 1; // 0-HDR WxN/1-HDR Stagger
			uint8_t rsv		 : 3;
			uint8_t hdriLe2DdrDis	 : 1; // hdri le to ddr disabled
			uint8_t dspIpcEn	 : 1; // enable dsp ipc for pwl or other features
			uint8_t vsMode		 : 1; // do not use
			uint8_t forceOfflineMode : 1; // 1-force Offline mode
		} hdrModeBits;
	};

	uint8_t	 expNum;	  // 1-single/2-2exp/3-3exp
	uint8_t	 ispPwlInFormat;  // pwlInFmt_e
	uint8_t	 ispRawOutFormat; // debug use
	uint8_t	 dvpDataType;	  // sensor data to ddr data type, 0: same as dataType/dataType_e
	uint8_t	 vinDataType;	  // ddr to isp pipe data type, 0: same as dataType/dataType_e
	uint16_t width;		  // mipi/hdmi input width
	uint16_t height;	  // mipi/hdmi input height
	uint16_t dvpDummyVal;	  // [15:x]=dummydata [x:0]=0, depend on sensor dummy data config
	// offline use dvpDummyLines for special sensors
	// [15] fixed dummy lines mode
	// [13:0] rawDummyMinLine if bit15=0
	// [13:0] fixed rawDummyLexpLinesNum if bit15=1
	uint16_t dvpDummyLines;
} raw_cfg_t;

typedef volatile struct _view_cfg_t {
	// view0: view0Fmt_e
	// view1: view1Fmt_e
	// view2: view2Fmt_e
	uint8_t viewFmt;     // view output format
	uint8_t scalerRemap; // scalerRemap_e

	union {
		struct _embedCfgBits {
			uint8_t comSwEmbedEn : 1;
			uint8_t embedBitsRsv : 7;
		} embedCfgBits;

		uint8_t embedCfg;
	};

	uint8_t	 rsv0;

	// crop before scaler [topCropBefore, botCropBefore) [lefCropBefore, rigCropBefore), disabled if all 0
	uint16_t topCropBefore; // crop pixel top position (include current position)
	uint16_t botCropBefore; // crop pixel bottom position (exclude current position)
	uint16_t lefCropBefore; // crop pixel left position (include current position)
	uint16_t rigCropBefore; // crop pixel right position (exclude current position)

	// scaler output size, sclaer disabled if following size is same as raw size
	uint16_t width;	 // view output width
	uint16_t height; // view output height

	// crop after scaler [topCropAfter, botCropAfter) [lefCropAfter, rigCropAfter), disabled if all 0
	uint16_t topCropAfter; // crop pixel top position (include current position)
	uint16_t botCropAfter; // crop pixel bottom position (exclude current position)
	uint16_t lefCropAfter; // crop pixel left position (include current position)
	uint16_t rigCropAfter; // crop pixel right position (exclude current position)
	uint16_t lineAlign;    // view line alignment
	uint16_t rsv1;
} view_cfg_t;

typedef volatile struct _embedded_data_cfg_t {
	uint16_t line_start; // Image begin from start N line after
	uint16_t line_end;   // embedded end line number
} embedded_data_cfg_t;

typedef volatile struct _ipc_reconf_t {
	uint32_t	    i2cRegBase;	     // i2c IP register base
	uint8_t		    mipiSensorIndex; // mipiSensorIndex_e
	uint8_t		    sensorIndex;     // sensorIndex_e
	uint8_t		    sensorDevID;     // sensor i2c addr
	uint8_t		    sensorRdWrMode;  // sensorRdWrMode_e
	uint16_t	    sensorOnline;    // Sensor is connected or disconnected
	// raw data crop for ltm, crop embed lines instead of video data.
	uint8_t		    ltmVinTopCrop;
	uint8_t		    ltmVinBotCrop;
	raw_cfg_t	    rawinfo;
	view_cfg_t	    viewinfo[3];
	uint32_t	    algo_addr;
	uint32_t	    algo_size;
	uint32_t	    iq_addr;
	uint32_t	    iq_size;
	// raw data crop before isp pipe and after ltm
	uint16_t	    ispInTopCrop; // crop top line num
	uint16_t	    ispInBotCrop; // crop bottom line num
	uint16_t	    ispInLefCrop; // crop left column num
	uint16_t	    ispInRigCrop; // crop right column num
	// embedded info
	embedded_data_cfg_t embeddedInfo[2];
	uint8_t		    semBank;
	uint8_t		    semId;
	uint16_t	    rsv;
} ipc_reconf_t;

#endif /* __BST_PROTO_ISP_IPC_H__ */
