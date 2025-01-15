// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

/*
 *  VERSION :
 *  Autosar Version      : 4.4.0
 *  SW Version           : 1.0.0
*/
#ifndef VCAN_COMMAND_H
#define VCAN_COMMAND_H

/**
*   @file    VCan_Command.h
*   @version 1.0.0
*
*   @brief   AUTOSAR VCan - module interface.
*   @details API header for VCAN driver.
*
*   @addtogroup VCAN_DRIVER
*   @{
*/

#ifdef __cplusplus
extern "C" {
#endif
	/*==================================================================================================
	*                                        INCLUDE FILES
	* 1) system and project includes
	* 2) needed interfaces from external units
	* 3) internal and external interfaces from this unit
	==================================================================================================*/



	/*==================================================================================================
	*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
	==================================================================================================*/

#pragma pack (1)
	/** 数据相关结构体 */
	typedef struct {
		u32 timestamp : 16;
		u32 dlc : 4;
		u32 rtr : 1;
		u32 ide : 1;
		u32 srr : 1;
		u32 rsvd : 6;
		u32 esi : 1;
		u32 brs : 1;
		u32 edl : 1;  //4Byte
		u32 id;       //4Byte
		u8 param[56]; //56Byte
		u8 data[64];  //64Byte
	} CAN2X;         //共计128Byte

	typedef struct
	{
		u8 CAN2ETH_DMAC[6];  //6Byte
		u8 rsvd0[2];    //2Byte

		u32 src_can_bus_id : 4;
		u32 filter_id : 8;
		u32 rsvd1 : 20;   //4Byte

		u32 gmac_timestamp_second_h;
		u32 gmac_timestamp_second_l;
		u32 gmac_timestamp_nanosecond;  //12Byte

		u8 rsvd2[32];
	} CAN2X_PARAMS;   //共计56B，对应上述的 param[56]


	typedef struct {
		u32 rsvd0 : 16;         //保留，填0
		u32 dlc : 4;           //dlc，can实际数据长度.注:dlc=9,并不代表数据长度为9，请百度can dlc
		u32 rtr : 1;           //是否为can远程帧
		u32 ide : 1;           //是否为can扩展帧
		u32 srr : 1;           //保留，填0
		u32 rsvd1 : 6;         //保留，填0
		u32 esi : 1;           //是否为错误帧，绝大部分情况下填0
		u32 brs : 1;           //是否为canfd加速帧
		u32 edl : 1;           //是否为canfd帧
		u32 id;                //canid
		u8 param[56];          //56Byte的额外参数，具体结构为下列的X2CAN_PARAMS
		u8 data[64];           //can报文携带的的实际数据
	} X2CAN;

	typedef struct
	{
		u32 client_id;           //预留，暂时非必填，vcan_client id用于下行方向路由表
		u32 timestamp_l;         //预留，暂时非必填
		u32 timestemp_h;         //预留，暂时非必填
		u32 des_can_bus_id : 4;  //从哪个can通道发送出去, 下个版本会删除此字段。
		u32 rsvd0 : 28;          //保留，填0
		u8 rsvd1[40];            //保留，填0
	} X2CAN_PARAMS;   //共计56B，对应上述的 param[56]


	typedef enum
	{
		LLCG_COMMAND_INIT_DONE = 0,
		LLCG_COMMAND_SET_CONTROLLER = 1,  //告诉VCAN核，有哪些Controller用于VCAN
		LLCG_COMMAND_INIT_BASE_ROUTE_TABLE = 2,    //告诉VCAN核，base路由表的位置，VCAN核先自己复制一份
		LLCG_COMMAND_INIT_COMMON_ROUTE_TABLE = 3,  //告诉VCAN核，common路由表的位置，VCAN核先自己复制一份
		LLCG_COMMAND_SET_ROUTE_TABLE = 4,  //告诉VCAN核，Controller+filter对应的转发规则
		LLCG_COMMAND_SEND_DATA = 5  //告诉VCAN核，需要通过VCAN发送数据
	}LLCG_COMMAND_ENUM;

	typedef struct
	{
		u32 can_controller_id; //用于vcan的controller编号，范围0~16
	}LLCG_COMMAND_PARAM_SET_CONTROLLER;


	typedef struct
	{
		u32 base_route_table_ptr; //指向LLCG_RouteTableBase
		u32 base_route_table_size;

	}LLCG_COMMAND_PARAM_INIT_BASE_ROUTE_TABLE;

	typedef struct
	{
		u8 can_controller_id;
		u32 common_route_table_ptr;
		u32 common_route_table_size;
	}LLCG_COMMAND_PARAM_INIT_COMMON_ROUTE_TABLE;



	typedef struct
	{
		u32 can_controller_id;
		u32 filter_id;
		u32 fast_route_table_ptr;
		u32 common_route_table_ptr;
	}LLCG_COMMAND_PARAM_SET_ROUTE_TABLE;

	typedef struct
	{
		X2CAN x2can;
	}LLCG_COMMAND_PARAM_SEND_DATA;

#pragma pack ()

#ifdef __cplusplus
}
#endif

#endif /* VCAN_COMMAND_H */
