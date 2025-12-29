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

		u32 src_can_bus_id : 8;
		u32 filter_id : 8;
		u32 rsvd1 : 16;   //4Byte

		u32 gmac_timestamp_second_h;
		u32 gmac_timestamp_second_l;
		u32 gmac_timestamp_nanosecond;  //12Byte

		u8 rsvd2[32];
	} CAN2X_PARAMS;   //共计56B，对应上述的 param[56]

	/**
	 * ETH2CAN、AUTOSAR2CAN、SST2CAN等结构体
	*/
	typedef struct {
		u32 rsvd0: 16;         //保留，填0
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
		u32 timestamp_h;         //预留，暂时非必填
		u32 src_vcan_port : 8;   //从哪个vcan通道来的数据，用于下行路由匹配
		u32 des_can_bus_id : 8;  //指定从哪个can通道发送出去, 如果无效则下行路由生效
		u32 rsvd0 : 16;          //保留，填0
		u8 rsvd1[40];            //保留，填0
	} X2CAN_PARAMS;   //共计56B，对应上述的 param[56]

	typedef enum
	{
		VCAN_CMD_SET_CONTROLLER  = 1,       // MCAL CLIENT-> SERVER: 物理Controller初始化完成
		VCAN_CMD_INIT_BASE_ROUTE_TABLE = 2,    // MCAL CLIENT-> SERVER: 发送base路由表的位置
		VCAN_CMD_INIT_COMMON_ROUTE_TABLE = 3,  // MCAL CLIENT-> SERVER: 发送common路由表的位置
		VCAN_CMD_SET_ROUTE_TABLE = 4,       // MCAL CLIENT-> SERVER: 发送Controller+filter对应的转发规则
		VCAN_CMD_GET_IF_CONFIG_BY_DTB = 5,  // MCAL CLIENT-> SERVER: CAN接口是否已经由DTB进行了初始化(如已初始化则放弃MCAL配置)
	} VCan_CommandEnum;

	typedef struct
	{
		u32 can_controller_id; //用于vcan的controller编号，范围0~16
	} VCan_CmdSetController;

	typedef struct
	{
		u32 base_route_table_ptr; //指向LLCG_RouteTableBase
		u32 base_route_table_size;
	} VCan_CmdInitBaseRouteTable;

	typedef struct
	{
		u8 can_controller_id;
		u32 common_route_table_ptr;
		u32 common_route_table_size;
	} VCan_CmdInitCommonRouteTable;

	typedef struct
	{
		u32 can_controller_id;
		u32 filter_id;
		u32 fast_route_table_ptr;
		u32 common_route_table_ptr;
		u32 common_route_table_num;
	} VCan_CmdSetRouteTable;

	typedef enum
	{
		VCAN_CMD_PORT_OPEN_REQ = 10,        // CLIENT-> SERVER: 打开虚拟端口
		VCAN_CMD_PORT_OPEN_EXT_REQ = 11,    // CLIENT-> SERVER: 打开虚拟端口，指定收发模式
		VCAN_CMD_PORT_CLOSE_REQ = 12,       // CLIENT-> SERVER: 关闭虚拟端口
		VCAN_CMD_PORT_SET_ROUTE_REQ = 13,   // CLIENT-> SERVER: 设置虚拟端口路由
		VCAN_CMD_PORT_SEND_REQ = 14,        // CLIENT-> SERVER: 通过MSGBOX发送CAN数据
		VCAN_CMD_PORT_DDR_SMMU_REQ = 15,    // CLIENT-> SERVER: 如果使用DDR需要先发送SMMU请求
		VCAN_CMD_PORT_DDR_SMMU_FREE = 16,   // CLIENT-> SERVER: 发送SMMU释放请求

		VCAN_CMD_PORT_DATA_NOTIFY = 60,     // SERVER-> CLIENT: 数据到达通知
		VCAN_CMD_PORT_DATA_IND = 61,        // SERVER-> CLIENT: 数据内容
	} VCan_PortCommandEnum;

	typedef enum
	{
		VCAN_ERR_OK = 0,
		VCAN_ERR_PARA_ERR = 1,
		VCAN_ERR_RESOURCE_UNAVALIABLE = 2,
		VCAN_ERR_MEM_NOT_ENOUGH = 3,
		VCAN_ERR_NO_PERMISSION = 4,
		VCAN_ERR_CMD_NOT_SUPPORT = 5,
		VCAN_ERR_SERVER_ERR = 6,
	} VCan_CmdErrEnum;

	typedef struct {
		u32 msg_len;
		u32 cmd_id;
	} VCan_PortMsgHeader;

	typedef struct {
		u32 err;
	} VCan_PortGeneralRsp;

	typedef struct {
		u32 port_id;    // 端口
		u32 rx_notify;  // 接收是否通知
	} VCan_PortOpenReq;

	typedef struct {
		u32 err;
		u32 port_id;
		u32 rx_sqb_addr;   // 分配的sqbuffer address，注：同一个client打开的多个端口使用同一份sqbuffer及index?不行
		u32 rx_sqb_index;  // 分配的sqbuffer index
	} VCan_PortOpenRsp;

	#define VCAN_RX_METHOD_TYPE_SERVER_SQB 0    // 由SERVER提供sqbuffer
	#define VCAN_RX_METHOD_TYPE_CLIENT_SQB 1    // 由CLIENT提供sqbuffer
	#define VCAN_RX_METHOD_TYPE_MSGBOX 2        // 采用MSGBOX方式接收

	#define VCAN_TX_METHOD_TYPE_MSGBOX 0    // 0 - 通过MSGBOX发送
	#define VCAN_TX_METHOD_TYPE_SQB    1    // 1 - client提供sq buffer来发送数据，由SERVER轮询(对发送有性能要求时使用)

	typedef struct {
		u32 port_id;    // 端口
		u32 rx_notify;  // 接收是否通知

		u32 rx_method_type;  // 参考VCAN_RX_METHOD_TYPE_*描述，配置为CLIENT_SQB后续字段有效：
		u32 rx_sqb_addr;
		u32 rx_sqb_index;

		u32 tx_method_type;  // 参考VCAN_TX_METHOD_TYPE_*描述；配置为SQB时后续字段有效
		u32 tx_sqb_addr;
		u32 tx_sqb_index;
	} VCan_PortOpenWithBufferReq;

	typedef struct {
		u32 err;
		u32 port_id;
	} VCan_PortOpenWithBufferRsp;

	typedef struct {
		u32 port_id;
	} VCan_PortCloseReq;

	typedef struct {
		u32 err;
		u32 port_id;
	} VCan_PortCloseRsp;

	typedef struct {
		u32 src_port_id;
		X2CAN data;  // x2can
	} VCan_PortSendReq;

	typedef struct {
		u32 err;
		u32 port_id;
	} VCan_PortSendRsp;

	typedef struct {
		u32 port;
	} VCan_PortDataArriveEvt;

	typedef struct {
		u32 port;
		CAN2X data;  // can2x
	} VCan_PortDataInd;

	#define MAX_FILTER_TO_PORTS_NUM 16
	typedef struct {
		u32 port_id;
		u32 hwobj_id;
		u32 canid_type;
		u32 filter_para0;
		u32 filter_para1;
		u32 dest_num;
		u8 dest_ports[MAX_FILTER_TO_PORTS_NUM];
		u8 eth_mac[6];
		u8 pad0[2];
	} VCan_PortSetRouteReq;

	typedef struct {
		u32 err;
		u32 port_id;
	} VCan_PortSetRouteRsp;

	typedef struct {
		u32 ddr_addr_H;
		u32 ddr_addr_L;
		u32 ddr_size;
	} VCan_PortDdrSmmuAllocReq;

	typedef struct {
		u32 err;
		u32 smm_va;  // SMMU返回的地址
	} VCan_PortDdrSmmuAllocRsp;

	typedef struct {
		u32 ddr_addr_H;
		u32 ddr_addr_L;
		u32 ddr_size;;
	} VCan_PortDdrSmmuFreeReq;

	typedef struct {
		u32 err;
	} VCan_PortDdrSmmuFreeRsp;
#pragma pack ()

#ifdef __cplusplus
}
#endif

#endif /* VCAN_COMMAND_H */
