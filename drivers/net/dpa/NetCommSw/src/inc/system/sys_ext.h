/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SYS_EXT_H
#define __SYS_EXT_H

#include "std_ext.h"


/**************************************************************************//**
 @Group         sys_grp     System Interfaces

 @Description   Linux system programming interfaces.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         sys_gen_grp     System General Interface

 @Description   General definitions, structures and routines of the linux
                system programming interface.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   System Modules and Sub-Modules Enumeration

                This enumeration type is expected by most system routines in
                order to identify the type of the referenced module or sub-module.
                The lower values of this enumeration type are modules, and the
                higher values are sub-modules.
*//***************************************************************************/
typedef enum e_SysModule
{
    e_SYS_MODULE_NONE = 0,

    /* ----------- Modules ----------- */
    e_SYS_MODULE_PLATFORM,
    e_SYS_MODULE_DEV_MNG,
    e_SYS_MODULE_DATA_POOL_MNG,
    e_SYS_MODULE_TRAFFIC_MNG,
    e_SYS_MODULE_APP,
    /* Stacks */
    e_SYS_MODULE_PCI_STACK,
    e_SYS_MODULE_ESDHC_STACK,
    e_SYS_MODULE_RIO_STACK,
    e_SYS_MODULE_I2C_STACK,
    e_SYS_MODULE_SPI_STACK,
    e_SYS_MODULE_USB_STACK,
    e_SYS_MODULE_IMA_STACK,
    /* Wrappers */
    e_SYS_MODULE_WRP_ECM,
    e_SYS_MODULE_WRP_ARBITER,
    e_SYS_MODULE_WRP_DUART,
    e_SYS_MODULE_WRP_UART,
    e_SYS_MODULE_WRP_QE,
    e_SYS_MODULE_WRP_UPC,
    e_SYS_MODULE_WRP_UCC_ATM,
    e_SYS_MODULE_WRP_UCC_POS,
    e_SYS_MODULE_WRP_UCC_GETH,
    e_SYS_MODULE_WRP_UCC_HDLC,
    e_SYS_MODULE_WRP_UCC_TRNS,
    e_SYS_MODULE_WRP_UCC_UART,
    e_SYS_MODULE_WRP_QMC,
    e_SYS_MODULE_WRP_QMC_HDLC,
    e_SYS_MODULE_WRP_QMC_TRANS,
    e_SYS_MODULE_WRP_L2_SWITCH,
    e_SYS_MODULE_WRP_SI,
    e_SYS_MODULE_WRP_TDM,
    e_SYS_MODULE_WRP_MCC,
    e_SYS_MODULE_WRP_MCC_HDLC,
    e_SYS_MODULE_WRP_MCC_TRANS,
    e_SYS_MODULE_WRP_MCC_SS7,
    e_SYS_MODULE_WRP_PPPOHT,
    e_SYS_MODULE_WRP_IW_PPPOHT,
    e_SYS_MODULE_WRP_PCI,
    e_SYS_MODULE_WRP_ESDHC,
    e_SYS_MODULE_WRP_RIO,
    e_SYS_MODULE_WRP_I2C,
    e_SYS_MODULE_WRP_SPI,
    e_SYS_MODULE_WRP_ETSEC,
    e_SYS_MODULE_WRP_SRIO,
    e_SYS_MODULE_WRP_IMA,
    e_SYS_MODULE_WRP_MTC,
    e_SYS_MODULE_WRP_QE_IW,
    e_SYS_MODULE_WRP_QE_FILTERING,
    e_SYS_MODULE_WRP_QE_RTC,
    e_SYS_MODULE_WRP_VP,
    e_SYS_MODULE_WRP_GTIMERS,
    e_SYS_MODULE_WRP_DMA,
    e_SYS_MODULE_WRP_PTP,

    /* Bus Device Drivers */
    e_SYS_MODULE_MPC8568_RIO_DEV,
    e_SYS_MODULE_MPC8568_PCI_DEV,
    e_SYS_MODULE_MPC836X_PCI_DEV,
    e_SYS_MODULE_MPC837x_PCI_DEV,
    e_SYS_MODULE_MPC832X_PCI_DEV,
    e_SYS_MODULE_MPC8568_PCIE_DEV,
    e_SYS_MODULE_MPC8569_PCIE_DEV,
    e_SYS_MODULE_MPC837x_PCIE_DEV,
    e_SYS_MODULE_MPC8568_SPI_FLASH_DEV,
    e_SYS_MODULE_MPC8568_SPI_LB_DEV,
    e_SYS_MODULE_SD_DEV,
    e_SYS_MODULE_SPD_I2C_DEV,
    e_SYS_MODULE_DS1374_I2C_DEV,
    e_SYS_MODULE_PCA9555_I2C_DEV,
    e_SYS_MODULE_M24256_I2C_DEV,
    e_SYS_MODULE_AT24C01A_I2C_DEV,

    /* Must close the modules list and open the sub-modules list */
    e_SYS_SUBMODULE_DUMMY_FIRST,

    /* --------- Sub-modules --------- */
    e_SYS_SUBMODULE_DATA_POOL,
    e_SYS_SUBMODULE_PG,
    e_SYS_SUBMODULE_TG,
    e_SYS_SUBMODULE_TA,
    e_SYS_SUBMODULE_DEC,
    e_SYS_SUBMODULE_PART,
    e_SYS_SUBMODULE_IPIC,
    e_SYS_SUBMODULE_EPIC,
    e_SYS_SUBMODULE_QE_IC,
    e_SYS_SUBMODULE_LAW,
    e_SYS_SUBMODULE_DDR,
    e_SYS_SUBMODULE_LBC,
    e_SYS_SUBMODULE_ECM,
    e_SYS_SUBMODULE_ARBITER,
    e_SYS_SUBMODULE_L2,
    e_SYS_SUBMODULE_QE,
    e_SYS_SUBMODULE_QE_TIMERS,
    e_SYS_SUBMODULE_GTIMERS,
    e_SYS_SUBMODULE_PAR_IO,
    e_SYS_SUBMODULE_GPIO,
    e_SYS_SUBMODULE_SI,
    e_SYS_SUBMODULE_TDM,
    e_SYS_SUBMODULE_TDM_RX_FRAME,
    e_SYS_SUBMODULE_TDM_TX_FRAME,
    e_SYS_SUBMODULE_MCC,
    e_SYS_SUBMODULE_MCC_HDLC_CH,
    e_SYS_SUBMODULE_MCC_TRANS_CH,
    e_SYS_SUBMODULE_MCC_SS7_CH,
    e_SYS_SUBMODULE_UPC,
    e_SYS_SUBMODULE_DUART,
    e_SYS_SUBMODULE_I2C_CTRL,
    e_SYS_SUBMODULE_DMA,
    e_SYS_SUBMODULE_DMA_CH,
    e_SYS_SUBMODULE_PM,
    e_SYS_SUBMODULE_RTC,
    e_SYS_SUBMODULE_TLU,
    e_SYS_SUBMODULE_SEC,
    e_SYS_SUBMODULE_PCI_CTRL,
    e_SYS_SUBMODULE_PCIE_CTRL,
    e_SYS_SUBMODULE_ESDHC,
    e_SYS_SUBMODULE_SRIO_PORT,
    e_SYS_SUBMODULE_SRIO_MU,
    e_SYS_SUBMODULE_SPI_CTRL,
    e_SYS_SUBMODULE_ETSEC,
    e_SYS_SUBMODULE_USB_CTRL,
    e_SYS_SUBMODULE_UCC_GETH,
    e_SYS_SUBMODULE_UCC_ATM_COMMON,
    e_SYS_SUBMODULE_UCC_ATM_AAL2_COMMON,
    e_SYS_SUBMODULE_UCC_ATM_CTRL,
    e_SYS_SUBMODULE_UCC_POS_COMMON,
    e_SYS_SUBMODULE_UCC_POS_CTRL,
    e_SYS_SUBMODULE_EOS_LINK,
    e_SYS_SUBMODULE_PPPOS_LINK,
    e_SYS_SUBMODULE_UTOPIA_PORT,
    e_SYS_SUBMODULE_UCC_ATM_POLICER,
    e_SYS_SUBMODULE_UCC_ATM_AAL0,
    e_SYS_SUBMODULE_UCC_ATM_AAL5,
    e_SYS_SUBMODULE_UCC_ATM_AAL1,
    e_SYS_SUBMODULE_UCC_ATM_AAL2,
    e_SYS_SUBMODULE_UCC_ATM_AAL2_TQD,
    e_SYS_SUBMODULE_UCC_ATM_AAL2_CID,
    e_SYS_SUBMODULE_UCC_HDLC,
    e_SYS_SUBMODULE_UCC_TRNS,
    e_SYS_SUBMODULE_UCC_UART,
    e_SYS_SUBMODULE_QMC,
    e_SYS_SUBMODULE_QMC_HDLC_CH,
    e_SYS_SUBMODULE_QMC_TRANS_CH,
    e_SYS_SUBMODULE_L2_SWITCH,
    e_SYS_SUBMODULE_L2_SWITCH_PORT,
    e_SYS_SUBMODULE_PPPOHT,
    e_SYS_SUBMODULE_PPPOHT_BUNDLE,
    e_SYS_SUBMODULE_PPPOHT_LINK,
    e_SYS_SUBMODULE_PPPOHT_RX_FBP,
    e_SYS_SUBMODULE_PPPOHT_TX_FBPS,
    e_SYS_SUBMODULE_IW_PPPOHT,
    e_SYS_SUBMODULE_IW_PPPOHT_BUNDLE,
    e_SYS_SUBMODULE_IW_PPPOHT_LINK,
    e_SYS_SUBMODULE_IMA,
    e_SYS_SUBMODULE_IMA_GROUP_PORT,
    e_SYS_SUBMODULE_IMA_GROUP,
    e_SYS_SUBMODULE_IMA_LINK,
    e_SYS_SUBMODULE_MTC_COMMON,
    e_SYS_SUBMODULE_MTC,
    e_SYS_SUBMODULE_MTC_UCC,
    e_SYS_SUBMODULE_MTC_MCC,
    e_SYS_SUBMODULE_MTC_MCC_COMMON,
    e_SYS_SUBMODULE_IW_COMMON,
    e_SYS_SUBMODULE_IW_DEV,
    e_SYS_SUBMODULE_IW_CONN,
    e_SYS_SUBMODULE_IW_CDESC,
    e_SYS_SUBMODULE_IW_MCAST_GROUP,
    e_SYS_SUBMODULE_IW_IP_REASS,
    e_SYS_SUBMODULE_IW_PLCR,
    e_SYS_SUBMODULE_IW_QM,
    e_SYS_SUBMODULE_IW_IPSEC,
    e_SYS_SUBMODULE_QE_FILTER,
    e_SYS_SUBMODULE_QE_TABLE,
    e_SYS_SUBMODULE_QE_RTC,
    e_SYS_SUBMODULE_VP_DEV,
    e_SYS_SUBMODULE_VP_PORT,
    e_SYS_SUBMODULE_MII_MNG,
    e_SYS_SUBMODULE_PTP,
    e_SYS_SUBMODULE_SATA,
    e_SYS_SUBMODULE_SERDES,
    e_SYS_SUBMODULE_BM,
    e_SYS_SUBMODULE_BM_PORTAL,
    e_SYS_SUBMODULE_BM_CE_PORTAL,
    e_SYS_SUBMODULE_BM_CI_PORTAL,
    e_SYS_SUBMODULE_QM,
    e_SYS_SUBMODULE_QM_PORTAL,
    e_SYS_SUBMODULE_QM_CE_PORTAL,
    e_SYS_SUBMODULE_QM_CI_PORTAL,
    e_SYS_SUBMODULE_FM,
    e_SYS_SUBMODULE_FM_MURAM,
    e_SYS_SUBMODULE_FM_PORT_HO,
    e_SYS_SUBMODULE_FM_PORT_10GRx,
    e_SYS_SUBMODULE_FM_PORT_1GRx,
    e_SYS_SUBMODULE_FM_PORT_10GTx,
    e_SYS_SUBMODULE_FM_PORT_1GTx,
    e_SYS_SUBMODULE_FM_PORT_10GMAC,
    e_SYS_SUBMODULE_FM_PORT_1GMAC,
    e_SYS_SUBMODULE_IW_IPHC_HC,
    e_SYS_SUBMODULE_IW_IPHC_HDEC,
    /* Must close the sub-modules list */
    e_SYS_SUBMODULE_DUMMY_LAST

} e_SysModule;

/**************************************************************************//**
 @Description   System Object Descriptor

                This is the generic object descriptor for the system layout.
                It contains only the module enumeration type, according to
                which the system can decide whether this is a module descriptor
                or a sub-module descriptor.
*//***************************************************************************/
typedef struct t_SysObjectDescriptor
{
    e_SysModule module; /**< Module/sub-module type */

} t_SysObjectDescriptor;

/**************************************************************************//**
 @Description   Module initialization function (with no parameters)
*//***************************************************************************/
typedef t_Handle (t_SysModuleInitFuncNoParams)(void);

/**************************************************************************//**
 @Description   Module initialization function (with pointer to parameters)
*//***************************************************************************/
typedef t_Handle (t_SysModuleInitFuncWithParams)(void *p_ModuleParams);

/**************************************************************************//**
 @Description   System Module Descriptor

                This structure should be used in the use case layout to request
                initialization of a module object.

                A module is by definition a singleton object, therefore no ID
                is required in this structure.
*//***************************************************************************/
typedef struct t_SysModuleDescriptor
{
    e_SysModule module;                     /**< Module type */
    void        *p_Settings;                /**< Pointer to the module's settings */

    struct
    {
        union
        {
            t_SysModuleInitFuncNoParams     *f_InitNoParams;
                                            /**< Module initialization routine (with no parameters) */
            t_SysModuleInitFuncWithParams   *f_InitWithParams;
                                            /**< Module initialization routine (with parameters) */
        } cbs;
        t_Error                             (*f_Free)(t_Handle h_Module);
                                            /**< Module free routine */
    } moduleInterface;

} t_SysModuleDescriptor;


/**************************************************************************//**
 @Description   System Sub-Module Descriptor

                This structure should be used in the use case layout to request
                initialization of a sub-module object.

                There may be multiple objects of the same sub-module type,
                therefore the ID variable must be unique for each instance of
                the same type.

                The f_UpdateSettings callback routine and the idIncrement value
                may be used to clone multiple objects of the same type. If the
                callback is not NULL, the system shall call this routine to
                generate a new settings structure for the next object, and then
                initialize the object. The system will continue to invoke the
                callback routine until a NULL pointer is returned. The callback
                routine is responsible for allocating and freeing the cloned
                settings structure. The original settings must not be modified.
*//***************************************************************************/
typedef struct t_SysSubModuleDescriptor
{
    e_SysModule module;             /**< Sub-module type */
    uint64_t    id;                 /**< The sub-module ID */
    void        *p_Settings;        /**< Pointer to the sub-module's settings */
    void *      (*f_UpdateSettings)(uint64_t newId, void *p_OrigSettings, void *p_PrevSettings);
                                    /**< Callback routine for cloning objects */
    uint16_t    idIncrement;        /**< Amount by which to increment the
                                         Sub-module ID when cloning objects */
} t_SysSubModuleDescriptor;


/**************************************************************************//**
 @Collection    Object Identifier Macros
 @{
*//***************************************************************************/
#define SYS_NULL_OBJECT_ID          0xFFFFFFFF
                                    /**< Object ID representing no object */

#define CAST_ID_TO_HANDLE(_id)      ((t_Handle)((uintptr_t)_id))
                                    /**< Macro for casting an object ID to a handle */
#define CAST_HANDLE_TO_ID(_h)       ((uintptr_t)(_h))
                                    /**< Macro for casting a handle to an object ID */
/* @} */

/**************************************************************************//**
 @Collection    Macros for Advanced Configuration Requests
 @{
*//***************************************************************************/
#define SYS_MAX_ADV_CONFIG_ARGS     4
                                    /**< Maximum number of arguments in
                                         an advanced confiuration entry */

#define ADV_CONFIG_DONE             NULL, { 0 }
                                    /**< Marks the end of an advanced
                                         configuration requests array \hideinitializer */
#define ADV_CONFIG_NONE             (t_SysObjectAdvConfigEntry[]){ {ADV_CONFIG_DONE} }
                                    /**< Marks an empty advanced
                                         configuration requests array \hideinitializer */
/* @} */

/**************************************************************************//**
 @Description   System Object Advanced Configuration Entry

                This structure represents a single request for an advanced
                configuration call on the initialized object. An array of such
                requests may be contained in the settings structure of the
                corresponding object.

                The maximum number of arguments is limited to #SYS_MAX_ADV_CONFIG_ARGS.
*//***************************************************************************/
typedef struct t_SysObjectAdvConfigEntry
{
    void        *p_Function;    /**< Pointer to advanced configuration routine */
#if 0
     /*@@@@ cannot use uint64_t (CW bug?) */
#endif
    uintptr_t    args[SYS_MAX_ADV_CONFIG_ARGS];
                                /**< Array of arguments for the specified routine;
                                     All arguments should be casted to uint32_t. */
} t_SysObjectAdvConfigEntry;


/** @} */ /* end of sys_gen_grp */

/**************************************************************************//**
 @Group         sys_app_grp     System Interface for Applications

 @Description   Linux system programming interface for applications and
                use cases.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection    Test Descriptor Text Length Limits
 @{
*//***************************************************************************/
#define SYS_MAX_TEST_MODULE_NAME_LENGTH     32  /**< Maximum test module name length */
#define SYS_MAX_TEST_GROUP_NAME_LENGTH      64  /**< Maximum test group name length */
#define SYS_MAX_TEST_DESCRIPTION_LENGTH     128 /**< Maximum test description length */
/* @} */

/**************************************************************************//**
 @Description   Test Descriptor

                This structure should be used in the use case layout to request
                execution of a specific test.
*//***************************************************************************/
typedef struct t_SysTestDescriptor
{
    char        testModule[SYS_MAX_TEST_MODULE_NAME_LENGTH];
                /**< Test module name */
    char        testGroup[SYS_MAX_TEST_GROUP_NAME_LENGTH];
                /**< Test group name */
    uint16_t    testId;
                /**< Test identifier */
    char        description[SYS_MAX_TEST_DESCRIPTION_LENGTH];
                /**< Test description */
    t_Error     (*f_RunTest)(void *p_TestParam);
                /**< Test routine */
    void        (*f_KillTest)(void);
                /**< Test termination routine */
    void        *p_TestParam;
                /**< Pointer to the test parameters structure */

} t_SysTestDescriptor;


/**************************************************************************//**
 @Description   Runtime Layout Structure

                This structure describes a requested runtime layout, and
                contains both logical object descriptors and test descriptors.
*//***************************************************************************/
typedef struct t_SysRuntimeLayout
{
    t_SysObjectDescriptor   **p_LogicalObjects;     /**< Logical-layer object descriptors */
    t_SysTestDescriptor     **p_TestDescriptors;    /**< Test descriptors */

} t_SysRuntimeLayout;

/**************************************************************************//**
 @Description   Peripheral Layout Structure

                This structure describes a peripheral layout, and contains both
                peripheral object descriptors and an array of runtime layouts
*//***************************************************************************/
typedef struct t_SysPeripheralLayout
{
    t_SysObjectDescriptor   **p_PeripheralObjects;  /**< Peripheral-layer object descriptors */
    t_SysRuntimeLayout      *p_RuntimeLayouts;      /**< An array of runtime layouts */

} t_SysPeripheralLayout;

/**************************************************************************//**
 @Description   System Layout Structure

                This structure describes a system layout, and contains both
                system object descriptors and an array of peripheral layouts
*//***************************************************************************/
typedef struct t_SysSystemLayout
{
    t_SysObjectDescriptor   **p_SystemObjects;      /**< System-layer object descriptors */
    t_SysPeripheralLayout   *p_PeripheralLayouts;   /**< An array of peripheral layouts */

} t_SysSystemLayout;

/**************************************************************************//**
 @Description   Use Case Layout Structure

                This structure describes a use case runtime layout, and
                contains a pointer to the platform settings as well as an
                array of system layouts.
*//***************************************************************************/
typedef struct t_SysUseCaseLayout
{
    t_SysObjectDescriptor   *p_PlatformObject;      /**< Platform object descriptor */
    t_SysSystemLayout       *p_SystemLayouts;       /**< An array of system layouts */

} t_SysUseCaseLayout;


/**************************************************************************//**
 @Collection    Macros for Building the Use Case Layout
 @{
*//***************************************************************************/
#define SYS_SYSTEM_OBJECTS(...)     (t_SysObjectDescriptor*[]){ __VA_ARGS__, NULL }
                                    /**< Defines an array of system object descriptors \hideinitializer */
#define SYS_PERIPHERAL_OBJECTS(...) (t_SysObjectDescriptor*[]){ __VA_ARGS__, NULL }
                                    /**< Defines an array of peripheral object descriptors \hideinitializer */
#define SYS_LOGICAL_OBJECTS(...)    (t_SysObjectDescriptor*[]){ __VA_ARGS__, NULL }
                                    /**< Defines an array of logical object descriptors \hideinitializer */
#define SYS_TEST_DESCRIPTORS(...)   (t_SysTestDescriptor*[]){ __VA_ARGS__, NULL }
                                    /**< Defines an array of test descriptors \hideinitializer */

#define SYS_BEGIN_SYSTEM_LAYOUTS \
        (t_SysSystemLayout[]){      /**< Starts an array of system layout structures \hideinitializer */
#define SYS_END_SYSTEM_LAYOUTS \
        , { NULL, NULL } }          /**< Ends an array of system layout structures \hideinitializer */

#define SYS_BEGIN_PERIPHERAL_LAYOUTS \
        (t_SysPeripheralLayout[]){  /**< Starts an array of peripheral layout structures \hideinitializer */
#define SYS_END_PERIPHERAL_LAYOUTS \
        , { NULL, NULL } }          /**< Ends an array of peripheral layout structures \hideinitializer */

#define SYS_BEGIN_RUNTIME_LAYOUTS \
        (t_SysRuntimeLayout[]){     /**< Starts an array of runtime layout structures \hideinitializer */
#define SYS_END_RUNTIME_LAYOUTS \
        , { NULL, NULL } }          /**< Ends an array of runtime layout structures \hideinitializer */

/* @} */

/**************************************************************************//**
 @Collection    Macros for Defining a Module Interface
 @{
*//***************************************************************************/
#define MODULE_INTERFACE_VOID_PARAM(_initFunc, _freeFunc) \
    { .cbs.f_InitNoParams = (_initFunc), .f_Free = (_freeFunc) }
                    /**< Defines a module interface (with no initialization parameters) \hideinitializer */

#define MODULE_INTERFACE_WITH_PARAM(_initFunc, _freeFunc) \
    { .cbs.f_InitWithParams = (t_SysModuleInitFuncWithParams *)(_initFunc), .f_Free = (_freeFunc) }
                    /**< Defines a module interface (with initialization parameters) \hideinitializer */
/* @} */

typedef t_Error (t_UpdateSettingsFunc)(uint64_t id, void *p_SubmoduleSettings);


/**************************************************************************//**
 @Function      SYS_Init

 @Description   System initialization routine.

                This routine calls the SYS_InternalInit() routine to initialize
                the internal system structures and services, such as memory
                management, objects repository and more.

 @Return        None.
*//***************************************************************************/
void SYS_Init(void);

/**************************************************************************//**
 @Function      SYS_Free

 @Description   System termination routine.

                This routine releases all internal structures that were
                initialized by the SYS_Init() routine.

 @Return        None.
*//***************************************************************************/
void SYS_Free(void);

/**************************************************************************//**
 @Function      SYS_RunLayout

 @Description   Executes a given use case layout.

                This routine may be called to process and execute a use case
                layout structure. The user should pass a pointer to a valid
                use case layout structure, and the system would initialize
                the requested objects and execute the requested test routines.

 @Param[in]     p_UcLayout - Pointer to use case layout structure.

 @Return        Zero for success; Non-zero value on error.
*//***************************************************************************/
int SYS_RunLayout(t_SysUseCaseLayout *p_UcLayout);

/**************************************************************************//**
 @Function      SYS_GetHandle

 @Description   Returns a specific object handle.

                This routine may be used to get the handle of any module or
                sub-module in the system.

                For singleton objects, it is recommended to use the
                SYS_GetUniqueHandle() routine.

 @Param[in]     module  - Module/sub-module type.
 @Param[in]     id      - For sub-modules, this is the unique object ID;
                          For modules, this value must always be zero.

 @Return        The handle of the specified object if exists;
                NULL if the object is not known or is not initialized.
*//***************************************************************************/
t_Handle SYS_GetHandle(e_SysModule module, uint64_t id);

/**************************************************************************//**
 @Function      SYS_GetUniqueHandle

 @Description   Returns a specific object handle (for singleton objects).

                This routine may be used to get the handle of any singleton
                module or sub-module in the system.

                This routine simply calls the SYS_GetHandle() routine with
                the \c id parameter set to zero.

 @Param[in]     module - Module/sub-module type.

 @Return        The handle of the specified object if exists;
                NULL if the object is not known or is not initialized.
*//***************************************************************************/
static __inline__ t_Handle SYS_GetUniqueHandle(e_SysModule module)
{
    return SYS_GetHandle(module, 0);
}

/**************************************************************************//**
 @Function      SYS_CreateObject

 @Description   Initializes a module or sub-module using the information
                provided in the object descriptor.

 @Param[in]     p_ObjectDesc - Object descriptor.

 @Return        E_OK in case of success; error code otherwise.
*//***************************************************************************/
t_Error SYS_CreateObject(t_SysObjectDescriptor *p_ObjectDesc);

/**************************************************************************//**
 @Function      SYS_FreeObject

 @Description   Frees a module or sub-module.

 @Param[in]     module  - Module/sub-module type.
 @Param[in]     id      - For sub-modules, this is the unique object ID;
                          For modules, this value must always be zero.

 @Return        E_OK in case of success; error code otherwise.
*//***************************************************************************/
t_Error SYS_FreeObject(e_SysModule module, uint64_t id);

/**************************************************************************//**
 @Function      USER_BuildLayout

 @Description   User-implemented routine for building the use case layout.

                Any memory allocations in this routine must be released in the
                USER_FreeLayout() routine.

 @Return        Pointer to the use case layout; NULL on failure.
*//***************************************************************************/
t_SysUseCaseLayout * USER_BuildLayout(void);

/**************************************************************************//**
 @Function      USER_FreeLayout

 @Description   User-implemented routine for freeing the use case layout.

                This routine is called by the main routine in order to free
                the use case layout before terminating the program.

 @Param[in]     p_UseCaseLayout - Pointer to the use case layout to free.

 @Return        None.
*//***************************************************************************/
void USER_FreeLayout(t_SysUseCaseLayout *p_UseCaseLayout);


/**************************************************************************//**
 @Function      USER_ModuleInit

 @Description   User-implemented routine for system initialization. It calls
                the necessary routines to build and run the use case layout.

                This must be called from the kernel module's init routine.

 @Return        0 on sucess, negative error code otherwise.
*//***************************************************************************/
int USER_ModuleInit(void);

/**************************************************************************//**
 @Function      USER_ModuleExit

 @Description   User-implemented routine for freeing system resources. out.

                This must be called from the kernel module's exit routine.

 @Return        None.
*//***************************************************************************/
void USER_ModuleExit(void);

/** @} */ /* end of sys_app_grp */

/**************************************************************************//**
 @Group         sys_mod_grp     System Interface for Modules

 @Description   Linux system programming interface for modules (such as
                wrappers and others).

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   Sub-Modules Registration Parameters.

                This structure provides registration parameters for a set of
                sub-modules. Each module should register its own sub-modules
                in the system using the SYS_RegisterSubModules() routine.
*//***************************************************************************/
typedef struct t_SysSubModuleRegisterParam
{
    e_SysModule owner;              /**< The type of the owner module */
    uint8_t     numOfSubModules;    /**< Number of sub-modules in the \c p_SubModules array */
    e_SysModule *p_SubModules;      /**< An array of sub-module types, of size \c numOfSubModules */
    t_Error     (*f_InitSubModule)(t_Handle h_Module, t_SysSubModuleDescriptor *p_SubModuleDesc);
                                    /**< Sub-modules initialization routine */
    t_Error     (*f_FreeSubModule)(t_Handle h_Module, e_SysModule subModule, uint64_t id);
                                    /**< Sub-modules free routine */
    t_Handle    (*f_GetSubModule)(t_Handle h_Module, e_SysModule subModule, uint64_t id);
                                    /**< Routine for retrieving sub-module handles */
} t_SysSubModuleRegisterParam;

/**************************************************************************//**
 @Description   Sub-Modules Deregistration Parameters.

                This structure provides deregistration parameters for a set of
                sub-modules. Each module should deregister its own sub-modules
                from the system during its termination and cleanup phase, using
                the SYS_UnregisterSubModules() routine.
*//***************************************************************************/
typedef struct t_SysSubModuleUnregisterParam
{
    e_SysModule owner;              /**< The type of the owner module */
    uint8_t     numOfSubModules;    /**< Number of sub-modules in the \c p_SubModules array */
    e_SysModule *p_SubModules;      /**< An array of sub-module types, of size \c numOfSubModules */

} t_SysSubModuleUnregisterParam;

/**************************************************************************//**
 @Function      SYS_RegisterSubModules

 @Description   Registers a set of sub-modules as being owned by a given module.

                This routine should be called by every system module in order
                to register its sub-modules in the system. Without registration,
                sub-modules cannot be initialized through the use case layout.

 @Param[in]     p_RegParam - Pointer to registration parameters structure.

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error SYS_RegisterSubModules(t_SysSubModuleRegisterParam *p_RegParam);

/**************************************************************************//**
 @Function      SYS_UnregisterSubModules

 @Description   Unregisters a set of sub-modules from their owner module.

                This routine should be called by every system module during
                its termination and cleanup phase, in order to deregister its
                sub-modules.

 @Param[in]     p_UnregParam - Pointer to de-registration parameters structure.

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error SYS_UnregisterSubModules(t_SysSubModuleUnregisterParam *p_UnregParam);

/**************************************************************************//**
 @Function      SYS_ForceHandle

 @Description   Forces a handle for a specific object in the system.

                This routine allows forcing an object handle into the system
                and thus bypassing the normal initialization flow.

                The forced handle must be removed as soon as it is not valid
                anymore, using the SYS_RemoveForcedHandle() routine.

 @Param[in]     module      - The object (module/sub-module) type.
 @Param[in]     id          - Unique object ID;
 @Param[in]     h_Module    - The object handle;

 @Return        E_OK on success; Error code otherwise.

 @Cautions      This routine must not be used in normal flow - it serves only
                rare and special cases in platform initialization.
*//***************************************************************************/
t_Error SYS_ForceHandle(e_SysModule module, uint64_t id, t_Handle h_Module);

/**************************************************************************//**
 @Function      SYS_RemoveForcedHandle

 @Description   Removes a previously forced handle of a specific object.

                This routine must be called to remove object handles that
                were previously forced using the SYS_ForceHandle() routine.

 @Param[in]     module      - The object (module/sub-module) type.
 @Param[in]     id          - Unique object ID;

 @Return        None.

 @Cautions      This routine must not be used in normal flow - it serves only
                rare and special cases in platform initialization.
*//***************************************************************************/
void SYS_RemoveForcedHandle(e_SysModule module, uint64_t id);

/** @} */ /* end of sys_mod_grp */
/** @} */ /* end of sys_grp */


#define PARAMS(_num, _params)   ADV_CONFIG_PARAMS_##_num _params
#define NO_PARAMS

#define ADV_CONFIG_PARAMS_1(_type) \
    , (_type)p_Entry->args[0]

#define ADV_CONFIG_PARAMS_2(_type0, _type1) \
    , (_type0)p_Entry->args[0], (_type1)p_Entry->args[1]

#define ADV_CONFIG_PARAMS_3(_type0, _type1, _type2) \
    , (_type0)p_Entry->args[0], (_type1)p_Entry->args[1], (_type2)p_Entry->args[2]

#define ADV_CONFIG_PARAMS_4(_type0, _type1, _type2, _type3) \
    , (_type0)p_Entry->args[0], (_type1)p_Entry->args[1], (_type2)p_Entry->args[2], (_type3)p_Entry->args[3]


#define SET_ADV_CONFIG_ARGS_1(_arg0)        \
    p_Entry->args[0] = (uintptr_t )(_arg0);   \

#define SET_ADV_CONFIG_ARGS_2(_arg0, _arg1) \
    p_Entry->args[0] = (uintptr_t )(_arg0);   \
    p_Entry->args[1] = (uintptr_t )(_arg1);   \

#define SET_ADV_CONFIG_ARGS_3(_arg0, _arg1, _arg2)  \
    p_Entry->args[0] = (uintptr_t )(_arg0);           \
    p_Entry->args[1] = (uintptr_t )(_arg1);           \
    p_Entry->args[2] = (uintptr_t )(_arg2);           \

#define SET_ADV_CONFIG_ARGS_4(_arg0, _arg1, _arg2, _arg3)   \
    p_Entry->args[0] = (uintptr_t )(_arg0);                   \
    p_Entry->args[1] = (uintptr_t )(_arg1);                   \
    p_Entry->args[2] = (uintptr_t )(_arg2);                   \
    p_Entry->args[3] = (uintptr_t )(_arg3);                   \

#define ARGS(_num, _params) SET_ADV_CONFIG_ARGS_##_num _params
#define NO_ARGS

#define ADD_ADV_CONFIG(_func, _param)       \
    do {                                    \
        if (i<max){                         \
            p_Entry = &p_Entrys[i];         \
            p_Entry->p_Function = _func;    \
            _param                          \
            i++;                            \
        }                                   \
        else                                \
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, \
                         ("Number of advanced-configuration entries exceeded")); \
    } while (0)

#define ADD_ADV_CONFIG_FIND_LOCATION(_func)                                     \
    for (i=0; TRUE; i++) {                                                      \
        if (i == max)                                                           \
            RETURN_ERROR(MAJOR, E_INVALID_VALUE,                                \
                         ("Number of advanced-configuration entries exceeded"));\
        p_Entry = &p_Entrys[i];                                             \
        if (!p_Entry->p_Function || (p_Entry->p_Function == _func))         \
            break;                                                          \
    }                                                                       \

#define ADD_ADV_CONFIG_START(_p_Entries, _maxEntries)           \
    {                                                           \
        t_SysObjectAdvConfigEntry   *p_Entry;                   \
        t_SysObjectAdvConfigEntry   *p_Entrys = (_p_Entries);   \
        int                         i=0, max = (_maxEntries);   \

#define ADD_ADV_CONFIG_END \
    }

#define ADV_CONFIG_CHECK_START(_p_Entry)                        \
    {                                                           \
        t_SysObjectAdvConfigEntry   *p_Entry = _p_Entry;        \
        t_Error                     errCode;                    \

#define ADV_CONFIG_CHECK(_handle, _func, _params)               \
        if (p_Entry->p_Function == _func)                       \
        {                                                       \
            errCode = _func(_handle _params);                   \
        } else

#define ADV_CONFIG_CHECK_END                                    \
        {                                                       \
            REPORT_ERROR(MAJOR, E_INVALID_SELECTION,            \
                         ("Advanced configuration routine"));   \
            return NULL;                                        \
        }                                                       \
        if (errCode != E_OK)                                    \
        {                                                       \
            REPORT_ERROR(MAJOR, errCode, NO_MSG);               \
            return NULL;                                        \
        }                                                       \
    }


#endif /* __SYS_EXT_H */
