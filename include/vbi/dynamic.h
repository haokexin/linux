/* dynamic.h - Dynamic VB VBI interface */

/* Copyright 2010 Wind River Systems, Inc. */

#ifndef _VBI_DYNAMIC_H
#define _VBI_DYNAMIC_H

#include <vbi/types.h>

#ifndef VB_ALIGN_FIELD_64
#if defined(LP64)
#define VB_ALIGN_FIELD_64(decl_var, pad_var)	\
			   __attribute__(( aligned(8) )) \
			   decl_var
#else
#if (__VBI_BYTE_ORDER == __VBI_LITTLE_ENDIAN)
#define VB_ALIGN_FIELD_64(decl_var, pad_var)	\
			    __attribute__(( aligned(8) )) \
			    decl_var; \
			    uint32_t pad_var
# else
#define VB_ALIGN_FIELD_64(decl_var, pad_var)	\
			    __attribute__(( aligned(8) )) \
			    uint32_t pad_var; \
			    decl_var
#endif
#endif
#endif

#define NAME_LENGTH 64
#define BOOTLINE_LENGTH 256

typedef char vbi_name_t[NAME_LENGTH];
typedef char vbi_bootline_t[BOOTLINE_LENGTH];

struct vbi_pci_device
{
	vbi_name_t	Name;			/*  64 bytes */
	uint32_t	Bus;			/*   4 bytes */
	uint32_t	Device;			/*   4 bytes */
	uint32_t	Function;		/*   4 bytes */
	uint32_t	WriteEnable;		/*   4 bytes */
};

struct vb_simple_information
{
	uint64_t	RamAliasSize;			/*   8 bytes */
	uint64_t	RamAliasAddr;			/*   8 bytes */
	uint64_t	RamSize;			/*   8 bytes */
	uint64_t	SystemRamSize;			/*   8 bytes */
	uint64_t	BackgroundModeStartAddr;	/*   8 bytes */
	uint32_t	EnableVmmuOnException;		/*   4 bytes */
	uint32_t	TickTimerFrequency;		/*   4 bytes */
	uint32_t	BoardConfig;			/*   4 bytes */
	vbi_name_t	Name;				/*  64 bytes */
	vbi_bootline_t	BootLine;			/* 256 bytes */
	int32_t		PassFaults;			/*   4 bytes */
	int32_t		TrapIoFault;			/*   4 bytes */
	int32_t		TraceIoFault;			/*   4 bytes */
	uint32_t	Cpu;				/*   4 bytes */
	int32_t		Preload;			/*   4 bytes */
	int32_t		Priority;			/*   4 bytes */
	uint32_t	SystemCallAccessRights;		/*   4 bytes */
	vbi_name_t	BoardType;			/*  64 bytes */
	int32_t		SupervisoryMode;		/*   4 bytes */
	uint32_t	Cores;				/*   4 bytes */
	uint32_t	GuestOS;			/*   4 bytes */
	uint32_t	VMSize;				/*   4 bytes */
	uint32_t	BackgroundModeEnabled;		/*   4 bytes */
	uint32_t	NumOfGuestDevices;		/*   4 bytes */
	uint32_t	NumPciDevices;			/*   4 bytes */
	uint32_t	Reserved[32];			/* reserved 128 bytes */
};


struct vb_information
{
	struct vb_simple_information vbSimpleInformation;
	VB_ALIGN_FIELD_64(uint32_t *pCpuList, pad1); /* already aligned
							- 8 bytes*/
	VB_ALIGN_FIELD_64(uint32_t *pCpuPriorityList, pad2);	/* 8 bytes */
	VB_ALIGN_FIELD_64(vbi_name_t *pDeviceList, pad3);	/* 8 bytes */
	VB_ALIGN_FIELD_64(struct vbi_pci_device *pPciDevices, pad4);
								/* 8 bytes */
	uint32_t    Reserved[32];    /* reserved for later use - 128 bytes */
};


/* Additive/Subtractive - i.e. not in zombie configuration */
int32_t vbiVbSharedMemoryAlloc(uint32_t vbId, vbi_name_t smRegionName, 
				   uint64_t *va, size_t size);
int32_t vbiVbSharedMemoryFree(uint32_t vbId, vbi_name_t smRegionName, 
				  uint64_t va);
int32_t vbiVbRamAlloc(uint32_t vbId, uint64_t *va, size_t size);
int32_t vbiVbRamFree(uint32_t vbId, uint64_t va);

/* Vb Core Migration */

/* Information can be optained for the Virtual boards such as prio and 
 * present Cpu using vbiBoardConfigGet() */

#define VBIMOVE_OPTION_RESUME    0
#define VBIMOVE_OPTION_NO_RESUME 1

#define VBIPRIOSET_OPTION_RESUME    0
#define VBIPRIOSET_OPTION_NO_RESUME 1

#ifdef CONFIG_WRHV_COREVBI_ONLY
static inline int32_t vbi_board_simple_config_get(uint32_t vbId,
				     struct vb_simple_information *pVbInfo)
{
	VBISTAT_VERBOSE(vbi_board_simple_config_get);
	return -1;
}
static inline int32_t vbi_board_config_get(uint32_t vbId,
			       struct vb_information *pVbInfo)
{
	VBISTAT_VERBOSE(vbi_board_config_get);
	return -1;
}
static inline uint32_t vbi_vb_create(struct vb_information *pVbInfo,
					uint32_t options)
{
	VBISTAT_VERBOSE(vbi_vb_create);
	return -1;
}
static inline int32_t vbi_vb_delete(uint32_t vbId)
{
	VBISTAT_VERBOSE(vbi_vb_delete);
	return -1;
}
static inline int32_t vbi_vb_move(uint32_t vbId, uint32_t *pCpuList,
		      uint32_t options)
{
	VBISTAT_VERBOSE(vbi_vb_move);
	return -1;
}
static inline int32_t vbi_vb_priority_set(uint32_t vbId,
		uint32_t *pCpuPriorityList, uint32_t options)
{
	VBISTAT_VERBOSE(vbi_vb_priority_set);
	return -1;
}
#else
/* This returns only the simple contiguous data */
int32_t vbi_board_simple_config_get(uint32_t vbId,
				     struct vb_simple_information *pVbInfo);

/* this returns also the complex data that has variable sixe e.g.device list */
int32_t vbi_board_config_get(uint32_t vbId,
			       struct vb_information *pVbInfo);

uint32_t vbi_vb_create(struct vb_information *pVbInfo, uint32_t options);
int32_t vbi_vb_delete(uint32_t vbId);
int32_t vbi_vb_move(uint32_t vbId, uint32_t *pCpuList,
		      uint32_t options);

int32_t vbi_vb_priority_set(uint32_t vbId, uint32_t *pCpuPriorityList,
			     uint32_t options);
#endif

#endif
