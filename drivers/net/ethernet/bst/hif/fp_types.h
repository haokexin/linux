/*
 * fp_types.h
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C)2024Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _FP_TYPES_H_
#define _FP_TYPES_H_

typedef unsigned int    UINT;
typedef int             INT;
typedef unsigned int    UINT32;
typedef unsigned short  USHORT;
typedef short           SHORT;
typedef unsigned char   UCHAR;
typedef char            CHAR;
typedef unsigned long   ULONG;
typedef unsigned long long   UINT64;
typedef long long   INT64;


typedef int             int32;
typedef unsigned int    uint32;
typedef short           int16;
typedef unsigned short  uint16;
typedef signed char     int8;
typedef unsigned char   uint8;
typedef unsigned char   uchar;


typedef enum {

    TABLE_TYPE_ROUTE,
    TABLE_TYPE_BRIDGE,
    TABLE_TYPE_ANY,

} fp_table_type_t;

#endif /*End of _FP_TYPES_H_ File*/

