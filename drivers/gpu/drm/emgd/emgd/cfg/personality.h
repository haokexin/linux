/* -*- pse-c -*-
 *-----------------------------------------------------------------------------
 * Filename: personality.h
 * $Revision: 1.5 $
 *-----------------------------------------------------------------------------
 * Copyright (c) 2002-2010, Intel Corporation.
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
 * Description:
 *  This file is used in conjunction with the default_config.h to
 *  gererate a full set of build defines.
 *-----------------------------------------------------------------------------
 */

#ifndef _PERSONALITY_H
#define _PERSONALITY_H
/*
TODO: After the 9.1.1 branch, re-enable this code and delete igd_version.h
#define IGD_MAJOR_NUM  9
#define IGD_MINOR_NUM  1
#define IGD_BUILD_NUM  1258

#define IGD_PCF_VERSION   0x00000400
*/

/* Enable COPP */
#define CONFIG_COPP

#include <igd_version.h>

#endif
