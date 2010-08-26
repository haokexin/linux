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

#include "std_ext.h"
#include "string_ext.h"
#include "sprint_ext.h"
#include "stdarg_ext.h"
#include "xx_ext.h"

#include "procbuff.h"


t_Handle    ProcBuff_Init(char *buffer,char **start,int offset,int length, int *eof)
{
    t_ProcBuff  * p_ProcBuff = XX_Malloc(sizeof(t_ProcBuff));

    if (p_ProcBuff)
    {
        p_ProcBuff->num_of_written_chars = 0;
        p_ProcBuff->buffer = buffer;
        p_ProcBuff->offset = offset;
        p_ProcBuff->length = length;
        p_ProcBuff->eof = eof;

        if(start)
            *start = buffer;
        *(p_ProcBuff->eof) = 0;
    }

    return (t_Handle)(p_ProcBuff);
}

void        ProcBuff_Free (t_Handle h_ProcBuff)
{
    t_ProcBuff  * p_ProcBuff = (t_ProcBuff *)(h_ProcBuff);
    XX_Free(p_ProcBuff);
}

void         ProcBuff_Write (t_Handle h_ProcBuff,const char *fmt, ...)
{
    t_ProcBuff  * p_ProcBuff = (t_ProcBuff *)(h_ProcBuff);
    int         len;
    va_list     args;
    char        *str = NULL;

    str = XX_Malloc(MAX_LINE_SIZE);
    if (!str) {
        return;
    }
    va_start(args, fmt);
    len=vsnprintf(str,MAX_LINE_SIZE,fmt,args);
    va_end(args);

    if (len == -1 || len >= MAX_LINE_SIZE)
    {
        /* formated string truncated, it is to big to fit into the MAX_LINE_SIZE */
        len = MAX_LINE_SIZE - 1;
        str[len] = 0;
    }

    if (len <= p_ProcBuff->offset)
        p_ProcBuff->offset -= len;
    else
    {
        len -= p_ProcBuff->offset;
        if (len > p_ProcBuff->length)
            len = p_ProcBuff->length;
        strncpy(p_ProcBuff->buffer,str + p_ProcBuff->offset,(unsigned int)(len));

        p_ProcBuff->buffer += len;
        p_ProcBuff->length -= len;
        if (p_ProcBuff->offset)
            p_ProcBuff->offset = 0;
        p_ProcBuff->num_of_written_chars += len;
    }
    XX_Free(str);
}

int          ProcBuff_GetNumOfWrittenChars(t_Handle h_ProcBuff)
{
    t_ProcBuff  * p_ProcBuff = (t_ProcBuff *)(h_ProcBuff);

    return p_ProcBuff->num_of_written_chars;
}

void         ProcBuff_Done (t_Handle h_ProcBuff)
{
    t_ProcBuff  * p_ProcBuff = (t_ProcBuff *)(h_ProcBuff);

    if (p_ProcBuff->length)
        *(p_ProcBuff->eof) = 1;
}
