#  Copyright (c) 2011 Wind River Systems, Inc.

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License version 2 as
#  published by the Free Software Foundation.

#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

#ifndef _SEC_TRAMPOLINE_H_
#define _SEC_TRAMPOLINE_H_

#ifndef __ASSEMBLY__

#ifdef CONFIG_SEC_TRAMPOLINE

extern const unsigned char sec_trampoline_data [];
extern const unsigned char sec_trampoline_end  [];

#endif /* CONFIG_SEC_TRAMPOLINE */

#endif /* __ASSEMBLY__ */

#endif /* _SEC_TRAMPOLINE_H_ */

