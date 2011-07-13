#ifndef _WRHV_BOOT_H
#define _WRHV_BOOT_H

/*
 * In current situation, there is no parameters protocol negotiated
 * between hypervisor and guest Linux, so we still define the hard
 * address for command line buffer like ENT_OFFSET as follows. Just
 * for notes to remove them in the future.
 * WRHV_CMDLINE_ADDR=offsetof(struct vb_config,bootLine)+ENT_OFFSET
 */
#define WRHV_CMDLINE_ADDR 0xF00000DC
#define WRHV_CMDLINE_SIZE 256
#define WRHV_EARLYCON_SIZE  14  /* sizeof("wrhv_earlycon=") */

#endif /* _WRHV_BOOT_H */
