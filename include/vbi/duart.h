#ifndef DAURT_H
#define DAURT_H

/* taken from wrhv-1.2/include/sys/devices/drivers/serial.h */
#define SIO_MODE_POLL   1       /* polling mode */
#define SIO_MODE_INT    2       /* interrupt mode */

/* options to SIO_HW_OPTS_SET (ala POSIX), bitwise or'ed together */

#define SIO_HW_OPTS_CLOCAL 0x1 /* ignore modem status lines */
#define SIO_HW_OPTS_CREAD  0x2 /* enable device reciever */

#define SIO_HW_OPTS_CSIZE  0xc /* bits 3 and 4 encode the character size */
#define SIO_HW_OPTS_CS5    0x0 /* 5 bits */
#define SIO_HW_OPTS_CS6    0x4 /* 6 bits */
#define SIO_HW_OPTS_CS7    0x8 /* 7 bits */
#define SIO_HW_OPTS_CS8    0xc /* 8 bits */

#define SIO_HW_OPTS_HUPCL  0x10 /* hang up on last close */
#define SIO_HW_OPTS_STOPB  0x20 /* send two stop bits (else one) */
#define SIO_HW_OPTS_PARENB 0x40 /* parity detection enabled (else disabled) */
#define SIO_HW_OPTS_PARODD 0x80 /* odd parity  (else even) */

#endif /* DUART_H */
