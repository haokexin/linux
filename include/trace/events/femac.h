/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM femac

#if !defined(_TRACE_FEMAC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_FEMAC_H

#include <linux/tracepoint.h>

TRACE_EVENT(rx_packet_start,

	    TP_PROTO(int tailroom,
		     unsigned long pdu_length),

	    TP_ARGS(tailroom, pdu_length),

	    TP_STRUCT__entry(
		    __field(int, tailroom)
		    __field(unsigned long, pdu_length)
		    ),

	    TP_fast_assign(
		    __entry->tailroom = tailroom;
		    __entry->pdu_length = pdu_length;
		    ),

	    TP_printk("tailroom=%d pdu_length=%ld",
		      __entry->tailroom, __entry->pdu_length)
	);

TRACE_EVENT(rx_packet_loop,

	    TP_PROTO(int tailroom,
		     unsigned long pdu_length,
		     int loop),

	    TP_ARGS(tailroom, pdu_length, loop),

	    TP_STRUCT__entry(
		    __field(int,   tailroom)
		    __field(unsigned long, pdu_length)
		    __field(int, loop)
		    ),

	    TP_fast_assign(
		    __entry->tailroom = tailroom;
		    __entry->pdu_length = pdu_length;
		    __entry->loop = loop;
		    ),

	    TP_printk("tailroom=%d pdu_length=%ld loop = %d",
		      __entry->tailroom, __entry->pdu_length, __entry->loop)
	);

TRACE_EVENT(rx_packet_pdu_overrun,

	    TP_PROTO(unsigned int bytes_copied,
		     unsigned int *words),

	    TP_ARGS(bytes_copied, words),

	    TP_STRUCT__entry(
		    __field(unsigned int, bytes_copied)
		    __array(unsigned int, words, 4)
		    ),

	    TP_fast_assign(
		    __entry->bytes_copied = bytes_copied;
		    memcpy(&__entry->words, words, 4 * 4);
		    ),

	    TP_printk("bytes_copied=%d words[0]=%x words[1]=%x words[2]=%x words[3]=%x",
		      __entry->bytes_copied, __entry->words[0],
		      __entry->words[1], __entry->words[2], __entry->words[3])
	);

TRACE_EVENT(rx_packet_error,

	    TP_PROTO(int dummy),

	    TP_ARGS(dummy),

	    TP_STRUCT__entry(
		    __field(int, dummy)
		    ),

	    TP_fast_assign(
		    __entry->dummy = dummy;
		    ),

	    TP_printk("%s", "")
	);

TRACE_EVENT(rx_packet_overrun_eop,

	    TP_PROTO(unsigned int bytes_copied),

	    TP_ARGS(bytes_copied),

	    TP_STRUCT__entry(
		    __field(unsigned int, bytes_copied)
		    ),

	    TP_fast_assign(
		    __entry->bytes_copied = bytes_copied;
		    ),

	    TP_printk("bytes_copied=%u", __entry->bytes_copied)
	);

TRACE_EVENT(rx_packet_finish,

	    TP_PROTO(int total_loops),

	    TP_ARGS(total_loops),

	    TP_STRUCT__entry(
		    __field(int, total_loops)
		    ),

	    TP_fast_assign(
		    __entry->total_loops = total_loops;
		    ),

	    TP_printk("total_loops=%d",
		      __entry->total_loops)
	);

TRACE_EVENT(rx_packets_start,

	    TP_PROTO(int max),

	    TP_ARGS(max),

	    TP_STRUCT__entry(__field(int, max)),

	    TP_fast_assign(__entry->max = max;),

	    TP_printk("max=%d", __entry->max)
	);

TRACE_EVENT(rx_packets_finish,

	    TP_PROTO(int packets),

	    TP_ARGS(packets),

	    TP_STRUCT__entry(__field(int, packets)),

	    TP_fast_assign(__entry->packets = packets;),

	    TP_printk("packets=%d", __entry->packets)
	);

TRACE_EVENT(rx_packet_not_for_us,

	    TP_PROTO(int packets),

	    TP_ARGS(packets),

	    TP_STRUCT__entry(__field(int, not_for_us_packets)),

	    TP_fast_assign(__entry->not_for_us_packets = packets;),

	    TP_printk("not_for_us_packets=%d", __entry->not_for_us_packets)
	);

#endif /* _TRACE_FEMAC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
