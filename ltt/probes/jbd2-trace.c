/*
 * ltt/probes/jbd2-trace.c
 *
 * JBD2 tracepoint probes.
 *
 * (C) Copyright 2009 - Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/module.h>
#include <trace/jbd2.h>

void probe_jbd2_checkpoint(journal_t *journal, int result)
{
	trace_mark_tp(jbd2, checkpoint, jbd2_checkpoint,
		probe_jbd2_checkpoint, "dev %s need_checkpoint %d",
		journal->j_devname, result);
}

void probe_jbd2_start_commit(journal_t *journal,
			     transaction_t *commit_transaction)
{
	trace_mark_tp(jbd2, start_commit, jbd2_start_commit,
		probe_jbd2_start_commit, "dev %s transaction %d",
		journal->j_devname, commit_transaction->t_tid);
}

void probe_jbd2_end_commit(journal_t *journal,
			   transaction_t *commit_transaction)
{
	trace_mark_tp(jbd2, end_commit, jbd2_end_commit,
		probe_jbd2_end_commit, "dev %s transaction %d head %d",
		journal->j_devname, commit_transaction->t_tid,
		journal->j_tail_sequence);
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("JBD2 Tracepoint Probes");
