/* SPDX-License-Identifier: LGPL-2.1 */
/*
 * Copyright (C) 2019, VMware, Tzvetomir Stoyanov <tz.stoyanov@gmail.com>
 *
 */
#ifndef _TRACE_FS_LOCAL_H
#define _TRACE_FS_LOCAL_H

#include <tracefs.h>
#include <pthread.h>

#define __hidden __attribute__((visibility ("hidden")))
#define __weak __attribute__((weak))

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/* Will cause a division by zero warning if cond is true */
#define BUILD_BUG_ON(cond)			\
	do { if (!(1/!(cond))) { } } while (0)

#define HASH_BITS 10

struct tracefs_options_mask {
	unsigned long long	mask;
};

struct follow_event {
	struct tep_event	*event;
	void			*callback_data;
	int (*callback)(struct tep_event *,
			struct tep_record *,
			int, void *);
};

struct tracefs_instance {
	struct tracefs_options_mask	supported_opts;
	struct tracefs_options_mask	enabled_opts;
	struct follow_event		*followers;
	struct follow_event		*missed_followers;
	char				*trace_dir;
	char				*name;
	pthread_mutex_t			lock;
	int				ref;
	int				flags;
	int				ftrace_filter_fd;
	int				ftrace_notrace_fd;
	int				ftrace_marker_fd;
	int				ftrace_marker_raw_fd;
	int				nr_followers;
	int				nr_missed_followers;
	bool				pipe_keep_going;
	bool				iterate_keep_going;
};

struct tracefs_buffer_stat {
	ssize_t				entries;
	ssize_t				overrun;
	ssize_t				commit_overrun;
	ssize_t				bytes;
	long long			oldest_ts;
	long long			now_ts;
	ssize_t				dropped_events;
	ssize_t				read_events;
};

extern const struct tep_format_field tfs_common_stacktrace;

extern pthread_mutex_t tfs_toplevel_lock;

static inline pthread_mutex_t *trace_get_lock(struct tracefs_instance *instance)
{
	return instance ? &instance->lock : &tfs_toplevel_lock;
}

void tfs_put_instance(struct tracefs_instance *instance);
int tfs_get_instance(struct tracefs_instance *instance);

/* Can be overridden */
void tracefs_warning(const char *fmt, ...);

char *tfs_strstrip(char *str);
int tfs_str_read_file(const char *file, char **buffer, bool warn);
char *tfs_append_file(const char *dir, const char *name);
char *tfs_find_tracing_dir(bool debugfs);

#ifndef ACCESSPERMS
#define ACCESSPERMS (S_IRWXU|S_IRWXG|S_IRWXO) /* 0777 */
#endif

#ifndef ALLPERMS
#define ALLPERMS (S_ISUID|S_ISGID|S_ISVTX|S_IRWXU|S_IRWXG|S_IRWXO) /* 07777 */
#endif

#ifndef DEFFILEMODE
#define DEFFILEMODE (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH) /* 0666*/
#endif

struct tracefs_options_mask *
tfs_supported_opts_mask(struct tracefs_instance *instance);

struct tracefs_options_mask *
tfs_enabled_opts_mask(struct tracefs_instance *instance);

char **tfs_list_create_empty(void);
int tfs_list_pop(char **list);

char *tfs_append_string(char *str, const char *delim, const char *add);
int tfs_test_state(int state);
bool tfs_verify_event_field(struct tep_event *event,
			    const char *field_name,
			    const struct tep_format_field **ptr_field);
int tfs_append_filter(char **filter, unsigned int *state,
		      unsigned int *open_parens,
		      struct tep_event *event,
		      enum tracefs_filter type,
		      const char *field_name,
		      enum tracefs_compare compare,
		      const char *val);

void *tfs_mmap(int fd, struct kbuffer *kbuf);
void tfs_unmap(void *mapping);
int tfs_mmap_load_subbuf(void *mapping, struct kbuffer *kbuf);
int tfs_mmap_read(void *mapping, void *buffer);

struct tracefs_synth *
tfs_synth_init_from(struct tep_handle *tep,
		    const char *start_system,
		    const char *start_event);

#define HIST_COUNTER_TYPE	(TRACEFS_HIST_KEY_MAX + 100)
int tfs_synth_add_start_field(struct tracefs_synth *synth,
			      const char *start_field,
			      const char *name,
			      enum tracefs_hist_key_type type,
			      int cnt);

/* Internal interface for ftrace dynamic events */

struct tracefs_dynevent {
	char *trace_file;
	char *prefix;
	char *system;
	char *event;
	char *address;
	char *format;
	enum tracefs_dynevent_type type;
};

struct tracefs_dynevent *
tfs_dynevent_alloc(enum tracefs_dynevent_type type, const char *system,
		   const char *event, const char *address, const char *format);
int tfs_dynevent_get_count(unsigned int types, const char *system);

int tfs_load_events(struct tep_handle *tep,
		    const char *tracing_dir,
		    const char *system);
int tfs_rescan_events(struct tep_handle *tep,
		      const char *tracing_dir,
		      const char *system);
struct tep_event *
tfs_get_tep_event(struct tep_handle *tep,
		  const char *system,
		  const char *name);

unsigned int tfs_quick_hash(const char *str);

#endif /* _TRACE_FS_LOCAL_H */
