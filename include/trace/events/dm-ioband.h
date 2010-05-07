#if !defined(_TRACE_DM_IOBAND_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DM_IOBAND_H

#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dm-ioband

TRACE_EVENT(ioband_hold_urgent_bio,

	TP_PROTO(struct ioband_group *gp, struct bio *bio),

	TP_ARGS(gp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		gp->c_banddev->g_name	)
		__field(	int,		c_id			)
		__array(	int,		g_blocked,	2	)
		__array(	int,		c_blocked,	2	)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, gp->c_banddev->g_name);
		__entry->c_id		= gp->c_id;
		memcpy(__entry->g_blocked, gp->c_banddev->g_blocked,
			sizeof(gp->c_banddev->g_blocked));
		memcpy(__entry->c_blocked, gp->c_blocked,
			sizeof(gp->c_blocked));
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s,%d: %d,%d %c %llu + %u %d/%d %d/%d",
		  __get_str(g_name), __entry->c_id,
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector,
		  __entry->c_blocked[0], __entry->g_blocked[0],
		  __entry->c_blocked[1], __entry->g_blocked[1])
);

TRACE_EVENT(ioband_hold_bio,

	TP_PROTO(struct ioband_group *gp, struct bio *bio),

	TP_ARGS(gp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		gp->c_banddev->g_name	)
		__field(	int,		c_id			)
		__array(	int,		g_blocked,	2	)
		__array(	int,		c_blocked,	2	)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, gp->c_banddev->g_name);
		__entry->c_id		= gp->c_id;
		memcpy(__entry->g_blocked, gp->c_banddev->g_blocked,
			sizeof(gp->c_banddev->g_blocked));
		memcpy(__entry->c_blocked, gp->c_blocked,
			sizeof(gp->c_blocked));
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s,%d: %d,%d %c %llu + %u %d/%d %d/%d",
		  __get_str(g_name), __entry->c_id,
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector,
		  __entry->c_blocked[0], __entry->g_blocked[0],
		  __entry->c_blocked[1], __entry->g_blocked[1])
);

TRACE_EVENT(ioband_make_pback_list,

	TP_PROTO(struct ioband_group *gp, struct bio *bio),

	TP_ARGS(gp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		gp->c_banddev->g_name	)
		__field(	int,		c_id			)
		__array(	int,		g_blocked,	2	)
		__array(	int,		c_blocked,	2	)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, gp->c_banddev->g_name);
		__entry->c_id		= gp->c_id;
		memcpy(__entry->g_blocked, gp->c_banddev->g_blocked,
			sizeof(gp->c_banddev->g_blocked));
		memcpy(__entry->c_blocked, gp->c_blocked,
			sizeof(gp->c_blocked));
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s,%d: %d,%d %c %llu + %u %d/%d %d/%d",
		  __get_str(g_name), __entry->c_id,
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector,
		  __entry->c_blocked[0], __entry->g_blocked[0],
		  __entry->c_blocked[1], __entry->g_blocked[1])
);

TRACE_EVENT(ioband_make_issue_list,

	TP_PROTO(struct ioband_group *gp, struct bio *bio),

	TP_ARGS(gp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		gp->c_banddev->g_name	)
		__field(	int,		c_id			)
		__array(	int,		g_blocked,	2	)
		__array(	int,		c_blocked,	2	)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, gp->c_banddev->g_name);
		__entry->c_id		= gp->c_id;
		memcpy(__entry->g_blocked, gp->c_banddev->g_blocked,
			sizeof(gp->c_banddev->g_blocked));
		memcpy(__entry->c_blocked, gp->c_blocked,
			sizeof(gp->c_blocked));
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s,%d: %d,%d %c %llu + %u %d/%d %d/%d",
		  __get_str(g_name), __entry->c_id,
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector,
		  __entry->c_blocked[0], __entry->g_blocked[0],
		  __entry->c_blocked[1], __entry->g_blocked[1])
);

TRACE_EVENT(ioband_release_urgent_bios,

	TP_PROTO(struct ioband_device *dp, struct bio *bio),

	TP_ARGS(dp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		dp->g_name		)
		__array(	int,		g_blocked,	2	)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, dp->g_name);
		memcpy(__entry->g_blocked, dp->g_blocked,
			sizeof(dp->g_blocked));
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s: %d,%d %c %llu + %u %d %d",
		  __get_str(g_name),
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector,
		  __entry->g_blocked[0], __entry->g_blocked[1])
);

TRACE_EVENT(ioband_make_request,

	TP_PROTO(struct ioband_device *dp, struct bio *bio),

	TP_ARGS(dp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		dp->g_name		)
		__field(	int,		c_id			)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, dp->g_name);
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s: %d,%d %c %llu + %u",
		  __get_str(g_name),
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector)
);

TRACE_EVENT(ioband_pushback_bio,

	TP_PROTO(struct ioband_device *dp, struct bio *bio),

	TP_ARGS(dp, bio),

	TP_STRUCT__entry(
		__string(	g_name,		dp->g_name		)
		__field(	dev_t,		dev			)
		__field(	sector_t,	sector			)
		__field(	unsigned int,	nr_sector		)
		__field(	char,		rw			)
	),

	TP_fast_assign(
		__assign_str(g_name, dp->g_name);
		__entry->dev		= bio->bi_bdev->bd_dev;
		__entry->sector		= bio->bi_sector;
		__entry->nr_sector	= bio->bi_size >> 9;
		__entry->rw	= (bio_data_dir(bio) == READ) ? 'R' : 'W';
	),

	TP_printk("%s: %d,%d %c %llu + %u",
		  __get_str(g_name),
		  MAJOR(__entry->dev), MINOR(__entry->dev), __entry->rw,
		  (unsigned long long)__entry->sector, __entry->nr_sector)
);

#endif /* _TRACE_DM_IOBAND_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
