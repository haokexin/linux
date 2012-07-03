#ifndef __AD9889B_H
#define __AD9889B_H

#include <linux/fb.h>

enum hdmi_audio_input {
	HDMI_AUDIO_IN_SPDIF,
	HDMI_AUDIO_IN_I2S,
};

struct ad9889b_pdata {
	u32 irq_gpio;
	u32 irq_type;
	int fb;
	enum hdmi_audio_input ain;
};
#endif /* __AD9889B_H */

