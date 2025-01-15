#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/scmi_protocol.h>
#include <asm/div64.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/io.h>


struct channel_handle{
   const struct scmi_channel_proto_ops  *chn_ops;
    struct scmi_protocol_handle *ph;
};

struct channel_handle * c_handle = NULL;

int scmi_read(u32 reg,u32 *val){
    int ret = -1;
   if(c_handle == NULL || c_handle->chn_ops == NULL || c_handle->chn_ops->read == NULL || c_handle->ph == NULL){
        return ret;
    }


    ret =c_handle->chn_ops->read(c_handle->ph,reg,val);


    return ret;
}

int scmi_write(u32 reg,u32 val){

    int ret = -1;
    if(c_handle == NULL || c_handle->chn_ops == NULL || c_handle->chn_ops->read == NULL || c_handle->ph == NULL){
        return ret;
    }


    ret = c_handle->chn_ops->write(c_handle->ph,reg,val);
    
    return ret;
}


static int scmi_channel_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
    struct device *dev = &sdev->dev;
 

	if (!handle)
		return -ENODEV;

    
    c_handle = devm_kzalloc(dev, sizeof(*c_handle), GFP_KERNEL);
	if (!c_handle)
			return -ENOMEM;

	c_handle->chn_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_CHANNEL, &c_handle->ph);
	if (IS_ERR(c_handle->chn_ops))
		return PTR_ERR(c_handle->chn_ops);

	return 0;
}


static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_CHANNEL, "channel" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_channel_driver = {
	.name = "scmi-channel",
	.probe = scmi_channel_probe,
	.id_table = scmi_id_table,
};
//module_scmi_driver(scmi_clocks_driver);


static int __init scmi_channel_driver_init(void) {
	return scmi_register(&scmi_channel_driver); 
} 


subsys_initcall(scmi_channel_driver_init);



MODULE_DESCRIPTION("ARM SCMI channel driver");
MODULE_AUTHOR("BST Ltd.");