// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Power Interface (SCMI) Protocol based clock driver
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2018-2020 ARM Ltd.
 */

#include <linux/clk-provider.h>
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

static const struct scmi_clk_proto_ops *clk_ops;


struct scmi_clk {
	u32 id;
	struct clk_hw hw;
	struct work_struct clk_work;

	const struct scmi_clock_info *info;
	const struct scmi_protocol_handle *ph;
};

DEFINE_MUTEX(send_mutex);

#define to_scmi_clk(clk) container_of(clk, struct scmi_clk, hw)


static unsigned long scmi_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	int ret;
	u64 rate = 0;
	struct scmi_clk *clk = to_scmi_clk(hw);

	

	
	mutex_lock(&send_mutex);
	ret = clk_ops->rate_get(clk->ph, clk->id, &rate);
	mutex_unlock(&send_mutex);


	if (ret)
		return 0;

	return rate;
}

static long scmi_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	u64 val;
	int ret;

	struct scmi_clk *clk = to_scmi_clk(hw);
	mutex_lock(&send_mutex);
	ret = clk_ops->rate_get(clk->ph, clk->id, &val);
	mutex_unlock(&send_mutex);
	if (ret)
		return 0;


	return rate;
}


static int scmi_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct scmi_clk *clk = to_scmi_clk(hw);

	return clk_ops->rate_set(clk->ph, clk->id, rate);
}




static int scmi_clk_enable(struct clk_hw *hw)
{
	struct scmi_clk *sclk = to_scmi_clk(hw);
#if 0
	int i = 0,isFound = 0;
#else 
    int isFound = 0;
#endif
	if(sclk->info->type != CLOCK_TREE_GAT){
		return 0;
	}


	isFound = clk_ops->has_child(sclk->ph,sclk->id);
	if(isFound == 1){
		return 0;
	}

//	printk("enable:%s\n",sclk->info->name);

	clk_ops->enable(sclk->ph, sclk->id);

	return 0;
}




static void scmi_clk_disable(struct clk_hw *hw)
{
	struct scmi_clk *sclk = to_scmi_clk(hw);
	#if 0
	int i = 0,isFound = 0;
	#else
	int isFound = 0;
	#endif


	if(sclk->info->type != CLOCK_TREE_GAT){
		return;
	}

	isFound = clk_ops->has_child(sclk->ph,sclk->id);
	if(isFound == 1){
		return;
	}

//	printk("disable:%s\n",sclk->info->name);

	
	clk_ops->disable(sclk->ph, sclk->id);


}


int	scmi_clk_is_enabled(struct clk_hw *hw){
	int is_enable = 0;
	struct scmi_clk *clk = to_scmi_clk(hw);

	is_enable = clk_ops->is_enable(clk->ph, clk->id);
	//printk("20230707 is_enable:%d\n",is_enable);

	return is_enable;
}





static u8 scmi_get_parent(struct clk_hw *hw)
{

	struct scmi_clk *clk = to_scmi_clk(hw);

	

	return clk_ops->get_parent(clk->ph, clk->id);
}




static int scmi_set_parent(struct clk_hw *hw,u8 index)
{

	struct scmi_clk *clk = to_scmi_clk(hw);

	return clk_ops->set_parent(clk->ph, clk->id,index);
}



static const struct clk_ops scmi_clk_ops = {
	.recalc_rate = scmi_clk_recalc_rate,
	.round_rate = scmi_clk_round_rate,
	.set_rate = scmi_clk_set_rate,
	/*
	 * We can't provide enable/disable callback as we can't perform the same
	 * in atomic context. Since the clock framework provides standard API
	 * clk_prepare_enable that helps cases using clk_enable in non-atomic
	 * context, it should be fine providing prepare/unprepare.
	 */
	.prepare =   scmi_clk_enable,
	.unprepare = scmi_clk_disable,

	//.get_parent = scmi_get_parent,
	//.set_parent = scmi_set_parent,
	.determine_rate = NULL,
};



static int scmi_clk_ops_init(struct device *dev, struct scmi_clk *sclk,const char * const * parents)
{
	int ret;
	unsigned long min_rate, max_rate;


	struct clk_init_data init = {
		.flags = CLK_GET_RATE_NOCACHE,
		.num_parents = sclk->info->parent_num,
		.parent_names = parents,
		.ops = &scmi_clk_ops,
		.name = sclk->info->name,
	};

	sclk->hw.init = &init;
	ret = devm_clk_hw_register(dev, &sclk->hw);
	if (ret)
		return ret;

	if (sclk->info->rate_discrete) {
		int num_rates = sclk->info->list.num_rates;

		if (num_rates <= 0)
			return -EINVAL;

		min_rate = sclk->info->list.rates[0];
		max_rate = sclk->info->list.rates[num_rates - 1];
	} else {
		min_rate = sclk->info->range.min_rate;
		max_rate = sclk->info->range.max_rate;
	}

	clk_hw_set_rate_range(&sclk->hw, min_rate, max_rate);
	return ret;
}



static long bst_clk_gate_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
		return rate;	
}


const struct clk_ops bst_clk_gate_ops = {
	.enable = scmi_clk_enable,
	.disable = scmi_clk_disable,
	.is_enabled = scmi_clk_is_enabled,
	.round_rate = bst_clk_gate_round_rate,
};

struct clk_hw *bst_clk_hw_register_gate(struct device *dev, struct scmi_clk *sclk,const char * const *  parent_names)
{
	struct clk_hw *hw = NULL;
	struct clk_init_data init = {};
	int ret = -EINVAL;

	init.name = sclk->info->name;
	init.ops = &bst_clk_gate_ops;
	init.flags = CLK_IGNORE_UNUSED;
	
	init.parent_names = parent_names;
	init.num_parents = 1;

	sclk->hw.init = &init;


	ret = clk_hw_register(dev, &sclk->hw);
	if (ret) {
		return hw;
	}

	hw = &sclk->hw;

	return hw;
}


static long bst_clk_mux_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
		return rate;	
}



const struct clk_ops bst_clk_mux_ops = {
	.get_parent = scmi_get_parent,
	.set_parent = scmi_set_parent,
	.round_rate = bst_clk_mux_round_rate,
};

struct clk_hw *bst_clk_hw_register_mux(struct device *dev, struct scmi_clk *sclk,const char * const *  parent_names)
{
	struct clk_hw *hw;
	struct clk_init_data init = {};
	int ret = -EINVAL;

	init.name = sclk->info->name;
	init.ops = &bst_clk_mux_ops;
	init.flags = 0;
	init.parent_names = parent_names;
	init.parent_data = NULL;
	init.parent_hws = NULL;
	init.num_parents = sclk->info->parent_num;

	sclk->hw.init = &init;
	ret = clk_hw_register(dev, &sclk->hw);
	if (ret) {
		hw = NULL;
	}

	hw = &sclk->hw;

	return hw;
}


static unsigned int _get_div(const struct clk_div_table *table,
			     unsigned int val, unsigned long flags, u8 width)
{
	if (flags & CLK_DIVIDER_ONE_BASED)
		return val;
	if (flags & CLK_DIVIDER_POWER_OF_TWO)
		return 1 << val;
	if (flags & CLK_DIVIDER_MAX_AT_ZERO)
		return val ? val : clk_div_mask(width) + 1;
	
	return val + 1;
}


unsigned long bst_divider_recalc_rate(struct clk_hw *hw, unsigned long parent_rate,
				  unsigned int val,
				  const struct clk_div_table *table,
				  unsigned long flags, unsigned long width)
{
	unsigned int div;

	div = _get_div(table, val, flags, width);
	if (!div) {
		WARN(!(flags & CLK_DIVIDER_ALLOW_ZERO),
			"%s: Zero divisor and CLK_DIVIDER_ALLOW_ZERO not set\n",
			clk_hw_get_name(hw));
		return parent_rate;
	}

	return DIV_ROUND_UP_ULL((u64)parent_rate, div);
}


static unsigned long bst_clk_divider_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{

	u64 val;
	int ret;

	struct scmi_clk *clk = to_scmi_clk(hw);


	mutex_lock(&send_mutex);
	ret = clk_ops->rate_get(clk->ph, clk->id, &val);
	mutex_unlock(&send_mutex);
	if (ret)
		return 0;
	

	return val;
	//return bst_divider_recalc_rate(hw, parent_rate, val, NULL, 0, 0);
}









static int scmi_set_div_rate(struct clk_hw *hw, unsigned long rate,unsigned long parent_rate){
	//unsigned int val;
	int ret;

	struct scmi_clk *clk = to_scmi_clk(hw);
	




	mutex_lock(&send_mutex);
	ret = clk_ops->dividor_set(clk->ph, clk->id, rate);
	mutex_unlock(&send_mutex);

	return ret;
}




static long bst_clk_divider_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
		u64 val;
		int ret;

		struct scmi_clk *clk = to_scmi_clk(hw);
		mutex_lock(&send_mutex);
		ret = clk_ops->rate_get(clk->ph, clk->id, &val);
		mutex_unlock(&send_mutex);
		if (ret)
			return 0;

	  divider_ro_round_rate(hw, rate, prate, NULL, 0, 0,val);

		return rate;	
}




const struct clk_ops bst_clk_divider_ro_ops = {
	.recalc_rate = bst_clk_divider_recalc_rate,
	.round_rate = bst_clk_divider_round_rate,
	.set_rate = scmi_set_div_rate,
};

struct clk_hw *bst_clk_hw_register_divider(struct device *dev, struct scmi_clk *sclk,const char * const *  parent_names)
{
	
	struct clk_hw *hw;
	struct clk_init_data init = {};
	int ret;

	init.name = sclk->info->name;
	init.ops = &bst_clk_divider_ro_ops;
	init.flags = CLK_SET_RATE_PARENT ;
	init.parent_names = parent_names;
	init.parent_data = NULL;
	init.parent_hws = NULL;
	init.num_parents = sclk->info->parent_num;


	sclk->hw.init = &init;


	ret = clk_hw_register(dev, &sclk->hw);
	if (ret) {
		hw = NULL;
	}

	hw = &sclk->hw;

	return hw;
}





static unsigned long clk_factor_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct scmi_clk *clk = to_scmi_clk(hw);
	unsigned long long int rate;

	rate = (unsigned long long int)parent_rate * clk->info->fix_factor_mult;
	do_div(rate, clk->info->fix_factor_div);
	return (unsigned long)rate;
}

static long clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct scmi_clk *clk = to_scmi_clk(hw);

	return (*prate / clk->info->fix_factor_div) * clk->info->fix_factor_mult;
}

static int clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	/*
	 * We must report success but we can do so unconditionally because
	 * clk_factor_round_rate returns values that ensure this call is a
	 * nop.
	 */

	return 0;
}

const struct clk_ops bst_clk_fixed_factor_ops = {
	.round_rate = clk_factor_round_rate,
	.set_rate = clk_factor_set_rate,
	.recalc_rate = clk_factor_recalc_rate,
};




static struct clk_hw * bst_clk_hw_register_fixed_factor(struct device *dev,struct scmi_clk *sclk,const char * const *  parent_names)
{
	struct clk_hw *hw;
	struct clk_init_data init = {};
	int ret;

	init.name = sclk->info->name;
	init.ops = &bst_clk_fixed_factor_ops;
	init.parent_names = parent_names;
	init.num_parents = sclk->info->parent_num;


	sclk->hw.init = &init;

	ret = clk_hw_register(dev, &sclk->hw);

	if (ret) {
		hw = ERR_PTR(ret);
	} 

	hw = &sclk->hw;


	return hw;
}



static int scmi_clocks_probe(struct scmi_device *sdev)
{
	

	int idx, count,err;
	struct clk_hw **hws;
	struct clk_hw_onecell_data *clk_data;
	struct device *dev = &sdev->dev;
	struct device_node *np = dev->of_node;
	const struct scmi_handle *handle = sdev->handle;
	struct scmi_protocol_handle *ph;


	if (!handle)
		return -ENODEV;






	clk_ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_CLOCK, &ph);
	if (IS_ERR(clk_ops))
		return PTR_ERR(clk_ops);

	count = clk_ops->count_get(ph);
	if (count < 0) {
		dev_err(dev, "%pOFn: invalid clock output count\n", np);
		return -EINVAL;
	}


	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, count),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;


	clk_data->num = count ;
	hws = clk_data->hws;

	for (idx = 0; idx < clk_data->num; idx++) {

		struct scmi_clk *sclk;
		
		char * parent_names[8];
		unsigned int i;
		const char * const *  parents;



		sclk = devm_kzalloc(dev, sizeof(*sclk), GFP_KERNEL);
		if (!sclk)
			return -ENOMEM;

		sclk->info = clk_ops->info_get(ph, idx);
		if (!sclk->info) {
			dev_err(dev, "invalid clock info for idx %d\n", idx);
			continue;
		}

		sclk->id = idx;
		sclk->ph = ph;

		for(i=0;i<sclk->info->parent_num;i++){
			parent_names[i] = (char *)sclk->info->parent_index[i];
		}


		parents = (	const char * const * )parent_names;
	

		switch(sclk->info->type){
			case CLOCK_TREE_PLL:{
				err = scmi_clk_ops_init(dev, sclk,parents);
				if (err) {
					dev_err(dev, "failed to register clock %d %s\n", idx,sclk->info->name);
					devm_kfree(dev, sclk);
					hws[idx] = NULL;
				} else {
					//dev_info(dev, "PLL Registered clock:%s\n", sclk->info->name);
					hws[idx] = &sclk->hw;

				}
				break;
			}
			case CLOCK_TREE_MUX:{
				struct clk_hw * hw;
				hw = bst_clk_hw_register_mux(dev, sclk,parents);
				if (!hw) {
					dev_err(dev, "failed to register clock %d %s\n", idx,sclk->info->name);
					devm_kfree(dev, sclk);
					hws[idx] = NULL;
				} else {
				//	dev_info(dev, "Mux Registered clock:%s\n", sclk->info->name);
					hws[idx] = hw;

				}
				break;
			}
			case CLOCK_TREE_GAT:{
				struct clk_hw * hw;

				hw = bst_clk_hw_register_gate(dev, sclk,parents);
				if (!hw) {
					dev_err(dev, "failed to register clock %d %s\n", idx,sclk->info->name);
					devm_kfree(dev, sclk);
					hws[idx] = NULL;
				} else {
					//dev_info(dev, "Gate Registered clock:%s\n", sclk->info->name);
					hws[idx] = hw;

				}
				break;
			}
			case CLOCK_TREE_DIV:{
				struct clk_hw * hw;
				
				hw = bst_clk_hw_register_divider(dev, sclk,parents);
				if (!hw) {
					dev_err(dev, "failed to register clock %d %s\n", idx,sclk->info->name);
					devm_kfree(dev, sclk);
					hws[idx] = NULL;
				} else {
					//dev_err(dev, "divider Registered clock:%s\n", sclk->info->name);
					hws[idx] = hw;

				}
				break;
			}
			case CLOCK_TREE_FIX:{
				if(sclk->info->parent_num == 0){
					struct clk * clk;
					clk = clk_register_fixed_rate(NULL, sclk->info->name, NULL, 0, sclk->info->fix_rate);
					if (!clk) {
					dev_err(dev, "failed to register clock %d %s\n", idx,sclk->info->name);
					devm_kfree(dev, sclk);
					hws[idx] = NULL;
					} else {
						//dev_info(dev, "Fix Registered clock:%s\n", sclk->info->name);
						hws[idx] = 	container_of(&clk,struct clk_hw,clk);

					}
				}
				break;
			}
			case CLOCK_TREE_FCT:{
				if(sclk->info->parent_num == 1){
					struct clk_hw * hw;
					hw = bst_clk_hw_register_fixed_factor(dev, sclk,parents);
					if (!hw) {
						dev_err(dev, "failed to register clock %d %s\n", idx,sclk->info->name);
						devm_kfree(dev, sclk);
						hws[idx] = NULL;
					} else {
						
						hws[idx] = hw;

					}
				}
				break;
			}
		}


		
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,clk_data);
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_CLOCK, "clocks" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_clocks_driver = {
	.name = "scmi-clocks",
	.probe = scmi_clocks_probe,
	.id_table = scmi_id_table,
};
//module_scmi_driver(scmi_clocks_driver);


static int __init scmi_clocks_driver_init(void) {
	return scmi_register(&scmi_clocks_driver); 
} 


subsys_initcall(scmi_clocks_driver_init);



MODULE_AUTHOR("Sudeep Holla <sudeep.holla@arm.com>");
MODULE_DESCRIPTION("ARM SCMI clock driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");