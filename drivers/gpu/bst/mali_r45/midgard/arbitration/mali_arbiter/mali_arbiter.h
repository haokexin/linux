/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note OR MIT */

/*
 * (C) COPYRIGHT 2019-2021 Arm Limited or its affiliates. All rights reserved.
 */

/*
 * Part of the Mali reference arbiter
 */

#ifndef _MALI_ARBITER_H_
#define _MALI_ARBITER_H_

/*
 * This define specifies the current version of the arbiter.
 * Whenever the arbiter change in its functionality, so that integration effort
 * is required, the version number will be increased.
 * Each module which interact with the arbiter must make an effort
 * to check that it implements the correct version.
 *
 * Version history:
 * 1 - First arbiter release with PV support
 * 2 - Added Partition Manager support
 */
#define MALI_ARBITER_VERSION 2

/*
 * VM_UNASSIGNED is used to indicate that no VM has that GPU or partition
 * assigned to at that moment.
 */
#define VM_UNASSIGNED -1

/**
 * NO_FREQ - is used in case platform doesn't support reporting frequency
 */
#define NO_FREQ 0

/* Forward Declarations */
struct mali_arb_data;
struct mali_vm_data;
struct mali_vm_priv_data;
struct mali_arb_priv_data;

/**
 * struct power_interface - GPU power integration interface
 * @dev: GPU power device.
 * @ops: GPU power exposed operations.
 *
 * Struct which contains the public GPU power interface. This is what defines
 * all the information needed by the Arbiter to interact with the GPU power
 * device.
 */
struct power_interface {
	struct device *dev;
	struct mali_gpu_power_ops *ops;
};

/**
 * struct vm_assign_interface - GPU to VM assign interfaces
 * @if_id: Interface index associated with the GPU or partition assing device.
 *         Used when multiple VM assign interfaces are available, usually when
 *         hardware separation is supported.
 * @dev:   VM interface device.
 * @ops:   VM assign exposed operations.
 *
 * Struct which contains the public GPU to VM assign interfaces. This is what
 * defines all the information needed by the Arbiter to interact with the
 * assign devices.
 * These interfaces are used by the arbiter to assign the GPU or the partition
 * to a specific VM.
 */
struct vm_assign_interface {
	uint32_t if_id;
	struct device *dev;
	struct vm_assign_ops *ops;
};

/**
 * struct repartition_interface - Repartition interfaces
 * @if_id: Interface index associated with the repartition device. Used when
 *         multiple repartition interfaces are available.
 * @dev:   Repartition device.
 * @ops:   Repartition exposed operations.
 *
 * Struct which contains the public Repartition interfaces. This is what
 * defines all the information needed by the Arbiter to interact with the
 * Repartition devices.
 * These interfaces are used by the arbiter to rearrange slices in different
 * partitions. Only available when hardware separation is supported.
 */
struct repartition_interface {
	uint32_t if_id;
	struct device *dev;
	struct part_cfg_ops *ops;
};

/**
 * struct mali_vm_ops - Functions provided by the VMs
 * @gpu_granted: Callback to indicate that GPU is granted to VM.
 *               vm - Opaque pointer to the VM data passed by the backend
 *                    during the registration.
 *               freq - current frequency of the GPU to be passed to the VM,
 *                      NO_FREQ value to be passed in case of no support.
 * @gpu_stop: Callback to request that VM must stop using GPU.
 *            vm - Opaque pointer to the VM data passed by the backend
 *                 during registration.
 * @gpu_lost: Callback to request that VM must stop using GPU.
 *            vm - Opaque pointer to the VM data passed by the backend during
 *                 registration.
 *
 *            The VM should attempt recovery by failing all
 *            outstanding work and then issue a fresh request for the GPU
 *            when ready.
 */
struct mali_vm_ops {
	void (*gpu_granted)(struct mali_vm_priv_data *vm, uint32_t freq);
	void (*gpu_stop)(struct mali_vm_priv_data *vm);
	void (*gpu_lost)(struct mali_vm_priv_data *vm);
};

/**
 * struct vm_assign_ops - VM assign callbacks
 *
 * This struct contains the VM assign callbacks. It is used by the Arbiter to
 * assign, unassign or check the VMs that currently have a GPU or partition
 * assigned to.
 *
 * @assign_vm: Callback to assign GPU or Partition to a given VM.
 *             dev -   Pointer to the VM assign interface device.
 *             vm_id - VM ID to assign the GPU or Partition to.
 *             Returns
 *             0	- successful.
 *             -EINVAL	- invalid argument.
 *             -ENODEV	- expected interface not available.
 *             -EIO	- failed with hardware fault.
 *             -EFAULT	- failed with software fault.
 *
 *             After this function completes, the GPU or partition will be
 *             assigned to the active VM and ready to be accessed.
 *             This is a blocking function which should not be called
 *             concurrently for the same device from different threads.
 * @unassign_vm: Callback to eemove the assignment of the GPU or Partition
 *               from the currently VM it is assigned to.
 *               dev - Pointer to the VM assign interface device.
 *               Returns
 *               0		- successful.
 *               -EINVAL	- invalid argument.
 *               -ENODEV	- expected interface not available.
 *               -EIO		- failed with hardware fault.
 *
 *               After this function completes, the GPU or partition will be
 *               unassigned from the VM that currently has access to it.
 *               This is a blocking function which should not be called
 *               concurrently for the same device from different threads.
 * @get_assigned_vm: Callback to get the VM which the GPU or Partition is
 *                   assigned to.
 *                   dev - Pointer to the VM assign interface device.
 *                   vm_id - Pointer to variable to receive the VM ID which
 *                   the GPU or Partition is assigned to or VM_UNASSIGNED
 *                   if no VM has access to it.
 *                   Returns
 *                   0		- successful.
 *                   -EINVAL	- invalid argument.
 *                   -ENODEV	- expected interface not available.
 *                   -EIO	- failed with hardware fault.
 *
 *                   Get the VM which the GPU or Partition is assigned to. The
 *                   get_assigned_vm callback is optional and may be NULL.
 */
struct vm_assign_ops {
	int (*assign_vm)(struct device *dev, uint32_t vm_id);
	int (*unassign_vm)(struct device *dev);
	int (*get_assigned_vm)(struct device *dev, int32_t *vm_id);
};

/**
 * struct mali_arb_ops - Functions provided by the Arbiter
 * @arb_reg_assign_if: Callback to register VM assign interface.
 *                     arb_data - Arbiter public data exposed
 *                                (see mali_arb_data).
 *                     vm_assign - GPU to VM assign interfaces.
 *                     Returns
 *                     0	- successful.
 *                     -EINVAL	- invalid argument.
 *                     -ENODEV	- expected interface not available.
 *                     -EFAULT	- assign interface already registered.
 *
 *                     Register a VM assign interface to be used by the Arbiter
 *                     to assign a GPU or Partition to a specific VM. If
 *                     hardware separation is not supported, just the interface
 *                     index zero (see vm_assign_interface) should be
 *                     registered.
 * @arb_unreg_assign_if: Callback to unregister VM assign interface.
 *                       arb_data - Arbiter public data exposed
 *                                  (see mali_arb_data).
 *                       if_id -    Index of the interface to be unregistered.
 *                                  Should be zero in case hardware separation
 *                                  is not supported.
 *
 *                       Unregister the VM assign interface of index if_id from
 *                       the Arbiter. If hardware separation is not supported,
 *                       just the interface index zero (see
 *                       vm_assign_interface) should have been registered
 *                       (see arb_reg_assign_if).
 * @register_vm: Callback to register a VM comms device.
 *               arb_data - Arbiter public data exposed (see mali_arb_data).
 *               vm_info - The VM info data populated by caller
 *                         (see mali_vm_data).
 *               vm - Opaque pointer to the private VM data used by the
 *                    backend to identify the VMs. The Arbiter must pass
 *                    this pointer back in the VM callbacks every time it
 *                    needs to communicate to this VM being registered by
 *                    this function.
 *               arb_vm - Opaque pointer to the private Arbiter VM data used
 *                        by the Arbiter to identify and work with each VM.
 *                        This is populated by the Arbiter in this function.
 *                        The backend must pass this pointer back in the
 *                        Arbiter callbacks every time the VM being registered
 *                        here needs to communicate with the Arbiter.
 *                Returns
 *                0		- successful.
 *                -EINVAL	- invalid argument.
 *                -ENOMEM	- out of memory.
 *
 *                This function must be called first before a VM comms device
 *                instance can use the arbiter functionality.
 * @unregister_vm: Callback to unregister a VM comms device.
 *                 arb_vm - Opaque pointer to the Arbiter VM data populated by
 *                          the arbiter during registration.
 *
 *                 This function must be called before the VM comms device is
 *                 unloaded to sever the connection.
 * @gpu_request: Callback to request GPU time.
 *               arb_vm - Opaque pointer to the Arbiter VM data populated by
 *                        the arbiter during registration.
 *
 *               Called by a VM comms device to request GPU time. The Arbiter
 *               should respond when ready with a callback to gpu_granted.
 * @gpu_active: Callback to notify Arbiter that GPU is being used.
 *              arb_vm - Opaque pointer to the Arbiter VM data populated by
 *                       the arbiter during registration.
 *
 *              Called by a VM comms device to notify the Arbiter that the VM
 *              is busy using the GPU. This information can be useful in
 *              making scheduling decisions.
 * @gpu_idle: Callback to notify Arbiter that GPU is not being used.
 *            arb_vm - Opaque pointer to the Arbiter VM data populated by
 *                     the arbiter during registration.
 *
 *            Called by a VM to notify the Arbiter that it is no longer using
 *            the GPU. This information can be useful in making scheduling
 *            decisions.
 * @gpu_stopped: Callback to indicate that a VM KBase driver is in a
 *               stopped state and GPU is not being used.
 *               arb_vm - Opaque pointer to the Arbiter VM data populated by
 *                        the arbiter during registration.
 *               req_again - VM has work pending and still wants the GPU
 *
 *               VM must no longer use the GPU once this function has been
 *               called until gpu_granted has been called by the Arbiter.
 * @get_max: Callback to notify Arbiter of a request for max config.
 *           arb_vm - Opaque pointer to the Arbiter VM data populated by
 *                        the arbiter during registration.
 *           max_l2_slices: Returns the maximum number of GPU slices that can
 *                          be assigned to a partition. Caller must ensure
 *                          not NULL.
 *           max_core_mask - Returns intersection of any possible partition
 *                           core mask for that resource group.
 *           Returns
 *           0		- successful.
 *           -EINVAL	- invalid argument.
 *           -EFAULT	- max config calculation failed.
 *
 *           Called by a Resource group module to Notify Arbiter that a
 *           request for getting max config have been made, and that the
 *           arbiter needs to supply max config data.
 */
struct mali_arb_ops {
	int (*arb_reg_assign_if)(struct mali_arb_data *arb_data,
					struct vm_assign_interface vm_assign);
	void (*arb_unreg_assign_if)(struct mali_arb_data *arb_data,
					uint32_t if_id);
	int (*register_vm)(struct mali_arb_data *arb_data,
			   struct mali_vm_data *vm_info,
			   struct mali_vm_priv_data *vm,
			   struct mali_arb_priv_data **arb_vm);
	void (*unregister_vm)(struct mali_arb_priv_data *arb_vm);
	void (*gpu_request)(struct mali_arb_priv_data *arb_vm);
	void (*gpu_active)(struct mali_arb_priv_data *arb_vm);
	void (*gpu_idle)(struct mali_arb_priv_data *arb_vm);
	void (*gpu_stopped)(struct mali_arb_priv_data *arb_vm, bool req_again);
	int (*get_max)(struct mali_arb_priv_data *arb_vm,
			uint32_t *max_l2_slices, uint32_t *max_core_mask);
};

/**
 * struct mali_arb_data - Arbiter data
 * @ops:  Arbiter exposed operations
 *
 * Struct which contains the public arbiter data. This data needs to be exposed
 * to the backend and it can be done via:
 * 1 - If the arbiter driver use the probe mechanism, it must set it as its
 * device data on any platform config devices using dev_set_drvdata() so that it
 * can be retrieved by using dev_get_drvdata().
 * 2 - If the arbiter driver does not use the probe mechanism, it must return a
 * pointer to this data on the arbiter_create() function (see below).
 */
struct mali_arb_data {
	struct mali_arb_ops ops;
};

/**
 * struct mali_vm_data - VM data
 * @dev: Communication device associated with the VM.
 * @id:  VM ID to identify the VM.
 * @ops: VM exposed operations.
 *
 * Struct which contains the public VM data. This is what defines a VM to the
 * Arbiter. When a register_vm() callback is called by the backend module it
 * must pass this VM data as a parameter (see register_vm) to expose to the
 * Arbiter the VM information to be registered.
 */
struct mali_vm_data {
	struct device *dev;
	uint32_t id;
	struct mali_vm_ops ops;
};

/**
 * struct resource_interfaces - Resources interfaces
 * @num_if:      Number of interfaces available from the resources.
 * @vm_assign:   VM to GPU assign interfaces.
 * @repartition: Repartition interfaces.
 *
 * Struct which contains the collection of all the resources related
 * interfaces. It is used for grouping the relevant interfaces when
 * they are associated. When hardware separation is supported for example
 * each partition has its VM assign and Repartition interface.
 */
struct resource_interfaces {
	uint32_t num_if;
	struct vm_assign_interface **vm_assign;
	struct repartition_interface **repartition;
};

/**
 * arbiter_destroy() - Release the resources of the arbiter device
 * @arb_data: Public arbiter data exposed to the backend module.
 * @dev:      Device that contains the arbiter instance. This could either be a
 *            dedicated virtual device for the arbiter or contained in the
 *            platform-specific virtualization device, such as resource group
 *            or xenbus driver
 *
 * This function is called when the device is removed to free up all the
 * resources.
 */
void arbiter_destroy(struct mali_arb_data *arb_data, struct device *dev);

/**
 * arbiter_create() - Creates an arbiter instance
 * @dev:      Device that contains the arbiter instance. This could either be a
 *            dedicated virtual device for the arbiter or contained in the
 *            platform-specific virtualization device, such as resource group
 *            or xenbus driver
 * @pwr:      GPU Power interfaces. If the GPU power device is NULL, the
 *            Arbiter considers that no power control platform is available and
 *            the interface to power management is not set
 * @res:      Resources interfaces. If the vm_assign interface is NULL, the
 *            Arbiter considers that there is no hardware support through
 *            partition manager. In this case the interfaces to register and
 *            unregister the vm_assign interfaces is set by the Arbiter.
 * @arb_data: Arbiter public data populated by the arbiter to expose its
 *            information to the backend
 *
 * This function is an alternative mechanism to create the arbiter instance.
 * The arbiter can either be a standalone device or it can be instantiated as
 * part of an existing device. Therefore, this function is the interface that
 * allows an existing device to instantiate the Arbiter.
 *
 * This function is not required in case the Arbiter is instantiated using the
 * probe mechanism. In this case all the necessary data can be accessed via the
 * device tree.
 *
 * Return:
 * * 0		- success.
 * * -EINVAL	- invalid argument.
 * * -ENODEV	- expected interface not available.
 * * -EFAULT	- failed with software fault.
 * * -EIO	- failed with hardware fault.
 * * -ENOMEM	- out of memory.
 */
int arbiter_create(struct device *dev,
			struct power_interface pwr,
			struct resource_interfaces res,
			struct mali_arb_data **arb_data);

#endif /* _MALI_ARBITER_H_ */
