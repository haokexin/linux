#----------------------------------------------------------------------------
# Filename: Makefile.gnu
# $Revision: 1.57 $
#----------------------------------------------------------------------------
# Copyright (c) 2002-2010, Intel Corporation.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#----------------------------------------------------------------------------
# Modified to build in the WR-Linux kernel
# Signed-off-by: Greg Moffatt <greg.moffatt@windriver.com>

ccflags-y := \
	   -Idrivers/gpu/drm/emgd/include \
	   -Idrivers/gpu/drm/emgd/emgd/display/mode/cmn \
	   -Idrivers/gpu/drm/emgd/emgd/video/overlay/cmn \
	   -Idrivers/gpu/drm/emgd/emgd/video/msvdx \
	   -Idrivers/gpu/drm/emgd/emgd/include \
	   -Idrivers/gpu/drm/emgd/emgd/cfg \
	   -Idrivers/gpu/drm/emgd/emgd/pal/lpd \
	   -Idrivers/gpu/drm/emgd/emgd/pal/lvds \
	   -Idrivers/gpu/drm/emgd/emgd/pal/ch7036 \
	   -Idrivers/gpu/drm/emgd/emgd/drm \
	   -Iinclude/drm \
	   -Idrivers/gpu/drm/emgd/pvr/include4 \
	   -Idrivers/gpu/drm/emgd/pvr/services4/include \
	   -Idrivers/gpu/drm/emgd/pvr/services4/include/env/linux \
	   -Idrivers/gpu/drm/emgd/pvr/services4/srvkm/env/linux \
	   -Idrivers/gpu/drm/emgd/pvr/services4/srvkm/include \
	   -Idrivers/gpu/drm/emgd/pvr/services4/srvkm/bridged \
	   -Idrivers/gpu/drm/emgd/pvr/services4/system/plb \
	   -Idrivers/gpu/drm/emgd/pvr/services4/system/tnc \
	   -Idrivers/gpu/drm/emgd/pvr/services4/system/include \
	   -Idrivers/gpu/drm/emgd/pvr/services4/srvkm/hwdefs \
	   -Idrivers/gpu/drm/emgd/pvr/services4/srvkm/bridged/sgx \
	   -Idrivers/gpu/drm/emgd/pvr/services4/srvkm/devices/sgx \
	   -Idrivers/gpu/drm/emgd/pvr/tools/intern/debug \
	   -DSUPPORT_DRI_DRM_EXT \
	   -DLINUX \
	   -DPVR_BUILD_DIR="\"emgd\"" \
	   -DPVR_BUILD_DATE="\"20111115\"" \
	   -DPVR_BUILD_TYPE="\"release\"" \
	   -DBUILD=release \
	   -DRELEASE \
	   -DPVR_SECURE_HANDLES \
	   -DPVR_PROC_USE_SEQ_FILE \
	   -DLDM_PCI \
	   -DSUPPORT_CACHEFLUSH_ON_ALLOC \
	   -DSUPPORT_DRI_DRM \
	   -DSGX535 \
	   -DSGX_CORE_REV=121 \
	   -UDEBUG_LOG_PATH_TRUNCATE \
	   -DDISPLAY_CONTROLLER=emgd_dc \
	   -D_XOPEN_SOURCE=600 \
	   -DSERVICES4 \
	   -DPVR2D_VALIDATE_INPUT_PARAMS \
	   -DSUPPORT_SRVINIT \
	   -DSUPPORT_SGX \
	   -DSUPPORT_PERCONTEXT_PB \
	   -DSUPPORT_LINUX_X86_WRITECOMBINE \
	   -DSUPPORT_SECURE_DRM_AUTH_EXPORT \
	   -DSUPPORT_PDUMP_DELAYED_INITPHASE_TERMINATION \
	   -DTRANSFER_QUEUE \
	   -DSYS_USING_INTERRUPTS \
	   -DSUPPORT_HW_RECOVERY \
	   -DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	   -DPVR_SECURE_HANDLES \
	   -DUSE_PTHREADS \
	   -DSUPPORT_SGX_EVENT_OBJECT \
	   -DSUPPORT_SGX_HWPERF \
	   -DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	   -DSUPPORT_LINUX_X86_PAT \
	   -DSUPPORT_SGX535 \
	   -DSUPPORT_CACHE_LINE_FLUSH \
	   -DSUPPORT_CPU_CACHED_BUFFERS \
	   -DDEBUG_MESA_OGL_TRACE \
	   -DSUPPORT_EGL_IMAGE_SYNC_DEPENDENCY \





EMGD_OBJS := \
	emgd/drm/emgd_fb.o \
	emgd/drm/emgd_fbcon.o \
	emgd/drm/emgd_crtc.o \
	emgd/drm/emgd_encoder.o \
	emgd/drm/emgd_connector.o \
	emgd/drm/emgd_mmap.o \
	emgd/drm/emgd_drv.o \
	emgd/drm/emgd_interface.o \
	emgd/drm/emgd_test_pvrsrv.o \
	emgd/drm/user_config.o \
	emgd/drm/splash_screen.o \
	emgd/display/pd/cmn/pd.o \
	emgd/display/pi/cmn/igd_pi.o \
	emgd/display/pi/cmn/displayid.o \
	emgd/display/pi/cmn/pd_init_all.o \
	emgd/display/pi/cmn/edid.o \
	emgd/display/pi/cmn/pi.o \
	emgd/display/pi/cmn/mode_table.o \
	emgd/display/pi/tnc/i2c_gmbus_tnc.o \
	emgd/display/pi/tnc/i2c_bitbash_tnc.o \
	emgd/display/pi/plb/i2c_plb.o \
	emgd/display/mode/cmn/match.o \
	emgd/display/mode/cmn/micro_mode.o \
	emgd/display/mode/cmn/vga_mode.o \
	emgd/display/mode/cmn/igd_mode.o \
	emgd/display/mode/tnc/micro_mode_tnc.o \
	emgd/display/mode/tnc/mode_tnc.o \
	emgd/display/mode/tnc/kms_mode_tnc.o \
	emgd/display/mode/tnc/clocks_tnc.o \
	emgd/display/mode/plb/micro_mode_plb.o \
	emgd/display/mode/plb/clocks_plb.o \
	emgd/display/mode/plb/mode_plb.o \
	emgd/display/mode/plb/kms_mode_plb.o \
	emgd/display/dsp/cmn/dsp.o \
	emgd/display/dsp/tnc/dsp_tnc.o \
	emgd/display/dsp/plb/dsp_plb.o \
	emgd/core/init/cmn/igd_global.o \
	emgd/core/init/cmn/igd_init.o \
	emgd/core/init/tnc/micro_init_tnc.o \
	emgd/core/init/tnc/init_tnc.o \
	emgd/core/init/plb/init_plb.o \
	emgd/core/init/plb/micro_init_plb.o \
	emgd/state/power/cmn/igd_pwr.o \
	emgd/state/power/plb/pwr_plb.o \
	emgd/state/appcontext/cmn/igd_appcontext.o \
	emgd/state/appcontext/plb/appcontext_plb.o \
	emgd/state/reg/cmn/reg.o \
	emgd/state/reg/tnc/reg_tnc.o \
	emgd/state/reg/plb/reg_plb.o \
	emgd/video/overlay/cmn/ovl_coeff.o \
	emgd/video/overlay/cmn/igd_ovl.o \
	emgd/video/overlay/cmn/micro_ovl.o \
	emgd/video/overlay/tnc/ovl_tnc_cache.o \
	emgd/video/overlay/tnc/ovl_tnc.o \
	emgd/video/overlay/tnc/ovl2_tnc.o \
	emgd/video/overlay/tnc/micro_ovl_tnc.o \
	emgd/video/overlay/plb/ovl_plb_cache.o \
	emgd/video/overlay/plb/ovl2_plb.o \
	emgd/video/overlay/plb/ovl_plb.o \
	emgd/video/overlay/plb/micro_ovl_plb.o \
	emgd/video/msvdx/msvdx_init.o \
	emgd/video/msvdx/msvdx.o \
	emgd/video/msvdx/msvdx_pvr.o \
	emgd/video/topaz/topaz_init.o \
	emgd/video/topaz/topaz.o \
	emgd/pal/sdvo/sdvo_attr.o \
	emgd/pal/sdvo/sdvo_hdmi.o \
	emgd/pal/sdvo/sdvo_port.o \
	emgd/pal/sdvo/sdvo_intf.o \
	emgd/pal/ch7036/ch7036_attr.o \
	emgd/pal/ch7036/ch7036_fw.o \
	emgd/pal/ch7036/ch7036_intf.o \
	emgd/pal/ch7036/ch7036_port.o \
	emgd/pal/ch7036/ch7036.o \
	emgd/pal/ch7036/ch7036_iic.o \
	emgd/pal/ch7036/ch7036_pm.o \
	emgd/pal/ch7036/ch7036_reg_table.o \
	emgd/pal/ch7036/lvds/lvds.o \
	emgd/pal/lvds/lvds.o \
	emgd/pal/lpd/lpd.o \
	emgd/gmm/gmm.o \
	emgd/gmm/gtt.o \
	emgd/utils/pci.o \
	emgd/utils/memmap.o \
	emgd/utils/math_fix.o \

ENVDIR = pvr/services4/srvkm/env/linux
COMMONDIR = pvr/services4/srvkm/common
BRIDGEDDIR = pvr/services4/srvkm/bridged
SYSCONFIGDIR = pvr/services4/system/common
SGXDIR = pvr/services4/srvkm/devices/sgx
DISPCLASSDIR = pvr/services4/3rdparty/emgd_displayclass


ENV_OBJS = $(ENVDIR)/osfunc.o \
	   $(ENVDIR)/mutils.o \
	   $(ENVDIR)/mmap.o \
	   $(ENVDIR)/module.o \
	   $(ENVDIR)/pdump.o \
	   $(ENVDIR)/proc.o \
	   $(ENVDIR)/pvr_bridge_k.o \
	   $(ENVDIR)/pvr_debug.o \
	   $(ENVDIR)/mm.o \
	   $(ENVDIR)/mutex.o \
	   $(ENVDIR)/event.o \
	   $(ENVDIR)/osperproc.o \
	   $(ENVDIR)/pvr_drm.o

COMMON_OBJS = $(COMMONDIR)/buffer_manager.o \
	    $(COMMONDIR)/devicemem.o \
	    $(COMMONDIR)/deviceclass.o \
	    $(COMMONDIR)/handle.o \
	    $(COMMONDIR)/hash.o \
	    $(COMMONDIR)/metrics.o \
	    $(COMMONDIR)/pvrsrv.o \
	    $(COMMONDIR)/queue.o \
	    $(COMMONDIR)/ra.o \
	    $(COMMONDIR)/resman.o \
	    $(COMMONDIR)/power.o \
	    $(COMMONDIR)/mem.o \
	    $(COMMONDIR)/pdump_common.o \
	    $(COMMONDIR)/perproc.o \
	    $(COMMONDIR)/lists.o \
	    $(COMMONDIR)/mem_debug.o

BRIDGED_OBJS = $(BRIDGEDDIR)/bridged_support.o \
	     $(BRIDGEDDIR)/bridged_pvr_bridge.o \
	     $(BRIDGEDDIR)/sgx/bridged_sgx_bridge.o

SYSCONFIG_OBJS = $(SYSCONFIGDIR)/sysconfig.o \
	        pvr/services4/system/tnc/sysconfig.o \
	        pvr/services4/system/plb/sysconfig.o \
		$(SYSCONFIGDIR)/sysutils.o
#		 $(SYSCONFIGDIR)/sysirq.o
#	 	 $(SYSCONFIGDIR)/ospm_power.o \

SGX_OBJS = $(SGXDIR)/sgxinit.o \
	 $(SGXDIR)/sgxpower.o \
	 $(SGXDIR)/sgxreset.o \
	 $(SGXDIR)/sgxutils.o \
	 $(SGXDIR)/sgxkick.o \
	 $(SGXDIR)/sgxtransfer.o \
	 $(SGXDIR)/mmu.o \
	 $(SGXDIR)/pb.o

DC_OBJS = $(DISPCLASSDIR)/emgd_dc.o \
	  $(DISPCLASSDIR)/emgd_dc_linux.o


emgd-y := \
	$(DC_OBJS) \
	$(EMGD_OBJS) \
	$(ENV_OBJS) \
	$(COMMON_OBJS) \
	$(BRIDGED_OBJS) \
	$(SYSCONFIG_OBJS) \
	$(SGX_OBJS) \


obj-$(CONFIG_DRM_EMGD) += emgd.o

