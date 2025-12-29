#!/bin/bash

CROSS_COMPILETOOL_GNU=${TOOLCHAIN}aarch64-linux-gnu-
CROSS_COMPILETOOL_BST=${TOOLCHAIN}aarch64-bst-linux-
BUILD_LOG=$(pwd)/build.log

AR_GNU=${CROSS_COMPILETOOL_GNU}ar
AR_BST=${CROSS_COMPILETOOL_BST}ar
if command -v ${AR_GNU} 1>/dev/null 2>&1; then
    CROSS_COMPILETOOL=${CROSS_COMPILETOOL_GNU}
fi
if command -v ${AR_BST} 1>/dev/null 2>&1; then
    CROSS_COMPILETOOL=${CROSS_COMPILETOOL_BST}
fi

if [ -z "$CROSS_COMPILETOOL" ]; then
    echo 'Failed to find compile tools.'
    exit 1
fi

export LOCALVERSION=""

file_path=`${CROSS_COMPILETOOL}gcc -print-file-name=plugin`/plugin-version.h
if [ -f "$file_path" ]
then
	echo "remove $file_path to adapt compiling";
	if [ -f "$file_path" ]
	then
		echo "you must remove file $file_path before building kernel !!!"
		exit
	fi
fi

function help()
{
cat << EOF
        This script is used for build c1200 ivi kernel use gki build mode.
        usage:
                `basename $0` <option>

                options:
                -h/--help               display this help info
                -c                      remove build dir and clean the previous build
                menuconfig              make menuconfig use the default build .config
                update                  update build kernel and dtb usd build .config
                all                     merge gki_defconfig and config_fragment build dtb and kernel
                default                 build all
EOF
exit;
}


# Step1: copy and merge bstc1200 basic config fragment
function generate_build_config() {
        if [ ! -d "build" ]; then
                mkdir build
        fi
        cp arch/arm64/configs/bstc1200_gki.fragment ./build/.config
        make CROSS_COMPILE=$CROSS_COMPILETOOL  ARCH=arm64 O=build olddefconfig

        ./scripts/kconfig/merge_config.sh -m -O \
                ./build/ \
                ./build/.config \
                ./arch/arm64/configs/android-u-base.config > /dev/null 2>&1

        make CROSS_COMPILE=$CROSS_COMPILETOOL  ARCH=arm64 O=build olddefconfig
}


# Step2: delete unsupport config items
function update_build_config() {
        ./scripts/config --file build/.config \
                -d CONFIG_ZONE_DMA32 \
                -d CONFIG_ZONE_DMA \
                -d CONFIG_MODVERSIONS \
                -e CONFIG_DEVMEM

        make CROSS_COMPILE=$CROSS_COMPILETOOL  ARCH=arm64 O=build olddefconfig
}


# Setp3: build default kernel image and copy related dtb and image
function build_kernel_image() {
        make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build -j32
        make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build modules -j32
        # remove previous install kernel modules in modules_dir, fix install disagree version git modules
        rm -rf ./build/modules_dir/lib/modules/*
        make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build INSTALL_MOD_PATH=modules_dir modules_install -j32
}


# setp4: copy kernel image、related dtbs
function copy_image_dtb() {
    local overlay
    readonly PREBUILT=../../device/bst/c1200/prebuilt
    IMAGE_FILES=" Image
                  bstc1200-ivi.dtb
                  c1200-evb-overlay.dtb
                  c1200-cdcu-overlay.dtb
		  c1296-cdcu1.0-ivi-8c8g-overlay.dtb
		  c1296-cdcu1.0-ivi-4c6g-overlay.dtb
		  c1296-cdcu2.0-ivi-8c8g-overlay.dtb
		  c1296-cdcu2.0-ivi-4c6g-overlay.dtb
                  c1296-evm1.0-ivi-4c6g-overlay.dtb "

    if [ ! -d "${PREBUILT}" ]; then
	    mkdir -p "${PREBUILT}"
    fi
    rm -rf "${PREBUILT}"/*.dtb
    for FILE in ${IMAGE_FILES}; do
        find ./build/arch/arm64/boot/ -name ${FILE} | xargs -i cp {} "${PREBUILT}"/
    done

    cp -dr ./build/arch/arm64/boot/dts/bst/overlay/isp "${PREBUILT}"/
    overlay=$(readlink ./build/arch/arm64/boot/dts/bst/overlay/bst-overlay.dtbo)
    overlay=${overlay#isp\/}
    ln -sf "${overlay}" "${PREBUILT}"/isp/isp-overlay.dtbo
    find "${PREBUILT}"/isp ! -name "*.dtbo" -type f  -exec rm '{}' + || true
}

# setp5: copy related bsp drivers ko modules, delete redundancy modules
function copy_ko_modules() {
        cp build/drivers/pci/bcmdhd/bcmdhd.ko ../../device/bst/c1200/prebuilt

	if [ ! -d ../../device/bst/c1200/prebuilt/kernel_modules ]; then
		mkdir -p ../../device/bst/c1200/prebuilt/kernel_modules
	fi

        KO_FILES="mali_kbase.ko
                bst-dc.ko
                dwc3-bst.ko
                dwc3.ko
                gadgetfs.ko
                dummy_hcd.ko
                usb_bst_ccgx.ko
                usb_bst_virt_msg.ko
                usb_virt_device.ko
                xhci-hcd.ko
                xhci-plat-hcd.ko
                roles.ko
                uas.ko
                usb-storage.ko
                ax88796b.ko
                asix.ko
                ax88179_178a.ko
                cdc_ether.ko
                cdc_ncm.ko
                cdc_subset.ko
                net1080.ko
                r8152.ko
                r8153_ecm.ko
                rtl8150.ko
                usbnet.ko
                zaurus.ko
                ch341.ko
                cp210x.ko
                ftdi_sio.ko
                option.ko
                pl2303.ko
                usb_wwan.ko
                usbserial.ko
                bst_hwcv.ko
                bstn_driver.ko"

        rm -rf ../../device/bst/c1200/prebuilt/kernel_modules/*.ko
        for FILE in ${KO_FILES}; do
                find ./build/modules_dir/ -name ${FILE} | xargs -i cp {} ../../device/bst/c1200/prebuilt/kernel_modules
        done
}

if [[ $# -eq 0 ]]
then
        generate_build_config
        update_build_config
        build_kernel_image 2>&1 | tee $BUILD_LOG
#        copy_image_dtb
#        copy_ko_modules
        exit
else
        args=( "$@" )
        for arg in ${args[*]} ; do
        case ${arg} in
                -h|-help)       help;;
                -c)             rm -rf build;;
                menuconfig)     make CROSS_COMPILE=$CROSS_COMPILETOOL  ARCH=arm64 O=build menuconfig;;
                update)         build_kernel_image;
                                copy_image_dtb;
                                copy_ko_modules;;
                all)            generate_build_config;
                                update_build_config;
                                build_kernel_image 2>&1 | tee $BUILD_LOG;
                                copy_image_dtb;
                                copy_ko_modules;;
                *)              help;;
        esac
        done
fi

