#!/bin/bash
#
# ----------------------------------------------------------------------------
# This confidential and proprietary software may be used only as
# authorised by a licensing agreement from ARM Limited.
# (C) COPYRIGHT 2020-2021 ARM Limited
# ALL RIGHTS RESERVED
# The entire notice above must be reproduced on all authorised
# copies and copies may only be made to the extent permitted
# by a licensing agreement from ARM Limited.
# ----------------------------------------------------------------------------
#
# Purpose:
#   To download source code and apply patches to enable them for use
#   with the DDK.
#
# Usage:
#   cd <your_path/xen>
#   ./download_and_patch.sh xen_version
#   example: ./download_and_patch.sh 4.14

set -e

PATCH_LIST=("host-xen.patch")
PATCHES=$PWD/patches

mkdir -p host

echo $PATCH_LIST

if [ $# -eq 0 ]
then
echo "xen version required, eg 4.11"
exit -1
fi

function prep_host_xen() {

    echo "Preparing Xen hypervisor..."
    pushd host
    if [ ! -d "xen/.git" ]; then
        git clone git://xenbits.xen.org/xen.git
    fi
    pushd xen/xen
    git stash
    git clean -fd
    if [ $1 = "4.11" ]
    then
        git checkout RELEASE-4.11.0
        echo "revert commit 9f954a5e90414d10632e6c2fef5a33ea8a4a1e4e"
        git revert --no-edit 9f954a5e90414d10632e6c2fef5a33ea8a4a1e4e
    elif [ $1 = "4.14" ]
    then
        git checkout c93b520a41f2787dd76bfb2e454836d1d5787505
    fi
    popd
    popd
}

function apply_patches() {
    echo "Applying patches"
    pushd host/xen/$1
    for f in "${PATCH_LIST[@]}" ; do
        echo "Patch file $f"
        patch -f -N -p 1 -i $PATCHES/$f
    done
    popd
}

prep_host_xen $1
if [ -d "$PATCHES" ]; then
    apply_patches
else
    echo "Error: missing patches folder!"
    echo "Please copy the patches folder in the current directory!"
    exit -1
fi
echo "Script complete"
