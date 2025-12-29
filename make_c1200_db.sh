#!/bin/sh

CROSS_COMPILETOOL_GNU=${TOOLCHAIN}aarch64-linux-gnu-
CROSS_COMPILETOOL_BST=${TOOLCHAIN}aarch64-bst-linux-

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

file_path=`${CROSS_COMPILETOOL}gcc -print-file-name=plugin`/include/plugin-version.h
if [ -f "$file_path" ]
then 
	echo "remove $file_path to adapt compiling";
	if [ -f "$file_path" ]
	then
		echo "you must remove file $file_path before building kernel !!!"
		exit
	fi	
fi

export LOCALVERSION=""

make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build bstc1200_db_defconfig
make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build -j32 
make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build modules -j32
