#!/bin/sh

CROSS_COMPILETOOL=aarch64-linux-gnu-

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

make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build bstc1200_recovery_defconfig
make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build -j32 
make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build modules -j32

