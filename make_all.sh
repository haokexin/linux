#!/bin/sh

Usage=" $0 [help] [CROSS_COMPILE] [OURDIR] [DEFCONFIG]"
CROSS=aarch64-linux-gnu-
OURDIR=./build/
DEFCONFIG=defconfig

if [ ! -d $OURDIR ]; then
	mkdir -p $OURDIR
fi

for arg in "$@"
do
	shift
	[[ $arg == *help ]]		&& echo -e "$Usage"			&& exit 0
	[[ $arg == CROSS_COMPILE* ]]	&& CROSS=${arg//CROSS_COMPILE=/}	&& continue
	[[ $arg == OURDIR* ]]		&& OURDIR=${arg//OURDIR=/}		&& continue
	[[ $arg == DEFCONFIG* ]]	&& DEFCONFIG=${arg//DEFCONFIG=/}	&& continue
done

echo "$0 $CROSS $OURDIR $DEFCONFIG"

make LOCALVERSION= ARCH=arm64 CROSS_COMPILE=$CROSS O=$OURDIR $DEFCONFIG -j32 && \
make LOCALVERSION= ARCH=arm64 CROSS_COMPILE=$CROSS O=$OURDIR all -j32 && \
make LOCALVERSION= ARCH=arm64 CROSS_COMPILE=$CROSS O=$OURDIR INSTALL_MOD_PATH=modules_install modules_install -j32
