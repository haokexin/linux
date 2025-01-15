#!/bin/sh

CROSS_COMPILETOOL=aarch64-linux-gnu-
#CROSS_COMPILETOOL=aarch64-bst-linux-

export LOCALVERSION=""

make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build $1 $2 $3 $4 $5 $6 $7 $8
