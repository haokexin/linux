#!/bin/bash

set -e
~/prj/msgbx_ipc/idl-tools/bin/bst_idl_code_gen touch-client.fdepl && {\
	cp -rv touch-client/src-gen/* ../touch-virt/
	rm -rf touch-client
	echo "==========success!============"
} || {
	echo "==========failure!============"
}
