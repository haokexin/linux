#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
# Test for gcc 'asm goto' support
# Copyright (C) 2010, Jason Baron <jbaron@redhat.com>

TEST=$1
shift

case $TEST in
    "goto")
cat << "END" | $@ -x c - -fno-PIE -c -o /dev/null
int main(void)
{
#if defined(__arm__) || defined(__aarch64__)
	/*
	 * Not related to asm goto, but used by jump label
	 * and broken on some ARM GCC versions (see GCC Bug 48637).
	 */
	static struct { int dummy; int state; } tp;
	asm (".long %c0" :: "i" (&tp.state));
#endif

entry:
	asm goto ("" :::: entry);
	return 0;
}
END
    ;;

    "goto_output")
cat << "END" | $@ -x c - -c -o /dev/null
int foo(int x) {
	asm goto ("": "=r"(x) ::: bar);
	return x;
	bar: return 0;
}
END
    ;;

    "goto_tied_output")
cat << "END" | $@ -x c - -c -o /dev/null
int foo(int *x) {
	asm goto (".long (%l[bar]) - .\n": "+m"(*x) ::: bar);
	return *x;
	bar: return 0;
}
END
    ;;

    *)
	exit -1
    ;;
esac
