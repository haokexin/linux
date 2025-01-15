CC = aarch64-linux-gnu-gcc
CFLAGS = -O2
CFLAGS1 = -O3

TESTS := test-map test-alloc-queue test-aec-ns hifi_test test-queue test-kws test-kws-by-file

$(CC) $(CFLAGS) all:  $(TESTS)
clean:
	rm -f $(TESTS)

%: %.c
