/*
 * Copyright (C) 2012 Red Hat, Inc., Frederic Weisbecker <fweisbec@redhat.com>
 *
 * Licensed under the terms of the GNU GPL License version 2
 *
 * Move a task to a cgroup and forkbomb there.
 */

#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char **argv)
{
	FILE *fp;
	int pid;
	char cpid[50];

	if (argc < 2)
		return -1;

	pid = getpid();
	fp = fopen(argv[1], "w");
	if (!fp)
		return -2;

	sprintf(cpid, "%d\n", pid);

	if (fwrite(cpid, strlen(cpid), 1, fp) != 1) {
		perror("can't write pid\n");
		return -3;
	}
	fclose(fp);

	for (;;)
		fork();

	return 0;
}
