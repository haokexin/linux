/*
 * Copyright (C) 2012 Red Hat, Inc., Frederic Weisbecker <fweisbec@redhat.com>
 *
 * Licensed under the terms of the GNU GPL License version 2
 *
 * Try to reproduce bug on common ancestor logic as described
 * at https://lkml.org/lkml/2011/11/8/218
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

volatile static int thread_end;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;


static void *thread_start(void *unused)
{
	pthread_mutex_lock(&mutex);

	while (!thread_end)
		pthread_cond_wait(&cond, &mutex);

	pthread_mutex_unlock(&mutex);

	return NULL;
}

int main(int argc, char **argv)
{
	int fd_root, fd;
	int pid;
	char cpid[50];
	pthread_t thread;

	if (argc < 3)
		return -1;

	if (pthread_create(&thread, NULL, thread_start, NULL) != 0)
		return -2;

	pid = getpid();
	fd = open(argv[1], O_WRONLY);
	if (fd < 0)
		return -3;

	sprintf(cpid, "%d\n", pid);

	/* Move group to /dev/cgroup/cgroup0 */
	if (write(fd, cpid, strlen(cpid)) < 0)
		return -4;

	fd_root = open(argv[2], O_WRONLY);
	if (fd < 0)
		return -5;

	/* Move group leader to /dev/cgroup/ root */
	if (write(fd_root, cpid, strlen(cpid)) < 0)
		return -6;

	/* Move back group to /dev/cgroup/cgroup0 */
	if (write(fd, cpid, strlen(cpid)) < 0)
		return -7;

	close(fd_root);
	close(fd);

	pthread_mutex_lock(&mutex);
	thread_end = 1;
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&mutex);

	pthread_join(thread, NULL);

	return 0;
}
