/*
 * Copyright (C) 2012 Red Hat, Inc., Frederic Weisbecker <fweisbec@redhat.com>
 *
 * Licensed under the terms of the GNU GPL License version 2
 *
 * Try to move a multithread proc to a cgroup.
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
	int fd;
	int pid;
	char cpid[50];
	pthread_t thread;

	if (argc < 2)
		return -1;

	if (pthread_create(&thread, NULL, thread_start, NULL) != 0)
		return -2;

	pid = getpid();
	fd = open(argv[1], O_WRONLY);
	if (fd < 0)
		return -3;

	sprintf(cpid, "%d\n", pid);

	if (write(fd, cpid, strlen(cpid)) < 0)
		return -4;

	close(fd);

	pthread_mutex_lock(&mutex);
	thread_end = 1;
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&mutex);

	pthread_join(thread, NULL);

	return 0;
}
