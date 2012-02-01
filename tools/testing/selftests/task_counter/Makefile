all:
	gcc fork.c -o fork
	gcc forkbomb.c -o forkbomb
	gcc multithread.c -o multithread -lpthread
	gcc spread_thread_group.c -o spread_thread_group -lpthread

clean:
	rm -f fork forkbomb multithread spread_thread_group *~
