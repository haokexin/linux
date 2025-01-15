1. code structure please refer to this link:
<https://q4sut0rjd7.feishu.cn/wiki/wikcne0IC63RvXHAYBLJGg4paKd>

2. use guide
You could download these codes and run ./build.sh, which can compile all code and copy program into ./lib/ directory. Then you could run related program to check running result.
prerequisite:
ubuntu environment

* ipc_hw_shm_recv / ipc_hw_shm_send : these are ipc hw_layer API test demo.
  resource code : ./test/ipc_hw_recv_test.c ./test/ipc_hw_send_test.c

* ipc_trans_bare_client.c / ipc_trans_bare_server.c : these are ipc trans_layer API test demo
  resource code : ./test/ipc_trans_bare_client.c ./test/ipc_trans_bare_server.c

* test_client / test_server : these are ipc app_layer API test demo.
  resource code : ./test/baremetal/

**IMPORTANT**
you could refer to these API test resource code to test your implementation, if necessary.
