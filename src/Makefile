all: sim sim2 test

sim: shell.c sim.c
	gcc -g -O2 $^ -o $@

sim2: shell2.c sim2.c
	gcc -g -O2 -DDEBUG_FLAG $^ -o $@

test: shell2.c sim2.c
	gcc -g -O2 -DTEST_MODE $^ -o $@

.PHONY: clean
clean:
	rm -rf *.o *~ sim
	rm -rf *.o *~ sim2
	rm -rf *.o *~ test
