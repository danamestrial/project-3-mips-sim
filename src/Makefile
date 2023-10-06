all: sim sim2

sim: shell.c sim.c
	gcc -g -O2 $^ -o $@

sim2: shell.c sim2.c
	gcc -g -O2 $^ -o $@

.PHONY: clean
clean:
	rm -rf *.o *~ sim
	rm -rf *.o *~ sim2
