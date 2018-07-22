CC=gcc
INC=-lpthread

ualarm_test:
	$(CC) -o $@ ualarm_test.c $(INC)

ualarm_test.s:
	$(CC) -S ualarm_test.c $(INC)

all: ualarm_test ualarm_test.s
clean: 
	@rm ualarm_test ualarm_test.s
