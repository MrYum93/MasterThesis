BIN = ../bin
NAME = shmem
CC = gcc
CFLAGS =  -Wall -pedantic
LDFLAGS = -lm
BINARIES =  $(BIN)/$(NAME)
CLEAN = clean

all: $(BINARIES) cleanobj

$(BIN)/$(NAME): shmem.o \
	sharedmem.o \
	semaphore.o
	$(CC) -o $@ $^ $(LDFLAGS)

cleanobj:
	$(RM) */*.o
	$(RM) *.o

clean: cleanobj
	$(RM) $(BINARIES)
