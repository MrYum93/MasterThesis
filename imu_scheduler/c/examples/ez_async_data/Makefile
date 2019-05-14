BIN = .
NAME = demo_encoder
OBJECTS = main.o vectornav.o vel_est.o

CC = gcc
INCLUDES = -I ../../include
CFLAGS = -Wall

all: ez_async_data

clean:
	rm -f main.o
	rm -f ez_async_data
	rm -f vel_est.o
	cd ../.. && make clean

ez_async_data: main.o vectornav.o vel_est.o proglib 
	$(CC) -o imu_sched main.o vectornav.o vel_est.o -L ../../build -lvnc -lpthread -lm -lrt 

proglib:
	cd ../.. && make

main.o: main.c 
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

vectornav.o: vectornav.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

vel_est.o: vel_est.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

