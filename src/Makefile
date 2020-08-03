CC=g++
LIB=-li2c
INC=-Icore/ -Iplatform
CFLAGS=-lstdc++

all: VL53L1X_api.o vl53l1_platform.o
	$(CC) $(LIB) $(INC) $(CFLAGS) main.cpp vl53l1_platform.o VL53L1X_api.o -o main

vl53l1_platform.o:
	$(CC) $(LIB) $(INC) $(CFLAGS) -c platform/vl53l1_platform.c

VL53L1X_api.o:
	$(CC) $(LIB) $(INC) $(CFLAGS) -c core/VL53L1X_api.c

clean:
	rm -rf *.o main
