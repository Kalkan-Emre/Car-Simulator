all: simulator

simulator: helper.o WriteOutput.o main.o
	g++ helper.o WriteOutput.o main.o -o simulator -lpthread

helper.o: helper.c helper.h
	gcc -Wall -g -c helper.c

WriteOutput.o: WriteOutput.c WriteOutput.h
	gcc -Wall -g -c WriteOutput.c

main.o: main.cpp monitor.h
	g++ -Wall -g -c main.cpp

clean:
	rm -f *.o simulator

.PHONY: all clean
