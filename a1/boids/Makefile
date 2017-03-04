CC=g++
SOURCES=Boids.cpp util.cpp
OBJS=$(SOURCES:.cpp=.o)


all: Boids

Boids: Boids.cpp util.o
	$(CC) -O4 -g -fopenmp Boids.cpp util.o -L./lib -lglui -l3ds -I./include -lGL -lglut -lGLU -lm -o Boids
util.o: util.cpp util.h
	$(CC) -c -g util.cpp

#not revelent to lab
test: test.cpp util.h
	$(CC) test.cpp -o test

clean:
	rm -f Boids $(OBJS) a.out test
