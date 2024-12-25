CXX=g++
CXX_FLAGS=-std=c++17

all: main

main: main.o
		${CXX} ${CXXFLAGS} $^ -o main

main.o: main.cpp
		${CXX} ${CXXFLAGS} $^ -c -l wiringPi

clean:
		rm -f main *.o *.gch a.out *.exe