CXX=g++
CXX_FLAGS=-std=c++17

DEPENDENCIES=wiringPi
HEADERS=constants.h

targets=main test 

all: main

main: main.o ${HEADERS}
		${CXX} ${CXXFLAGS} $^ -o main -l ${DEPENDENCIES}

test: test.o ${HEADERS}
		${CXX} ${CXXFLAGS} $^ -o test -l ${DEPENDENCIES}
		./test

main.o: main.cpp
		${CXX} ${CXXFLAGS} $^ -c

test.o: test.cpp
		${CXX} ${CXXFLAGS} $^ -c

clean:
		rm -f ${targets} *.o *.gch a.out *.exe
