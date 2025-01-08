# Compiler settings
CXX      := g++
CXXFLAGS := -Wall -Iinclude
CC := gcc
CFLAGS := -Wall -Iinclude
LIBS     := -lwiringPi

# Directories
SRC   := src
UTIL   := src/util
BIN   := bin
INCLUDE   := include

# Object files
OBJS     := $(BIN)/led.o $(BIN)/setup.o $(BIN)/BMP_390.o $(BIN)/ICM_20948.o 
MAIN_OBJ := $(BIN)/main.o
TEST_OBJ := $(BIN)/test.o

# Executables
main := $(BIN)/main
test := $(BIN)/test

# Default target
.PHONY: all
all: $(main) $(test)
test: $(test)
	./bin/test

$(main): $(OBJS) $(MAIN_OBJ)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

$(test): $(OBJS) $(TEST_OBJ)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

$(BIN)/led.o: $(SRC)/led.cpp $(INCLUDE)/led.h $(INCLUDE)/constants.h
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN)/setup.o: $(SRC)/setup.cpp $(INCLUDE)/setup.h $(INCLUDE)/constants.h
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN)/ICM_20948.o: $(BIN)/ICM_20948.cpp $(INCLUDE)/ICM_20948.h $(UTIL)/ICM_20948_REGISTERS.h $(UTIL)/AK09916_REGISTERS.h
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN)/BMP_390.o: $(BIN)/BMP_390.c $(INCLUDE)/BMP_390.h
	@mkdir -p $(BIN)
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN)/main.o: $(SRC)/main.cpp $(INCLUDE)/led.h $(INCLUDE)/setup.h $(BIN)/ICM_20948.h $(BIN)/BMP_390.h 
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN)/test.o: $(SRC)/test.cpp $(INCLUDE)/led.h $(INCLUDE)/setup.h $(BIN)/ICM_20948.h $(BIN)/BMP_390.h 
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(BIN)/*.o
	rm -f $(main)
	rm -f $(test)
