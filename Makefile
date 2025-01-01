# Compiler settings
CXX      := g++
CXXFLAGS := -Wall -Iinclude   # Add -g for debugging or -O2 for optimization
LIBS     := -lwiringPi        # Link wiringPi

# Directories
SRC   := src
BIN   := bin
INCLUDE   := include

# Object files
OBJS     := $(BIN)/led.o $(BIN)/setup.o
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

$(BIN)/main.o: $(SRC)/main.cpp $(INCLUDE)/led.h $(INCLUDE)/setup.h
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN)/test.o: $(SRC)/test.cpp $(INCLUDE)/led.h $(INCLUDE)/setup.h
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(BIN)/*.o
	rm -f $(main)
	rm -f $(test)
