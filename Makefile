CXX        := g++
CXXFLAGS   := -std=c++17 -Iinclude
SRC        := src
BIN        := bin
DEPENDENCIES := wiringPi

SRCS       := $(wildcard $(SRC)/*.cpp)
BINS       := $(patsubst $(SRC)/%.cpp, $(BIN)/%, $(SRCS))
HDRS       := $(wildcard include/*.h)

.PHONY: all clean

all: $(BINS)

$(BIN)/%: $(SRC)/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -l$(DEPENDENCIES)

clean:
	rm -rf $(BIN)/*
