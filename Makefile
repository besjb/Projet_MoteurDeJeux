MAKEFLAGS += -j -k

CXX = g++

SRC_DIR = src
INCLUDE_DIR = -Iinclude -Icommon/include
LIB_DIR = -Lcommon/lib
OBJ_DIR = obj

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

CXXFLAGS = -Ofast -std=c++23 $(INCLUDE_DIR)

LIBS = -lglfw3 -lGL -lGLEW

TARGET = main

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIR) $(LIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm $(OBJ_DIR)/*.o
	rm ./main

.PHONY: all clean