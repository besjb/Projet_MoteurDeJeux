MAKEFLAGS += -j -k

CXX = g++

SRC_DIR = src
ifeq ($(OS),Windows_NT)
	INCLUDE_DIR = -Iinclude -IC:/mingw64-new/include
	LIB_DIR = -LC:/mingw64-new/lib
	LIBS = -lglfw3 -lgdi32 -lopengl32 -lglew32
else
	INCLUDE_DIR = -Iinclude -Icommon/include
	LIB_DIR = -Lcommon/lib
	LIBS = -lglfw3 -lGL -lGLEW -ldl -lpthread
endif

OBJ_DIR = obj

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

CXXFLAGS = -Ofast -std=c++23 $(INCLUDE_DIR)

TARGET = main

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIR) $(LIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
ifeq ($(OS),Windows_NT)
	del /f $(OBJ_DIR)\*.o $(TARGET).exe 2> NUL
else
	rm $(OBJ_DIR)/*.o
	rm ./main
endif

.PHONY: all clean
