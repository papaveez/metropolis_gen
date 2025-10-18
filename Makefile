CXX = clang++
CXXFLAGS = -g -std=c++20 -arch arm64 -I/opt/homebrew/include \
		   $(shell pkg-config --cflags raylib) \
		   -Iexternal/raygui/src

LIB = $(shell pkg-config --libs raylib)

SRC_DIR = src
BUILD_DIR = build


SRCS = $(shell find $(SRC_DIR) -name "*.cpp")
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRCS))
TARGET = $(BUILD_DIR)/a.out

all: $(TARGET)

# Ensure all build subdirectories exist
$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LIB)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run

