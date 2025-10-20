CXX = clang++
CXXFLAGS = -g -std=c++20 -arch arm64 -I/opt/homebrew/include \
		   $(shell pkg-config --cflags raylib)

LIB = $(shell pkg-config --libs raylib)

SRC_DIR = src
BUILD_DIR = build

EXTERNAL_DIRS = external/raygui/src external/SimplexNoise/src

CXXFLAGS += $(addprefix -I, $(EXTERNAL_DIRS))

SRCS = $(shell find $(SRC_DIR) -name "*.cpp") \
	   $(foreach dir, $(EXTERNAL_DIRS), $(shell find $(dir) -name "*.cpp"))

OBJS = $(patsubst %.cpp, $(BUILD_DIR)/%.o, $(SRCS))

TARGET = $(BUILD_DIR)/a.out

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LIB)

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
