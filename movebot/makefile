CXX = g++
CXXFLAGS = -Wall -std=c++17 -Iinclude
TARGET = movebot
SRC_DIR = src
SRC = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_DIR = obj
OBJ = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC))
DEP = $(OBJ:.o=.d)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@

-include $(DEP)

clean:
	rm -rf $(OBJ_DIR) $(TARGET)