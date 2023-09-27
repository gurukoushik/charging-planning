PURPLE = \033[1;35m
NC = \033[00m

.PHONY: all
all: build format tidy

.PHONY: format
format:
		@echo "${PURPLE}Running clang-format on all cpp and hpp files...${NC}"
		find . -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.h" -not -path "**/build/*" | xargs clang-format -style=Google -i

.PHONY: build
build: clean
		@echo "${PURPLE}Building the target...${NC}"
		g++ -std=c++11 -O1 main.cpp network.cpp -o plan

.PHONY: build-debug
build-debug: clean
		@echo "${PURPLE}Building the target in debug mode...${NC}"
		g++ -std=c++11 -g main.cpp network.cpp -o plan

.PHONY: clean
clean:
		@echo "${PURPLE}Clean build files...${NC}"
		rm plan | true
