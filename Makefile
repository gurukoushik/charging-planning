PURPLE = \033[1;35m
NC = \033[00m

.PHONY: all
all: build format

.PHONY: format
format:
		@echo "${PURPLE}Running clang-format on all cpp and hpp files...${NC}"
		find . -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.h" -not -path "**/build/*" | xargs clang-format -style=Google -i

.PHONY: build
build: clean
		@echo "${PURPLE}Building the targets...${NC}"
		mkdir build && cd build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. && make

.PHONY: build-debug
build-debug: clean
		@echo "${PURPLE}Building the targets in debug mode...${NC}"
		mkdir build && cd build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug .. && make

.PHONY: clean
clean:
		@echo "${PURPLE}Clean build files...${NC}"
		rm -r build | true
