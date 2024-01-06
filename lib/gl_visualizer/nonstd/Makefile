# Directory layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Configure the directory tree.
# - Object files will be placed in OBJ_DIR.
# - Source code is in SRC_DIR.
# - Test code is in TEST_DIR.

OBJ_DIR  := build
SRC_DIR  := nonstd
TEST_DIR := test
DOC_DIR  := docs

$(OBJ_DIR):
	@mkdir $(OBJ_DIR)

vpath %.h        $(SRC_DIR)
vpath %.cpp      $(SRC_DIR)
vpath test_%.cpp $(TEST_DIR)


# Dependency Tracking ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Set up automatic dependency tracking.

DEP_FILE   := $(OBJ_DIR)/Dependencies
DEP_GLOB    = $(OBJ_DIR)/$*.d
TRACK_DEPS  = cat $(DEP_GLOB) >> $(DEP_FILE) ; rm -f $(DEP_GLOB)

-include $(DEP_FILE)


# Compiler configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Configure compiler, flags, and options.

CXX   := g++
FLAGS := -std=c++11 -fPIC -g3 -O3
WARN  := -Wall -Werror -Wextra -Wno-parentheses
INCL  := -I.
DEPS   = -MMD -MF $(DEP_GLOB)

COMPILE = $(CXX) $(FLAGS) $(WARN) $(INCL) $(DEPS)


# Source file configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Scan the directory tree for source/test files with suffix '.cpp'.

SRCS      := $(shell find $(SRC_DIR) -name '*.cpp')
TEST_SRCS := $(shell find $(TEST_DIR) -name 'test_*.cpp')


# Object file configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Create an object name for each source file and define object compilation
# recipes.

OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o, $(SRCS))

$(OBJ_DIR)/%.o: %.cpp %.h | $(OBJ_DIR)
	@echo Compiling $(shell echo $< | sed 's|.*/||' )...
	@$(COMPILE) -c $< -o $@
	@$(TRACK_DEPS)


# Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Define recipes for building static and shared libraries.

NAME   := nonstd
SHARED := lib$(NAME).so
STATIC := lib$(NAME).a

.DEFAULT_GOAL := $(STATIC)

.PHONY: shared
shared: $(SHARED)
$(SHARED): $(OBJS)
	@echo Linking shared library...
	@$(CXX) -shared $^ -o $@

.PHONY: static
static: $(STATIC)
$(STATIC): $(OBJS)
	@echo Linking static library...
	@ar rcs $@ $^


# Test configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Create an object name for each test file and define object compilation
# recipes.

TEST_OBJS := $(subst .cpp,.o, $(TEST_SRCS))

$(TEST_DIR)/%.o: %.cpp $(.DEFAULT_GOAL)
	@echo Building $(shell echo $< | sed s/.cpp// | sed 's|.*/||' )...
	@$(COMPILE) $< -L. -l$(NAME) -o $@
	@$(TRACK_DEPS)


# Tests ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Define recipe for building and running the tests.

.PHONY: tests
tests: $(TEST_OBJS)
	@echo Testing...
	@for testobj in $(TEST_OBJS); do $$testobj; done


# Docs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Define recipe for building the docs.

HEADERS := $(shell find . -name '*.h')

.PHONY: docs
docs: $(SRCS) $(HEADERS)
	@echo Building docs...
	@doxygen $(DOC_DIR)/Doxyfile


# Clean ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#
# Define cleanup recipes.

.PHONY: clean
clean: cleandocs cleantests
	@rm -rf $(SHARED) $(STATIC) $(OBJ_DIR)

.PHONY: cleantests
cleantests:
	@rm -f $(TEST_OBJS)

.PHONY: cleandocs
cleandocs:
	@rm -rf $(DOC_DIR)/html


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
