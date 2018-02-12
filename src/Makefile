# Configuration Options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Set platform, debug mode, and parallel compile if not already set.
platform ?= LINUX_gcc
debug    ?= 0
parallel ?= 0

# Temporary setting for making vizmo-compatible maps for nonholonomic robots.
vizmo_map := #-DVIZMO_MAP

# Define robot type if it isn't already set.
ROBOT_DEF ?= PMPCfg

# Build support for iCreate hardware?
icreate ?= 0

# Directory Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

PMPL_DIR  := .
UTILS_DIR := ../utils

# The OBJ_DIR is the landing zone for our object files. Create it with a
# directory structure to match our source tree.
OBJ_DIR := build
MOC_DIR := $(OBJ_DIR)/moc

# Treat all sub-directories except the build tree as possible source locations.
SRC_DIRS  := $(patsubst ./%,%, $(shell find . -type d \
               ! \( -path "./$(OBJ_DIR)*" -o -name '.' \) ))

# Always try to rebuild the build directory to catch newly added dirs.
#.PHONY: $(OBJ_DIR)
$(OBJ_DIR):
	@mkdir -p $@ $(patsubst %,$(OBJ_DIR)/%,$(SRC_DIRS))
$(MOC_DIR): | $(OBJ_DIR)
	@mkdir $@

# Look for files in the source directories.
vpath %.h       $(SRC_DIRS)
vpath %.cpp     $(SRC_DIRS)
vpath moc_%.cpp $(MOC_DIR)

# External Makefiles ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

include $(UTILS_DIR)/makefile_includes/Makefile.defaults
include $(PMPL_DIR)/Makefile.PMPLdefaults

# Compiler Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

DEFS = $(PMPL_DEF) $(UTILS_DEF) $(SIM_DEF) $(vizmo_map)
INCL = $(PMPL_INCL) $(UTILS_INCL) $(STAPL_INCL) $(BOOST_INCL) $(SIM_INCL)
LIBS = $(PMPL_LIB) $(UTILS_LIB) $(STAPL_LIB) $(BOOST_LIB) $(SIM_LIBS)

# Object File Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Pre-pend the build directory to the object file names.
PMPL_OBJS := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(PMPL_SRCS))
SIM_OBJS  := $(patsubst %.cpp,$(OBJ_DIR)/%.o, $(SIM_SRCS))

# Make the object files depend on updated utility libraries so the libs are
# built first.
$(PMPL_OBJS): $(UTILS_LIBFILE) $(STAPL_LIBFILE)
$(SIM_OBJS): $(UTILS_LIBFILE) $(STAPL_LIBFILE)

ifeq ($(parallel), 1)
  MAIN = parallel_main.o
else
  MAIN = main.o
endif
$(MAIN): $(PMPL_LIBFILE)

# Library Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Default is to build the library object.
.DEFAULT_GOAL := $(PMPL_LIBFILE)

$(PMPL_LIBFILE): $(PMPL_OBJS)
	@echo Linking pmpl library...
	@$(AR) $@ $^

# Executable Recipes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Build the traditional pmpl executable.
pmpl: $(MAIN) $(PMPL_LIBFILE)
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(MAIN) $(LIBS) -o $@

# Build the simulator.
sim: SIM_DEF := -DPMPL_SIMULATOR
sim: $(PMPL_LIBFILE) $(SIM_OBJS) $(MOC_OBJS) \
     | $(patsubst %,$(MOC_DIR)/%,$(MOC_SRCS))
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(SIM_OBJS) $(MOC_OBJS) $(LIBS) -o $@

# Object File Recipes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Set up dependency tracking.
DEPS_OUT = $(OBJ_DIR)/$*.d
DEPS     = -MMD -MF $(DEPS_OUT)

DEPENDENCIES := $(OBJ_DIR)/Dependencies
-include $(DEPENDENCIES)

# This recipe runs headers with Q_OBJECT through the qt MOC.
$(MOC_DIR)/moc_%.cpp: %.h | $(MOC_DIR)
	@echo Moc\'ing $(shell echo $< | sed 's|.*/||' )...
	@$(QT_MOC) $< -o $@

# This recipe is for MOC sources.
$(MOC_DIR)/moc_%.o: $(MOC_DIR)/moc_%.cpp %.h | $(MOC_DIR)
	@echo Compiling Moc\'d source $(shell echo $< | sed 's|.*/||' )...
	@$(CXX) -c $(CXXFLAGS) $(OPTS) $(DEFS) $(INCL) $(DEPS) $< -o $@
	@cat $(DEPS_OUT) >> $(DEPENDENCIES)
	@rm -f $(DEPS_OUT)

# This recipe is for regular objects.
$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR)
	@$(MAKE) -s check_platform
	@echo Compiling $<...
	@$(CXX) -c $(CXXFLAGS) $(OPTS) $(DEFS) $(INCL) $(DEPS) $< -o $@
	@cat $(DEPS_OUT) >> $(DEPENDENCIES)
	@rm -f $(DEPS_OUT)

# Cleanup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

.PHONY: clean
clean:
	@echo Cleaning pmpl library and executables...
	@rm -f $(PMPL_LIBFILE) $(MAIN) pmpl $(SIM_OBJS) sim

.PHONY: reallyclean
reallyclean: clean
	@echo Cleaning all pmpl objects...
	@rm -f $(PMPL_OBJS)
	@rm -rf $(OBJ_DIR)
	@rm -f $(DEPENDENCIES)

.PHONY: reallyreallyclean
reallyreallyclean: reallyclean
	@echo Cleaning all utilities...
	@cd $(STAPL_DIR) && $(MAKE) platform=$(platform) stl=$(STL_LIB) clean
	@cd $(CD_DIR) && $(MAKE) clean
	@cd $(TINYXML_DIR) && $(MAKE) clean
	@cd $(MPNN_SRC) && $(MAKE) clean
	@cd $(MATHTOOL_DIR) && $(MAKE) clean
	@cd $(MODELLOADER_DIR) && $(MAKE) clean
	@cd $(KMEANS_DIR) && $(MAKE) clean
	@cd $(TETGEN_DIR) && $(MAKE) clean
	@cd $(SANDBOX_DIR) && $(MAKE) clean
	@cd $(BULLET_DIR) && $(MAKE) clean
	@cd $(PHYSICAL_ROBOTS_DIR) && $(MAKE) clean

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
.PHONY: test
test: SIM_DEF := -DPMPL_SIMULATOR
test:
	@echo srcdirs $(SRC_DIRS)
