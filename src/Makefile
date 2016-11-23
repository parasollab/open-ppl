# Configuration Options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Set platform, debug mode, and parallel compile if not already set.
platform ?= LINUX_gcc
debug    ?= 0
parallel ?= 0

# Define robot type if it isn't already set.
ROBOT_DEF ?= PMPCfg
#ROBOT_DEF ?= PMPState
#ROBOT_DEF ?= PMPReachableVolume

# Directory Layout ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

PMPL_DIR  ?= .
UTILS_DIR ?= ../utils

# External Makefiles ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

include $(UTILS_DIR)/makefile_includes/Makefile.defaults
include $(PMPL_DIR)/Makefile.PMPLdefaults

# Compiler Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

DEFS := $(PMPL_DEF) $(UTILS_DEF) $(STAPL_DEF) $(BOOST_DEF)
INCL := $(PMPL_INCL) $(UTILS_INCL) $(STAPL_INCL) $(BOOST_INCL) $(SIM_INCL)
LIBS := $(PMPL_LIB) $(UTILS_LIB) $(STAPL_LIB) $(BOOST_LIB) $(SIM_LIBS)
DEPS = -MMD -MF $*.d

# Object File Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

$(PMPL_OBJS): $(UTILS_LIBFILE) $(STAPL_LIBFILE)

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
pmpl: $(PMPL_LIBFILE) $(MAIN)
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(MAIN) $(LIBS) -o $@

# Build the simulator.
sim: $(PMPL_LIBFILE) $(SIM_LIBFILE) $(SIM_OBJS)
	@echo Linking $@...
	@$(CXX) $(CXXFLAGS) $(OPTS) $(SIM_OBJS) $(LIBS) -o $@

# Object File Recipes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

%.o: %.cpp
	@$(MAKE) -s check_platform
	@echo Compiling $@...
	@$(CXX) -c $(CXXFLAGS) $(OPTS) $(DEFS) $(INCL) $(DEPS) $< -o $@
	@cat $*.d >> Dependencies
	@rm -f $*.d

# Cleanup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

.PHONY: clean
clean:
	@echo Cleaning pmpl library and executables...
	@rm -f $(PMPL_LIBFILE) $(MAIN) pmpl $(SIM_OBJS) sim

.PHONY: reallyclean
reallyclean: clean
	@echo Cleaning all pmpl objects...
	@rm -f $(PMPL_OBJS) *.d
	@rm -f Dependencies

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

# Other Stuff ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# include automatically generated dependencies
-include Dependencies

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
