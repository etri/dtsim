DTSIM_VERSION=`cat ../../../version`
REPAST_HPC_VERSION = 2.3.0

# (DTSim) modify this path if you installed SWs to other directory
BASE = /opt/twin

ENABLE_SHARED = @ENABLE_SHARED@
ENABLE_STATIC = @ENABLE_STATIC@

CPPFLAGS = -I.
CPPFLAGS += -I$(BASE)/Boost/Boost_1.61/include
CPPFLAGS += -I$(BASE)/CURL/include
CPPFLAGS += -I$(BASE)/GEOS/include
CPPFLAGS += -I$(BASE)/NetCDF/include
CPPFLAGS += -I$(BASE)/NetCDF-cxx/include
CPPFLAGS += -I$(BASE)/Shapelib/include
CPPFLAGS += -I$(BASE)/dtsim/${DTSIM_VERSION}/include

LDFLAGS = -L$(BASE)/Boost/Boost_1.61/lib
LDFLAGS += -L$(BASE)/CURL/lib
LDFLAGS += -L$(BASE)/GEOS/lib
LDFLAGS += -L$(BASE)/NetCDF/lib
LDFLAGS += -L$(BASE)/NetCDF-cxx/lib
LDFLAGS += -L$(BASE)/Shapelib/lib
LDFLAGS += -L$(BASE)/dtsim/${DTSIM_VERSION}/lib

CXX = mpicxx
CXXLD = mpicxx

CXXFLAGS = -g -Wall -O2 -std=c++11 -MMD -MP -Wno-reorder -Wno-unused-variable -Wno-sign-compare
CXXFLAGS += $(CPPFLAGS)
CXXFLAGS += -DUSE_UNSTABLE_GEOS_CPP_API

LIBS = -lboost_mpi-mt -lboost_serialization-mt -lboost_filesystem-mt -lboost_system-mt
LIBS += -lcurl
LIBS += -lnetcdf -lnetcdf_c++
LIBS += -lgeos
LIBS += -lshp
LIBS += -lrepast_hpc-$(REPAST_HPC_VERSION)
LIBS += -lcity-$(REPAST_HPC_VERSION)

REPAST_HPC_LIB_DIR = $(BASE)/dtsim/${DTSIM_VERSION}/lib
NETCDF_LIB_DIR = $(BASE)/NetCDF/lib
NETCDF_CXX_LIB_DIR = $(BASE)/NetCDF-cxx/lib
BOOST_LIB_DIR = $(BASE)/Boost/Boost_1.61/lib
GEOS_LIB_DIR = $(BASE)/GEOS/lib
SHAPELIB_LIB_DIR = $(BASE)/Shapelib/lib

INSTALL_DIR = $(BASE)/dtsim/${DTSIM_VERSION}
INSTALL_BIN_DIR = $(INSTALL_DIR)/bin
INSTALL_MODEL_DIR = $(INSTALL_BIN_DIR)/examples/exam04

RPATHS :=

ifneq ($(REPAST_HPC_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(REPAST_HPC_LIB_DIR)
endif

ifneq ($(NETCDF_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(NETCDF_LIB_DIR)
endif

ifneq ($(NETCDF_CXX_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(NETCDF_CXX_LIB_DIR)
endif

ifneq ($(BOOST_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(BOOST_LIB_DIR)
endif

ifneq ($(GEOS_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(GEOS_LIB_DIR)
endif

ifneq ($(SHAPELIB_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(SHAPELIB_LIB_DIR)
endif

USE_MAC = no
SO_SUFFIX=so
ifeq ($(USE_MAC),no)
	LIB_CXX_FLAGS += -fPIC
	LIB_LD_FLAGS += -shared
else
	LIB_LD_FLAGS += -dynamiclib
	SO_SUFFIX=dylib
endif

HAVE_CP_U = yes
ifeq ($(HAVE_CP_U),yes)
	CP_ARGS = -uvf
else
	CP_ARGS = -vf
endif

ifeq ($(V),)
	# Prints a short description of the action, does not show command
	Q=@echo
	E=@
else
ifeq ($(V),1)
	# Echoes the entire command
	Q=@echo >/dev/null
	E=
else # V=2
	# Echoes the description and the command
	Q=@echo ; echo
	E=
endif
endif

# (DTSim) binary release directory
EXAMPLE_EXEC = ./exam04
EXAMPLE_DEPS = ./*.d
EXAMPLE_PARAMS = ./model.props

EXAMPLE_SRC :=
include ./module.mk

# OBJECT FILES
EXAMPLE_OBJECTS=$(patsubst %.cpp,%.o,$(EXAMPLE_SRC))

-include $(EXAMPLE_OBJECTS:.o=.d)

.PHONY: all clean

.DEFAULT_GOAL := all

all: exam04 install

exam04: $(EXAMPLE_OBJECTS)
	$(CXXLD) $(LDFLAGS) $(EXAMPLE_OBJECTS) -o $(EXAMPLE_EXEC) $(LIBS) $(RPATHS)

%.o : %.cpp
	$(Q) "CC	$(@)"
	$(E) $(CXX) $(CXXFLAGS) -Wno-deprecated-declarations $(LIB_CXX_FLAGS) -c $< -o $@

install: exam04
	$(Q) "Install exam04 to $(INSTALL_MODEL_DIR)"
	@mkdir -p $(INSTALL_MODEL_DIR)/logs
	@mkdir -p $(INSTALL_MODEL_DIR)/results
	$(E) cp $(CP_ARGS) $(EXAMPLE_EXEC) $(INSTALL_MODEL_DIR)
	$(E) cp $(CP_ARGS) $(EXAMPLE_PARAMS) $(INSTALL_MODEL_DIR)

clean: clean-all
clean-all:
	@echo "Clean Objects, Deps, Executions"
	@rm -rf logs
	@rm -fv core.* 
	@rm -fv $(EXAMPLE_OBJECTS)
	@rm -fv $(EXAMPLE_DEPS)
	@rm -fv $(EXAMPLE_EXEC)

clear:
	@echo "Clean Objects, Deps, Executions"
	@rm -rf logs
	@rm -fv core.* 
	@rm -fv $(EXAMPLE_OBJECTS)
	@rm -fv $(EXAMPLE_DEPS)
	@rm -fv $(EXAMPLE_EXEC)
	@echo "Clean Executions from install dir"
	@rm -fv $(INSTALL_MODEL_DIR)/$(EXAMPLE_EXEC)
