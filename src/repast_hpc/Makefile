DTSIM_VERSION=`cat ../../version`
REPASTHPC_VERSION = 2.3.0

# (DTSim) modify this path if you installed SWs to other directory
BASE = /opt/twin

ENABLE_SHARED = @ENABLE_SHARED@
ENABLE_STATIC = @ENABLE_STATIC@

CPPFLAGS = -I../../include/repast_hpc
CPPFLAGS += -I$(BASE)/Boost/Boost_1.61/include
CPPFLAGS += -I$(BASE)/CURL/include
CPPFLAGS += -I$(BASE)/NetCDF/include
CPPFLAGS += -I$(BASE)/NetCDF-cxx/include
CPPFLAGS += -I$(BASE)/GEOS/include

LDFLAGS = -L$(BASE)/Boost/Boost_1.61/lib
LDFLAGS += -L$(BASE)/CURL/lib
LDFLAGS += -L$(BASE)/NetCDF/lib
LDFLAGS += -L$(BASE)/NetCDF-cxx/lib
LDFLAGS += -L$(BASE)/GEOS/lib

CXX = mpicxx
CXXLD = mpicxx

CXXFLAGS = -g -Wall -O2 -std=c++11 -MMD -MP -Wno-reorder -Wno-unused-variable -Wno-sign-compare
CXXFLAGS += -DUSE_UNSTABLE_GEOS_CPP_API
CXXFLAGS += $(CPPFLAGS)

LIBS = -lboost_mpi-mt -lboost_serialization-mt -lboost_filesystem-mt -lboost_system-mt
LIBS += -lnetcdf -lnetcdf_c++
LIBS += -lgeos

BOOST_LIB_DIR = $(BASE)/Boost/Boost_1.61/lib
CURL_LIB_DIR = $(BASE)/CURL/lib
NETCDF_LIB_DIR = $(BASE)/NetCDF/lib
NETCDF_CXX_LIB_DIR = $(BASE)/NetCDF-cxx/lib
GEOS_LIB_DIR = $(BASE)/GEOS/lib

AR      = ar
ARFLAGS = cr
RANLIB  = ranlib

RPATHS :=
ifneq ($(BOOST_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(BOOST_LIB_DIR)
endif

ifneq ($(CURL_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(CURL_LIB_DIR)
endif

ifneq ($(NETCDF_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(NETCDF_LIB_DIR)
endif

ifneq ($(NETCDF_CXX_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(NETCDF_CXX_LIB_DIR)
endif

ifneq ($(GEOS_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(GEOS_LIB_DIR)
endif

USE_MAC = no
SO_SUFFIX=so
ifeq ($(USE_MAC),no)
	LIB_CXX_FLAGS += -fPIC
 	LDFLAGS += -shared
else
	LDFLAGS += -dynamiclib
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
INSTALL_DIR = $(BASE)/dtsim/${DTSIM_VERSION}
INSTALL_BIN_DIR = $(INSTALL_DIR)/bin
INSTALL_INCLUDE_DIR = $(INSTALL_DIR)/include
INSTALL_REPASTHPC_INCLUDE_DIR = $(INSTALL_INCLUDE_DIR)/repast_hpc
INSTALL_LIB_DIR = $(INSTALL_DIR)/lib

REPASTHPC_LIB := repast_hpc
REPASTHPC_SO = ../../lib/lib$(REPASTHPC_LIB)-$(REPASTHPC_VERSION).$(SO_SUFFIX)
REPASTHPC_A = ../../lib/lib$(REPASTHPC_LIB)-$(REPASTHPC_VERSION).a
REPASTHPC_HEADERS = ../../include/repast_hpc/*.h
REPASTHPC_DEPS = ./*.d

REPASTHPC_MAC_NAME=
ifeq ($(USE_MAC),yes)
	REPASTHPC_MAC_NAME= -install_name "$(INSTALL_DIR)/$(REPASTHPC_SO)"
endif

REPASTHPC_SRC :=
include ./module.mk

# OBJECT FILES
REPASTHPC_OBJECTS=$(patsubst %.cpp,%.o,$(REPASTHPC_SRC))

-include $(REPASTHPC_OBJECTS:.o=.d)

.PHONY: all clean install

.DEFAULT_GOAL := all

all: repast_hpc

DIRS = $(INSTALL_DIR) $(INSTALL_LIB_DIR) $(INSTALL_INCLUDE_DIR) \
	   $(INSTALL_BIN_DIR) $(INSTALL_REPASTHPC_INCLUDE_DIR)

install: repast_hpc
	$(E) mkdir -pv $(DIRS)
	$(Q) "Install Repast-HPC include to $(INSTALL_REPASTHPC_INCLUDE_DIR)"
	$(E) cp $(CP_ARGS) $(REPASTHPC_HEADERS) $(INSTALL_REPASTHPC_INCLUDE_DIR)
	$(Q) ""
	$(Q) "Install Repast-HPC library to $(INSTALL_LIB_DIR)"
	$(E) cp $(CP_ARGS) $(REPASTHPC_SO) $(INSTALL_LIB_DIR)
	$(E) cp $(CP_ARGS) $(REPASTHPC_A) $(INSTALL_LIB_DIR)

repast_hpc: $(REPASTHPC_SO) $(REPASTHPC_A)

$(REPASTHPC_SO): $(REPASTHPC_OBJECTS)
	$(CXXLD) $(REPASTHPC_MAC_NAME) $(LDFLAGS) $(REPASTHPC_OBJECTS) -o $(REPASTHPC_SO) $(LIBS) $(RPATHS)

$(REPASTHPC_A): $(REPASTHPC_OBJECTS)
	$(Q) "AR	$(@)"
	$(E) $(AR) $(ARFLAGS) $(@) $(REPASTHPC_OBJECTS)
	$(E) $(RANLIB) $(@)

%.o : %.cpp
	$(Q) "CC	$(@)"
	$(E) $(CXX) $(CXXFLAGS) -Wno-deprecated-declarations $(LIB_CXX_FLAGS) -c $< -o $@

clean:
	@echo "Clean Objects, Deps"
	@rm -fv $(REPASTHPC_OBJECTS)
	@rm -fv $(REPASTHPC_DEPS)

clean-all:
	@echo "Clean Objects, Deps, libraries"
	@rm -fv $(REPASTHPC_OBJECTS)
	@rm -fv $(REPASTHPC_SO)
	@rm -fv $(REPASTHPC_A)
	@rm -fv $(REPASTHPC_DEPS)

clear:
	@echo "Clean Objects, Deps, libraries"
	@rm -fv $(REPASTHPC_OBJECTS)
	@rm -fv $(REPASTHPC_SO)
	@rm -fv $(REPASTHPC_A)
	@rm -fv $(REPASTHPC_DEPS)
	@echo "Clean headers, libraries from install dir"
	@rm -fv $(INSTALL_LIB_DIR)/lib$(REPASTHPC_LIB)-$(REPASTHPC_VERSION).$(SO_SUFFIX)
	@rm -fv $(INSTALL_LIB_DIR)/lib$(REPASTHPC_LIB)-$(REPASTHPC_VERSION).a
	@rm -fv $(INSTALL_REPASTHPC_INCLUDE_DIR)/*.h
