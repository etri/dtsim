#
# Makefile for City
#
# $Revision: 1023 $
# $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
#

DTSIM_VERSION=`cat ../../version`
REPAST_HPC_VERSION = 2.3.0

# (DTSim) modify this path if you installed SWs to other directory
BASE = /opt/twin

ENABLE_SHARED = @ENABLE_SHARED@
ENABLE_STATIC = @ENABLE_STATIC@

CPPFLAGS = -I../../include
CPPFLAGS += -I../../include/city
CPPFLAGS += -I$(BASE)/Boost/Boost_1.61/include
CPPFLAGS += -I$(BASE)/CURL/include
CPPFLAGS += -I$(BASE)/NetCDF/include
CPPFLAGS += -I$(BASE)/NetCDF-cxx/include
CPPFLAGS += -I$(BASE)/GEOS/include
CPPFLAGS += -I$(BASE)/Shapelib/include

LDFLAGS = -L../../lib
LDFLAGS += -L$(BASE)/Boost/Boost_1.61/lib
LDFLAGS += -L$(BASE)/CURL/lib
LDFLAGS += -L$(BASE)/NetCDF/lib
LDFLAGS += -L$(BASE)/NetCDF-cxx/lib
LDFLAGS += -L$(BASE)/GEOS/lib
LDFLAGS += -L$(BASE)/Shapelib/lib

CXX = mpicxx
CXXLD = mpicxx

CXXFLAGS = -g -Wall -O2 -std=c++11 -MMD -MP -Wno-reorder -Wno-unused-variable -Wno-sign-compare
CXXFLAGS += -DUSE_UNSTABLE_GEOS_CPP_API
CXXFLAGS += $(CPPFLAGS)

LIBS = -lboost_mpi-mt -lboost_serialization-mt -lboost_filesystem-mt -lboost_system-mt
LIBS += -lnetcdf -lnetcdf_c++
LIBS += -lgeos
LIBS += -lshp
LIBS += -lrepast_hpc-$(REPAST_HPC_VERSION)

REPAST_HPC_LIB_DIR = ../../lib
BOOST_LIB_DIR = $(BASE)/Boost/Boost_1.61/lib
CURL_LIB_DIR = $(BASE)/CURL/lib
NETCDF_LIB_DIR = $(BASE)/NetCDF/lib
NETCDF_CXX_LIB_DIR = $(BASE)/NetCDF-cxx/lib
GEOS_LIB_DIR = $(BASE)/GEOS/lib
SHAPELIB_LIB_DIR = $(BASE)/Shapelib/lib

AR      = ar
ARFLAGS = cr
RANLIB  = ranlib

RPATHS :=
ifneq ($(REPAST_HPC_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(REPAST_HPC_LIB_DIR)
endif

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

ifneq ($(SHAPELIB_LIB_DIR),)
	RPATHS += -Wl,-rpath -Wl,$(SHAPELIB_LIB_DIR)
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
INSTALL_CITY_INCLUDE_DIR = $(INSTALL_INCLUDE_DIR)/city
INSTALL_LIB_DIR = $(INSTALL_DIR)/lib

CITY_LIB := city
CITY_SO = ../../lib/lib$(CITY_LIB)-$(REPAST_HPC_VERSION).$(SO_SUFFIX)
CITY_A = ../../lib/lib$(CITY_LIB)-$(REPAST_HPC_VERSION).a
CITY_HEADERS = ../../include/city/*.h
CITY_DEPS = ./*.d

CITY_SRC :=
include ./module.mk

# OBJECT FILES
CITY_OBJECTS=$(patsubst %.cpp,%.o,$(CITY_SRC))

-include $(CITY_OBJECTS:.o=.d)

.PHONY: all clean install

.DEFAULT_GOAL := all

all: city

install: city
	$(E) mkdir -pv $(INSTALL_CITY_INCLUDE_DIR)
	$(Q) "Install City include to $(INSTALL_CITY_INCLUDE_DIR)"
	$(E) cp $(CP_ARGS) $(CITY_HEADERS) $(INSTALL_CITY_INCLUDE_DIR)
	$(Q) ""
	$(Q) "Install City library to $(INSTALL_LIB_DIR)"
	$(E) cp $(CP_ARGS) $(CITY_SO) $(INSTALL_LIB_DIR)
	$(E) cp $(CP_ARGS) $(CITY_A) $(INSTALL_LIB_DIR)

city: $(CITY_SO) $(CITY_A)

$(CITY_SO): $(CITY_OBJECTS)
	$(CXXLD) $(LDFLAGS) $(CITY_OBJECTS) -o $(CITY_SO) $(LIBS) $(RPATHS)

$(CITY_A): $(CITY_OBJECTS)
	$(Q) "AR	$(@)"
	$(E) $(AR) $(ARFLAGS) $(@) $(CITY_OBJECTS)
	$(E) $(RANLIB) $(@)

%.o : %.cpp
	$(Q) "CC	$(@)"
	$(E) $(CXX) $(CXXFLAGS) -Wno-deprecated-declarations $(LIB_CXX_FLAGS) -c $< -o $@

clean:
	@echo "Clean Objects, Deps"
	@rm -fv $(CITY_OBJECTS)
	@rm -fv $(CITY_DEPS)

clean-all:
	@echo "Clean Objects, Deps, libraries"
	@rm -fv $(CITY_OBJECTS)
	@rm -fv $(CITY_SO)
	@rm -fv $(CITY_A)
	@rm -fv $(CITY_DEPS)

clear:
	@echo "Clean Objects, Deps, libraries"
	@rm -fv $(CITY_OBJECTS)
	@rm -fv $(CITY_SO)
	@rm -fv $(CITY_A)
	@rm -fv $(CITY_DEPS)
	@echo "Clean headers, libraries from install dir"
	@rm -fv $(INSTALL_LIB_DIR)/lib$(CITY_LIB)-$(REPAST_HPC_VERSION).$(SO_SUFFIX)
	@rm -fv $(INSTALL_LIB_DIR)/lib$(CITY_LIB)-$(REPAST_HPC_VERSION).a
	@rm -fv $(INSTALL_CITY_INCLUDE_DIR)/*.h
