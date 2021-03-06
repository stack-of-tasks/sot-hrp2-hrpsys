# Copyright 2010, 2011, François Bleibel, Thomas Moulard Olivier
# Stasse, JRL, CNRS/AIST
#
# This file is part of sot-hrp2-hrpsys.
# sot-hrp2-hrpsys is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-hrp2-hrpsys is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-hrp2-hrpsys. If not, see <http://www.gnu.org/licenses/>.

include(../cmake/python.cmake)
include(../cmake/idl.cmake)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

# Add OpenHRP and robot headers to include directory.
INCLUDE_DIRECTORIES(
  ${GRX_PREFIX}/OpenHRP/Controller/corba
  ${GRX_PREFIX}/OpenHRP/Common
  ${GRX_PREFIX}/OpenHRP/DynamicsSimulator/server
  ${GRX_PREFIX}/include
  ${ROBOT_PATH}
  )

# Create and install python module
GENERATE_IDL_FILE(OpenHRPCommon ${GRX_PREFIX}/OpenHRP/Common/corba)
GENERATE_IDL_FILE(HRPcontroller ${GRX_PREFIX}/include/idl)
GENERATE_IDL_FILE(ViewSimulator ${GRX_PREFIX}/include/idl)

# OpenHRP generates stubs headers with '.h' suffix but they are
# generated with '.hh' suffix. Provide an alternative version to
# please OpenHRP headers.
ADD_CUSTOM_COMMAND(
  OUTPUT OpenHRPCommon.h
  COMMAND cp
  ARGS OpenHRPCommon.hh OpenHRPCommon.h
  MAIN_DEPENDENCY OpenHRPCommon.hh
  )
ADD_CUSTOM_COMMAND(
  OUTPUT HRPcontroller.h
  COMMAND cp
  ARGS HRPcontroller.hh HRPcontroller.h
  MAIN_DEPENDENCY HRPcontroller.hh
  )

# Declare the shared library containing the OpenHRP specific entity.
ADD_LIBRARY(openhrp
  SHARED
  OpenHRPCommon.hh
  OpenHRPCommonSK.cc
  HRPcontroller.hh
  HRPcontrollerSK.cc
  ViewSimulator.hh
  ViewSimulatorSK.cc
  )

SET_TARGET_PROPERTIES(
  openhrp PROPERTIES SOVERSION ${PROJECT_VERSION})

INSTALL(TARGETS openhrp DESTINATION lib)

# Declare library dependencies.
PKG_CONFIG_USE_DEPENDENCY(openhrp "omniORB4")

# Install the sample openhrp shell
INSTALL(FILES
  openhrp-shell
  DESTINATION bin
  PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE
  )


# Compile the OpenHRP plug-in.
ADD_LIBRARY(StackOfTasks SHARED plugin.cc)

# Remove 'lib' prefix in library filename.
SET_TARGET_PROPERTIES(StackOfTasks PROPERTIES PREFIX ""
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Declare plug-in dependencies.
TARGET_LINK_LIBRARIES(StackOfTasks openhrp)
PKG_CONFIG_USE_COMPILE_DEPENDENCY(StackOfTasks "sot-core")

INSTALL(TARGETS StackOfTasks DESTINATION "${ROBOT_PATH}/bin")


LIST(APPEND LOGGING_WATCHED_TARGETS openhrp StackOfTasks)
