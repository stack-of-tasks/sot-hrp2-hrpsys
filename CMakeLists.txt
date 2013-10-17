# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-openhrp.
# sot-openhrp is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-openhrp is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-openhrp. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/openhrp.cmake)

SET(PROJECT_NAME sot-hrp2-hrpsys)
SET(PROJECT_DESCRIPTION "OpenHRP plugin implementing the Stack of Tasks for HRP-2.")
SET(PROJECT_URL "https://softs.laas.fr/git/jrl/robots/sot-hrp2-hrpsys")

SET(CUSTOM_HEADER_DIR "dynamic_graph/sot/hrp2-hrpsys")

# Define the location of the CMakeModules directory.
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMakeModules)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

# Remove pedantic flag as Corba stubs won't compile with it.
STRING(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

# Search for Boost.
SET(BOOST_COMPONENTS thread)
SEARCH_FOR_BOOST()

# Search for openhrp and its robots
SEARCH_GRX3()
SEARCH_GRX_ROBOTS("HRP2JRL;HRP2LAAS")

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.1")
ADD_REQUIRED_DEPENDENCY("sot-core")
ADD_SUBDIRECTORY(src)

ADD_SUBDIRECTORY(script)
ADD_SUBDIRECTORY(project)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
