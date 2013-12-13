# Copyright (C) 2011 Thomas Moulard, JRL, CNRS/AIST.
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

FOREACH(ROBOT ${GRX_ROBOTS})
  SET(CMAKE_TEMPORARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/${ROBOT})
  FILE(MAKE_DIRECTORY ${CMAKE_TEMPORARY_DIR})
  CONFIGURE_FILE(sot.xml.in      ${CMAKE_TEMPORARY_DIR}/sot.xml      @ONLY)
  CONFIGURE_FILE(sot-bush.xml.in ${CMAKE_TEMPORARY_DIR}/sot-bush.xml @ONLY)

  INSTALL(
    FILES
    ${CMAKE_TEMPORARY_DIR}/sot.xml
    ${CMAKE_TEMPORARY_DIR}/sot-bush.xml
    DESTINATION ${GRX_PREFIX}/${ROBOT}/project
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
  )
ENDFOREACH(ROBOT)
