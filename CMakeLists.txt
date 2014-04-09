# Copyright 2014, Mehdi Benallegue, Nicolas Mansard, LAAS-CNRS
#
# This file is part of sot-stabilizer.
# sot-dyninv is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-dyninv is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-dyninv. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME sot-stabilizer)
SET(PROJECT_DESCRIPTION "Robot's stabilizer")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()


SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("state-observation")
ADD_REQUIRED_DEPENDENCY("sot-state-observation")

SET(${PROJECT_NAME}_HEADERS
    include/sot-stabilizer/hrp2-decoupled-stabilizer.hh
)


# Add subdirectories.
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unit-testing)

SETUP_PROJECT_FINALIZE()
