# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux
#
# This file is part of kpp-interfacestep.
# kpp-interfacestep is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# kpp-interfacestep is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# kpp-interfacestep.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)
SET(CUSTOM_HEADER_DIR "kpp/interfacestep")

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/kineo.cmake)

SET(PROJECT_NAME kpp-interfacestep)
SET(PROJECT_DESCRIPTION
  "Kineo Path Planner add-on for hpp-wholebody-step-planner"
  )
SET(PROJECT_URL "http://github.com/laas/kpp-interfacestep")

SET(${PROJECT_NAME}_HEADERS
  include/kpp/interfacestep/fwd.hh
  include/kpp/interfacestep/kppinterface-nowarning.hh
  include/kpp/interfacestep/interface.hh
  include/kpp/interfacestep/command-setinitgoal.hh
  include/kpp/interfacestep/command-dynamicpath.hh
  include/kpp/interfacestep/command-removeconstraints.hh
  include/kpp/interfacestep/command-grabobject.hh
)

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("hpp-util >= 0.7")
ADD_REQUIRED_DEPENDENCY("hpp-wholebody-step-planner >= 0.2")
ADD_REQUIRED_DEPENDENCY("hpp-wholebody-step-corba >= 0.1")
ADD_REQUIRED_DEPENDENCY("kpp-interface >= 2.1")

# Reinitialize directory where libs are installed since for Kineo addon, this
# directory is non standard: lib/modules/${UNAME_N}
SET(PKG_CONFIG_LIBS "")

SET(KINEO_ADDON_SOURCES
  src/interface.cc
  src/command-setinitgoal.cc
  src/command-dynamicpath.cc
  src/command-removeconstraints.cc
  src/command-grabobject.cc
)

KINEO_ADDON(${PROJECT_NAME})

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-wholebody-step-planner)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-wholebody-step-corba)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} kpp-interface)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
