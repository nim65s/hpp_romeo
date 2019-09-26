#
# Copyright (c) 2014 CNRS
# Authors: Renaud Viry
#
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/python.cmake)

FINDPYTHON()

SET(PROJECT_NAME hpp_romeo)
SET(PROJECT_DESCRIPTION
  "Python and ros launch files for Romeo robot in hpp."
  )

SETUP_HPP_PROJECT()

ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("romeo_description")

INSTALL (FILES
  src/hpp/corbaserver/romeo/robot.py
  src/hpp/corbaserver/romeo/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/romeo
  )

INSTALL (FILES
  src/hpp/corbaserver/manipulation/romeo/robot.py
  src/hpp/corbaserver/manipulation/romeo/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/romeo
  )

SETUP_HPP_PROJECT_FINALIZE()
