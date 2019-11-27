#
# Copyright (c) 2014 CNRS
# Authors: Renaud Viry
#
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAME hpp_romeo)
SET(PROJECT_DESCRIPTION "Python and ros launch files for Romeo robot in hpp.")

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

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
