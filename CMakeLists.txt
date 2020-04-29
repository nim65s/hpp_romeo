#
# Copyright (c) 2014, 2020 CNRS
# Authors: Renaud Viry, Guilhem Saurel
#
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp_romeo)
SET(PROJECT_DESCRIPTION "Python and ros launch files for Romeo robot in hpp.")

SET(PROJECT_USE_CMAKE_EXPORT TRUE)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

ADD_PROJECT_DEPENDENCY("hpp-corbaserver" REQUIRED)
ADD_PROJECT_DEPENDENCY("example-robot-data" REQUIRED)

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

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
  )

ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
