#
# Copyright (c) 2014, 2020 CNRS Authors: Renaud Viry, Guilhem Saurel
#

cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME hpp_romeo)
set(PROJECT_DESCRIPTION "Python and ros launch files for Romeo robot in hpp.")

set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/hpp.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

findpython()

add_project_dependency("hpp-corbaserver" REQUIRED)
add_project_dependency("example-robot-data" REQUIRED)

install(FILES src/hpp/corbaserver/romeo/robot.py
              src/hpp/corbaserver/romeo/__init__.py
        DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/romeo)

install(FILES src/hpp/corbaserver/manipulation/romeo/robot.py
              src/hpp/corbaserver/manipulation/romeo/__init__.py
        DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/romeo)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
