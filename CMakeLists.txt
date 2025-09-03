cmake_minimum_required(VERSION 3.16.3)
project(reachy_mini_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package (Eigen3 3 REQUIRED NO_MODULE)
find_package(eigenpy REQUIRED)

if (NOT PYTHON_SITELIB)
set(PYTHON_SITELIB "lib")
endif()

if ("${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" STREQUAL "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
endif()

set(CMAKE_SHARED_MODULE_PREFIX "")

find_package(Python3 REQUIRED)
find_package(Python COMPONENTS Interpreter)

set(SOURCES
    src/kinematics.cpp
)

add_library(libreachy_mini_kinematics SHARED ${SOURCES})

target_link_libraries(libreachy_mini_kinematics PUBLIC
    eigenpy::eigenpy
    Eigen3::Eigen
)

target_include_directories(libreachy_mini_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

set(MODULE_SOURCES
    bindings/module.cpp

    bindings/expose_eigen.cpp
    bindings/expose_kinematics.cpp
)

add_library(reachy_mini_kinematics MODULE ${MODULE_SOURCES})
target_compile_definitions(reachy_mini_kinematics PUBLIC -DBOOST_DISABLE_PRAGMA_MESSAGE)

set_target_properties(reachy_mini_kinematics PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB})
target_link_libraries(reachy_mini_kinematics ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} libreachy_mini_kinematics)
target_include_directories(reachy_mini_kinematics PRIVATE ${Python3_INCLUDE_DIRS})

add_custom_command(
    TARGET reachy_mini_kinematics POST_BUILD
    COMMAND doxystub
        --module reachy_mini_kinematics
        --doxygen_directory "${CMAKE_CURRENT_SOURCE_DIR}"
        --output "${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/reachy_mini_kinematics.pyi"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}"
    COMMENT "Generating stubs..."
)

if (APPLE)
set_target_properties(reachy_mini_kinematics PROPERTIES INSTALL_RPATH "@loader_path/../..")
set_target_properties(libreachy_mini_kinematics PROPERTIES INSTALL_RPATH "@loader_path")
else()
set_target_properties(reachy_mini_kinematics PROPERTIES INSTALL_RPATH "\$ORIGIN/../..")
set_target_properties(libreachy_mini_kinematics PROPERTIES INSTALL_RPATH "\$ORIGIN")
endif()

install(TARGETS libreachy_mini_kinematics DESTINATION lib)
install(TARGETS reachy_mini_kinematics DESTINATION ${PYTHON_SITELIB})
install(FILES ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/reachy_mini_kinematics.pyi DESTINATION ${PYTHON_SITELIB})

