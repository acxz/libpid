cmake_minimum_required(VERSION 3.3)

project(libpid VERSION 0.0.1 LANGUAGES CXX)

# Dependencies
## TODO pybind11

# Build
## add include directory
include_directories(include)

## build examples
add_subdirectory(examples)

# Install
## for correct library locations across platforms
include(GNUInstallDirs)

## install the header files
install(DIRECTORY include/libpid DESTINATION include)

## copy the cmake files required for find_package()
install(FILES cmake/libpidConfig.cmake DESTINATION share/libpid/cmake)

### install library and targets
#install(
#    # Below is useful if there are pre specified templates like a double
#    # template or a float template
#    #TARGETS libpid ${PRESPEC_LIB_NAMES}
#    EXPORT libpid_export
#    #ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    #LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    #RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    DESTINATION ${CMAKE_INSTALL_LIBDIR}
#    )
#
### Create the libpid.cmake file which holds target includes and dependencies
#install(EXPORT libpid_export DESTINATION share/libpid/cmake)

# Uninstall
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
