cmake_minimum_required(VERSION 3.25)
project(ydlidar_sdk C CXX)

#########################################################
# version
set(YDLIDAR_SDK_VERSION_MAJOR 1)
set(YDLIDAR_SDK_VERSION_MINOR 2)
set(YDLIDAR_SDK_VERSION_PATCH 19)
set(YDLIDAR_SDK_VERSION ${YDLIDAR_SDK_VERSION_MAJOR}.${YDLIDAR_SDK_VERSION_MINOR}.${YDLIDAR_SDK_VERSION_PATCH})

##########################################################
# Detect wordsize:
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MRPT_WORD_SIZE 64)
ELSE()
	SET(CMAKE_MRPT_WORD_SIZE 32)
ENDIF()
##################################################

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG        12.0.0)

FetchContent_MakeAvailable(fmt)

##########################################################
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "Current platform: Linux")
	#Linux add -fPIC
	add_compile_options(-fPIC)
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "Current platform: Windows")
	MESSAGE(STATUS "Current compiler: ${CMAKE_CXX_COMPILER_ID}")
	IF (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	#MSVC compiler
	add_compile_options("/utf-8")
	add_definitions(-DWIN32_LEAN_AND_MEAN)
    add_definitions(-DNOMINMAX)
	ELSEIF (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	#MinGW compiler
	ENDIF (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")
#or
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Debug builds: enable sanitizers for critical runtime checks
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-ggdb -fsanitize=leak -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=leak -fsanitize=address -fsanitize=undefined)
endif()

#####################################################
# add cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#############################################################################
# include cmake file
include(common/ydlidar_base)
include(install_package)

#############################################################################
# Policy CMP0023 allows to mix old and new interfaces of target_link_libraries
#
# The upstream SDK also pinned CMP0053/CMP0037/CMP0043 to OLD. All three were dropped:
# CMake deprecated the OLD behaviour, and none of them is actually needed here —
# no target is named "test"/"all"/"install" (CMP0037) and no COMPILE_DEFINITIONS_<Config>
# property is ever set (CMP0043), so both were pure noise. CMP0053 only changes how
# exotic ${} escapes are parsed, and the SDK has none.
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW) # We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0005 NEW) # Escape definitions (-D) strings
endif()

#############################################################################
# option
option( BUILD_SHARED_LIBS "Build shared libraries." OFF)
option( BUILD_EXAMPLES "Build Example." ON)
option( BUILD_CSHARP "Build CSharp." OFF)
option( BUILD_TEST "Build Test." OFF)

############################################################################
# find package
find_package(SWIG)
find_package(GTest)

# FindPythonInterp / FindPythonLibs were removed from CMake (policy CMP0148), so the
# modern FindPython3 module is used instead. The python/ and csharp/ subprojects and
# the summary script still read the legacy variable names, so those are mapped over.
find_package(Python3 QUIET COMPONENTS Interpreter Development)

set(PYTHONLIBS_FOUND FALSE)

if(Python3_Development_FOUND)
	set(PYTHONLIBS_FOUND TRUE)
	set(PYTHON_INCLUDE_PATH ${Python3_INCLUDE_DIRS})
	set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
	set(PYTHON_VERSION_STRING ${Python3_VERSION})
endif()

if(Python3_Interpreter_FOUND)
	set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
endif()

############################################################################
# include headers
include_directories(.)
include_directories(core)
include_directories(src)

########################################################
## Create configure file for inclusion in library
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ydlidar_config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ydlidar_config.h" )
set( GENERATED_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/ydlidar_config.h )

# Include binary directory for generated headers (MUST be before add_subdirectory calls)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#############################################################################
# addd subdirectory
add_subdirectory(core)
add_subdirectory(src)

##############################
#build examples
if(BUILD_EXAMPLES)
add_subdirectory(examples)
endif()

#############################################################################
# PARSE libraries
include(common/ydlidar_parse)
include_directories(${SDK_INCS})


#############################################################################
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#############################################################################################
#shared library
if(BUILD_SHARED_LIBS)
ydlidar_add_library(${PROJECT_NAME} SHARED ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
else()
ydlidar_add_library(${PROJECT_NAME} STATIC ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${SDK_LIBS} fmt::fmt)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

# Add install command for the library with EXPORT
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

if(TARGET fmt)
    # fmt is a FetchContent dependency linked PRIVATE into the SDK, so its headers are
    # not part of our public API and were never installed. Clearing PUBLIC_HEADER makes
    # that explicit and silences the "no PUBLIC_HEADER DESTINATION" warning — declaring
    # a destination instead would make install() chase paths relative to fmt's own
    # source tree and fail.
    set_target_properties(fmt PROPERTIES PUBLIC_HEADER "")

    install(TARGETS fmt
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
endif()

#############################################################################################
#static library
#ydlidar_add_library(${PROJECT_NAME}_static STATIC ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
#SET_TARGET_PROPERTIES(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
#SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
#SET_TARGET_PROPERTIES(${PROJECT_NAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

#target_link_libraries(${PROJECT_NAME}_static ${SDK_LIBS})
#target_include_directories(${PROJECT_NAME}_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)


############################################################################
# enable test
enable_testing()

############################################################################
# build ydlidar sdk python version
# Undereferenced form: ${VAR} expands to nothing when a find_package fails, which would
# turn this into the syntax error "if( AND )".
if(SWIG_FOUND AND PYTHONLIBS_FOUND)
    message(STATUS "build python API....")
    include_directories(python)
    add_subdirectory(python)
endif()

##############################################
# build ydlidar sdk c#
if(WIN32)
if(BUILD_CSHARP)
    message(STATUS "build csharp API....")
    include_directories(csharp)
    add_subdirectory(csharp)
endif()
endif()

###############################################################################
# test
if(GTEST_FOUND AND BUILD_TEST)
    message(STATUS "build test is ON.....")
    include_directories(test)
    add_subdirectory(test)
endif()

###############################################################################
# append path
list(APPEND SDK_INCS ${CMAKE_INSTALL_PREFIX}/include/src
    ${CMAKE_INSTALL_PREFIX}/include)


###############################################################################
# install package
string(TOUPPER ${PROJECT_NAME} PROJECT_PKG_NAME)
install_package(
    PKG_NAME ${PROJECT_PKG_NAME}
    LIB_NAME ${PROJECT_NAME}
    VERSION ${YDLIDAR_SDK_VERSION}
    INSTALL_HEADERS ${SDK_HEADERS}
    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    INCLUDE_DIRS ${SDK_INCS}
    LINK_LIBS ${SDK_LIBS}
    )
#########################################################################################
# install doc
install(DIRECTORY cmake/ DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_PKG_NAME})
install(FILES LICENSE.txt README.md DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_PKG_NAME}/)
install(DIRECTORY doc DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_PKG_NAME})
IF (WIN32)
ELSE()
INSTALL(PROGRAMS startup/initenv.sh DESTINATION startup)
ENDIF()

include_directories(${CMAKE_BINARY_DIR} )

##################################################################################
# make an uninstall target
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in)

# Check if uninstall target already exists
if(NOT TARGET uninstall)
    # Create new uninstall target
    add_custom_target(uninstall
        COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        COMMENT "Running YDLIDAR SDK uninstall script")
else()
    # Create a unique target for this SDK and make it depend on the existing uninstall target
    add_custom_target(uninstall_ydlidar_sdk
        COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        COMMENT "Running YDLIDAR SDK uninstall script")

    # Make the existing uninstall target depend on our SDK uninstall target
    add_dependencies(uninstall uninstall_ydlidar_sdk)
endif()


########################################################
### make package or cpack -C CPackConfig.cmake
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${YDLIDAR_SDK_VERSION})
set(CPACK_PACKAGE_VERSION ${YDLIDAR_SDK_VERSION})
set(CPACK_PACKAGE_CONTACT ${package-contact})
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF) # TODO: review packaging for linux boards
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "YDLIDAR SDK.")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Tony Yang")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${YDLIDAR_SDK_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${YDLIDAR_SDK_VERSION}")
set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")
set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
set(CPACK_SET_DESTDIR true)
set(CPACK_OUTPUT_FILE_PREFIX  ${CMAKE_CURRENT_BINARY_DIR})
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
        set(CPACK_GENERATOR "TBZ2")
        find_program(DPKG_PROGRAM dpkg)
        if (EXISTS ${DPKG_PROGRAM})
                list (APPEND CPACK_GENERATOR "DEB")
        endif()
else()
        set(CPACK_GENERATOR "ZIP")
endif()

include(CPack)

# Summary
include(cmake/script_show_final_summary.cmake REQUIRED)
