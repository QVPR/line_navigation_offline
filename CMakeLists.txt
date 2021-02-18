# Top level cmake file
# Suman Raj Bista

cmake_minimum_required( VERSION 3.0.0)
project(line_navigation_ros)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
    sensor_msgs
    geometry_msgs
    image_transport
)

catkin_package(
# 	INCLUDE_DIRS include
# 	LIBRARIES miro_teach_repeat
#   CATKIN_DEPENDS message_runtime
# 	DEPENDS system_lib
)

## Libary Paths
    #set(CMAKE_PREFIX_PATH
    #        /home/suman/soft/third_party/naoqi/install/share/naoqi_libqi/cmake
    #        /home/suman/soft/third_party/naoqi/install/share/naoqi_libqicore/cmake
    #   )

    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    set (CMAKE_BUILD_TYPE  RelWithDebInfo )

    option(USE_BIAS_LIBRARY "Use BIAS library for line matching if not OpenCV version is used" ON)
    option(USE_SYSTEM_ARPAK_SUPERLU "Use system Arpac, lapack, blas and superlu" OFF)

    # Custom BIAS library
    if(USE_BIAS_LIBRARY)
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/BIAS_CMakeLists.txt)
    endif()



    set (CMAKE_CXX_STANDARD 11)

    include_directories ( ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
    include_directories(".")

find_package(OpenCV 3 REQUIRED)

    include_directories(    SYSTEM
                            ${OpenCV_INCLUDE_DIRS}
                            
                        )




## Look for Arpack and SuperlU library required for line mataching

## use system arpack, blas,superlu and lapack libaries

if(USE_SYSTEM_ARPAK_SUPERLU)

    FIND_LIBRARY(ARPACK_LIBRARIES NAMES arpack)
    if(NOT ARPACK_LIBRARIES)
        message(FATAL_ERROR "ARPACK library not found")
    endif()

    FIND_LIBRARY(SUPERLU_LIBRARIES NAMES superlu)
    if(NOT SUPERLU_LIBRARIES)
        message(FATAL_ERROR "SUPERLU library not found")
    endif()

    FIND_LIBRARY(BLAS_LIBRARIES NAMES blas)
    if(NOT BLAS_LIBRARIES)
        message(FATAL_ERROR "BLAS library not found")
    endif()

    FIND_LIBRARY(LAPACK_LIBRARIES NAMES lapack)
    if(NOT LAPACK_LIBRARIES)
        message(FATAL_ERROR "LAPACK library not found")
    endif()

    set(ARLPSLU_LIBRARIES       ${ARPACK_LIBRARIES}
                                ${SUPERLU_LIBRARIES}
                                ${BLAS_LIBRARIES}
                                ${LAPACK_LIBRARIES} )
else()

## use custom built from sources => Required if built to run inside the Pepper Robot

    if(EXISTS ${CMAKE_SOURCE_DIR}/linenav/arpack++/lib/libaplpsl_lin64.a)
  	    set(ARLPSLU_LIBRARIES ${CMAKE_SOURCE_DIR}/linenav/arpack++/lib/libaplpsl_lin64.a)
    else()
        message("building custom arpack_superlu")
        add_custom_target(ARLPSLU_LIBRARIES ALL)
        add_custom_command(TARGET ARLPSLU_LIBRARIES COMMAND make all WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/linenav/arpack++ COMMENT "Building" USES_TERMINAL PRE_BUILD)
        set(ARLPSLU_LIBRARIES ${CMAKE_SOURCE_DIR}/linenav/arpack++/lib/libaplpsl_lin64.a)
    endif()

endif()


#find fortan libary (dependencies of ARPACK library)
    FIND_LIBRARY(FORTRAN_LIBRARIES NAMES gfortran HINTS /usr/lib/gcc/x86_64-linux-gnu/5)


    include_directories(linenav/arpack++/include)


# Line Matching sources and headers

if(USE_BIAS_LIBRARY)

    set(BIAS_linematching_SRCS      linenav/edlbd/EDLineDetector.cpp
                                    linenav/edlbd/LineDescriptor.cpp
                                    linenav/edlbd/PairwiseLineMatching.cpp  )

    set(BIAS_linematching_HDRS      linenav/edlbd/EDLineDetector.hh
                                    linenav/edlbd/LineDescriptor.hh
                                    linenav/edlbd/LineStructure.hh
                                    linenav/edlbd/PairwiseLineMatching.hh    )

    set(LineMatching_HDRS       ${BIAS_HEADERS}
                                ${BIAS_linematching_HDRS}    )

    set(LineMatching_SRCS       ${BIAS_SOURCES}
                                ${BIAS_linematching_SRCS}    )

else()

    set(LineMatching_HDRS               linenav/EDLineDetector.cpp
                                        linenav/LineDescriptor.cpp
                                        linenav/PairwiseLineMatching.cpp     )

    set(LineMatching_SRCS               linenav/EDLineDetector.hh
                                        linenav/LineDescriptor.hh
                                        linenav/LineStructure.hh
                                        linenav/PairwiseLineMatching.hh      )

 endif()





set(HEADERS        ${LineMatching_HDRS}
                    navmain/navigation.h
                    navmain/RobotInterface.h
                    navmain/ImagesOffline.h
                    linenav/linenavigation.h
                    linenav/linematch.h
                    linenav/dispnav.h
                    linenav/kimread.h
                     )

set(SOURCES     ${LineMatching_SRCS}
                    navmain/navigation.cpp
                    navmain/RobotInterface.cpp
                    navmain/ImagesOffline.cpp
                    linenav/linenavigation.cpp
                    linenav/linematch.cpp
                    linenav/dispnav.cpp
                    linenav/kimread.cpp
                      )


set(SRCS  navmain/nav_lineoffline.cpp   )



include_directories(navmain)
include_directories(${catkin_INCLUDE_DIRS})


set(extraLIBS ${ARLPSLU_LIBRARIES} ${FORTRAN_LIBRARIES})
message("${extraLIBS}")


add_executable (navigation_offline ${SRCS} ${SOURCES} ${HEADERS} )
target_link_libraries (navigation_offline ${extraLIBS} ${OpenCV_LIBS})

add_executable (navigation_online navmain/nav_online.cpp ${SOURCES} ${HEADERS} )
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries (navigation_online ${extraLIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})