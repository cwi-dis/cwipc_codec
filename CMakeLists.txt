
cmake_minimum_required(VERSION 3.16.0)
project(cwipc_codec)


#
# Find cwipc_util
#
if( CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR )
	message(WARNING Building cwipc_codec outside cwipc umbrella is not tested)
	find_package(cwipc_util REQUIRED)
	include_directories(${cwipc_util_INCLUDE_DIRS})
	link_directories(${cwipc_util_LIBRARY_DIRS})
endif()

#set(SUBSYS_NAME cwi_pcl_codec)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMakeFiles)
#set(QUIET_FLAG QUIET)

# Expose public includes and libraries to subprojects
#set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
#                                 ${Boost_INCLUDE_DIR}
#                                 ${PCL_INCLUDE_DIRS}
#                                 ${EIGEN_INCLUDE_DIRS}
#    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
#
#set(${PROJECT_NAME}_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/lib
#                                 ${Boost_LIBRARY_DIR}
#                                 ${PCL_LIBRARY_DIRS}
#    CACHE INTERNAL "${PROJECT_NAME}: Library Directories" FORCE)
#
#set(${PROJECT_NAME}_LIBRARIES
#                                 ${VTK_LIBRARIES}
#                                 ${Boost_LIBRARIES}
#                                 ${PCL_LIBRARIES}
#								 libjpeg-turbo::jpeg
#    CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)
#
#set(${PROJECT_NAME}_DEFINITIONS
#                                 ${Boost_DEFINITIONS}
#                                 ${PCL_DEFINITIONS}
#    CACHE INTERNAL "${PROJECT_NAME}: Definitions" FORCE)
#
#add_definitions( ${${PROJECT_NAME}_DEFINITIONS})
add_subdirectory("cloud_codec_v2")
add_subdirectory("src")
#add_subdirectory("apps/evaluate_compression")
add_subdirectory("apps/cwipc_encode")
add_subdirectory("apps/cwipc_decode")
add_subdirectory("apps/cwipc_enc_perftest")
add_subdirectory("python")
add_subdirectory("tests")
