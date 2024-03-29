# Software License Agreement (BSD License)
# 
# Point Cloud Library (PCL) - www.pointclouds.org
# Copyright (c) 2009-2012, Willow Garage, Inc.
# Copyright (c) 2012-, Open Perception, Inc.
# Copyright (c) CodeShop B.V. 2016-
# Copyright (c) 2017-, Stichting Centrum Wiskunde en Informatica.
# 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met: 
# 
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder(s) nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.16.0)


add_library(cwipc_codec SHARED cwipc_codec.cpp cwipc_pcl_additions.cpp)

target_sources(cwipc_codec
	PRIVATE	"../include/cwipc_codec/api.h"
	"readerwriterqueue.h"
	"atomicops.h"
	"cwipc_codec_config.h"
)

target_include_directories(cwipc_codec
	PUBLIC 
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> 
		$<INSTALL_INTERFACE:include> 
	PRIVATE 
		"../cloud_codec_v2/include" 
		"../jpeg_io/include" 
		${PCL_INCLUDE_DIRS}
)

target_link_libraries(cwipc_codec 
	PUBLIC cwipc_util 
	PRIVATE cloud_codec_v2 ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${JPEG_Turbo_LIBRARY})

install(TARGETS cwipc_codec
    EXPORT cwipc_codec
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION include/cwipc_codec
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/cwipc_codec
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${PROJECT_SOURCE_DIR}/CMakeFiles/cwipc_codec-config.cmake 
	DESTINATION lib/cmake/cwipc_codec)

install(EXPORT cwipc_codec 
	DESTINATION lib/cmake/cwipc_codec)

if(WIN32)
	# Copy the dependent DLLs that cmake/vcpkg have created
	install(FILES $<TARGET_RUNTIME_DLLS:cwipc_codec> DESTINATION ${CMAKE_INSTALL_BINDIR})
	# Copy the PDB file, if it exists
	install(FILES $<TARGET_PDB_FILE:cwipc_codec> DESTINATION ${CMAKE_INSTALL_BINDIR} OPTIONAL)
endif()
