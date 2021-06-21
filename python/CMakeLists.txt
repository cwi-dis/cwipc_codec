cmake_minimum_required(VERSION 3.10.0)

set(INSTALL_PATH ${CMAKE_INSTALL_DATADIR}/cwipc_codec/python)
set(DEST_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_PATH})

# This is gross. We need to set the directory where the cwipc module can be found while building with submodules.
if(NOT CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
	get_filename_component(cwipc_util_PYTHON_DIRS ${PROJECT_SOURCE_DIR}/../cwipc_util/python ABSOLUTE)
endif()

find_package(Python3 COMPONENTS Interpreter)

# Unfortunately Ubuntu 18.04 doesn't have the Python3 package (introduced in cmake 3.12 or so). Work around it.
if(NOT Python3_FOUND)
    find_program(PY3INTERPRETER python3)
    if(PY3INTERPRETER)
        set(Python3_EXECUTABLE ${PY3INTERPRETER})
        set(Python3_FOUND TRUE)
    endif()
endif()

install(FILES test_cwipc_codec.py setup.py DESTINATION ${INSTALL_PATH})
install(DIRECTORY _cwipc_codec DESTINATION ${INSTALL_PATH} PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
if(Python3_FOUND)
	install(CODE "MESSAGE(\"Installing Python modules with ${DEST_INSTALL_PATH}/setup.py\")")
	install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} setup.py install WORKING_DIRECTORY ${DEST_INSTALL_PATH} )")
else()
	MESSAGE("Python3 not found. Install python modules manually from ${INSTALL_PATH}")
endif()

if(Python3_FOUND)
	add_test(NAME cwipc_codec_python_tests
		COMMAND ${Python3_EXECUTABLE} test_cwipc_codec.py
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	
	# Ensure the cwipc_codec dynamic library and cwipc python module can be found by the tests
	if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set_property(TEST cwipc_codec_python_tests PROPERTY ENVIRONMENT DYLD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{DYLD_LIBRARY_PATH})
		set_property(TEST cwipc_codec_python_tests APPEND PROPERTY ENVIRONMENT PYTHONPATH=${cwipc_util_PYTHON_DIRS}:$ENV{PYTHONPATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set_property(TEST cwipc_codec_python_tests PROPERTY ENVIRONMENT LD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:${cwipc_util_LIBRARY_DIRS}:$ENV{LD_LIBRARY_PATH})
		set_property(TEST cwipc_codec_python_tests APPEND PROPERTY ENVIRONMENT PYTHONPATH=${cwipc_util_PYTHON_DIRS}:$ENV{PYTHONPATH})
	endif()
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		string(REPLACE ";" "\\;" prevpath "$ENV{PATH}")
		set_property(
			TEST cwipc_codec_python_tests
			APPEND PROPERTY ENVIRONMENT "PATH=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}Release\;${prevpath}"
		)
		set_property(TEST cwipc_codec_python_tests APPEND PROPERTY ENVIRONMENT PYTHONPATH=${cwipc_util_PYTHON_DIRS};$ENV{PYTHONPATH})
	endif()
endif()
