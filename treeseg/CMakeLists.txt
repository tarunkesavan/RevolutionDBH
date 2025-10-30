cmake_minimum_required(VERSION 3.10)
project(treeseg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Python setup ---
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
message(STATUS "Using Python includes from: ${Python3_INCLUDE_DIRS}")

# --- PCL and Armadillo ---
find_package(PCL 1.10 REQUIRED)
find_package(Armadillo 9.8 REQUIRED)

# --- nlohmann/json setup ---
# Try to find it first, if not found, fetch it
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
	message(STATUS "nlohmann_json not found, fetching from GitHub...")
	include(FetchContent)
	FetchContent_Declare(json
			GIT_REPOSITORY https://github.com/nlohmann/json.git
			GIT_TAG v3.11.3
	)
	FetchContent_MakeAvailable(json)
else()
	message(STATUS "Found nlohmann_json")
endif()

include_directories(
		${PCL_INCLUDE_DIRS}
		${ARMADILLO_INCLUDE_DIRS}
		${CMAKE_SOURCE_DIR}/include
		${CMAKE_SOURCE_DIR}/include/riegl
)
link_directories(
		${CMAKE_SOURCE_DIR}/lib
		${CMAKE_SOURCE_DIR}/lib/riegl
)

# --- XLSX Writer ---
find_library(XLSXWRITER_LIBRARY NAMES xlsxwriter)
if(NOT XLSXWRITER_LIBRARY)
	message(FATAL_ERROR "xlsxwriter library not found")
endif()

# --- Treeseg Core Library ---
add_library(treeseg SHARED ${CMAKE_SOURCE_DIR}/src/treeseg.cpp)
target_link_libraries(treeseg ${PCL_LIBRARIES})

add_library(leafsep SHARED ${CMAKE_SOURCE_DIR}/src/leafsep.cpp)
target_link_libraries(leafsep treeseg ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})

# --- Executables ---
add_executable(findstems ${CMAKE_SOURCE_DIR}/src/findstems.cpp)
target_include_directories(findstems PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(findstems
		treeseg
		${PCL_LIBRARIES}
		${XLSXWRITER_LIBRARY}
		${Python3_LIBRARIES}
		stdc++fs
		nlohmann_json::nlohmann_json
)

add_executable(segmentstem ${CMAKE_SOURCE_DIR}/src/segmentstem.cpp)
target_link_libraries(segmentstem treeseg ${PCL_LIBRARIES})

add_executable(getdtmslice ${CMAKE_SOURCE_DIR}/src/getdtmslice.cpp)
target_link_libraries(getdtmslice treeseg ${PCL_LIBRARIES})

add_executable(downsample ${CMAKE_SOURCE_DIR}/src/downsample.cpp)
target_link_libraries(downsample treeseg ${PCL_LIBRARIES})

add_executable(thin ${CMAKE_SOURCE_DIR}/src/thin.cpp)
target_link_libraries(thin treeseg ${PCL_LIBRARIES})

add_executable(segmentcrown ${CMAKE_SOURCE_DIR}/src/segmentcrown.cpp)
target_link_libraries(segmentcrown treeseg ${PCL_LIBRARIES})

add_executable(sepwoodleaf ${CMAKE_SOURCE_DIR}/src/sepwoodleaf.cpp)
target_link_libraries(sepwoodleaf treeseg leafsep ${PCL_LIBRARIES})

add_executable(pcdPointTreeseg2txt ${CMAKE_SOURCE_DIR}/src/pcdPointTreeseg2txt.cpp)
target_link_libraries(pcdPointTreeseg2txt treeseg ${PCL_LIBRARIES})

add_executable(txtPointTreeseg2pcd ${CMAKE_SOURCE_DIR}/src/txtPointTreeseg2pcd.cpp)
target_link_libraries(txtPointTreeseg2pcd treeseg ${PCL_LIBRARIES})

add_executable(pcdPointXYZRGB2txt ${CMAKE_SOURCE_DIR}/src/pcdPointXYZRGB2txt.cpp)
target_link_libraries(pcdPointXYZRGB2txt ${PCL_LIBRARIES})
