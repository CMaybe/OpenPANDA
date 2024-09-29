cmake_minimum_required(VERSION 3.10)

project(SimPandaControl)
set(PROJECT_VERSION 0.1.0)


list(INSERT CMAKE_MODULE_PATH 0  ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -fPIC")


# set default build to release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release)
endif()


include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/utils)

# include Franka for Panda
find_package(Franka REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

# include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Sai2 REQUIRED)

# include Chai3d
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
link_directories(${CHAI3D_LIBRARY_DIRS})

# SAI2-COMMON
include_directories(${SAI2-COMMON_INCLUDE_DIRS})

# SAI2-URDF
include_directories(${SAI2-URDF_INCLUDE_DIRS})

# SAI2-MODEL
include_directories(${SAI2-MODEL_INCLUDE_DIRS})

# SAI2-SIMULATION
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# SAI2-GRAPHICS
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# SAI2-PRIMITIVES
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})

# SAI2-INTERFACES
include_directories(${SAI2-INTERFACES_INCLUDE_DIRS})

# application
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

set(OPENSAI_LIBRARIES
${SAI2-GRAPHICS_LIBRARIES}
${SAI2-URDF_LIBRARIES}
${CHAI3D_LIBRARIES}
${SAI2-INTERFACES_LIBRARIES}
${SAI2-PRIMITIVES_LIBRARIES}
${SAI2-MODEL_LIBRARIES}
${SAI2-SIMULATION_LIBRARIES}
${SAI2-COMMON_LIBRARIES}
)


# add apps
set (PANDA_SIMULATION_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PANDA_SIMULATION_BINARY_DIR})

# Source files
set(SOURCES
    src/main.cpp
    src/sai2/control.cpp
    src/sai2/simulation.cpp
    src/robot/panda_controller.cpp
	src/utils/trajectory_manager.tpp
)

# Headers
set(HEADERS
    include/sai2/control.hpp 
    include/sai2/simulation.hpp
    include/robot/panda_controller.hpp
	include/utils/trajectory_manager.hpp
)

ADD_EXECUTABLE (PandaSim ${SOURCES} ${HEADERS})

# and link the library against the executable
target_link_libraries(PandaSim ${OPENSAI_LIBRARIES} Franka::Franka Eigen3::Eigen)

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY model/world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY model/panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR}/model)
FILE(COPY model/meshes DESTINATION ${APP_RESOURCE_DIR}/model)
