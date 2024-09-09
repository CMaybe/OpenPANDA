# options
set(CHAI3D_CMAKE_DIR "/opt/sai2/chai3d/build")
set(CHAI3D_INCLUDE_DIRS "/opt/sai2/chai3d/src;/opt/sai2/chai3d/external/glew/include;/usr/include;/usr/include/AL;/opt/sai2/chai3d/external/DHD/include")
set(CHAI3D_LIBRARIES "chai3d;/usr/lib/x86_64-linux-gnu/libOpenGL.so;/usr/lib/x86_64-linux-gnu/libGLX.so;/usr/lib/x86_64-linux-gnu/libGLU.so;/usr/lib/x86_64-linux-gnu/libopenal.so;drd;usb-1.0;rt;pthread;dl")
set(CHAI3D_LIBRARY_DIRS "/opt/sai2/chai3d/external/DHD/lib/lin-x86_64")
set(CHAI3D_DEFINITIONS -DLINUX)
set(CHAI3D_SOURCE_DIR /opt/sai2/chai3d)

# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET chai3d AND NOT CHAI3D_BINARY_DIR)
  include("${CHAI3D_CMAKE_DIR}/CHAI3DTargets.cmake")
endif()


set(SAI2-COMMON_CMAKE_DIR "/opt/sai2/sai2-common/build")
set(SAI2-COMMON_INCLUDE_DIRS "/opt/sai2/sai2-common/src;/usr/include/jsoncpp")
set(SAI2-COMMON_DEFINITIONS "")
 
# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-common AND NOT SAI2-COMMON_BINARY_DIR)
  include("${SAI2-COMMON_CMAKE_DIR}/SAI2-COMMONTargets.cmake")
endif()

# IMPORTED target created by SAI2-COMMONTargets.cmake
set(SAI2-COMMON_LIBRARIES "sai2-common;jsoncpp;/usr/lib/x86_64-linux-gnu/libhiredis.so")


set(SAI2-GRAPHICS_CMAKE_DIR "/opt/sai2/sai2-graphics/build")
set(SAI2-GRAPHICS_INCLUDE_DIRS "/opt/sai2/sai2-graphics/src")
set(SAI2-GRAPHICS_DEFINITIONS "")
 
# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-graphics AND NOT SAI2-GRAPHICS_BINARY_DIR)
  include("${SAI2-GRAPHICS_CMAKE_DIR}/SAI2-GRAPHICSTargets.cmake")
endif()

# IMPORTED target created by SAI2-GRAPHICSTargets.cmake
set(SAI2-GRAPHICS_LIBRARIES "sai2-graphics;/usr/lib/x86_64-linux-gnu/libglfw.so")

# compute paths
set(SAI2-MODEL_CMAKE_DIR "/opt/sai2/sai2-model/build")
set(SAI2-MODEL_INCLUDE_DIRS "/opt/sai2/sai2-model/src;/opt/sai2/sai2-model/rbdl/include;/opt/sai2/sai2-model/rbdl/build/include")
set(SAI2-MODEL_DEFINITIONS "")
 
# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-model AND NOT SAI2-MODEL_BINARY_DIR)
  include("${SAI2-MODEL_CMAKE_DIR}/SAI2-MODELTargets.cmake")
endif()

# IMPORTED target created by SAI2-MODELTargets.cmake
set(SAI2-MODEL_LIBRARIES "sai2-model;/opt/sai2/sai2-model/rbdl/build/librbdl.so")


# compute paths
set(SAI2-PRIMITIVES_CMAKE_DIR "/opt/sai2/sai2-primitives/build")
set(SAI2-PRIMITIVES_INCLUDE_DIRS "/opt/sai2/sai2-primitives/src;/opt/sai2/sai2-primitives/ruckig/include/")
set(SAI2-PRIMITIVES_DEFINITIONS "")
 
# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-primitives AND NOT SAI2-PRIMITIVES_BINARY_DIR)
  include("${SAI2-PRIMITIVES_CMAKE_DIR}/SAI2-PRIMITIVESTargets.cmake")
endif()

# IMPORTED target created by SAI2-PRIMITIVESTargets.cmake
set(SAI2-PRIMITIVES_LIBRARIES "sai2-primitives;/opt/sai2/sai2-primitives/ruckig/build/libruckig.so")


# compute paths
set(SAI2-SIMULATION_CMAKE_DIR "/opt/sai2/sai2-simulation/build")
set(SAI2-SIMULATION_INCLUDE_DIRS "/opt/sai2/sai2-simulation/src;/opt/sai2/sai2-simulation/src/headers_core")
set(SAI2-SIMULATION_DEFINITIONS " -DBUILD64")
set(SAI2-SIMULATION_LIBRARIES "sai2-simulation;/opt/sai2/sai2-simulation/lib/linux/x86_64/libsai2-simulation-core.a")

# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-simulation AND NOT SAI2-SIMULATION_BINARY_DIR)
  include("${SAI2-SIMULATION_CMAKE_DIR}/SAI2-SIMULATIONTargets.cmake")
endif()

# compute paths
set(SAI2-URDF_CMAKE_DIR "/opt/sai2/sai2-urdfreader/build")
set(SAI2-URDF_INCLUDE_DIRS "/opt/sai2/sai2-urdfreader/src;/opt/sai2/sai2-urdfreader/src/urdf/urdfdom/urdf_parser/include")
set(SAI2-URDF_DEFINITIONS "")
 
# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-urdf AND NOT SAI2-URDF_BINARY_DIR)
  include("${SAI2-URDF_CMAKE_DIR}/SAI2-URDFTargets.cmake")
endif()

# IMPORTED target created by SAI2-URDFTargets.cmake
set(SAI2-URDF_LIBRARIES "sai2-urdf;/usr/lib/x86_64-linux-gnu/libtinyxml2.so")
