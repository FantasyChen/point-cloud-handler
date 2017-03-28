FIND_PACKAGE( OpenCV 2.4.11 )
FIND_PACKAGE( PCL 1.7 )

# OpenCV
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${OpenCV_LIBRARY_DIRS} )

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
#INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )
# Eigen
FIND_PACKAGE( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR})
### CERES
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("ceres found ${Ceres_FOUND} at ${Ceres_DIR}")
### GFLAGS
FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
MESSAGE("gflags found ${Gflags_FOUND} at ${GFLAGS_INCLUDE_DIRS} with namespace: ${GFLAGS_NAMESPACE}")

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/json)

add_library(CeresICP CeresICP.cpp LidarPointCloudHandler.cpp ${PROJECT_SOURCE_DIR}/include/LidarPointCloudHandler.h)
TARGET_LINK_LIBRARIES(CeresICP ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES} ${PCL_LIBRARIES})

add_library(JsonCPP jsoncpp.cpp  ${PROJECT_SOURCE_DIR}/include/json/json.h)
TARGET_LINK_LIBRARIES(JsonCPP)

add_executable (LidarBoudingBoxCluster LidarBoundingBoxCluster.cpp ${PROJECT_SOURCE_DIR}/include/json/json.h ${PROJECT_SOURCE_DIR}/include/utils.h ModelRepresentation.cpp ${PROJECT_SOURCE_DIR}/include/ModelRepresentation.h utils.cpp LidarPointCloudHandler.cpp ${PROJECT_SOURCE_DIR}/include/LidarPointCloudHandler.h)
TARGET_LINK_LIBRARIES(LidarBoudingBoxCluster JsonCPP ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CSPARSE_LIBRARY} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})


add_executable (points_grouping ExtractionAndMatching.cpp ${PROJECT_SOURCE_DIR}/include/utils.h ModelRepresentation.cpp ${PROJECT_SOURCE_DIR}/include/ModelRepresentation.h utils.cpp LidarPointCloudHandler.cpp ${PROJECT_SOURCE_DIR}/include/LidarPointCloudHandler.h)
TARGET_LINK_LIBRARIES(points_grouping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CSPARSE_LIBRARY} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable (CurbDetection CurbDetection.cpp ${PROJECT_SOURCE_DIR}/include/utils.h ModelRepresentation.cpp ${PROJECT_SOURCE_DIR}/include/ModelRepresentation.h utils.cpp LidarPointCloudHandler.cpp ${PROJECT_SOURCE_DIR}/include/LidarPointCloudHandler.h)
TARGET_LINK_LIBRARIES(CurbDetection ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CSPARSE_LIBRARY} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})


add_executable (MultiframeIntegration MultiframeIntegration.cpp ${PROJECT_SOURCE_DIR}/include/utils.h ModelRepresentation.cpp ${PROJECT_SOURCE_DIR}/include/ModelRepresentation.h utils.cpp LidarPointCloudHandler.cpp ${PROJECT_SOURCE_DIR}/include/LidarPointCloudHandler.h)
TARGET_LINK_LIBRARIES(MultiframeIntegration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CSPARSE_LIBRARY} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})

