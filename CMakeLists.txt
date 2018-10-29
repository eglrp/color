project(PointCloudGenerator)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
${EIGEN_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
D:\coding\Compiled\WpdPack\Include)

add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE)

add_executable(PointCloudGenerator 
src/PointCloudGenerator.cpp src/PcapParse.cpp 
src/GPRMCInfo.cpp src/TrjFile.cpp 
src/PointColorize.cpp
src/ImageParse.cpp
src/PointRGBGenerator.cpp)

target_link_libraries(PointCloudGenerator 
${PCL_LIBRARIES}
${OpenCV_LIBS}
D:\coding\Compiled\WpdPack\Lib\x64\Packet.lib
D:\coding\Compiled\WpdPack\Lib\x64\wpcap.lib)
