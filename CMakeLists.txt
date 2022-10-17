cmake_minimum_required(VERSION 3.19)
project(billiard)

set(CMAKE_CXX_STANDARD 11)

set(curl_DIR "C:/Users/chibi/Documents/libs64_test/curl/curl-7.69.1")
set(OpenCV_DIR "C:\\Users\\chibi\\Documents\\opencv\\opencv\\cmake-gui-build\\install")

include_directories(${OpenCV_DIR}/include)
link_directories(${OpenCV_DIR}/x64/mingw/lib)

add_executable(billiard main.cpp
#        SimpleSerial.cpp
        UDPServer.cpp)

target_link_libraries(billiard
        libopencv_core460 libopencv_highgui460  libopencv_imgproc460
        libopencv_imgcodecs460 libopencv_videoio460
        ws2_32.lib)
