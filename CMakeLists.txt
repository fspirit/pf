project(pf)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.13.1/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(SOURCE_FILES main.cpp WebSocketMessageHandler.cpp WebSocketMessageHandler.h ParticleFilter.cpp ParticleFilter.h GpsMeasurement.h
        ControlMeasurement.h LandmarkObservation.h GpsMeasurementNoise.h ObservationsNoise.cpp ObservationsNoise.h Landmark.cpp Landmark.h)
add_executable(pf ${SOURCE_FILES})
target_link_libraries(pf z ssl uv uWS)