cmake_minimum_required(VERSION 3.25)
project(GModPose VERSION 1.1.3 LANGUAGES CXX)

if(NOT WIN32 OR NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "GModPose requires Windows x64 and MSVC.")
endif()
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GMODPOSE_DEPENDENCIES_ONLY "Build the pinned dependency packages" OFF)
if(GMODPOSE_DEPENDENCIES_ONLY)
    add_subdirectory(cmake/dependencies)
    return()
endif()

set(GMODPOSE_CACHE_ROOT "" CACHE PATH "Optional compatible prebuilt dependency cache")
if(GMODPOSE_CACHE_ROOT)
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "19.51.36260.0")
        message(FATAL_ERROR "The dependency cache requires Release /MD and MSVC 19.51.36260.0.")
    endif()
    find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
    execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/check-cache.py"
        "${GMODPOSE_CACHE_ROOT}" COMMAND_ERROR_IS_FATAL ANY)
    list(PREPEND CMAKE_PREFIX_PATH "${GMODPOSE_CACHE_ROOT}/prefix")
endif()
find_package(spdlog 1.16.0 EXACT CONFIG REQUIRED)
find_package(directxmath CONFIG REQUIRED)
find_package(directxtk CONFIG REQUIRED)

if(GMODPOSE_CACHE_ROOT)
    include(cmake/CachedDependencies.cmake)
else()
    include(cmake/SourceDependencies.cmake)
endif()

add_library(GModPoseImGui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp")
target_include_directories(GModPoseImGui SYSTEM PUBLIC "${imgui_SOURCE_DIR}")
# Keep supplementary Unicode characters through the preset name input fields.
target_compile_definitions(GModPoseImGui PUBLIC IMGUI_USE_WCHAR32)
target_compile_options(GModPoseImGui PRIVATE /utf-8)
target_link_libraries(GModPoseImGui PUBLIC d3d11 dxgi d3dcompiler)

include(FetchContent)
FetchContent_Declare(json
    URL https://raw.githubusercontent.com/nlohmann/json/v3.12.0/single_include/nlohmann/json.hpp
    URL_HASH SHA256=aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63
    DOWNLOAD_NO_EXTRACT TRUE)
FetchContent_MakeAvailable(json)

add_library(GModPose SHARED src/Plugin.cpp src/core/Editor.cpp
    src/core/DragView.cpp src/core/PosingCamera.cpp src/core/PoseSettings.cpp
    src/core/FingerPanel.cpp src/core/FacePanel.cpp src/core/FaceActor.cpp
    src/core/PoseMath.cpp src/core/FingerPose.cpp src/core/SeatBlock.cpp)
target_link_libraries(GModPose PRIVATE CommonLibSSE GModPoseImGui Microsoft::DirectXTK Microsoft::DirectXMath)
target_include_directories(GModPose SYSTEM PRIVATE "${json_SOURCE_DIR}")
target_compile_definitions(GModPose PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_compile_options(GModPose PRIVATE /W4 /permissive- /utf-8 /Zc:preprocessor /bigobj
    /wd4005 /wd4061 /wd4068 /wd4200 /wd4201 /wd4265 /wd4266 /wd4371
    /wd4514 /wd4582 /wd4583 /wd4623 /wd4625 /wd4626 /wd4702 /wd4710
    /wd4711 /wd4820 /wd4996 /wd5026 /wd5027 /wd5045 /wd5053 /wd5204 /wd5220)
set_target_properties(GModPose PROPERTIES PREFIX "" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")
