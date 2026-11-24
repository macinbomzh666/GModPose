include(FetchContent)
set(ENABLE_SKYRIM_SE ON CACHE BOOL "" FORCE)
set(ENABLE_SKYRIM_AE ON CACHE BOOL "" FORCE)
set(ENABLE_SKYRIM_VR OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SKSE_SUPPORT_PATCH_SAFETY OFF CACHE BOOL "" FORCE)
set(COMMONLIB_PREBUILT OFF CACHE BOOL "" FORCE)
if(CMAKE_CROSSCOMPILING)
    set(COMMONLIB_ENABLE_IPO OFF CACHE BOOL "Enable CommonLibSSE-NG link-time optimization")
endif()

FetchContent_Declare(commonlibsse
    GIT_REPOSITORY https://github.com/alandtse/CommonLibSSE-NG.git
    GIT_TAG 8ffc2b74af015f837de53cc02b0cb470a15678e4
    GIT_SUBMODULES extern/openvr)
FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.9.tar.gz
    URL_HASH SHA256=af97ed649182c39314320514a672b82008ab462b9293fe23d37b30bfa5d05519
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(commonlibsse imgui)
target_link_libraries(CommonLibSSE PUBLIC Microsoft::DirectXMath)
