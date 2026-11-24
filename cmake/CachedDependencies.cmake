add_library(CommonLibSSE STATIC IMPORTED)
set_target_properties(CommonLibSSE PROPERTIES
    IMPORTED_LOCATION "${GMODPOSE_CACHE_ROOT}/linux/_deps/commonlibsse-build/CommonLibSSE.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${GMODPOSE_CACHE_ROOT}/linux/_deps/commonlibsse-src/include"
    INTERFACE_COMPILE_FEATURES cxx_std_23
    INTERFACE_COMPILE_DEFINITIONS "ENABLE_SKYRIM_SE=1;ENABLE_SKYRIM_AE=1;HAS_SKYRIM_MULTI_TARGETING=1;WINVER=0x0601;_WIN32_WINNT=0x0601"
    INTERFACE_LINK_LIBRARIES "spdlog::spdlog;Microsoft::DirectXTK;Microsoft::DirectXMath;advapi32;bcrypt;d3d11;d3dcompiler;dbghelp;dxgi;ole32;shell32;user32;version")
set(imgui_SOURCE_DIR "${GMODPOSE_CACHE_ROOT}/linux/_deps/imgui-src")
