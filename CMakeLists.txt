cmake_minimum_required(VERSION 3.24)

project(PointCloudRenderer LANGUAGES C CXX)

include(FetchContent)

set(
    LASZIP_BUILD_STATIC
    ON
    CACHE BOOL
    "Build LASzip as a static library"
    FORCE
)

FetchContent_Declare(
    laszip

    GIT_REPOSITORY
        https://github.com/LASzip/LASzip.git

    GIT_TAG
        3.5.0

    GIT_SHALLOW
        TRUE
)

FetchContent_MakeAvailable(laszip)

find_program(
    DXC_EXECUTABLE
    NAMES dxc.exe
    HINTS
        "$ENV{WindowsSdkDir}/bin/$ENV{WindowsSDKVersion}/x64"
        "C:/Program Files (x86)/Windows Kits/10/bin/${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}/x64"
    REQUIRED
)

set(
    GENERATED_SHADER_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/generated/shaders"
)

file(
    MAKE_DIRECTORY
    "${GENERATED_SHADER_DIRECTORY}"
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/ReferenceVS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T vs_6_0
        -E VSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/ReferenceVS.h"
        -Vn g_referenceVertexShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/Reference.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/Reference.hlsl"

    COMMENT
        "Compiling reference vertex shader"

    VERBATIM
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/ReferencePS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T ps_6_0
        -E PSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/ReferencePS.h"
        -Vn g_referencePixelShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/Reference.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/Reference.hlsl"

    COMMENT
        "Compiling reference pixel shader"

    VERBATIM
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/ClodReduceCS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T cs_6_0
        -E CSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/ClodReduceCS.h"
        -Vn g_clodReduceComputeShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ClodReduce.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ClodReduce.hlsl"

    COMMENT
        "Compiling CLOD reduction compute shader"

    VERBATIM
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/ClodDrawVS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T vs_6_0
        -E VSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/ClodDrawVS.h"
        -Vn g_clodDrawVertexShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ClodDraw.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ClodDraw.hlsl"

    COMMENT
        "Compiling CLOD splat vertex shader"

    VERBATIM
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/ClodDrawPS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T ps_6_0
        -E PSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/ClodDrawPS.h"
        -Vn g_clodDrawPixelShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ClodDraw.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ClodDraw.hlsl"

    COMMENT
        "Compiling CLOD splat pixel shader"

    VERBATIM
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/DebugLinesVS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T vs_6_0
        -E VSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/DebugLinesVS.h"
        -Vn g_debugLinesVertexShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DebugLines.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DebugLines.hlsl"

    COMMENT
        "Compiling debug line vertex shader"

    VERBATIM
)

add_custom_command(
    OUTPUT
        "${GENERATED_SHADER_DIRECTORY}/DebugLinesPS.h"

    COMMAND
        "${DXC_EXECUTABLE}"
        -T ps_6_0
        -E PSMain
        -WX
        -Fh "${GENERATED_SHADER_DIRECTORY}/DebugLinesPS.h"
        -Vn g_debugLinesPixelShader
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DebugLines.hlsl"

    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DebugLines.hlsl"

    COMMENT
        "Compiling debug line pixel shader"

    VERBATIM
)

add_custom_target(
    PointCloudShaders

    DEPENDS
        "${GENERATED_SHADER_DIRECTORY}/ReferenceVS.h"
        "${GENERATED_SHADER_DIRECTORY}/ReferencePS.h"
        "${GENERATED_SHADER_DIRECTORY}/ClodReduceCS.h"
        "${GENERATED_SHADER_DIRECTORY}/ClodDrawVS.h"
        "${GENERATED_SHADER_DIRECTORY}/ClodDrawPS.h"
        "${GENERATED_SHADER_DIRECTORY}/DebugLinesVS.h"
        "${GENERATED_SHADER_DIRECTORY}/DebugLinesPS.h"
)

add_executable(PointCloudRenderer WIN32
    src/main.cpp
    src/Application.cpp
    src/Application.h
    src/Camera.cpp
    src/Camera.h
    src/D3D12Context.cpp
    src/D3D12Context.h
    src/GpuPoint.h
    src/PointCloudHierarchy.h
    src/PointCloudHierarchyBuilder.cpp
    src/PointCloudHierarchyBuilder.h
    src/LasPointCloudLoader.cpp
    src/LasPointCloudLoader.h
    src/PointCloudData.cpp
    src/PointCloudData.h
    src/PointCloudDataset.cpp
    src/PointCloudDataset.h
    src/Win32Helpers.h
    src/TileLoadWorker.h
    src/TileLoadWorker.cpp
    src/ClodPoint.h
    src/ReducedClodPoint.h
)

target_sources(
    PointCloudRenderer
    PRIVATE
        shaders/Reference.hlsl
        shaders/ClodReduce.hlsl
        shaders/ClodDraw.hlsl
        shaders/DebugLines.hlsl
)

add_dependencies(
    PointCloudRenderer
    PointCloudShaders
)

target_include_directories(
    PointCloudRenderer
    PRIVATE
        "${GENERATED_SHADER_DIRECTORY}"
        "${laszip_SOURCE_DIR}/dll"
        "${laszip_SOURCE_DIR}/include/laszip"
)

target_compile_features(PointCloudRenderer PRIVATE cxx_std_20)

target_compile_definitions(PointCloudRenderer PRIVATE
    UNICODE
    _UNICODE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

target_link_libraries(
    PointCloudRenderer
    PRIVATE
        d3d12
        dxgi
        dxguid
        shell32
        laszip3
)

target_compile_options(PointCloudRenderer PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /EHsc>
)

set_property(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTY VS_STARTUP_PROJECT PointCloudRenderer
)
