##
# Open Space Program
# Copyright © 2019-2020 Open Space Program Project
#
# MIT License
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
##

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED
    GL
    Shaders
    MeshTools
    Primitives
    #SceneGraph
    Trade
    Sdl2Application
    AnyImageImporter
    )

find_package(MagnumPlugins REQUIRED
    TinyGltfImporter
    StbImageImporter)

find_package(spdlog)

find_package(Threads)

# Gather paths to OSP headers and sources
add_executable(osp-magnum)
file (GLOB_RECURSE CPP_FILES CONFIGURE_DEPENDS *.cpp)
file (GLOB_RECURSE H_FILES CONFIGURE_DEPENDS *.h)
target_sources(osp-magnum PRIVATE "${CPP_FILES}" "${H_FILES}")

# For some reason, source_group() requires all files to be added to the executable
set(SHADERS_DIR "../bin/OSPData/adera/Shaders")
file (GLOB_RECURSE SHADER_FILES CONFIGURE_DEPENDS
    "${SHADERS_DIR}/*.vert"
    "${SHADERS_DIR}/*.frag"
    "${SHADERS_DIR}/*.comp"
)
target_sources(osp-magnum PRIVATE "${SHADER_FILES}")

# Set osp-magnum as startup project
set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT osp-magnum)

target_include_directories(osp-magnum PRIVATE .)

# Include Newton Dynamics to the project

# TODO: find a better way to do this.
#       Somehow Newton needs some defines for headers to work properly.
#       This section was based on Newton Dynamic's CMakeLists.txt
if(UNIX)
    if (BUILD_64)
        target_compile_definitions(osp-magnum PRIVATE -D_POSIX_VER_64)
    else (BUILD_64)
        target_compile_definitions(osp-magnum PRIVATE -D_POSIX_VER)
    endif (BUILD_64)

elseif(MSVC)
    if(WIN32)
        if(CMAKE_CL_64)
            target_compile_definitions(osp-magnum PRIVATE -D_WIN_64_VER)
        else()
            target_compile_definitions(osp-magnum PRIVATE -D_WIN_32_VER)
        endif()
    else()
        target_compile_definitions(osp-magnum PRIVATE -D_ARM_VER)
    endif()

elseif(MINGW)
    if(CMAKE_CL_64)
        target_compile_definitions(osp-magnum PRIVATE -D_MINGW_64_VER)
        target_compile_definitions(osp-magnum PRIVATE -D_WIN_64_VER)
    else()
        target_compile_definitions(osp-magnum PRIVATE -D_MINGW_32_VER)
        target_compile_definitions(osp-magnum PRIVATE -D_WIN_32_VER)
    endif()

endif()

# MSVC quality of life improvements for Visual Studio

# Set the MSVC debug working directory
set_property(TARGET osp-magnum PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ../../bin)

# Enforce conformance mode for osp-magnum
target_compile_options(osp-magnum PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/permissive->)

# Segregate headers, shaders into filters
source_group("Shader Files" FILES ${SHADER_FILES})
source_group("Header Files" FILES ${H_FILES})

# Include ENTT (header only lib)
target_include_directories(osp-magnum PRIVATE ../3rdparty/entt/src)

# Put executable in the bin folder
set_target_properties(osp-magnum PROPERTIES RUNTIME_OUTPUT_DIRECTORY ../bin)

target_link_libraries(osp-magnum PRIVATE
    ${CMAKE_THREAD_LIBS_INIT}
    EnTT::EnTT
    Corrade::Main
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    #Magnum::SceneGraph
    Magnum::Shaders
    Magnum::Trade
    Magnum::AnyImageImporter
    MagnumPlugins::TinyGltfImporter
    MagnumPlugins::StbImageImporter
    dNewton dScene dModel dVehicle
    toml11
    spdlog
    )

if(OSP_ENABLE_IWYU)
  find_program(iwyu_path NAMES include-what-you-use iwyu)
  if(iwyu_path)
      set_property(TARGET osp-magnum
                   PROPERTY
                   CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path}
                   -Xiwyu
                   --mapping_file=${CMAKE_SOURCE_DIR}/iwyu.imp)
  endif()
endif()

if(OSP_ENABLE_CLANG_TIDY)
  find_program(tidy_path NAMES clang-tidy)
  if(tidy_path)
      set_property(TARGET osp-magnum
                   PROPERTY
                   CXX_CLANG_TIDY ${tidy_path}
                   --checks="clang-diagnostic-*,clang-analyzer-*,bugprone-*,performance-*,readability-*,modernize-*,-modernize-use-trailing-return-type,-modernize-use-auto")
  endif()
endif()

# Copy root/bin to build/bin
FILE (COPY "${CMAKE_SOURCE_DIR}/bin/" DESTINATION "${CMAKE_BINARY_DIR}/bin/")

