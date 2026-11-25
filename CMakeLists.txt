cmake_minimum_required(VERSION 3.20)
project(SourceVoice LANGUAGES CXX RC)

if(NOT WIN32)
    message(FATAL_ERROR "Mic-Sourceify currently targets Windows.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vcpkg
set(VCPKG_INSTALLED "C:/vcpkg/installed/x64-windows")

# PortAudio
set(PORTAUDIO_INCLUDE_DIR "${VCPKG_INSTALLED}/include")
set(PORTAUDIO_LIBRARY "${VCPKG_INSTALLED}/lib/portaudio.lib")

# Speex
set(SPEEX_INCLUDE_DIR "${VCPKG_INSTALLED}/include")
set(SPEEX_LIBRARY "${VCPKG_INSTALLED}/lib/speex.lib")

# Проверка зависимостей
foreach(FILE_PATH
    "${PORTAUDIO_INCLUDE_DIR}/portaudio.h"
    "${SPEEX_INCLUDE_DIR}/speex/speex.h"
    "${PORTAUDIO_LIBRARY}"
    "${SPEEX_LIBRARY}"
)
    if(NOT EXISTS "${FILE_PATH}")
        message(FATAL_ERROR "Dependency not found: ${FILE_PATH}")
    endif()
endforeach()

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/resource.rc.in"
    "${CMAKE_CURRENT_BINARY_DIR}/resource.rc"
    @ONLY
)

add_executable(source-voice main.cpp "${CMAKE_CURRENT_BINARY_DIR}/resource.rc")

target_include_directories(source-voice PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${PORTAUDIO_INCLUDE_DIR}"
    "${SPEEX_INCLUDE_DIR}"
)

target_link_libraries(source-voice PRIVATE
    "${PORTAUDIO_LIBRARY}"
    "${SPEEX_LIBRARY}"
    Comctl32
    UxTheme
    Shell32
)

target_compile_definitions(source-voice PRIVATE
    _CRT_SECURE_NO_WARNINGS
)

if(MSVC)
    target_compile_options(source-voice PRIVATE
        /W4
    )
endif()

