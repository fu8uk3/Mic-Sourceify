cmake_minimum_required(VERSION 3.20)
project(SourceVoice LANGUAGES CXX)

if(NOT WIN32)
  message(FATAL_ERROR "Source Voice currently targets Windows (WASAPI via PortAudio).")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_path(PORTAUDIO_INCLUDE_DIR portaudio.h REQUIRED)
find_library(PORTAUDIO_LIBRARY NAMES portaudio REQUIRED)
find_path(SPEEX_INCLUDE_DIR speex/speex.h REQUIRED)
find_library(SPEEX_LIBRARY NAMES speex REQUIRED)

add_executable(source-voice main.cpp)
target_include_directories(source-voice PRIVATE ${PORTAUDIO_INCLUDE_DIR} ${SPEEX_INCLUDE_DIR})
target_link_libraries(source-voice PRIVATE ${PORTAUDIO_LIBRARY} ${SPEEX_LIBRARY})
target_compile_definitions(source-voice PRIVATE _CRT_SECURE_NO_WARNINGS)
if(MSVC)
  target_compile_options(source-voice PRIVATE /W4 /O2)
endif()
