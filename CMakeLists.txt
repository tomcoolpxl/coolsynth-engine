cmake_minimum_required(VERSION 3.22)

project(CoolSynthEngine VERSION 0.1.0 LANGUAGES C CXX) # C as well: JUCE 9 builds zlib, libpng, libjpeg and libflac as C

# Built on its own (its tests, its CI), the engine brings the shared build settings, JUCE and CTest itself. Inside
# CoolSynth or CoolSynth 808 the app has done all three before it adds this folder.
if(PROJECT_IS_TOP_LEVEL)
    include(cmake/CoolSynthFlags.cmake)
    add_subdirectory(external/JUCE)
    include(CTest)
endif()

include(cmake/CoolSynthEngine.cmake)
