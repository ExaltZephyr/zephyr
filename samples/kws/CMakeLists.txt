cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kws_demo)

# Disable thread-safe local static initialization (not needed on Zephyr)
set(NO_THREADSAFE_STATICS $<TARGET_PROPERTY:compiler-cpp,no_threadsafe_statics>)
zephyr_compile_options($<$<COMPILE_LANGUAGE:CXX>:${NO_THREADSAFE_STATICS}>)

# Path to the tflite-micro module
set(TFLM_DIR $ENV{ZEPHYR_BASE}/../optional/modules/lib/tflite-micro)

# -- Embed binary assets as C include files --
set(gen_dir ${GENERATED_HEADER_ROOT_DIR}/kws)

# Optional test WAV set (auto-discovered under src/test_wavs/*.wav)
file(GLOB TEST_WAV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/test_wavs/*.wav")

set(TEST_WAV_GEN_HPP ${CMAKE_CURRENT_BINARY_DIR}/test_wavs_gen.hpp)
set(TEST_WAV_GEN_CPP ${CMAKE_CURRENT_BINARY_DIR}/test_wavs_gen.cpp)

file(WRITE ${TEST_WAV_GEN_HPP}
"#pragma once\n"
"#include <cstddef>\n"
"#include <cstdint>\n\n"
"struct EmbeddedTestWav {\n"
"    const char* name;\n"
"    const uint8_t* data;\n"
"    size_t size;\n"
"};\n\n"
"extern const EmbeddedTestWav* g_test_wavs;\n"
"extern const size_t g_test_wavs_count;\n")

file(WRITE ${TEST_WAV_GEN_CPP}
"#include \"test_wavs_gen.hpp\"\n\n")

set(TEST_WAV_ENTRIES "")
foreach(TEST_WAV ${TEST_WAV_FILES})
    get_filename_component(WAV_STEM ${TEST_WAV} NAME_WE)
    string(MAKE_C_IDENTIFIER "${WAV_STEM}" WAV_ID)

    set(WAV_INC ${gen_dir}/test_wav_${WAV_ID}.inc)
    generate_inc_file_for_target(app ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_WAV} ${WAV_INC})
    get_filename_component(WAV_INC_NAME ${WAV_INC} NAME)

    file(APPEND ${TEST_WAV_GEN_CPP}
    "static const uint8_t test_wav_${WAV_ID}[] = {\n"
    "#include \"${WAV_INC_NAME}\"\n"
    "};\n\n")

    list(APPEND TEST_WAV_ENTRIES "{\"${WAV_STEM}\", test_wav_${WAV_ID}, sizeof(test_wav_${WAV_ID})}")
endforeach()

list(LENGTH TEST_WAV_ENTRIES TEST_WAV_COUNT)
if(TEST_WAV_COUNT GREATER 0)
    file(APPEND ${TEST_WAV_GEN_CPP} "static const EmbeddedTestWav g_test_wavs_storage[] = {\n")
    foreach(TEST_WAV_ENTRY ${TEST_WAV_ENTRIES})
        file(APPEND ${TEST_WAV_GEN_CPP} "    ${TEST_WAV_ENTRY},\n")
    endforeach()
    file(APPEND ${TEST_WAV_GEN_CPP} "};\n\n")
    file(APPEND ${TEST_WAV_GEN_CPP}
    "const EmbeddedTestWav* g_test_wavs = g_test_wavs_storage;\n"
    "const size_t g_test_wavs_count = sizeof(g_test_wavs_storage) / sizeof(g_test_wavs_storage[0]);\n")
else()
    file(APPEND ${TEST_WAV_GEN_CPP}
    "const EmbeddedTestWav* g_test_wavs = nullptr;\n"
    "const size_t g_test_wavs_count = 0;\n")
endif()

# TFLite model files
generate_inc_file_for_target(app
    ${TFLM_DIR}/tensorflow/lite/micro/examples/micro_speech/models/audio_preprocessor_int8.tflite
    ${gen_dir}/audio_preprocessor_int8.tflite.inc)

# Include path for generated headers
target_include_directories(app PRIVATE ${gen_dir})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# -- Signal processing kernels not compiled by default in the Zephyr module --
# The audio preprocessor model needs these additional signal ops.
set(SIG_KERN ${TFLM_DIR}/signal/micro/kernels)
set(SIG_SRC  ${TFLM_DIR}/signal/src)

target_sources(app PRIVATE
    # Application sources
    src/main.cpp
    ${TEST_WAV_GEN_CPP}
    model.cc

    # Signal micro kernels (ops needed by audio_preprocessor_int8.tflite)
    ${SIG_KERN}/energy.cc
    ${SIG_KERN}/energy_flexbuffers_generated_data.cc
    ${SIG_KERN}/fft_auto_scale_common.cc
    ${SIG_KERN}/fft_auto_scale_kernel.cc
    ${SIG_KERN}/filter_bank.cc
    ${SIG_KERN}/filter_bank_flexbuffers_generated_data.cc
    ${SIG_KERN}/filter_bank_log.cc
    ${SIG_KERN}/filter_bank_log_flexbuffers_generated_data.cc
    ${SIG_KERN}/filter_bank_spectral_subtraction.cc
    ${SIG_KERN}/filter_bank_spectral_subtraction_flexbuffers_generated_data.cc
    ${SIG_KERN}/filter_bank_square_root.cc
    ${SIG_KERN}/filter_bank_square_root_common.cc
    ${SIG_KERN}/pcan.cc
    ${SIG_KERN}/pcan_flexbuffers_generated_data.cc
    ${SIG_SRC}/circular_buffer.cc
    ${SIG_SRC}/energy.cc
    ${SIG_SRC}/fft_auto_scale.cc
    ${SIG_SRC}/filter_bank.cc
    ${SIG_SRC}/filter_bank_log.cc
    ${SIG_SRC}/filter_bank_spectral_subtraction.cc
    ${SIG_SRC}/filter_bank_square_root.cc
    ${SIG_SRC}/log.cc
    ${SIG_SRC}/max_abs.cc
    ${SIG_SRC}/msb_32.cc
    ${SIG_SRC}/msb_64.cc
    ${SIG_SRC}/pcan_argc_fixed.cc
    ${SIG_SRC}/square_root_32.cc
    ${SIG_SRC}/square_root_64.cc
)
