# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
#
# Build of the canceller harness. The file
# runs in two ways:
#
# - from src/CMakeLists.txt under UDSP_BENCH, on the canceller source of the
#   cache path UDSP_REPLAY_LEC_C. It builds lec_hz, lec_replay_coh and
#   lec_tone_sweep beside the lec_replay of that build;
# - as a project of its own, for one canceller variant, with the cache path
#   UDSP_HARNESS_LEC_C. It then also builds lec_replay and test_lec on that
#   source, and bench_lec when pkg-config finds speexdsp. lec_harness.py
#   build configures it so.
#
# Three build steps write instrumented copies into the build directory and
# never touch a source: patch_hz.py for lec_hz, patch_coh.py for
# lec_replay_coh and patch_sweep.py for lec_tone_sweep. A copy whose anchor
# is gone fails its step, so the build reports a canceller source that the
# hooks do not fit.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	cmake_minimum_required(VERSION 3.13)
	project(udsp_lec_harness C)
	set(CMAKE_C_STANDARD 11)
	set(CMAKE_C_STANDARD_REQUIRED ON)
	# The options of src/CMakeLists.txt less -Werror: a variant rule can
	# leave a helper that the variant does not call.
	add_compile_options(-Os -Wall -Wextra -Wmissing-declarations
			    -Wno-unused-parameter -D_GNU_SOURCE)
	set(HARNESS_STANDALONE ON)
	get_filename_component(HARNESS_UDSP_SRC
			       ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
	set(UDSP_HARNESS_LEC_C "${HARNESS_UDSP_SRC}/dsp/lec.c" CACHE FILEPATH
	    "the canceller source of the harness build")
	set(HARNESS_LEC_C ${UDSP_HARNESS_LEC_C})
	include_directories(${HARNESS_UDSP_SRC}/include)
else()
	set(HARNESS_STANDALONE OFF)
	set(HARNESS_UDSP_SRC ${PROJECT_SOURCE_DIR})
	get_filename_component(HARNESS_LEC_C ${UDSP_REPLAY_LEC_C} ABSOLUTE
			       BASE_DIR ${PROJECT_SOURCE_DIR})
endif()

find_program(HARNESS_PYTHON3 python3)
if(NOT HARNESS_PYTHON3)
	message(STATUS "python3 not found: the canceller harness is skipped")
	return()
endif()

set(HARNESS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(HARNESS_BIN ${CMAKE_CURRENT_BINARY_DIR})
set(HARNESS_DSP ${HARNESS_UDSP_SRC}/dsp)
set(HARNESS_TEST ${HARNESS_UDSP_SRC}/test)

add_custom_command(OUTPUT ${HARNESS_BIN}/lec_hz.c
		   COMMAND ${HARNESS_PYTHON3} ${HARNESS_DIR}/patch_hz.py
			   ${HARNESS_LEC_C} ${HARNESS_BIN}/lec_hz.c
		   DEPENDS ${HARNESS_LEC_C} ${HARNESS_DIR}/patch_hz.py
		   COMMENT "Instrumenting the canceller source for lec_hz")
add_custom_command(OUTPUT ${HARNESS_BIN}/lec_coh.c
		   COMMAND ${HARNESS_PYTHON3} ${HARNESS_DIR}/patch_coh.py
			   ${HARNESS_LEC_C} ${HARNESS_BIN}/lec_coh.c
		   DEPENDS ${HARNESS_LEC_C} ${HARNESS_DIR}/patch_coh.py
		   COMMENT "Instrumenting the canceller source for lec_replay_coh")
add_custom_command(OUTPUT ${HARNESS_BIN}/tone_sweep.c
		   COMMAND ${HARNESS_PYTHON3} ${HARNESS_DIR}/patch_sweep.py
			   ${HARNESS_TEST}/test_lec.c ${HARNESS_DIR}/sweep_tail.c
			   ${HARNESS_BIN}/tone_sweep.c
		   DEPENDS ${HARNESS_TEST}/test_lec.c ${HARNESS_DIR}/sweep_tail.c
			   ${HARNESS_DIR}/patch_sweep.py
		   COMMENT "Writing the source of lec_tone_sweep")

# hz.c includes the instrumented copy whole, so the copy is a dependency of
# the target and not a source of its own.
set_source_files_properties(${HARNESS_BIN}/lec_hz.c PROPERTIES
			    HEADER_FILE_ONLY ON GENERATED ON)
add_executable(lec_hz ${HARNESS_DIR}/hz.c ${HARNESS_BIN}/lec_hz.c
	       ${HARNESS_DSP}/cn.c ${HARNESS_DSP}/resamp.c)
target_compile_definitions(lec_hz PRIVATE
			   LEC_HZ_C="${HARNESS_BIN}/lec_hz.c")
target_include_directories(lec_hz PRIVATE ${HARNESS_DSP})
target_link_libraries(lec_hz m)

add_executable(lec_replay_coh ${HARNESS_TEST}/bench/lec_replay.c
	       ${HARNESS_BIN}/lec_coh.c ${HARNESS_DSP}/cn.c
	       ${HARNESS_DSP}/resamp.c)
target_include_directories(lec_replay_coh PRIVATE ${HARNESS_DSP})
target_link_libraries(lec_replay_coh m)

add_executable(lec_tone_sweep ${HARNESS_BIN}/tone_sweep.c ${HARNESS_LEC_C}
	       ${HARNESS_DSP}/cn.c ${HARNESS_DSP}/resamp.c)
target_include_directories(lec_tone_sweep PRIVATE ${HARNESS_TEST}
			   ${HARNESS_DSP})
target_link_libraries(lec_tone_sweep m)

if(HARNESS_STANDALONE)
	add_executable(lec_replay ${HARNESS_TEST}/bench/lec_replay.c
		       ${HARNESS_LEC_C} ${HARNESS_DSP}/cn.c
		       ${HARNESS_DSP}/resamp.c)
	target_include_directories(lec_replay PRIVATE ${HARNESS_DSP})
	target_link_libraries(lec_replay m)

	add_executable(test_lec ${HARNESS_TEST}/test_lec.c ${HARNESS_LEC_C}
		       ${HARNESS_DSP}/cn.c ${HARNESS_DSP}/resamp.c)
	target_include_directories(test_lec PRIVATE ${HARNESS_TEST}
				   ${HARNESS_DSP})
	target_link_libraries(test_lec m)

	find_package(PkgConfig QUIET)
	if(PKG_CONFIG_FOUND)
		pkg_check_modules(SPEEXDSP QUIET speexdsp)
	endif()
	if(SPEEXDSP_FOUND)
		add_executable(bench_lec ${HARNESS_TEST}/bench/bench_lec.c
			       ${HARNESS_LEC_C} ${HARNESS_DSP}/cn.c
			       ${HARNESS_DSP}/resamp.c)
		target_include_directories(bench_lec PRIVATE ${HARNESS_DSP}
					   ${SPEEXDSP_INCLUDE_DIRS})
		target_link_directories(bench_lec PRIVATE
					${SPEEXDSP_LIBRARY_DIRS})
		target_link_libraries(bench_lec m ${SPEEXDSP_LIBRARIES})
	else()
		message(STATUS "speexdsp not found: bench_lec skipped")
	endif()
endif()
