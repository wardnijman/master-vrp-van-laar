cmake_minimum_required(VERSION 3.16)
project(sa_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -march=native")


# Path to your static OR-Tools build
set(ORTOOLS_ROOT "${CMAKE_SOURCE_DIR}/../or-tools/build/install")

set(ORTOOLS_STATIC_LIBS
  ${ORTOOLS_ROOT}/lib/libCbc.a
  ${ORTOOLS_ROOT}/lib/libCbcSolver.a
  ${ORTOOLS_ROOT}/lib/libCgl.a
  ${ORTOOLS_ROOT}/lib/libClp.a
  ${ORTOOLS_ROOT}/lib/libClpSolver.a
  ${ORTOOLS_ROOT}/lib/libCoinUtils.a
  ${ORTOOLS_ROOT}/lib/libOsi.a
  ${ORTOOLS_ROOT}/lib/libOsiCbc.a
  ${ORTOOLS_ROOT}/lib/libOsiClp.a
  ${ORTOOLS_ROOT}/lib/libabsl_bad_any_cast_impl.a
  ${ORTOOLS_ROOT}/lib/libabsl_bad_optional_access.a
  ${ORTOOLS_ROOT}/lib/libabsl_bad_variant_access.a
  ${ORTOOLS_ROOT}/lib/libabsl_base.a
  ${ORTOOLS_ROOT}/lib/libabsl_city.a
  ${ORTOOLS_ROOT}/lib/libabsl_civil_time.a
  ${ORTOOLS_ROOT}/lib/libabsl_cord.a
  ${ORTOOLS_ROOT}/lib/libabsl_cord_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_cordz_functions.a
  ${ORTOOLS_ROOT}/lib/libabsl_cordz_handle.a
  ${ORTOOLS_ROOT}/lib/libabsl_cordz_info.a
  ${ORTOOLS_ROOT}/lib/libabsl_cordz_sample_token.a
  ${ORTOOLS_ROOT}/lib/libabsl_crc32c.a
  ${ORTOOLS_ROOT}/lib/libabsl_crc_cord_state.a
  ${ORTOOLS_ROOT}/lib/libabsl_crc_cpu_detect.a
  ${ORTOOLS_ROOT}/lib/libabsl_crc_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_debugging_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_demangle_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_die_if_null.a
  ${ORTOOLS_ROOT}/lib/libabsl_examine_stack.a
  ${ORTOOLS_ROOT}/lib/libabsl_exponential_biased.a
  ${ORTOOLS_ROOT}/lib/libabsl_failure_signal_handler.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_commandlineflag.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_commandlineflag_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_config.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_marshalling.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_parse.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_private_handle_accessor.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_program_name.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_reflection.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_usage.a
  ${ORTOOLS_ROOT}/lib/libabsl_flags_usage_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_graphcycles_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_hash.a
  ${ORTOOLS_ROOT}/lib/libabsl_hashtablez_sampler.a
  ${ORTOOLS_ROOT}/lib/libabsl_int128.a
  ${ORTOOLS_ROOT}/lib/libabsl_leak_check.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_entry.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_flags.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_globals.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_initialize.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_check_op.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_conditions.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_format.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_globals.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_log_sink_set.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_message.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_nullguard.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_internal_proto.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_severity.a
  ${ORTOOLS_ROOT}/lib/libabsl_log_sink.a
  ${ORTOOLS_ROOT}/lib/libabsl_low_level_hash.a
  ${ORTOOLS_ROOT}/lib/libabsl_malloc_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_periodic_sampler.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_distributions.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_distribution_test_util.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_platform.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_pool_urbg.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_randen.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_randen_hwaes.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_randen_hwaes_impl.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_randen_slow.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_internal_seed_material.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_seed_gen_exception.a
  ${ORTOOLS_ROOT}/lib/libabsl_random_seed_sequences.a
  ${ORTOOLS_ROOT}/lib/libabsl_raw_hash_set.a
  ${ORTOOLS_ROOT}/lib/libabsl_raw_logging_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_scoped_set_env.a
  ${ORTOOLS_ROOT}/lib/libabsl_spinlock_wait.a
  ${ORTOOLS_ROOT}/lib/libabsl_stacktrace.a
  ${ORTOOLS_ROOT}/lib/libabsl_status.a
  ${ORTOOLS_ROOT}/lib/libabsl_statusor.a
  ${ORTOOLS_ROOT}/lib/libabsl_str_format_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_strerror.a
  ${ORTOOLS_ROOT}/lib/libabsl_strings.a
  ${ORTOOLS_ROOT}/lib/libabsl_strings_internal.a
  ${ORTOOLS_ROOT}/lib/libabsl_symbolize.a
  ${ORTOOLS_ROOT}/lib/libabsl_synchronization.a
  ${ORTOOLS_ROOT}/lib/libabsl_throw_delegate.a
  ${ORTOOLS_ROOT}/lib/libabsl_time.a
  ${ORTOOLS_ROOT}/lib/libabsl_time_zone.a
  ${ORTOOLS_ROOT}/lib/libgmock.a
  ${ORTOOLS_ROOT}/lib/libgmock_main.a
  ${ORTOOLS_ROOT}/lib/libgtest.a
  ${ORTOOLS_ROOT}/lib/libgtest_main.a
  ${ORTOOLS_ROOT}/lib/libortools.a
  ${ORTOOLS_ROOT}/lib/libortools_flatzinc.a
  ${ORTOOLS_ROOT}/lib/libprotobuf-lite.a
  ${ORTOOLS_ROOT}/lib/libprotobuf.a
  ${ORTOOLS_ROOT}/lib/libprotoc.a
  ${ORTOOLS_ROOT}/lib/libre2.a
  ${ORTOOLS_ROOT}/lib/libscip.a
  ${ORTOOLS_ROOT}/lib/libutf8_range.a
  ${ORTOOLS_ROOT}/lib/libutf8_validity.a
  ${ORTOOLS_ROOT}/lib/libz.a
)

# Include dirs
include_directories(
  ${ORTOOLS_ROOT}/include
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/lib
)

# Link dirs
link_directories(${ORTOOLS_ROOT}/lib)

# Collect all .a files (order matters: protobuf and absl should go after ortools)
file(GLOB ORTOOLS_STATIC_LIBS "${ORTOOLS_ROOT}/lib/*.a")

# Executable
add_executable(sa_solver
  src/main.cpp
  src/cache.cpp
  src/validation.cpp
  src/day_solver.cpp
  src/penalties.cpp
  src/sa.cpp
  src/schedule_state.cpp
  src/seed_generators.cpp
  src/schedule_state.cpp
  src/seed.cpp
  src/first_solution_parser.cpp
  src/first_schedule_solver.cpp
)

# set(CMAKE_BUILD_TYPE Debug)
# target_compile_options(sa_solver PRIVATE -O0 -g -fno-omit-frame-pointer)

if(APPLE)
  set(ORTOOLS_LINK_MODE "MACOS")
  target_link_libraries(sa_solver "-framework CoreFoundation")
else()
  set(ORTOOLS_LINK_MODE "LINUX")
endif()

if(ORTOOLS_LINK_MODE STREQUAL "MACOS")
  target_link_libraries(sa_solver
    ${ORTOOLS_STATIC_LIBS}
    pthread
  )
else()
  target_link_libraries(sa_solver
    -Wl,--start-group
      ${ORTOOLS_STATIC_LIBS}
      pthread
    -Wl,--end-group
  )
endif()
