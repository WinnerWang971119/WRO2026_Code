# =============================================================================
# chain_speed_freshness.cmake
#
# The T4 "executable tether" freshness gate (ADR-059). Invoked via
# `add_test(... COMMAND ${CMAKE_COMMAND} -P chain_speed_freshness.cmake ...)`
# with three `-D` variables set by CMakeLists.txt:
#   DUMP_EXE     - path to the built `chain_speed_dump` binary.
#   GOLDEN_FILE  - path to the committed
#                  `tools/path_planner/tests/fixtures/chain_speed_goldens.json`.
#   OUT_FILE     - scratch path (inside the build tree) to capture a FRESH run's stdout.
#
# Mirrors the `routes.h` / `tshape_profile` CI freshness pattern (regenerate, then
# byte-compare) but implemented as a ctest so it runs locally too, not just in a dedicated CI
# job: RUN the dump binary fresh, write its stdout to OUT_FILE, then BYTE-COMPARE it against the
# already-committed GOLDEN_FILE via `${CMAKE_COMMAND} -E compare_files`. Any mismatch (a firmware
# retune that changed `Config::TSHAPE_ACCEL_PWM_PER_CM` / `PWM_MIN_MAGNITUDE` or a change to the
# `ChainSpeedPolicy.h` math itself) fails this test with both paths printed so a human can `diff`
# them directly. The committed fixture is regenerated-and-compared on every ctest run, never
# merely produced once and trusted forever. The Python mirror `chain_speed.py` (T5) reads the
# SAME fixture, so this gate keeps the C++ and Python sides pinned to one another.
# =============================================================================

if(NOT DEFINED DUMP_EXE)
    message(FATAL_ERROR "chain_speed_freshness.cmake: DUMP_EXE not set")
endif()
if(NOT DEFINED GOLDEN_FILE)
    message(FATAL_ERROR "chain_speed_freshness.cmake: GOLDEN_FILE not set")
endif()
if(NOT DEFINED OUT_FILE)
    message(FATAL_ERROR "chain_speed_freshness.cmake: OUT_FILE not set")
endif()

if(NOT EXISTS "${GOLDEN_FILE}")
    message(FATAL_ERROR
        "chain_speed_freshness: committed golden fixture is missing: ${GOLDEN_FILE}\n"
        "Run the dump binary once and commit its stdout as this file.")
endif()

# Regenerate: run the dump binary fresh, capture stdout to OUT_FILE.
execute_process(
    COMMAND "${DUMP_EXE}"
    OUTPUT_FILE "${OUT_FILE}"
    RESULT_VARIABLE _dump_rc
)
if(NOT _dump_rc EQUAL 0)
    message(FATAL_ERROR "chain_speed_freshness: ${DUMP_EXE} exited with code ${_dump_rc}")
endif()

# Compare: byte-equal against the committed fixture.
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUT_FILE}" "${GOLDEN_FILE}"
    RESULT_VARIABLE _cmp_rc
)
if(NOT _cmp_rc EQUAL 0)
    message(FATAL_ERROR
        "chain_speed_freshness: FRESHLY REGENERATED goldens differ from the committed "
        "fixture.\n"
        "  Fresh:     ${OUT_FILE}\n"
        "  Committed: ${GOLDEN_FILE}\n"
        "This means Config::TSHAPE_ACCEL_PWM_PER_CM / PWM_MIN_MAGNITUDE (or ChainSpeedPolicy.h "
        "itself) changed since the fixture was last regenerated. Re-run the chain_speed_dump "
        "binary and commit its stdout as "
        "tools/path_planner/tests/fixtures/chain_speed_goldens.json.")
endif()

message(STATUS "chain_speed_freshness: OK — goldens match the committed fixture")
