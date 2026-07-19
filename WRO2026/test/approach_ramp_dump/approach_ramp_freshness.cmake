# =============================================================================
# approach_ramp_freshness.cmake
#
# The T3 "executable tether" freshness gate (2026-07-17 anticipatory-line-approach-ramp
# plan). Invoked via `add_test(... COMMAND ${CMAKE_COMMAND} -P approach_ramp_freshness.cmake
# ...)` with three `-D` variables set by CMakeLists.txt:
#   DUMP_EXE     - path to the built `approach_ramp_dump` binary.
#   GOLDEN_FILE  - path to the committed
#                  `tools/path_planner/tests/fixtures/approach_ramp_goldens.json`.
#   OUT_FILE     - scratch path (inside the build tree) to capture a FRESH run's stdout.
#
# Mirrors the `routes.h` / `tshape_profile` / `chain_speed` CI freshness pattern (regenerate,
# then `git diff --exit-code`) but implemented as a ctest so it runs locally too, not just in
# a dedicated CI job: RUN the dump binary fresh, write its stdout to OUT_FILE, then
# BYTE-COMPARE it against the already-committed GOLDEN_FILE via
# `${CMAKE_COMMAND} -E compare_files`. Any mismatch (a firmware retune that changed
# `Config::TSHAPE_*` / `Config::LINE_APPROACH_*` or a change to the policy math itself) fails
# this test with both paths printed so a human can `diff` them directly. This is NOT a
# dump-only target — the committed fixture is regenerated-and-compared on every ctest run,
# never merely produced once and trusted forever. The Python cross-language test (T5 / TC13)
# reads the SAME fixture, so this gate keeps the C++ and Python sides pinned to one another.
# =============================================================================

if(NOT DEFINED DUMP_EXE)
    message(FATAL_ERROR "approach_ramp_freshness.cmake: DUMP_EXE not set")
endif()
if(NOT DEFINED GOLDEN_FILE)
    message(FATAL_ERROR "approach_ramp_freshness.cmake: GOLDEN_FILE not set")
endif()
if(NOT DEFINED OUT_FILE)
    message(FATAL_ERROR "approach_ramp_freshness.cmake: OUT_FILE not set")
endif()

if(NOT EXISTS "${GOLDEN_FILE}")
    message(FATAL_ERROR
        "approach_ramp_freshness: committed golden fixture is missing: ${GOLDEN_FILE}\n"
        "Run the dump binary once and commit its stdout as this file.")
endif()

# Regenerate: run the dump binary fresh, capture stdout to OUT_FILE.
execute_process(
    COMMAND "${DUMP_EXE}"
    OUTPUT_FILE "${OUT_FILE}"
    RESULT_VARIABLE _dump_rc
)
if(NOT _dump_rc EQUAL 0)
    message(FATAL_ERROR "approach_ramp_freshness: ${DUMP_EXE} exited with code ${_dump_rc}")
endif()

# Compare: byte-equal against the committed fixture.
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUT_FILE}" "${GOLDEN_FILE}"
    RESULT_VARIABLE _cmp_rc
)
if(NOT _cmp_rc EQUAL 0)
    message(FATAL_ERROR
        "approach_ramp_freshness: FRESHLY REGENERATED goldens differ from the committed "
        "fixture.\n"
        "  Fresh:     ${OUT_FILE}\n"
        "  Committed: ${GOLDEN_FILE}\n"
        "This means Config::TSHAPE_* / Config::LINE_APPROACH_* (or ApproachRampPolicy.h "
        "itself) changed since the fixture was last regenerated. Re-run the "
        "approach_ramp_dump binary and commit its stdout as "
        "tools/path_planner/tests/fixtures/approach_ramp_goldens.json.")
endif()

message(STATUS "approach_ramp_freshness: OK — goldens match the committed fixture")
