# Stable anchor file for the toy golden fixtures' Source SHA256 stamp.
#
# This file is NOT imported by the test suite -- it is read as raw bytes
# by `compute_source_hash()` so the golden routes_toy.h carries a
# deterministic Source SHA256 line. The actual toy field-data graph is
# constructed in Python by `_build_toy_field_data()` inside
# `test_header_writer.py`.
#
# Editing this file (even whitespace) invalidates the SHA256 baked into
# routes_toy.h and forces a golden rebaseline. Don't touch unless you
# also regenerate the goldens.
