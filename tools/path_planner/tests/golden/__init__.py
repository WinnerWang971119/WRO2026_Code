"""Golden snapshot package for header_writer tests.

The bytes in ``routes_toy.h`` and ``routes_toy.cpp`` are the canonical
serialisation of the toy field_data fixture. Tests compare emitted
output against these files byte-for-byte to lock the C++ schema.

Regenerate when the schema legitimately changes by re-running the
header writer against the toy field and copying the output here. Do
not hand-edit.
"""
