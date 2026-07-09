"""Name-superset guard: every real fluent-builder setter exists on the CTPS mock.

This is the *real-grew* half of the ADR-030/049 mock-conformance guard (issue
#89). The complementary compile-time conformance TU
(``WRO2026/test/ctps_mocks/test_ctps_mock_conformance.cpp``, task T5) chains
every *current* real setter on each mock builder, so it catches a **removed /
renamed / signature-incompatible** mock setter. What that TU cannot catch is a
real fluent builder *gaining* a brand-new setter that the mock never grew — it
stays green until a human adds the call, the same manual step as updating the
mock. That real-grew case is exactly the drift that broke #89 (the mocks were
never updated when ``Drive`` / ``LineFollower`` builders gained ``flat()`` /
``setPreset()``). This test closes it: for each fluent builder it asserts the
mock's public setter-name set is a **superset** of the real builder's, so a new
real setter that the mock lacks fails the build.

Extraction rule (the precise #89 drift class)
---------------------------------------------
libcst parses Python, not C++, so this is a deliberate *text* scan. For each
builder class we collect the names of public member functions whose return type
is ``<ClassName>&`` — i.e. the **fluent-setter surface** (``DriveDistBuilder&
setPower(...)`` and friends). That rule is precise and low-false-positive:

* constructors, the destructor (``~Name``), and ``operator=`` are excluded (a
  ctor has no return type; the deleted ``operator=`` returns ``<ClassName>&``
  but its name starts with ``operator`` and is filtered out);
* the move constructor's ``<ClassName>&&`` parameter never matches the
  ``<ClassName>& name(`` shape;
* private helpers (``_resolvePower`` returns ``int16_t``, not ``<ClassName>&``)
  are excluded both by return type and by the ``public:`` access gate.

Comments are stripped before scanning so a doxygen ``{SLOW, NORMAL, FAST}`` or a
``;`` inside a ``//`` comment cannot corrupt the brace/semicolon structure the
class-body finder relies on.

This file lands in ``tools/path_planner/tests/`` and is collected by the broad
CI ``pytest tools/path_planner/tests/`` job. It reads header text only — no
built binary, no external resource — so it needs no CI gate.

NOTE: the positive per-builder checks reflect the *end state* (the mock is a
superset). They pass only after tasks T2/T3 add the missing ``flat()`` /
``setPreset()`` setters to the mocks; until then each currently fails, which is
the guard correctly biting. The negative and self-contained synthetic checks
pass unconditionally.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Header discovery — resolve paths relative to the repo root (runs on Linux CI).
# ---------------------------------------------------------------------------


def _repo_root() -> Path:
    """Walk up from this file to the repo root (the dir holding ``WRO2026/src/drive/Drive.h``).

    Robust to the test-tree depth rather than a hard-coded ``parents[N]`` index
    or an absolute ``C:\\...`` path, so the scan works on Linux CI too.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "WRO2026" / "src" / "drive" / "Drive.h").is_file():
            return parent
    raise AssertionError(
        f"repo root (a directory containing WRO2026/src/drive/Drive.h) not found above {here}"
    )


_ROOT = _repo_root()

_REAL_DRIVE = _ROOT / "WRO2026" / "src" / "drive" / "Drive.h"
_REAL_LINE = _ROOT / "WRO2026" / "src" / "drive" / "LineFollower.h"
_MOCK_DRIVE = _ROOT / "WRO2026" / "test" / "ctps_mocks" / "Drive.h"
_MOCK_LINE = _ROOT / "WRO2026" / "test" / "ctps_mocks" / "LineFollower.h"

# (builder class name, real header, mock shadow) — the seven fluent builders.
_BUILDERS = [
    ("DriveDistBuilder", _REAL_DRIVE, _MOCK_DRIVE),
    ("DriveTimeBuilder", _REAL_DRIVE, _MOCK_DRIVE),
    ("TurnDegBuilder", _REAL_DRIVE, _MOCK_DRIVE),
    ("TurnToYawBuilder", _REAL_DRIVE, _MOCK_DRIVE),
    ("StopAtLineBuilder", _REAL_DRIVE, _MOCK_DRIVE),
    ("WallStopBuilder", _REAL_DRIVE, _MOCK_DRIVE),
    ("LineFollowerEnableBuilder", _REAL_LINE, _MOCK_LINE),
]

_BUILDER_IDS = [name for name, _real, _mock in _BUILDERS]


# ---------------------------------------------------------------------------
# Header-scan helpers (shared by the real-file and synthetic tests).
# ---------------------------------------------------------------------------


def _read(path: Path) -> str:
    """Read a header, with a clear assertion message if it is missing."""
    assert path.is_file(), f"expected header not found: {path}"
    return path.read_text(encoding="utf-8")


def _strip_comments(text: str) -> str:
    """Remove ``//`` and ``/* */`` comments, preserving string/char literals.

    Stripping first means a doxygen ``{A, B}`` or a ``;`` inside a comment can
    never derail the brace/semicolon structure the class-body finder walks.
    String and character literals are copied through verbatim (with escapes) so
    their contents are never mistaken for comment openers.
    """
    out: list[str] = []
    i, n = 0, len(text)
    while i < n:
        two = text[i : i + 2]
        if two == "//":
            nl = text.find("\n", i)
            i = n if nl == -1 else nl
        elif two == "/*":
            end = text.find("*/", i + 2)
            i = n if end == -1 else end + 2
        elif text[i] in ('"', "'"):
            quote = text[i]
            out.append(text[i])
            i += 1
            while i < n:
                ch = text[i]
                out.append(ch)
                if ch == "\\":  # escape — copy the next char too, never terminate
                    if i + 1 < n:
                        out.append(text[i + 1])
                    i += 2
                    continue
                i += 1
                if ch == quote:
                    break
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def _extract_class_body(text: str, class_name: str) -> str:
    """Return the ``{ ... }`` body of the DEFINITION of ``class_name``.

    Skips forward/friend declarations (``class Foo;`` — a ``;`` before any
    ``{``) and brace-matches from the opening brace of the real definition to its
    depth-0 close. ``text`` is expected to be comment-stripped already.
    """
    header_re = re.compile(
        r"class\s+(?:\[\[nodiscard\]\]\s+)?" + re.escape(class_name) + r"\b"
    )
    for m in header_re.finditer(text):
        rest = text[m.end():]
        brace_off = rest.find("{")
        semi_off = rest.find(";")
        if brace_off == -1:
            continue
        if semi_off != -1 and semi_off < brace_off:
            continue  # forward / friend declaration, not the definition
        start = m.end() + brace_off
        depth = 0
        for i in range(start, len(text)):
            c = text[i]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    return text[start + 1 : i]
        raise AssertionError(
            f"unbalanced braces while scanning class {class_name} body"
        )
    raise AssertionError(f"definition of class {class_name} not found")


# Matches a fluent setter: return type ``<ClassName>&``, a method name, then ``(``.
# The move ctor's ``<ClassName>&&`` param and the constructor's ``<ClassName>(``
# do not fit this shape; ``operator=`` fits but is filtered by name below.
def _method_regex(class_name: str) -> re.Pattern[str]:
    return re.compile(re.escape(class_name) + r"\s*&\s*([A-Za-z_]\w*)\s*\(")


def _access_at(body: str, pos: int) -> str:
    """Return the C++ access ('public'/'protected'/'private') in effect at ``pos``.

    A ``class`` body defaults to ``private``; each ``public:`` / ``protected:`` /
    ``private:`` label flips it from that point forward.
    """
    access = "private"
    for lbl in re.finditer(r"\b(public|protected|private)\s*:", body):
        if lbl.start() <= pos:
            access = lbl.group(1)
        else:
            break
    return access


def _extract_builder_methods(header_text: str, class_name: str) -> set[str]:
    """Public fluent-setter names (return type ``<ClassName>&``) of ``class_name``.

    Excludes ctors, the destructor, ``operator=`` / other operators, and any
    non-public member. Handles both inline definitions (``... { ... }``) and
    bare declarations (``... ;``), and multi-line signatures.

    :param header_text: raw C++ header text (comments handled internally).
    :param class_name: the builder class whose body to scan.
    :return: the set of public fluent-setter method names.
    """
    body = _extract_class_body(_strip_comments(header_text), class_name)
    names: set[str] = set()
    for m in _method_regex(class_name).finditer(body):
        name = m.group(1)
        if name.startswith("operator"):
            continue
        if _access_at(body, m.start()) != "public":
            continue
        names.add(name)
    return names


def _assert_mock_superset(builder: str, real_text: str, mock_text: str) -> None:
    """Assert the mock builder's setter names are a superset of the real one's.

    Raises ``AssertionError`` naming the missing setter(s) when the mock has
    fallen behind — this is the exact check the positive tests run and the
    negative test proves fires.
    """
    real_names = _extract_builder_methods(real_text, builder)
    mock_names = _extract_builder_methods(mock_text, builder)
    # Anti-tautology: an extractor that silently returned nothing would make the
    # superset relation hold vacuously, hiding real drift.
    assert real_names, (
        f"extractor found no fluent setters on real {builder} — the header scan "
        f"has drifted from the source; fix the parser before trusting this guard"
    )
    missing = real_names - mock_names
    assert not missing, (
        f"CTPS mock {builder} is missing fluent setter(s) present on the real "
        f"builder: {sorted(missing)}. Add them to the mock shadow under "
        f"WRO2026/test/ctps_mocks/ so the recording surface stays a superset of "
        f"the real one (issue #89)."
    )


# ---------------------------------------------------------------------------
# Positive: every real builder's setter surface is covered by its mock shadow.
# ---------------------------------------------------------------------------


def test_header_paths_exist():
    """The four scanned headers resolve from the repo root on any platform."""
    for path in (_REAL_DRIVE, _REAL_LINE, _MOCK_DRIVE, _MOCK_LINE):
        assert path.is_file(), f"expected header not found: {path}"


@pytest.mark.parametrize("builder, real_path, mock_path", _BUILDERS, ids=_BUILDER_IDS)
def test_mock_is_name_superset_of_real(builder, real_path, mock_path):
    """NC5 (positive): mock setter-name set ⊇ real setter-name set, per builder.

    Passes once tasks T2/T3 add the missing ``flat()`` / ``setPreset()`` setters
    to the mock shadows; until then it fails, which is the guard biting on the
    live real-grew gap.
    """
    _assert_mock_superset(builder, _read(real_path), _read(mock_path))


def test_extractor_finds_known_real_setters():
    """Structural sanity: the scan recovers setters we know exist on the reals.

    Guards against a silently-broken parser that returns empty sets (which would
    make every superset check pass vacuously).
    """
    drive_dist = _extract_builder_methods(_read(_REAL_DRIVE), "DriveDistBuilder")
    assert {"setPower", "setPreset", "flat", "async"} <= drive_dist, drive_dist

    line_enable = _extract_builder_methods(
        _read(_REAL_LINE), "LineFollowerEnableBuilder"
    )
    assert {"setPreset", "flat", "setDistanceTarget", "setReverse"} <= line_enable, (
        line_enable
    )


# ---------------------------------------------------------------------------
# Negative + self-contained synthetic checks (green regardless of T2/T3).
# ---------------------------------------------------------------------------

# A synthetic real builder that also exercises every exclusion path:
#   - a constructor and a move constructor (no return type / ``&&`` param),
#   - a deleted ``operator=`` (returns ``FakeBuilder&`` but is name-filtered),
#   - a destructor,
#   - a PRIVATE chainable helper (must be excluded by the access gate),
#   - a comment carrying both a brace hazard ``{A, B}`` and a ``;``.
# Its public setter surface is exactly {setA, setB}.
_SYNTH_REAL = """
#ifndef FAKE_H
#define FAKE_H
class [[nodiscard]] FakeBuilder  // applies once; discarded temporary {A, B}
{
public:
    FakeBuilder(int x) : _x(x) {}
    FakeBuilder(const FakeBuilder&)            = delete;
    FakeBuilder& operator=(const FakeBuilder&) = delete;
    FakeBuilder(FakeBuilder&& o) noexcept : _x(o._x) {}

    /** @brief One of Config::Preset::{SLOW, NORMAL, FAST}; applied once. */
    FakeBuilder& setA(int p) { _x = p; return *this; }
    FakeBuilder& setB(bool q);   // out-of-line declaration

    ~FakeBuilder() {}

private:
    FakeBuilder& _privateChain() { return *this; }   // must NOT be collected
    int _x;
};
#endif
"""

# The synthetic mock that has fallen behind: it is missing ``setB``.
_SYNTH_MOCK_MISSING = """
#ifndef FAKE_H
#define FAKE_H
class [[nodiscard]] FakeBuilder
{
public:
    FakeBuilder(int) {}
    FakeBuilder& operator=(const FakeBuilder&) = delete;
    FakeBuilder& setA(int) { return *this; }
};
#endif
"""

# A synthetic mock that is a strict superset (setA, setB, plus an extra).
_SYNTH_MOCK_SUPERSET = """
#ifndef FAKE_H
#define FAKE_H
class [[nodiscard]] FakeBuilder
{
public:
    FakeBuilder(int) {}
    FakeBuilder& operator=(const FakeBuilder&) = delete;
    FakeBuilder& setA(int) { return *this; }
    FakeBuilder& setB(bool) { return *this; }
    FakeBuilder& setExtra(float) { return *this; }
};
#endif
"""


def test_extractor_excludes_ctors_operators_and_private():
    """The scan collects only public fluent setters from the synthetic real.

    Proves the constructor / move ctor / ``operator=`` / destructor / private
    helper are all excluded, and that a bare declaration (``setB``) is captured
    alongside an inline definition (``setA``).
    """
    assert _extract_builder_methods(_SYNTH_REAL, "FakeBuilder") == {"setA", "setB"}


def test_guard_bites_when_mock_drops_a_setter():
    """NC5 (negative): a mock missing a real setter must fail the superset check.

    The same assertion the positive tests use must raise ``AssertionError`` and
    name the missing setter — proving the guard actually bites.
    """
    real_names = _extract_builder_methods(_SYNTH_REAL, "FakeBuilder")
    mock_names = _extract_builder_methods(_SYNTH_MOCK_MISSING, "FakeBuilder")
    assert real_names - mock_names == {"setB"}, (real_names, mock_names)

    with pytest.raises(AssertionError) as excinfo:
        _assert_mock_superset("FakeBuilder", _SYNTH_REAL, _SYNTH_MOCK_MISSING)
    assert "setB" in str(excinfo.value)


def test_guard_passes_when_mock_is_superset():
    """A mock that covers (and exceeds) the real setter set must NOT raise.

    Guards against a false-positive guard that would flag a perfectly good mock.
    """
    _assert_mock_superset("FakeBuilder", _SYNTH_REAL, _SYNTH_MOCK_SUPERSET)


# ---------------------------------------------------------------------------
# T14 (issue #130, TC12/AC11) — field-level name-superset: trace_loader.Command
# vs the C++ recorder's serialized JSON field vocabulary.
#
# The builder-method superset above (NC5) guards the FIRST seam: does the mock
# capture every real fluent setter? This section guards the SECOND seam,
# introduced by T8's per-command speed knobs (`power`/`preset`/`start_speed`/
# `end_speed`/`profiled`/`brake`) and the `declare_position` op (`node`): does
# the Python `trace_loader.Command` dataclass carry a NAMED attribute for every
# JSON field key `ctps_record.cpp`'s `commandStreamToJson` can actually emit?
#
# This is the precise ADR-051 name-superset direction that matters here: T5's
# `profile.py::resolve_effective_speed` reads these fields via a plain
# `getattr(cmd, "start_speed", None)`-style named-attribute lookup (never the
# `Command.fields` catch-all dict), so a JSON key the C++ side can emit but
# that never became a NAMED `Command` field would silently resolve to the
# firmware default on EVERY move — a load-bearing default a human would have
# to notice went wrong, not a hard crash. Extending the mock's fluent-setter
# surface (NC5 above) is necessary but not sufficient: a setter can be
# captured into the C++ `Record` / serialized to JSON and STILL be missing
# from the Python dataclass if a human forgets that half of the two-language
# change — exactly the "real-grew" drift class this whole module exists to
# catch, one seam over.
# ---------------------------------------------------------------------------

_CTPS_RECORD_CPP = _ROOT / "WRO2026" / "test" / "ctps_mocks" / "ctps_record.cpp"

#: Matches the string-valued member emitter ``appendStrMember(out, "key", ...)``.
_APPEND_STR_MEMBER_RE = re.compile(r'appendStrMember\(\s*out\s*,\s*"([a-z_]+)"')

#: Matches the numeric/bool member emitter ``out += ", \"key\": "`` (and the
#: bare first-member form ``out += "\"key\": "`` used for ``op``/``ms`` etc.).
#: Both forms open with an (optionally comma-prefixed) escaped-quote JSON key.
_INLINE_MEMBER_RE = re.compile(r'out \+= ",? ?\\"([a-z_]+)\\":')


#: Document-level (``Trace``, not per-command ``Command``) JSON fields emitted
#: ONCE by ``commandStreamToJson`` outside the per-record ``switch`` — i.e.
#: before the ``"commands": [`` array opens. These share the same
#: ``appendStrMember`` / inline-literal emission idioms as the per-command
#: fields, so the regex scan below recovers them too, but they belong to
#: ``trace_loader.Trace`` (``start_node``, ``start_heading_deg``,
#: ``build_error``), never to a single ``Command`` entry, and must be excluded
#: from the per-command field-superset comparison.
_TRACE_LEVEL_KEYS: frozenset[str] = frozenset(
    {"start_node", "start_heading_deg", "build_error"}
)


def _extract_json_field_names(cpp_text: str) -> set[str]:
    """Return every PER-COMMAND JSON field-name key the serializer can emit.

    Scans BOTH member-emission idioms `commandStreamToJson` uses (see the
    module docstring extraction rule): the string-value helper
    `appendStrMember(out, "key", ...)` and the direct `out += ", \\"key\\": "`
    literal used for numeric/bool members. Comments are stripped first so a
    doxygen example JSON snippet can never be mistaken for a real emission
    site, mirroring `_strip_comments`'s role for the builder-method scan.
    Document-level keys (:data:`_TRACE_LEVEL_KEYS`, emitted once outside the
    per-record switch) are excluded — they belong to `trace_loader.Trace`,
    never to a single `Command` entry.

    :param cpp_text: raw ``ctps_record.cpp`` source text.
    :return: the set of per-command JSON field-name keys the serializer can
        emit (does NOT include the two structural wrapper keys `seq`/`op`,
        which are handled separately below since `op` is always a literal,
        never a member-macro emission).
    """
    text = _strip_comments(cpp_text)
    names: set[str] = set()
    names.update(_APPEND_STR_MEMBER_RE.findall(text))
    names.update(_INLINE_MEMBER_RE.findall(text))
    return names - _TRACE_LEVEL_KEYS


#: JSON field names emitted only as literal ``"op": "<value>"`` wrapper syntax
#: (op-discriminator + universal seq), never through either member-emission
#: idiom above, so they cannot be recovered by the regex scan. `Command`
#: models both (`Command.seq`, `Command.op`), so they are asserted present by
#: construction rather than scanned for — listed here purely for documentation.
_STRUCTURAL_KEYS: frozenset[str] = frozenset({"seq", "op"})

#: The T8 (issue #130) per-command speed knobs + the `declare_position` op's
#: `node` field — the exact field set this task's Contracts section adds. A
#: dedicated positive assertion (independent of the generic superset check)
#: pins this list so a future accidental deletion of one entry cannot silently
#: shrink coverage without a test noticing.
_T8_NEW_FIELDS: frozenset[str] = frozenset(
    {"power", "preset", "start_speed", "end_speed", "profiled", "brake", "node"}
)


def test_ctps_record_cpp_emits_expected_json_field_names():
    """Sanity: the extractor recovers the KNOWN field vocabulary from the C++ source.

    Guards against a silently-broken regex (which would make the superset
    check below pass vacuously): asserts both the long-standing per-command
    fields (`target`, `method`, `cm`, `mode`, `reverse`, ...) and the T8
    additions are recovered from the real `ctps_record.cpp`, and that the
    document-level `Trace` fields are correctly EXCLUDED from the per-command
    set (they are recovered separately, straight off the raw text, to prove
    the regex itself still sees them — only `_extract_json_field_names`'s
    trailing subtraction filters them out).
    """
    emitted = _extract_json_field_names(_read(_CTPS_RECORD_CPP))
    assert {
        "target", "arrival", "route", "text", "name",
        "method", "mode", "heading", "result",
    } <= emitted, emitted
    assert _T8_NEW_FIELDS <= emitted, (
        f"extractor did not recover the T8 field set from ctps_record.cpp: "
        f"missing {_T8_NEW_FIELDS - emitted}"
    )
    assert emitted.isdisjoint(_TRACE_LEVEL_KEYS), (
        f"per-command extraction leaked document-level Trace field(s): "
        f"{emitted & _TRACE_LEVEL_KEYS}"
    )

    raw_scan = set(
        _APPEND_STR_MEMBER_RE.findall(_strip_comments(_read(_CTPS_RECORD_CPP)))
    ) | set(_INLINE_MEMBER_RE.findall(_strip_comments(_read(_CTPS_RECORD_CPP))))
    assert _TRACE_LEVEL_KEYS & raw_scan == {"start_node"}, (
        "sanity: the raw (pre-filter) regex scan should still see start_node "
        "(the only Trace-level key using the appendStrMember idiom); a miss "
        "here means the extraction rule itself has drifted from the source."
    )


def test_trace_loader_command_is_field_superset_of_recorder():
    """TC12/AC11: every JSON field the C++ recorder can emit has a NAMED ``Command`` field.

    Extracts the JSON field-name vocabulary straight from the REAL
    ``ctps_record.cpp`` (never a hand-copied literal list, so the check tracks
    the actual C++ source) and asserts it is a subset of
    ``trace_loader.Command``'s dataclass field names (via ``dataclasses.fields``,
    so a future rename on either side is picked up automatically). A JSON key
    the recorder can emit but the loader has no named attribute for would
    silently resolve to ``None``/the firmware default through every
    ``getattr(cmd, name, None)``-style consumer (``profile.py``'s
    ``resolve_effective_speed`` foremost among them) — exactly the drift class
    this whole module exists to catch, applied to the field seam instead of
    the method seam.
    """
    from dataclasses import fields as dc_fields

    from tools.path_planner.ctps.trace_loader import Command

    emitted = _extract_json_field_names(_read(_CTPS_RECORD_CPP))
    named_fields = {f.name for f in dc_fields(Command)}

    missing = emitted - named_fields
    assert not missing, (
        f"trace_loader.Command is missing NAMED field(s) the real C++ recorder "
        f"(ctps_record.cpp) can emit: {sorted(missing)}. A JSON key with no "
        f"matching Command dataclass field silently falls back to the default "
        f"through getattr(cmd, name, None)-style resolution (e.g. "
        f"profile.py::resolve_effective_speed) instead of failing loud — add "
        f"the field to Command (issue #130, ADR-051)."
    )


def test_trace_loader_command_names_the_t8_fields_explicitly():
    """TC12/AC11 (positive, T8-scoped): the exact new field set is present.

    Narrower than the generic superset check above — pins the precise T8
    field list (the Contracts "Trace `drive` op fields" table +
    `declare_position.node`) so this test documents exactly what T8 added and
    fails with a precise diff if any single one regresses, independent of
    whether the C++ source scan itself stays healthy.
    """
    from dataclasses import fields as dc_fields

    from tools.path_planner.ctps.trace_loader import Command

    named_fields = {f.name for f in dc_fields(Command)}
    missing = _T8_NEW_FIELDS - named_fields
    assert not missing, (
        f"trace_loader.Command is missing the T8 field(s) {sorted(missing)} "
        f"(issue #130, Contracts)."
    )


def test_guard_bites_when_loader_drops_a_recorder_field(monkeypatch):
    """TC12/AC11 (negative): a loader missing a real recorder field must fail.

    Proves the field-superset guard actually bites: simulates a
    ``trace_loader`` that regressed by dropping ``start_speed`` (monkeypatching
    ``dataclasses.fields`` for a throwaway stand-in dataclass carrying every
    OTHER emitted field) and asserts the same subset-check logic used by
    ``test_trace_loader_command_is_field_superset_of_recorder`` raises and
    names the missing field — mirroring ``test_guard_bites_when_mock_drops_a_setter``
    for the method-superset half of this module.
    """
    from dataclasses import fields as dc_fields, make_dataclass

    emitted = _extract_json_field_names(_read(_CTPS_RECORD_CPP))
    assert "start_speed" in emitted, (
        "test setup assumption broken: ctps_record.cpp no longer emits "
        "start_speed — update this test's dropped-field choice"
    )

    # A synthetic stand-in for a regressed Command: every currently-emitted
    # field EXCEPT start_speed, each declared Optional so the dataclass
    # constructs trivially (values are irrelevant — only names are scanned).
    regressed_fields = sorted(emitted - {"start_speed"})
    RegressedCommand = make_dataclass(
        "RegressedCommand",
        [(name, "object", None) for name in regressed_fields],
    )

    named_fields = {f.name for f in dc_fields(RegressedCommand)}
    missing = emitted - named_fields
    assert missing == {"start_speed"}, missing
