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
