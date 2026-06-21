"""Shared pytest fixtures for the path-planner test suite.

This module exists primarily to host the ONE session-scoped ``tk.Tk()`` root
the editor GUI tests share (Issue #78, T14).  Previously the blit tests
(``test_editor_canvas_blit.py``) and the panels smoke tests
(``test_editor_panels_smoke.py``) each defined their OWN session-scoped
``tk_root`` fixture.  A session-scoped fixture defined in two different modules
is two DISTINCT fixtures, so a full-suite run created two ``tk.Tk()`` roots in
one process.  Repeatedly instantiating ``tk.Tk()`` within a single interpreter
is unstable -- it intermittently raises ``TclError`` -- which is exactly the
flakiness T4 fixed for a single file, resurfaced ACROSS files (a full run was
observed to SKIP the GUI tests instead of running them).  Under xvfb on CI that
manifests as silent green-by-skip.  Consolidating into one conftest-level
fixture means the whole session shares a single root, so it is created exactly
once and the cross-file repeated-``tk.Tk()`` flakiness cannot recur.
"""

from __future__ import annotations

import matplotlib

# Pin the headless-safe Agg backend before anything imports the editor.  pytest
# imports conftest very early -- before any test module -- so pinning here makes
# Agg the backend in force when ``tk.Tk()`` (below) and the per-file editor
# imports run.  The editor's ``editor.canvas`` module performs a
# ``matplotlib.use("TkAgg")`` at import time, which would raise on headless CI
# and clobber Agg; the test modules each no-op ``matplotlib.use`` around that
# import.  This pin is the centralized headless default they layer on top of.
matplotlib.use("Agg")

import tkinter as tk  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture(scope="session")
def tk_root():
    """One shared (withdrawn) Tk root for the whole test session.

    The editor GUI tests need a Tk root: the blit tests embed a
    ``FigureCanvasTkAgg`` inside a real ``FieldCanvas`` and the panels smoke
    tests build a ``SidePanels`` (a ``tk.Frame``).  Repeatedly creating and
    destroying ``tk.Tk()`` within a single pytest process is unstable -- it
    intermittently raises ``TclError``, making the GUI tests skip
    nondeterministically.

    Defining this fixture ONCE here in ``conftest.py`` (rather than once per
    test module) guarantees a SINGLE root for the whole session: a session-scoped
    fixture duplicated across modules would be two distinct fixtures and would
    create two roots in one process, which is precisely the cross-file
    repeated-``tk.Tk()`` flakiness this consolidation removes.  Every GUI test --
    in either module -- resolves ``tk_root`` to this same instance; each test
    still gets a fresh canvas / panel (function-scoped fixtures) layered on top.

    ``pytest.skip`` here is a genuine last resort: it fires only when this single
    ``tk.Tk()`` truly cannot be created (no display even under xvfb), never
    merely because a root already exists.
    """
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        pytest.skip(f"Tk unavailable (no display, even under xvfb): {exc}")
    root.withdraw()
    yield root
    try:
        root.destroy()
    except tk.TclError:
        pass
