"""Warm "drafting-table" theme for the WRO 2026 live field editor.

This is a **self-contained, side-effect-free** theme module that the later
de-lag tasks (T11 panels, T12 canvas, T13 app) consume. It owns three things
and nothing else:

1. **A warm "drafting-table" palette** -- a cream paper background, warm
   near-black ink, and three saturated accents (terracotta, teal, brick) that
   double as the node/edge semantic colors. Every color used anywhere in the
   editor should be sourced from a constant here; no naked hex literals should
   live in the panels/canvas/app modules.

2. **Font fallback chains.** No ``.ttf`` is bundled with the editor (see
   ``assets/fonts/README.md``), so the display/serif, sans-UI, and numeric-mono
   roles are expressed as ordered fallback tuples of family names. A loader
   helper *attempts* to private-load any bundled font and otherwise degrades
   silently down the chain -- a missing font must never crash the editor.

3. **Theme appliers** -- :func:`apply_ttk_theme` for the Tk/ttk widget tree and
   :func:`matplotlib_facecolors` / :func:`apply_to_axes` for the embedded
   matplotlib figure.

Policy notes
------------
* **``sv_ttk`` is optional.** If the (third-party) `Sun Valley ttk theme
  <https://github.com/rdbende/Sun-Valley-ttk-theme>`_ happens to be installed,
  :func:`apply_ttk_theme` uses it as a base and overlays the palette on top.
  It is **not** a dependency of this repo and is **not** present in CI, so the
  import is soft (``try/except ImportError``); when absent the function falls
  back to a pure :class:`ttk.Style` configuration over a built-in theme. This
  module must import cleanly with only the stdlib + matplotlib present.

* **No side effects at import.** Importing this module creates no Tk root and
  never calls :func:`matplotlib.use`. Backend selection is the canvas module's
  responsibility (it picks ``TkAgg``); the static visualizer picks ``Agg``.
  Touching the backend here would race those selectors. All matplotlib helpers
  therefore operate on objects passed in by the caller and import lazily.

* **Headless tolerance.** The font and ttk helpers are written so the headless
  panels-smoke test can import this module and call the pure helpers without a
  display. :func:`apply_ttk_theme` does require a real Tk root (a ``ttk.Style``
  needs one); it is wrapped so it degrades to a no-op rather than raising if a
  root cannot be obtained.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:  # Imported only for type checking; no runtime Tk/mpl import.
    import tkinter as tk
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure


# ===========================================================================
# Palette -- warm "drafting-table" colors. Every hex literal in the editor
# should originate here. Roles, not raw values, are what the rest of the
# editor references. All values are 6-digit ``#rrggbb`` strings.
# ===========================================================================

# --- Core surfaces & text -------------------------------------------------
# Warm off-white "drafting paper" -- the dominant background tone for both the
# matplotlib figure and the ttk frames.
PAPER: str = "#F3EAD8"
# A slightly lighter cream used for the plotting region so the axes lift very
# subtly off the surrounding paper.
PAPER_LIGHT: str = "#FAF4E6"
# Panel surface -- a hair darker than PAPER so side panels read as a distinct
# plane from the canvas without introducing a cold gray.
SURFACE: str = "#EADFC8"
# Warm near-black ink for body text and line-follow geometry.
INK: str = "#2B2622"
# Muted ink for secondary text, hints, and disabled labels.
INK_MUTED: str = "#6E6457"

# --- Accents (also the node/edge semantic colors) ------------------------
# Terracotta -- the primary accent: selection rings, active buttons, focus.
TERRACOTTA: str = "#C56B45"
# Teal -- reset nodes (and any "reset/zero" semantic).
TEAL: str = "#2A8C82"
# Brick -- blind nodes and blind (dead-reckoned) edges.
BRICK: str = "#B0413E"
# Ochre -- projection nodes (a line crossing that is not a junction). A warm
# amber that stays distinct from ink (line), brick (blind), and teal (reset).
OCHRE: str = "#C8951F"

# --- Supporting tones ----------------------------------------------------
# Subtle grid line color on the plotting surface -- a warm low-contrast tan.
GRID: str = "#D7C9AC"
# Muted spine / tick / axis-label color -- darker than the grid, lighter than
# ink, so the frame reads as structure rather than content.
SPINE: str = "#9C8C72"
# Validation badges. VALID is a warm, slightly desaturated green that sits on
# the cream paper without clashing; INVALID reuses the brick family.
VALID: str = "#5C8A3A"
INVALID: str = "#B0413E"


# ===========================================================================
# UI chrome tokens -- consumed by the panel (T6) and canvas (T7) modules.
# Every chrome constant below maps to a named UI role in the mockup; callers
# reference the role name, never a naked hex literal.
# ===========================================================================

# --- Title bar & section headers ------------------------------------------

# Point size for the title-bar display text. Paired with FONT_DISPLAY (Fraunces
# fallback chain) so the caller writes e.g. ``(resolve_font(FONT_DISPLAY), FONT_SIZE_TITLE)``.
FONT_SIZE_TITLE: int = 16

# Point size for section-header labels. The "small-caps" look is approximated
# by rendering the label text in UPPERCASE; no special font variant is needed.
FONT_SIZE_SECTION: int = 9

# Section-header text color. INK_MUTED gives a quieter hierarchy without
# introducing a new hue.
SECTION_HEADER_FG: str = INK_MUTED  # alias — intent is explicit for callers

# Hairline rule color drawn under each section header. SPINE is the right
# weight: darker than the grid, but still subordinate to body text.
HAIRLINE: str = SPINE

# --- Ghost button ---------------------------------------------------------

# A "ghost" button is flat/borderless at rest: text in INK_MUTED on SURFACE,
# with a thin border that matches the button edge for a barely-visible outline.
GHOST_BUTTON_FG: str = INK_MUTED        # resting foreground (text/icon)
GHOST_BUTTON_BG: str = SURFACE          # resting background (transparent feel)
GHOST_BUTTON_BORDER: str = SPINE        # thin 1-px border; SPINE is subtle

# Foreground override when the ghost button uses the primary (terracotta) accent
# instead of the muted default -- e.g., a "delete" or "arm" action.
GHOST_BUTTON_FG_ACCENT: str = TERRACOTTA

# --- Collapsible-section chevrons ----------------------------------------

# Glyph shown when the section is expanded (open / pointing down-right).
CHEVRON_OPEN: str = "◢"
# Glyph shown when the section is collapsed (closed / pointing right).
CHEVRON_COLLAPSED: str = "▸"

# --- Dirty-state indicator ------------------------------------------------

# Color for the unsaved-changes dot (⬤) in the title bar. TERRACOTTA is the
# primary attention accent and reads clearly against PAPER/SURFACE.
DIRTY_DOT: str = TERRACOTTA  # alias -- intent is explicit for callers

# --- Active-mode accent ---------------------------------------------------

# Soft terracotta wash drawn behind the active tool-mode row in the toolbar.
# Pale enough to not overpower the panel but clearly warmer than SURFACE.
ACCENT_SOFT: str = "#E7C9B8"

# Solid accent bar color for the active-mode left-edge stripe. TERRACOTTA is
# the right token; this alias makes the intent unambiguous for panels.py.
ACCENT_BAR: str = TERRACOTTA  # alias

# --- Status-badge colors (intent aliases for panels) ---------------------

# These alias the existing VALID / INVALID palette entries so the panels can
# reference them by their UI role (badge ⬤ indicator) rather than by the
# generic name, which also doubles as a matplotlib annotation color.
STATUS_VALID: str = VALID    # green ⬤ in the status-strip badge
STATUS_INVALID: str = INVALID  # brick ⬤ in the status-strip badge


# ===========================================================================
# Node / edge palette mapping -- consumed by the canvas (T12). Keeping the
# semantic-to-color mapping here lets the canvas replace its hardcoded
# black/red literals with a single lookup table that tracks the palette.
# ===========================================================================

# Node fill color keyed by the model's ``type`` field. ``projection`` (a line
# crossing that is not a junction) draws in ochre; ``reset`` in teal. An unknown
# type should fall back to :data:`NODE_COLOR_FALLBACK`.
NODE_COLORS: dict[str, str] = {
    "line": INK,
    "projection": OCHRE,
    "blind": BRICK,
    "reset": TEAL,
}
# Fallback fill for a node whose type is not in :data:`NODE_COLORS`.
NODE_COLOR_FALLBACK: str = INK_MUTED

# Edge stroke color keyed by edge kind. ``line`` = line-follow segment,
# ``blind`` = dead-reckoned (blind) edge.
EDGE_COLORS: dict[str, str] = {
    "line": INK,
    "blind": BRICK,
}

# The ring drawn around the selected / highlighted node.
SELECTION_RING: str = TERRACOTTA
# A proposed (not-yet-committed) edge while the user is mid-gesture.
PROPOSED_EDGE: str = TEAL


# ===========================================================================
# Font fallback chains. No ``.ttf`` is bundled (see assets/fonts/README.md),
# so each role is an ordered tuple of family names: the loader walks it and
# returns the first family Tk actually has, ending in a CSS/Tk generic family
# that is guaranteed to resolve.
# ===========================================================================

# Display / headings -- a warm serif.
FONT_DISPLAY: tuple[str, ...] = ("Fraunces", "Cambria", "Georgia", "serif")
# General UI text (labels, buttons, list items).
FONT_SANS: tuple[str, ...] = ("IBM Plex Sans", "Segoe UI", "Helvetica", "sans-serif")
# Numeric / tabular fields (coordinates, headings) -- a monospace.
FONT_MONO: tuple[str, ...] = ("IBM Plex Mono", "Cascadia Mono", "Consolas", "monospace")

# Directory that *would* hold bundled ``.ttf`` files if any were shipped.
_FONT_ASSET_DIR: Path = Path(__file__).resolve().parent / "assets" / "fonts"
# File suffixes the loader will attempt to private-load.
_FONT_SUFFIXES: tuple[str, ...] = (".ttf", ".otf")


# ===========================================================================
# matplotlib facecolor mapping. Pure data -- the canvas (T12) reads this dict
# instead of hardcoding figure/axes/grid/spine/tick colors.
# ===========================================================================

# Default font size (points) for matplotlib tick/axis labels. Kept here so the
# canvas does not reintroduce a magic number when wiring tick params.
_MPL_TICK_LABELSIZE: float = 8.0


def matplotlib_facecolors() -> dict[str, str]:
    """Return the editor's matplotlib color roles as a plain dict.

    The returned mapping is intended to be applied to an embedded figure so the
    plotting surface matches the warm ttk theme. It is **pure data**: it neither
    imports matplotlib nor touches any figure, which keeps this safe to call in
    a headless context.

    :return: A dict with the keys ``figure`` (figure facecolor), ``axes`` (axes
        facecolor), ``grid`` (grid line color), ``spine`` (axes spine color),
        and ``tick`` (tick mark and tick/axis-label color). All values are
        ``#rrggbb`` strings drawn from the module palette.
    """
    return {
        "figure": PAPER,
        "axes": PAPER_LIGHT,
        "grid": GRID,
        "spine": SPINE,
        "tick": SPINE,
    }


def apply_to_axes(ax: "Axes", fig: Optional["Figure"] = None) -> None:
    """Apply the editor palette to a matplotlib axes (and optional figure).

    This is the convenience counterpart to :func:`matplotlib_facecolors`: it
    sets the figure facecolor, the axes facecolor, the grid color, every spine
    color, and the tick / tick-label color so the embedded plot matches the
    warm ttk theme. matplotlib is imported lazily inside the call so importing
    this module never pulls in matplotlib or selects a backend.

    The function is defensive: any individual styling step that the active
    matplotlib backend rejects is skipped rather than raised, so a partially
    themed axes is preferable to a crash.

    :param ax: The :class:`~matplotlib.axes.Axes` to style. Required.
    :param fig: The :class:`~matplotlib.figure.Figure` whose facecolor to set.
        Defaults to ``ax.figure`` when omitted.
    :return: ``None``.
    """
    colors = matplotlib_facecolors()
    figure = fig if fig is not None else getattr(ax, "figure", None)

    if figure is not None:
        try:
            figure.set_facecolor(colors["figure"])
        except Exception:  # noqa: BLE001 -- styling must never crash the editor.
            pass

    try:
        ax.set_facecolor(colors["axes"])
    except Exception:  # noqa: BLE001
        pass

    try:
        ax.grid(True, color=colors["grid"])
    except Exception:  # noqa: BLE001
        pass

    try:
        for spine in ax.spines.values():
            spine.set_color(colors["spine"])
    except Exception:  # noqa: BLE001
        pass

    try:
        ax.tick_params(
            colors=colors["tick"],
            labelcolor=colors["tick"],
            labelsize=_MPL_TICK_LABELSIZE,
        )
    except Exception:  # noqa: BLE001
        pass


# ===========================================================================
# Font loading. Attempts to private-load any bundled font then resolves a role
# chain to the first family Tk actually exposes. Never raises.
# ===========================================================================


def load_bundled_fonts() -> None:
    """Attempt to private-load every bundled ``.ttf`` / ``.otf`` font.

    Walks :data:`_FONT_ASSET_DIR` and tries to register each font file with the
    process so its family becomes resolvable by name. This is **best-effort**:

    * The asset directory currently ships no fonts, so this is normally a no-op.
    * Private font loading is only attempted on Windows (via ``gdi32`` /
      ``AddFontResourceExW``), the platform the editor runs on. On other
      platforms, or if the ``ctypes`` call fails for any reason, the function
      returns silently and callers fall back down the font chain.

    Loading a private font does *not* by itself make Tk see it without a
    refresh, so this should be treated as a hint, not a guarantee. Callers must
    still resolve through :func:`resolve_font` and tolerate the chain's generic
    tail.

    :return: ``None``. Never raises.
    """
    try:
        if not _FONT_ASSET_DIR.is_dir():
            return
        font_files = [
            path
            for path in _FONT_ASSET_DIR.iterdir()
            if path.is_file() and path.suffix.lower() in _FONT_SUFFIXES
        ]
        if not font_files:
            return
        _private_load_windows_fonts(font_files)
    except Exception:  # noqa: BLE001 -- font loading must never crash the editor.
        pass


def _private_load_windows_fonts(font_files: list[Path]) -> None:
    """Register fonts for the current process via the Windows GDI API.

    Uses ``AddFontResourceExW`` with the ``FR_PRIVATE`` flag so the fonts are
    visible only to this process and need no installation. A non-Windows
    platform (where ``ctypes.windll`` is absent) raises ``AttributeError``,
    which the caller swallows.

    :param font_files: Font file paths to register.
    :return: ``None``.
    """
    import ctypes  # Local import -- only needed on the Windows font path.

    fr_private = 0x10  # FR_PRIVATE: load for this process only.
    gdi32 = ctypes.windll.gdi32  # type: ignore[attr-defined]  # Windows-only.
    for path in font_files:
        # A return of 0 means "no fonts added"; we ignore it and fall back.
        gdi32.AddFontResourceExW(str(path), fr_private, 0)


def _available_families() -> frozenset[str]:
    """Return the set of font family names Tk currently exposes.

    Resolving family availability needs a default Tk root. When none exists yet
    (the common headless case at import time, or before the app builds its
    window) this returns an empty set so :func:`resolve_font` degrades to the
    generic tail of the chain instead of raising.

    :return: A frozenset of available family names, possibly empty.
    """
    try:
        import tkinter.font as tkfont

        return frozenset(tkfont.families())
    except Exception:  # noqa: BLE001 -- no root / no display -> "nothing known".
        return frozenset()


def resolve_font(chain: tuple[str, ...]) -> str:
    """Resolve a font fallback chain to the first family Tk actually has.

    Walks ``chain`` in order and returns the first family that Tk reports as
    available (via :func:`tkinter.font.families`). If Tk cannot be queried (no
    root / headless) or none of the named families are present, the **last**
    entry in the chain is returned -- by convention a generic family name
    (``serif`` / ``sans-serif`` / ``monospace``) that Tk and CSS both resolve.

    The chain is assumed non-empty (all module ``FONT_*`` constants satisfy
    this); an empty chain returns an empty string rather than raising.

    :param chain: Ordered tuple of family names, most-preferred first, ending
        in a generic family.
    :return: The resolved family name, or the chain's last entry as a fallback.
    """
    if not chain:
        return ""
    available = _available_families()
    if available:
        for family in chain:
            if family in available:
                return family
    return chain[-1]


# ===========================================================================
# ttk theming. Soft-imports sv_ttk; otherwise configures a plain ttk.Style.
# Requires a real Tk root (a ttk.Style needs one) but degrades to a no-op
# rather than raising when a root cannot be obtained.
# ===========================================================================

# Preferred built-in ttk themes, best-looking first. ``clam`` honors
# background/foreground overrides on most platforms; the others are
# last-resort fallbacks that are always present.
_TTK_FALLBACK_THEMES: tuple[str, ...] = ("clam", "alt", "default", "classic")

# Base point size for ttk label / entry fonts configured below.
_TTK_FONT_SIZE: int = 10

# ttk style names we explicitly recolor. Configuring the bare element names
# (``TFrame`` etc.) recolors all widgets of that class.
_STYLED_ELEMENTS: tuple[str, ...] = (
    "TFrame",
    "TLabel",
    "TLabelframe",
    "TLabelframe.Label",
    "TButton",
    "TRadiobutton",
    "TCheckbutton",
    "TEntry",
    "TCombobox",
)


def apply_ttk_theme(root: "tk.Misc") -> None:
    """Apply the warm drafting-table theme to a Tk/ttk widget tree.

    If the optional third-party ``sv_ttk`` package is importable, its light
    theme is applied first as a base and the palette is overlaid on top;
    otherwise a built-in ttk theme is selected and recolored directly. Either
    way the result is the same warm cream-on-ink look.

    ``sv_ttk`` is **not** a dependency of this repo and is absent in CI, so the
    import is soft. The function never raises on a missing theme package, a
    missing display, or an unsupported built-in theme -- it degrades to a no-op
    so the headless panels-smoke test stays green.

    :param root: Any Tk widget or the root window. A :class:`ttk.Style` is
        created against it; a real Tk root must already exist (this does not
        create one). Passing ``None`` makes the call a no-op.
    :return: ``None``.

    :note: A ``ttk.Style`` requires a live Tk root, so this cannot be exercised
        in a truly headless (no-``$DISPLAY``) environment; it is designed to
        fail soft (no-op) there rather than crash the import or smoke test.
    """
    if root is None:
        return

    # Opportunistically register any bundled fonts before resolving families.
    load_bundled_fonts()

    sv_ttk = _try_import_sv_ttk()
    sv_ttk_active = False
    if sv_ttk is not None:
        sv_ttk_active = _apply_sv_ttk_base(sv_ttk)
    # Overlay (or fully build, when sv_ttk is absent) the palette on a Style.
    # When sv_ttk set a base theme we must NOT re-select a built-in theme, or
    # we would clobber it; the palette is overlaid on top instead.
    _configure_ttk_style(root, sv_ttk_active=sv_ttk_active)


def _try_import_sv_ttk():
    """Soft-import the optional ``sv_ttk`` theme package.

    :return: The imported ``sv_ttk`` module, or ``None`` if it is not installed
        (the expected case in this repo and in CI).
    """
    try:
        import sv_ttk  # type: ignore[import-not-found]  # Optional dependency.

        return sv_ttk
    except ImportError:
        return None


def _apply_sv_ttk_base(sv_ttk) -> bool:
    """Apply ``sv_ttk``'s light theme as the base look, if usable.

    :param sv_ttk: The imported ``sv_ttk`` module.
    :return: ``True`` if the sv_ttk theme was set successfully, ``False`` if the
        call failed. Swallows any error so a flaky optional dependency cannot
        crash the editor; a ``False`` return tells the caller to fall back to a
        built-in theme instead of overlaying onto a half-set sv_ttk base.
    """
    try:
        sv_ttk.set_theme("light")
        return True
    except Exception:  # noqa: BLE001 -- optional dependency, must not crash us.
        return False


def _configure_ttk_style(root: "tk.Misc", sv_ttk_active: bool = False) -> None:
    """Recolor a :class:`ttk.Style` from the module palette.

    When ``sv_ttk`` supplied a base theme (``sv_ttk_active`` is ``True``) this
    overlays palette colors on top of it and leaves the active theme untouched;
    when it did not, this first selects the best available built-in theme and
    then recolors it. Every step is guarded so a missing root or an
    override-resistant platform theme degrades to a partial/no-op result rather
    than raising.

    :param root: The Tk root / widget the style is bound to.
    :param sv_ttk_active: ``True`` when :func:`_apply_sv_ttk_base` set an sv_ttk
        theme; when set, the built-in theme selection is skipped so the sv_ttk
        base is preserved.
    :return: ``None``.
    """
    try:
        from tkinter import ttk
    except Exception:  # noqa: BLE001 -- no Tk available; nothing to do.
        return

    try:
        style = ttk.Style(root)
    except Exception:  # noqa: BLE001 -- no live root; fail soft.
        return

    # If sv_ttk did not set a theme, pick the first available built-in that we
    # know honors background/foreground overrides. When sv_ttk IS active we
    # leave its theme in place and only overlay the palette below.
    if not sv_ttk_active:
        _select_builtin_theme(style)

    sans = resolve_font(FONT_SANS)
    mono = resolve_font(FONT_MONO)

    # Class-wide backgrounds/foregrounds. Configuring the bare element names
    # recolors all widgets of that ttk class.
    for element in _STYLED_ELEMENTS:
        _safe_style_configure(style, element, background=SURFACE, foreground=INK)

    # The plotting/content frames sit on PAPER, not the panel SURFACE tone.
    _safe_style_configure(style, "TFrame", background=SURFACE)

    # Entry / Combobox: text fields read on the lighter paper tone with ink
    # text; the field area uses fieldbackground (Entry/Combobox-specific).
    _safe_style_configure(
        style,
        "TEntry",
        fieldbackground=PAPER_LIGHT,
        background=PAPER_LIGHT,
        foreground=INK,
    )
    _safe_style_configure(
        style,
        "TCombobox",
        fieldbackground=PAPER_LIGHT,
        background=SURFACE,
        foreground=INK,
    )

    # Buttons get the terracotta accent on press/active; keep the resting state
    # on the panel surface so they don't shout.
    _safe_style_configure(style, "TButton", background=SURFACE, foreground=INK)
    _safe_style_map(
        style,
        "TButton",
        background=[("pressed", TERRACOTTA), ("active", TERRACOTTA)],
        foreground=[("pressed", PAPER), ("active", PAPER)],
    )

    # Selection ring color is surfaced via the accent for any widget that maps
    # a "selected" state.
    _safe_style_map(
        style,
        "TRadiobutton",
        foreground=[("selected", TERRACOTTA)],
    )

    # Mono font for any explicitly numeric entry style. Callers (T11) opt in by
    # assigning style="Mono.TEntry"; defining it here keeps the family in one
    # place. Guarded because creating a derived style still needs a root.
    if mono:
        _safe_style_configure(
            style,
            "Mono.TEntry",
            fieldbackground=PAPER_LIGHT,
            foreground=INK,
            font=(mono, _TTK_FONT_SIZE),
        )
    if sans:
        _safe_style_configure(style, "TLabel", font=(sans, _TTK_FONT_SIZE))


def _select_builtin_theme(style: "object") -> None:
    """Select the first usable built-in ttk theme on ``style``.

    Only switches the theme when the currently active one is not already in our
    preferred list -- this avoids clobbering a base theme that ``sv_ttk`` set.

    :param style: A live :class:`ttk.Style` instance.
    :return: ``None``.
    """
    try:
        available = set(style.theme_names())  # type: ignore[attr-defined]
        current = style.theme_use()  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        return
    if current in _TTK_FALLBACK_THEMES:
        return
    for theme in _TTK_FALLBACK_THEMES:
        if theme in available:
            try:
                style.theme_use(theme)  # type: ignore[attr-defined]
            except Exception:  # noqa: BLE001
                continue
            return


def _safe_style_configure(style: "object", element: str, **kwargs) -> None:
    """Call ``style.configure`` while swallowing backend-specific failures.

    Some option names (``fieldbackground``) are rejected by some platform
    themes; rather than branch per platform we attempt the configure and ignore
    a rejection so the rest of the theming still applies.

    :param style: A live :class:`ttk.Style` instance.
    :param element: The ttk style/element name to configure.
    :param kwargs: Style options forwarded to ``style.configure``.
    :return: ``None``.
    """
    try:
        style.configure(element, **kwargs)  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        pass


def _safe_style_map(style: "object", element: str, **kwargs) -> None:
    """Call ``style.map`` while swallowing backend-specific failures.

    :param style: A live :class:`ttk.Style` instance.
    :param element: The ttk style/element name to map.
    :param kwargs: State-keyed option lists forwarded to ``style.map``.
    :return: ``None``.
    """
    try:
        style.map(element, **kwargs)  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        pass
