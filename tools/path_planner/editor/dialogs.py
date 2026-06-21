"""Modal dialog windows for the WRO 2026 live field editor (Issue #10, T3).

The dialogs here are pure widget shells: they collect input from the
operator and expose the result through a ``.result`` attribute set
synchronously inside the OK handler. The owning :class:`EditorApp`
constructs the dialog, calls ``wait_window()`` to block until the
modal is dismissed, then reads ``.result``.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Optional, Sequence

from tools.path_planner.editor import theme

# One keyboard shortcut: (key_label, human description).
ShortcutRow = tuple[str, str]
# One titled group of shortcuts: (category_title, [rows...]).
ShortcutGroup = tuple[str, "Sequence[ShortcutRow]"]


# Allowed node-type values, mirroring panels._NODE_TYPE_VALUES and the
# validator's rules. Duplicated here so dialogs.py doesn't import panels.
# "reset" is selectable here; the app layer seeds a default reset spec and
# opens the inspector after the node is created (T8) -- this dialog does not
# collect the reset spec, so the (name, type) result contract is unchanged.
_NODE_TYPE_VALUES: tuple[str, ...] = ("line", "blind", "reset")
_DEFAULT_NODE_TYPE: str = "line"

# Dialog layout constants -- no magic numbers in the code body.
_DIALOG_PADDING_PX: int = 10
_FIELD_PAD_Y_PX: int = 4
_BUTTON_PAD_Y_PX: int = 8
_ENTRY_WIDTH_CH: int = 24
_COMBO_WIDTH_CH: int = 22

# ShortcutsDialog layout constants.
_SHORTCUTS_KEY_PAD_X_PX: int = 12  # gap between the key label and its description
_SHORTCUTS_ROW_PAD_Y_PX: int = 2  # vertical breathing room between rows
_SHORTCUTS_HEADER_PAD_TOP_PX: int = 10  # space above each category header
_SHORTCUTS_HAIRLINE_HEIGHT_PX: int = 1  # thickness of the rule under a header
_SHORTCUTS_KEY_MIN_WIDTH_CH: int = 16  # min width of the key column for alignment
_SHORTCUTS_FONT_SIZE: int = 10  # point size for the key/description body rows

# Fallback font families used when the theme module is unavailable or its
# chains resolve to nothing (e.g. a headless root with no families known).
_MONO_FALLBACK_FAMILY: str = "Consolas"
_SANS_FALLBACK_FAMILY: str = "Segoe UI"


class AddNodeDialog(tk.Toplevel):
    """Modal Toplevel that collects (name, type) for a new node.

    The dialog is constructed, packed, and made modal via
    ``transient(parent) + grab_set()``. The owning :class:`EditorApp`
    then calls ``wait_window()`` on this instance and reads ``.result``:

    * ``(name_str, type_str)`` on successful OK click -- *name_str* is the
      already-stripped operator entry; *type_str* is one of
      ``"line"`` / ``"blind"`` / ``"reset"``.
    * ``None`` on Cancel, on Escape, or on window-close (the "X" button).

    Keyboard behaviour:

    * Initial focus is on the Name entry.
    * ``<Return>`` while the Name entry has focus advances focus to the
      Type combobox.
    * ``<Return>`` on the Type combobox or the OK button commits.
    * ``<Escape>`` cancels.

    :param parent: Parent Tk widget (usually the main window's root).
    :param initial_type: Default value for the Type combobox.
        Defaults to ``"line"``.
    """

    result: Optional[tuple[str, str]]

    def __init__(
        self,
        parent: tk.Widget | tk.Tk,
        *,
        initial_type: str = _DEFAULT_NODE_TYPE,
    ) -> None:
        super().__init__(parent)
        self.result = None

        self.title("Add node")
        self.resizable(False, False)

        self.transient(parent)
        self.protocol("WM_DELETE_WINDOW", self._on_cancel)

        body = tk.Frame(self, padx=_DIALOG_PADDING_PX, pady=_DIALOG_PADDING_PX)
        body.pack(side="top", fill="both", expand=True)

        # Name row
        name_row = tk.Frame(body)
        name_row.pack(side="top", fill="x", pady=(0, _FIELD_PAD_Y_PX))
        tk.Label(name_row, text="Name:", width=6, anchor="w").pack(side="left")
        self._name_var = tk.StringVar(value="")
        self._name_entry = tk.Entry(
            name_row,
            textvariable=self._name_var,
            width=_ENTRY_WIDTH_CH,
        )
        self._name_entry.pack(side="left", fill="x", expand=True)

        # Type row
        type_row = tk.Frame(body)
        type_row.pack(side="top", fill="x", pady=(0, _FIELD_PAD_Y_PX))
        tk.Label(type_row, text="Type:", width=6, anchor="w").pack(side="left")
        if initial_type not in _NODE_TYPE_VALUES:
            initial_type = _DEFAULT_NODE_TYPE
        self._type_var = tk.StringVar(value=initial_type)
        self._type_combo = ttk.Combobox(
            type_row,
            textvariable=self._type_var,
            values=list(_NODE_TYPE_VALUES),
            state="readonly",
            width=_COMBO_WIDTH_CH,
        )
        self._type_combo.pack(side="left", fill="x", expand=True)

        # Button row
        button_row = tk.Frame(body)
        button_row.pack(side="top", fill="x", pady=(_BUTTON_PAD_Y_PX, 0))
        tk.Button(button_row, text="Cancel", command=self._on_cancel, width=8).pack(
            side="right", padx=(4, 0)
        )
        self._ok_button = tk.Button(
            button_row, text="OK", command=self._on_ok, width=8, default="active"
        )
        self._ok_button.pack(side="right")

        self._name_entry.bind("<Return>", self._handle_name_return)
        self._type_combo.bind("<Return>", lambda _e: self._on_ok())
        self._ok_button.bind("<Return>", lambda _e: self._on_ok())
        self.bind("<Escape>", lambda _e: self._on_cancel())

        self.update_idletasks()
        self._center_on_parent(parent)
        self._name_entry.focus_set()
        self.grab_set()

    def _on_ok(self) -> None:
        """Commit: stash ``(name, type)`` on ``self.result`` and destroy.

        If the name is empty after stripping, the dialog stays open and
        focus returns to the Name entry. Duplicate-name detection is
        handled downstream by ``AddNode.apply``.
        """
        name = self._name_var.get().strip()
        if not name:
            self._name_entry.focus_set()
            return
        self.result = (name, self._type_var.get())
        self.destroy()

    def _on_cancel(self) -> None:
        """Cancel: leave ``self.result`` as ``None`` and destroy."""
        self.result = None
        self.destroy()

    def _handle_name_return(self, _event: tk.Event) -> str:
        """``<Return>`` on the Name entry -> advance focus to the Type combobox.

        Returning ``"break"`` stops the event from also triggering the OK
        button's default-action binding, so the user can confirm the name
        before they commit.
        """
        self._type_combo.focus_set()
        return "break"

    def _center_on_parent(self, parent: tk.Widget | tk.Tk) -> None:
        """Position the dialog at the centre of *parent*.

        Falls back to "no reposition" if the parent reports zero size
        (e.g. a hidden test root).
        """
        try:
            parent.update_idletasks()
            px = parent.winfo_rootx()
            py = parent.winfo_rooty()
            pw = parent.winfo_width()
            ph = parent.winfo_height()
            if pw <= 1 or ph <= 1:
                return
            dw = self.winfo_width()
            dh = self.winfo_height()
            x = px + (pw - dw) // 2
            y = py + (ph - dh) // 2
            self.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        except tk.TclError:
            pass


class ShortcutsDialog(tk.Toplevel):
    """Modal Toplevel that displays the editor's keyboard shortcuts.

    The dialog is purely informational: it renders whatever list of shortcut
    groups it is handed and offers no result. The owning :class:`EditorApp`
    (T8) is the single source of truth for the real key bindings and passes
    them in; the dialog stays dumb and decoupled so it can be unit-tested with
    synthetic data under a bare Tk root.

    Layout: each group is a small-caps (UPPERCASE) header with a hairline rule
    underneath, followed by one row per shortcut -- the *key label* left-aligned
    in a mono font and the *description* to its right in the UI sans font.

    Modal behaviour mirrors :class:`AddNodeDialog`: ``transient(parent)`` +
    ``grab_set()``. The dialog dismisses on ``<Escape>``, on the "Close" button,
    and on the window-manager close ("X"); on dismiss it releases the grab and
    returns focus to *parent*.

    The rendered widgets are retained on :attr:`_row_labels` and
    :attr:`_header_labels` so a test can construct the dialog (without the
    window ever being mapped) and assert every key label and description string
    is present in the widget tree.

    :param parent: Parent Tk widget (usually the main window's root).
    :param shortcut_groups: Ordered groups of shortcuts to render. Each item is
        ``(category_title, [(key_label, description), ...])``. An empty list, or
        a group with no rows, renders nothing for that group -- it never raises.
    """

    def __init__(
        self,
        parent: tk.Widget | tk.Tk,
        shortcut_groups: "Sequence[ShortcutGroup]",
    ) -> None:
        super().__init__(parent)
        self._parent = parent

        # Retained widget references so tests can introspect rendered text
        # without the window being visible.
        self._header_labels: list[tk.Label] = []
        self._row_labels: list[tuple[tk.Label, tk.Label]] = []

        self.title("Keyboard shortcuts")
        self.resizable(False, False)

        self.transient(parent)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        mono_family = self._resolve_mono_family()
        sans_family = self._resolve_sans_family()
        header_fg = self._theme_color("SECTION_HEADER_FG", "#6E6457")
        hairline_color = self._theme_color("HAIRLINE", "#9C8C72")

        body = tk.Frame(self, padx=_DIALOG_PADDING_PX, pady=_DIALOG_PADDING_PX)
        body.pack(side="top", fill="both", expand=True)

        self._build_groups(
            body,
            shortcut_groups,
            mono_family=mono_family,
            sans_family=sans_family,
            header_fg=header_fg,
            hairline_color=hairline_color,
        )

        # Button row
        button_row = tk.Frame(body)
        button_row.pack(side="top", fill="x", pady=(_BUTTON_PAD_Y_PX, 0))
        self._close_button = tk.Button(
            button_row,
            text="Close",
            command=self._on_close,
            width=8,
            default="active",
        )
        self._close_button.pack(side="right")

        self._close_button.bind("<Return>", lambda _e: self._on_close())
        self.bind("<Escape>", lambda _e: self._on_close())

        self.update_idletasks()
        self._center_on_parent(parent)
        self._close_button.focus_set()
        self.grab_set()

    def _build_groups(
        self,
        body: tk.Frame,
        shortcut_groups: "Sequence[ShortcutGroup]",
        *,
        mono_family: str,
        sans_family: str,
        header_fg: str,
        hairline_color: str,
    ) -> None:
        """Render every shortcut group into *body*.

        Each group becomes an UPPERCASE header label, a hairline rule, and one
        row per ``(key_label, description)`` pair. The created labels are stored
        on :attr:`_header_labels` / :attr:`_row_labels` for test introspection.

        :param body: The padded frame the groups are packed into.
        :param shortcut_groups: Ordered groups to render.
        :param mono_family: Resolved mono font family for key labels.
        :param sans_family: Resolved sans font family for descriptions/headers.
        :param header_fg: Foreground color for category headers.
        :param hairline_color: Color for the rule drawn under each header.
        :return: ``None``.
        """
        mono_font = (mono_family, _SHORTCUTS_FONT_SIZE)
        sans_font = (sans_family, _SHORTCUTS_FONT_SIZE)
        header_font = (sans_family, theme.FONT_SIZE_SECTION, "bold")

        for group_index, (category_title, rows) in enumerate(shortcut_groups):
            # First header hugs the top; later headers get breathing room above.
            top_pad = 0 if group_index == 0 else _SHORTCUTS_HEADER_PAD_TOP_PX

            header_label = tk.Label(
                body,
                text=str(category_title).upper(),
                font=header_font,
                fg=header_fg,
                anchor="w",
            )
            header_label.pack(side="top", fill="x", pady=(top_pad, 0))
            self._header_labels.append(header_label)

            # Hairline rule under the header.
            rule = tk.Frame(
                body,
                height=_SHORTCUTS_HAIRLINE_HEIGHT_PX,
                background=hairline_color,
            )
            rule.pack(side="top", fill="x", pady=(0, _FIELD_PAD_Y_PX))

            for key_label, description in rows:
                self._build_row(
                    body,
                    key_label=str(key_label),
                    description=str(description),
                    mono_font=mono_font,
                    sans_font=sans_font,
                )

    def _build_row(
        self,
        body: tk.Frame,
        *,
        key_label: str,
        description: str,
        mono_font: tuple[str, int],
        sans_font: tuple[str, int],
    ) -> None:
        """Render one shortcut row: mono key label on the left, description right.

        :param body: The frame the row is packed into.
        :param key_label: The key combination text (e.g. ``"Ctrl+S"``).
        :param description: The human-readable action text.
        :param mono_font: ``(family, size)`` tuple for the key label.
        :param sans_font: ``(family, size)`` tuple for the description.
        :return: ``None``.
        """
        row = tk.Frame(body)
        row.pack(side="top", fill="x", pady=(0, _SHORTCUTS_ROW_PAD_Y_PX))

        key = tk.Label(
            row,
            text=key_label,
            font=mono_font,
            width=_SHORTCUTS_KEY_MIN_WIDTH_CH,
            anchor="w",
            justify="left",
        )
        key.pack(side="left")

        desc = tk.Label(
            row,
            text=description,
            font=sans_font,
            anchor="w",
            justify="left",
        )
        desc.pack(side="left", fill="x", expand=True, padx=(_SHORTCUTS_KEY_PAD_X_PX, 0))

        self._row_labels.append((key, desc))

    def _on_close(self) -> None:
        """Dismiss: release the modal grab, restore parent focus, and destroy.

        Each teardown step is guarded so a half-built or already-destroyed
        widget tree (possible in a headless test) cannot raise on close.
        """
        try:
            self.grab_release()
        except tk.TclError:
            pass
        try:
            self._parent.focus_set()
        except tk.TclError:
            pass
        self.destroy()

    def _resolve_mono_family(self) -> str:
        """Resolve the mono font family via the theme, with a safe fallback.

        :return: A concrete font family name -- ``theme.resolve_font`` over
            :data:`theme.FONT_MONO` when it yields a non-empty family, else
            :data:`_MONO_FALLBACK_FAMILY`.
        """
        try:
            family = theme.resolve_font(theme.FONT_MONO)
        except Exception:  # noqa: BLE001 -- font resolution must not crash us.
            family = ""
        return family or _MONO_FALLBACK_FAMILY

    def _resolve_sans_family(self) -> str:
        """Resolve the UI sans font family via the theme, with a safe fallback.

        :return: A concrete font family name -- ``theme.resolve_font`` over
            :data:`theme.FONT_SANS` when it yields a non-empty family, else
            :data:`_SANS_FALLBACK_FAMILY`.
        """
        try:
            family = theme.resolve_font(theme.FONT_SANS)
        except Exception:  # noqa: BLE001 -- font resolution must not crash us.
            family = ""
        return family or _SANS_FALLBACK_FAMILY

    @staticmethod
    def _theme_color(token_name: str, fallback: str) -> str:
        """Read a color token from :mod:`theme`, falling back if it is absent.

        ``theme`` is being extended in parallel, so a token may not yet exist;
        this reads it defensively rather than assuming the attribute is present.

        :param token_name: The attribute name to read from :mod:`theme`.
        :param fallback: A literal ``#rrggbb`` to use if the token is missing.
        :return: The token's value, or *fallback* when it is not a string.
        """
        value = getattr(theme, token_name, None)
        return value if isinstance(value, str) else fallback

    def _center_on_parent(self, parent: tk.Widget | tk.Tk) -> None:
        """Position the dialog at the centre of *parent*.

        Falls back to "no reposition" if the parent reports zero size
        (e.g. a hidden test root).
        """
        try:
            parent.update_idletasks()
            px = parent.winfo_rootx()
            py = parent.winfo_rooty()
            pw = parent.winfo_width()
            ph = parent.winfo_height()
            if pw <= 1 or ph <= 1:
                return
            dw = self.winfo_width()
            dh = self.winfo_height()
            x = px + (pw - dw) // 2
            y = py + (ph - dh) // 2
            self.geometry(f"+{max(x, 0)}+{max(y, 0)}")
        except tk.TclError:
            pass
