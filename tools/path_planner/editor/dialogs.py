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
from typing import Optional


# Allowed node-type values, mirroring panels._NODE_TYPE_VALUES and the
# validator's rules. Duplicated here so dialogs.py doesn't import panels.
_NODE_TYPE_VALUES: tuple[str, ...] = ("line", "blind")
_DEFAULT_NODE_TYPE: str = "line"

# Dialog layout constants -- no magic numbers in the code body.
_DIALOG_PADDING_PX: int = 10
_FIELD_PAD_Y_PX: int = 4
_BUTTON_PAD_Y_PX: int = 8
_ENTRY_WIDTH_CH: int = 24
_COMBO_WIDTH_CH: int = 22


class AddNodeDialog(tk.Toplevel):
    """Modal Toplevel that collects (name, type) for a new node.

    The dialog is constructed, packed, and made modal via
    ``transient(parent) + grab_set()``. The owning :class:`EditorApp`
    then calls ``wait_window()`` on this instance and reads ``.result``:

    * ``(name_str, type_str)`` on successful OK click -- *name_str* is the
      already-stripped operator entry; *type_str* is one of
      ``"line"`` / ``"blind"``.
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
