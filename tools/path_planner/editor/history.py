"""Undo / redo stack for the live field editor.

The :class:`History` class is a thin coordinator on top of the
:class:`~tools.path_planner.editor.commands.Command` protocol: it pushes
successfully applied commands onto an undo stack, and shuffles them between
the undo and redo stacks as the user invokes :meth:`History.undo` and
:meth:`History.redo`.

Behaviour summary:

- :meth:`do` -- apply the command; on success push it onto the undo stack and
  clear the redo stack (the classical "any new edit invalidates the redo
  branch" rule).  On failure (the command raised), propagate the exception
  and do *not* mutate either stack: the model is presumed untouched.
- :meth:`undo` -- pop the most recent command, revert it, push it onto the
  redo stack.  Returns ``False`` if there is nothing to undo.
- :meth:`redo` -- pop from the redo stack, re-apply it, push it back onto
  the undo stack.  Returns ``False`` if there is nothing to redo.
- :meth:`clear` -- empty both stacks (used by I/O paths that load a fresh
  model from disk).

The stacks are unbounded in memory, per spec.  No persistence is performed.
T12 wires keyboard shortcuts (Ctrl+Z / Ctrl+Y) onto :meth:`undo` and
:meth:`redo`.
"""

from __future__ import annotations

from tools.path_planner.editor.commands import Command
from tools.path_planner.editor.model import FieldModel


class History:
    """LIFO undo + LIFO redo stacks bound to a single :class:`FieldModel`.

    A single :class:`History` instance is owned by the editor application
    (T12).  All user mutations are funnelled through :meth:`do`, and the
    keyboard shortcuts call :meth:`undo` / :meth:`redo`.

    Thread-safety: not thread-safe.  The editor is single-threaded Tk; all
    callers run on the Tk main loop.
    """

    def __init__(self) -> None:
        """Create an empty history with no commands recorded."""
        self._undo_stack: list[Command] = []
        self._redo_stack: list[Command] = []

    # ------------------------------------------------------------------
    # Mutation entry point
    # ------------------------------------------------------------------

    def do(self, command: Command, model: FieldModel) -> None:
        """Apply *command* to *model*, recording it on success.

        On success, *command* is pushed onto the undo stack and the redo
        stack is cleared (any in-progress redo branch is now stale).

        On failure (``command.apply`` raises), the exception propagates and
        neither stack is modified.  The command itself is *not* pushed,
        which matches the documented contract that commands must leave the
        model untouched when they reject their pre-conditions.

        :param command: A :class:`Command` instance to apply.
        :param model: The model to mutate.
        :raises Exception: Whatever ``command.apply`` raises is re-raised
            unmodified.
        """
        command.apply(model)
        # ``apply`` succeeded -- record it and invalidate the redo branch.
        self._undo_stack.append(command)
        self._redo_stack.clear()

    # ------------------------------------------------------------------
    # Undo / redo
    # ------------------------------------------------------------------

    def undo(self, model: FieldModel) -> bool:
        """Revert the most recently applied command.

        :param model: The model the command was originally applied to.
            Must be the same instance ``do`` mutated -- snapshots are not
            sufficient because commands store inverse data referenced to
            the same dict identities.
        :return: ``True`` if a command was reverted, ``False`` if the undo
            stack was empty.
        """
        if not self._undo_stack:
            return False
        command = self._undo_stack.pop()
        command.revert(model)
        self._redo_stack.append(command)
        return True

    def redo(self, model: FieldModel) -> bool:
        """Re-apply the most recently undone command.

        :param model: The model to mutate.  See :meth:`undo` for the
            instance-identity requirement.
        :return: ``True`` if a command was re-applied, ``False`` if the
            redo stack was empty.
        """
        if not self._redo_stack:
            return False
        command = self._redo_stack.pop()
        # Re-apply rebuilds the command's captured inverse data, so a
        # subsequent ``undo`` is safe.  If re-application fails (it
        # shouldn't, since the command succeeded once already with this
        # model state), the exception propagates and the command is
        # discarded -- it is NOT returned to the redo stack, because the
        # model is now in an indeterminate state and the caller needs to
        # see the error.
        command.apply(model)
        self._undo_stack.append(command)
        return True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """Empty both stacks.

        Called by file-load paths (Open, migrate) once the model has been
        replaced wholesale: there is nothing meaningful to undo across a
        full reload.
        """
        self._undo_stack.clear()
        self._redo_stack.clear()

    # ------------------------------------------------------------------
    # State queries (used by menu enable/disable in T12)
    # ------------------------------------------------------------------

    @property
    def can_undo(self) -> bool:
        """``True`` iff there is at least one command on the undo stack.

        :return: Whether :meth:`undo` would do useful work.
        """
        return bool(self._undo_stack)

    @property
    def undo_stack(self) -> tuple[Command, ...]:
        """Read-only view of the undo stack, oldest-applied first.

        Callers that need to re-derive auxiliary state (e.g. the editor's
        pending-EDGES-rename queue) walk this view after every undo / redo
        to keep their state in lockstep with the history. Returning a tuple
        prevents accidental mutation through the view.

        :return: A tuple of :class:`Command` instances in apply order.
        """
        return tuple(self._undo_stack)

    @property
    def can_redo(self) -> bool:
        """``True`` iff there is at least one command on the redo stack.

        :return: Whether :meth:`redo` would do useful work.
        """
        return bool(self._redo_stack)

    @property
    def redo_stack(self) -> tuple[Command, ...]:
        """Read-only view of the redo stack.

        The next command :meth:`redo` will re-apply is the last element
        (``redo_stack[-1]``), mirroring :meth:`undo`'s relationship to
        :attr:`undo_stack`. Callers peek this to mirror a redone command's
        effect onto auxiliary state (e.g. the editor's pending-rename delta)
        without mutating the stack. Returning a tuple prevents that mutation.

        :return: A tuple of :class:`Command` instances; ``[-1]`` is redone next.
        """
        return tuple(self._redo_stack)
