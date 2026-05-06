"""CLI demo: run the PathTracker simulator on the toy field and print the trace.

Usage::

    uv run python -m tools.path_planner.sim.demo

By default runs the same sequence the R4's NAV_DEMO_MODE will run::

    goTo("C")
    goTo("G")

starting from `field_data.START_NODE` facing `field_data.START_HEADING_DEG`.
Every emitted primitive is printed with the chassis pose AFTER it executes,
plus a final summary of the cumulative pose.

The optional ``--goals A,B,C`` flag overrides the default pair so you can
explore other paths without editing this file.

The optional ``--plot`` flag renders the chassis trace on top of the
existing matplotlib visualisation (`tools/path_planner/output/sim_trace.png`).

This module is purely a developer aid; it is NOT exercised by the test
suite. Tests live in `tools/path_planner/tests/test_runtime_sim.py`.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Iterable, List

# Allow running as ``python tools/path_planner/sim/demo.py`` (no -m flag) by
# threading the repo root onto sys.path. ``uv run python -m tools...`` works
# regardless because uv puts the project root on the path.
_HERE = Path(__file__).resolve()
_REPO_ROOT = _HERE.parent.parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tools.path_planner import field_data as toy_field_data
from tools.path_planner.compiler import (
    ACT_TURN_REL,
    ACT_TURN_TO,
    Primitive,
)
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import compute_all_pairs_next_hop
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    PathTrackerSim,
    UnreachableGoalError,
    walk_next_hop,
)


def _format_primitive(p: Primitive) -> str:
    """Render a primitive as a readable one-line string."""
    action, value = p
    if value is None:
        return f"{action}()"
    if action in (ACT_TURN_REL, ACT_TURN_TO):
        return f"{action}({value:+.3f} deg)"
    return f"{action}({value:.3f} cm)"


def _print_step(idx: int, primitive: Primitive, chassis: VirtualChassis) -> None:
    print(
        f"  step {idx:>2}  {_format_primitive(primitive):<40}"
        f"-> pose ({chassis.x_cm:7.3f}, {chassis.y_cm:7.3f}) "
        f"heading {chassis.heading_deg:7.3f}"
    )


def _run_one_goto(
    sim: PathTrackerSim,
    goal: str,
    step_counter: List[int],
    plot_traces: List[List[tuple]],
) -> bool:
    """Execute one goTo. Returns True on success, False if the FSM faulted.

    Mirrors PathTracker's C++ behaviour: an UNREACHABLE fault aborts the
    leg without crashing the program — the operator sees a clear message
    and the demo continues to attempt subsequent goals (which will also
    fault if currentNode hasn't advanced, but the chassis is correctly
    parked, EventTrace would have surfaced NAV FAULT UNR, etc.).
    """
    print(f"\n>>> goTo({goal!r})")
    print(
        f"    start pose ({sim.chassis.x_cm:7.3f}, {sim.chassis.y_cm:7.3f}) "
        f"heading {sim.chassis.heading_deg:7.3f}; "
        f"current_node = {sim.localization.current_node()!r}"
    )
    try:
        walked = walk_next_hop(
            sim._next_hop, sim.localization.current_node(), goal
        )    # pylint: disable=protected-access
    except UnreachableGoalError as err:
        print(
            f"    !! UNREACHABLE: NEXT_HOP[{err.current!r}][{goal!r}] = None\n"
            f"    !! C++ runtime would enter Fault::UNREACHABLE here and brake.\n"
            f"    !! (note: blind edges are unidirectional by design — goal "
            f"nodes have no outgoing edges in the toy graph.)"
        )
        return False
    print(f"    NEXT_HOP walk: {' -> '.join(walked)}")

    leg_trace = [(sim.chassis.x_cm, sim.chassis.y_cm)]
    try:
        for primitive, snapshot in sim.step_through(goal):
            step_counter[0] += 1
            _print_step(step_counter[0], primitive, snapshot)
            leg_trace.append((snapshot.x_cm, snapshot.y_cm))
    except UnreachableGoalError as err:
        # Defensive: walk_next_hop already proved the path exists, but a
        # mid-traverse fault could still occur in a future planner change.
        print(
            f"    !! UNREACHABLE mid-traverse at {err.current!r} -> {goal!r}"
        )
        plot_traces.append(leg_trace)
        return False
    plot_traces.append(leg_trace)
    return True


def _maybe_plot(
    sequence_traces: List[List[tuple]],
    field_data,
    out_path: Path,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:    # pragma: no cover -- matplotlib is a project dep
        print("matplotlib not available — skipping --plot", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    for name, (x, y) in field_data.NODES.items():
        ax.scatter(x, y, c="black", s=50, zorder=3)
        ax.annotate(
            name,
            (x, y),
            textcoords="offset points",
            xytext=(6, 6),
            fontsize=9,
            zorder=4,
        )
    palette = ["tab:green", "tab:blue", "tab:orange", "tab:red"]
    for i, trace in enumerate(sequence_traces):
        xs = [p[0] for p in trace]
        ys = [p[1] for p in trace]
        ax.plot(
            xs,
            ys,
            color=palette[i % len(palette)],
            linewidth=2,
            marker="o",
            markersize=4,
            label=f"leg {i + 1}",
            zorder=2,
        )
    for x_min, y_min, x_max, y_max, _kind in field_data.OBSTACLES:
        ax.add_patch(
            plt.Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                facecolor="lightgray",
                edgecolor="gray",
                alpha=0.6,
                zorder=1,
            )
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title("PathTracker simulator trace")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"\nWrote trace plot -> {out_path}")


def main(argv: Iterable[str] | None = None) -> int:
    """CLI entry point. Returns a process exit code."""
    parser = argparse.ArgumentParser(
        description="Run the PathTracker simulator on the toy field."
    )
    parser.add_argument(
        "--goals",
        default="C,G",
        help="Comma-separated list of goal node names. Default: C,G "
             "(matches WRO2026.ino's NAV_DEMO_MODE).",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Also render the chassis trace as PNG to "
             "tools/path_planner/output/sim_trace.png.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    goals = [g.strip() for g in args.goals.split(",") if g.strip()]
    if not goals:
        print("error: --goals must contain at least one goal name", file=sys.stderr)
        return 2

    print("=" * 70)
    print("PathTracker simulator demo")
    print("=" * 70)
    print(f"start node:    {toy_field_data.START_NODE}")
    print(f"start heading: {toy_field_data.START_HEADING_DEG} deg")
    print(f"goals:         {goals}")

    graph = build_graph(toy_field_data)
    next_hop = compute_all_pairs_next_hop(
        graph, toy_field_data.COST_BLIND_MULTIPLIER
    )
    chassis = VirtualChassis.from_field_data(toy_field_data)
    sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    print(f"\nLocalization zeroed at IMU yaw 0 -> world heading "
          f"{toy_field_data.START_HEADING_DEG} deg.")

    step_counter = [0]
    sequence_traces: List[List[tuple]] = []

    aborted_at: int | None = None
    for i, goal in enumerate(goals, start=1):
        if goal not in toy_field_data.NODES:
            print(f"error: goal {goal!r} is not a known node "
                  f"(known: {sorted(toy_field_data.NODES.keys())})",
                  file=sys.stderr)
            return 2
        ok = _run_one_goto(sim, goal, step_counter, sequence_traces)
        if not ok:
            aborted_at = i
            break

    print("\n" + "=" * 70)
    print("FINAL POSE")
    print("=" * 70)
    print(f"x:       {chassis.x_cm:.3f} cm")
    print(f"y:       {chassis.y_cm:.3f} cm")
    print(f"heading: {chassis.heading_deg:.3f} deg")
    print(f"current_node: {sim.localization.current_node()}")
    print(f"primitives total: {step_counter[0]}")
    if aborted_at is not None:
        print(
            f"\nNOTE: aborted at goal #{aborted_at} ({goals[aborted_at - 1]!r}); "
            f"{len(goals) - aborted_at} subsequent goals skipped."
        )

    if args.plot:
        out = _REPO_ROOT / "tools" / "path_planner" / "output" / "sim_trace.png"
        _maybe_plot(sequence_traces, toy_field_data, out)

    return 1 if aborted_at is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
