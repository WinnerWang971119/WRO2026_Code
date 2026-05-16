"""Pure-Python end-to-end simulator for the WRO 2026 PathTracker R4 runtime.

This package mirrors the C++ FSM in :mod:`WRO2026/PathTracker.cpp` so that
end-to-end navigation behaviour can be verified entirely on the laptop
without flashing a real robot. It is the verification bench for Phase 2
PR 2 — the user opted out of physical-fixture testing this PR.

Public surface:

* :class:`tools.path_planner.sim.chassis.VirtualChassis` — minimal
  ``(x, y, heading_deg)`` bookkeeping with primitive transitions
  (``turn_relative``, ``turn_to_heading``, ``follow_line_to_intersection``,
  ``follow_line_distance``, ``forward``).
* :class:`tools.path_planner.sim.runtime.LocalizationSim` — Python
  mirror of ``LocalizationClass``: holds ``current_node`` and the
  world-frame yaw offset captured at start-button press.
* :class:`tools.path_planner.sim.runtime.PathTrackerSim` — the FSM
  driver. ``go_to(goal_name)`` walks ``Routes::NEXT_HOP`` (here a Python
  dict from :func:`tools.path_planner.planner.compute_all_pairs_next_hop`)
  and returns the dispatched primitive sequence while mutating the
  attached chassis.

The simulator is intentionally a level above the underlying C++ runtime
in two ways:

1. It does NOT model time, encoder pulses, IMU drift, or motor PWM —
   each primitive applies its full effect atomically. PR 2 PathTracker
   advances state on tick boundaries; the sim collapses every dispatch +
   await pair into a single deterministic transition.
2. It mirrors compile_path's primitive emission rather than re-walking
   it independently. The end-to-end equivalence proof is then a check
   that the sim's emitted sequence equals
   ``compile_path(walked_path, start_heading, graph, field_data)``.
"""

from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    LocalizationSim,
    PathTrackerSim,
    UnreachableGoalError,
    walk_next_hop,
)

__all__ = [
    "VirtualChassis",
    "LocalizationSim",
    "PathTrackerSim",
    "UnreachableGoalError",
    "walk_next_hop",
]
