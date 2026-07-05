"""tools.reorg -- sketch-src reorg support scripts (issue #66).

Encodes the WRO2026 sketch-tree reorg (moving 39 framework modules from the
flat ``WRO2026/`` root into ``WRO2026/src/<domain>/``) as data + deterministic
transforms, per ``docs/plans/2026-07-05-sketch-src-reorg.md``:

* :mod:`tools.reorg.mapping` -- the file -> destination mapping table (the
  single source of truth every other module and downstream task consumes).
* :mod:`tools.reorg.git_mv_plan` -- lists the `git mv` operations for the 39
  moved files (never runs git itself).
* :mod:`tools.reorg.include_rules` -- the deterministic production
  ``#include "..."`` rewriter (the Contracts "Include-form rules" table).
* :mod:`tools.reorg.test_includes` -- ``WRO2026/test/`` real-vs-mock include
  classification + rewriter.
* :mod:`tools.reorg.rewriter` -- file-walking orchestration over the two
  rewriters above.
* :mod:`tools.reorg.cli` -- the ``python -m tools.reorg ...`` entry point.
"""
