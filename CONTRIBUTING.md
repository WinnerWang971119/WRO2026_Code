# WRO 2026 — Contributing

How we file pull requests and issues on this repo. Keep it human, keep it
short, anchor everything to the code.

## Branches

- Feature branches only. Never commit to `main`.
- Name the branch by intent, matching the prefixes already in `git log`:
  `feat/<slug>`, `fix/<slug>`, `docs/<slug>`, `test/<slug>`, `chore/<slug>`.
- One concern per branch so the PR stays reviewable.

## Commits

- Match the repo's Conventional-Commit style — `type(scope): subject`, e.g.
  `feat(editor): override edge type in place`, `fix(routes): regenerate
  routes.h/.cpp`, `docs: sync README with current code`.
- Subject: lowercase, imperative, no trailing period.
- Body (optional): 1–3 sentences on *why*, not a play-by-play of *how*.
- Write it the way a person would. No machine-generated footers, no
  AI-attribution trailers.
- If you touch the field or the emitter, regenerate `routes.h` / `routes.cpp`
  and commit them in the **same** commit — CI fails on a stale header.

## Documentation-first rule

- Before writing or changing code, read the Doxygen comments for the target
  module and any related APIs.
- Confirm the public contract first: preconditions, postconditions,
  blocking/async behavior, and return/error conventions.
- If the docs are missing or unclear, update the docs first (or in the same
  PR) before implementation.
- In the PR description, include one explicit line:
  `Doxygen reviewed for affected APIs before implementation.`

## Pull requests

### Title

Same rules as a commit subject: `type(scope): imperative summary`. No bracket
tags, no boilerplate prefixes.

### Description — the main comment

Write it in **rich Markdown with clear sections**, so a teammate (or your future
self) can review it and try the change on the bench without reading the diff.
Use headings, lists, code fences, and tables where they help. Cover:

- **What this PR does** — the change and the *why* behind it, anchored to the
  modules and files it touches.
- **How it affects the robot** — the on-field / runtime behavior that changes
  (motion, sensing, mission flow). Write "no runtime effect" for tooling / test
  / docs changes, and call out when a change is latent (built, but not yet on an
  exercised path).
- **How to manually test** — the exact commands or bench steps to exercise it
  (`arduino-cli` build / flash, `uv run ...`, a CTPS render, a calibration-menu
  path) and what a correct result looks like.

Keep the prose human — no AI-attribution trailers or machine-generated footers,
and do not pad the sections with a hollow `## Summary` / `## Test plan` skeleton
that restates the headings without content. For a trivial change (typo, dep
bump) a sentence or two under **What this PR does** is enough; drop the sections
that do not apply.

If an issue exists, link it so the merge closes it: `Closes #NN`.

### Review — inline comments are preferred

Leave review feedback as **inline comments on the specific lines or hunks** it
applies to, not as one big summary comment at the bottom. Inline comments keep
each point next to the code that triggered it, are easier to resolve one at a
time, and read better in the thread. Save the top-level review comment for a
one-line verdict (approve / needs work) or a remark that genuinely spans the
whole change.

## Issues

### Title

Short and descriptive — state the problem or the ask, not the solution.

### Labels are required — we have many, so use them

Every new issue gets **at least one type label and one area label**, plus a
priority once it is known. The taxonomy:

| Group | Labels | Use |
|---|---|---|
| Type | `bug`, `enhancement`, `documentation`, `question` | what kind of work it is |
| Priority | `P0: critical`, `P1: high`, `P2: medium`, `P3: low` | `P0` blocks a scoring run; `P3` is do-when-triggered |
| Area | `area: nav`, `area: editor`, `area: sim/ctps`, `area: vision`, `area: testing`, `area: tooling` | which subsystem it touches |
| Workflow | `epic`, `blocked`, `needs-design`, `deferred` | `epic` = parent tracking issue; `blocked` = waiting on another issue or hardware; `needs-design` = idea-stage, design first; `deferred` = parked until a trigger |
| Triage | `good first issue`, `help wanted`, `duplicate`, `invalid`, `wontfix` | onboarding + housekeeping |

Apply them when you open the issue:

```
gh issue create --title "..." --body "..." \
  --label "bug" --label "area: nav" --label "P1: high"
```

### Conventions

- A bug in **built** code gets a `bug` issue and a fix now, even if the
  current test map does not exercise that path — see the note in
  [`CLAUDE.md`](./CLAUDE.md). Only a genuinely unbuilt feature gets `deferred`.
- Use `epic` for a parent that fans out into sub-issues; link the children
  back to it.
- Reach for `needs-design` before writing code on anything idea-stage.

## See also

- [`CLAUDE.md`](./CLAUDE.md) / [`AGENTS.md`](./AGENTS.md) — coding rules and the
  ADR process.
- [`DECISIONS.md`](./DECISIONS.md) — when a change deserves an ADR.
