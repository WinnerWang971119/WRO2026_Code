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

## Pull requests

### Title

Same rules as a commit subject: `type(scope): imperative summary`. No bracket
tags, no boilerplate prefixes.

### Description — the main comment

Write a short paragraph, two to five sentences, on **what changed and why**.
That is the whole job. Do not paste a templated `## Summary` / `## Test plan`
skeleton, and do not append a generated footer or attribution line. If there
is something non-obvious to verify, add a couple of bullet lines for it;
otherwise leave it out.

A good description reads like a note to a teammate:

> Sync the top-level docs with the shipped tree. README / CLAUDE / Roadmap
> still called `Manipulator` a stub and claimed there was no host test runner;
> both are wrong now. Docs only, no code change.

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
