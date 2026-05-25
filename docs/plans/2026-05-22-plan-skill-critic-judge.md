# Plan Skill: Critic + Judge Workflow

**Goal:** Rewrite the `plan` skill so that after a draft spec is produced, an independent critic sub-agent reviews it; Claude reconciles findings (escalating to a judge sub-agent under a tiered-symmetric trigger), optionally asks the user one more question batch if new assumptions surfaced, then reports a summary + file path instead of dumping the full spec.

**Approach:** Insert four new phases between the current Phase 5 (Draft Spec) and the current Phase 6 (Show Full Spec). The save step moves earlier so the critic operates on the persisted file. The "show full spec inline" step is replaced by a summary-and-path message. `~/.claude/skills/plan/skill.md` is the only file edited.

## Scope

- **In scope:**
  - Edit `C:\Users\wangd\.claude\skills\plan\skill.md` to add Phases 5.5 (Save), 5.6 (Critic), 5.7 (Judge under tiered-symmetric trigger), 5.8 (Conditional Re-Question)
  - Renumber existing Phase 6 → 9, Phase 7 → 10, Phase 8 → 11 (Learning stays last)
  - Define the critic sub-agent prompt template (`general-purpose` sub-agent, prompt is self-contained, receives only original user request verbatim + saved spec file path + explicit permission to Read named reference files)
  - Define the judge sub-agent prompt template (3-way ruling: critic / Claude / synthesis)
  - Define the tiered-symmetric judge-fire rule
  - Define the trigger rule for the conditional user re-question batch
  - Define the summary-message format presented to the user (with 0-finding fallback)
  - Update the Red Flags table to reflect the new workflow
  - Update the top-level process `digraph` to reflect new phase ordering
- **Out of scope:**
  - Changes to the `execute` skill
  - Changes to the `structured-debug` skill's plan handoff (the Phase 0 short-circuit still works)
  - Creating a new agent type — reuse `general-purpose`
  - Persisting critic/judge transcripts to disk (only the final spec is saved; critique lives in the conversation, ephemeral)
  - Any project code edits in the WRO2026 repo
  - Fixing the existing Phase 8 (Learning) `.claude/memory/` save patterns even though they violate the user's mem0 hygiene rule (separate fix in a future PR)

## Decisions

- **Judge fires on a tiered-symmetric trigger** — (a) for every CRITICAL-severity finding, the judge always rules regardless of whether Claude accepts or dismisses; (b) for WARNING and NIT findings, the judge fires only when Claude dismisses (accepting is free). Rationale: critical findings have asymmetric downside in both directions (wrong accept ships a bug; wrong dismiss hides one), so they warrant a tribunal unconditionally. Lower-severity findings have bounded downside, so the cheaper asymmetric path is acceptable. Mitigation for critic mislabeling: Claude may dismiss any finding it believes is mislabeled (e.g., a real critical tagged as nit), which triggers the standard asymmetric judge fire and surfaces the severity dispute as part of the ruling. *(Established by judge ruling 2026-05-22 on the meta-spec; resolved a rubber-stamp-bias finding from the critic.)*
- **Critic runs after save, not before** — the artifact is persisted first so the critic operates on the same bytes the user will later read; also lets the user open the file in parallel while critic runs.
- **User must NOT open the spec file until the summary message arrives** — Claude tells the user "spec drafted; running review, do not open yet" before dispatching the critic, to avoid the read-vs-edit race. Once the summary lands, the file is final-for-this-revision.
- **Revisions overwrite the saved spec in place** — no version suffix. The conversation transcript is the audit log; git can be used by the user if they want versioned history.
- **Critic is `general-purpose` with a self-contained prompt** — the prompt receives the original user request verbatim AND explicit permission to Read a named set of reference files: the saved spec, `~/.claude/CLAUDE.md` (user-global rules), and the relevant project's `CLAUDE.md`. The critic does NOT receive the Q&A transcript and does NOT preload arbitrary codebase context. "Blank-slate" means "no plan-session memory and no implicit preloads," not "cannot read anything."
- **"Original user request" = the first user-authored message in the plan session, captured verbatim** before any Q&A transformation. Stored as a string Claude carries through Phase 5 for the critic prompt.
- **For skill-file plans (plans that edit a file under `~/.claude/skills/`), the critic does NOT check project ADRs** — only `~/.claude/CLAUDE.md`. Project ADRs are scoped to the project, not user-global skills.
- **Critique scope is fixed (5 axes)** — scope/assumptions, task graph correctness, approach vs alternatives, AC/test coverage, "wrong truth" (API claims, codebase contradictions, CLAUDE.md/ADR violations, internal logical contradictions). Encoded in the critic prompt.
- **Judge can rule 3-way** — critic wins, Claude wins, or propose synthesis. Synthesis must be a concrete spec edit, not "consider both."
- **User re-question batch is conditional** — only fires when an *accepted* critic finding (after any judge ruling) introduces an assumption Claude cannot resolve from existing context (codebase + CLAUDE.md + prior Q&A).
- **User sees a summary, not the full spec** — summary includes: one-paragraph spec recap, critic findings table (status: accepted / dismissed-by-judge / synthesized), file path. The trade is that the user must open the file to approve intelligently; this is intentional cost of removing the inline spec dump.
- **Critic/judge transcripts are ephemeral** — they live in the conversation only. If the user wants to retain them, they can scroll up; nothing is persisted.
- **Token cost is accepted as the cost of the feature** — the user requested critic+judge knowing it adds sub-agent invocations. The skill text will note the cost so future users can opt out by skipping the skill on trivial plans.

### Alternatives considered (rejected)

| Alternative | Why rejected |
|---|---|
| **Inline critic reflection** — Claude self-critiques without sub-agent | Defeats the point: same model, same context, same biases. Would not catch Claude's own assumptions. |
| **Two parallel critics with a vote** | 2× cost for marginal benefit; two general-purpose agents in parallel often converge on the same findings since they have identical prompts. |
| **Critic-then-revise without judge** | Removes the bias-catching brake. Critic becomes dictator; Claude has no formal way to push back. |
| **Symmetric judge on every finding** | Heaviest cost (5–30 findings × judge call); most findings are clearly correct or clearly wrong. Tiered-symmetric concentrates judge spend where bias is most expensive. |

## Acceptance Criteria

- [ ] AC1: `skill.md` contains 12 phases in this order: Phase 0 (Debug Handoff, unchanged), Phase 1 (Explore), Phase 2 (Ask), Phase 3 (Confirm), Phase 4 (Approaches), Phase 5 (Draft), Phase 5.5 (Save), Phase 5.6 (Critic), Phase 5.7 (Judge), Phase 5.8 (Re-Question), Phase 9 (Summary), Phase 10 (Approval & Handoff), Phase 11 (Learning). Decimal numbering for the four new phases is intentional to minimize diff churn in cross-references.
- [ ] AC2: `skill.md` contains a critic prompt template specifying: (a) self-contained prompt with original user request verbatim, (b) 5-axis critique scope, (c) explicit list of files the critic is permitted to Read, (d) output format (numbered findings with severity tags `critical|warning|nit`).
- [ ] AC3: `skill.md` contains a judge prompt template specifying: receives critic finding + Claude rebuttal + spec excerpt, must return one of `{CRITIC_WINS, CLAUDE_WINS, SYNTHESIS: "<concrete edit>"}`.
- [ ] AC4: `skill.md` specifies the tiered-symmetric judge-fire rule in plain prose: critical = always; warning/nit = only on dismissal.
- [ ] AC5: `skill.md` specifies the re-question trigger rule and requires the new question batch to reference the specific accepted finding that introduced the new assumption (no generic "any thoughts?" questions).
- [ ] AC6: `skill.md` specifies the summary message format. Format must render correctly with 0 findings (table omitted, replaced by "No findings.").
- [ ] AC7: `skill.md`'s process `digraph` reflects the new ordering: save → critic → judge? → re-question? → summary → approval.
- [ ] AC8: `skill.md`'s HARD-GATE still blocks any implementation skill until user approval; approval now happens after summary + path, not after inline spec dump.
- [ ] AC9a: Red Flags table has the "Showing spec section-by-section" row REMOVED.
- [ ] AC9b: Red Flags table has THREE new rows added: "Skipping critic step," "Dismissing critic finding without judge (when judge is required by tiered rule)," "Dumping full spec in chat instead of summary + path."
- [ ] AC10: `skill.md` notes the token-cost implication of critic+judge in a single sentence near the top of Phase 5.6.
- [ ] AC11: `skill.md` includes a one-sentence instruction in Phase 5.5 telling Claude to message the user "spec drafted; running review, do not open the file yet" before dispatching the critic.

## Critic Prompt Template (as it will appear in `skill.md`)

```
You are a critic sub-agent reviewing a planning spec. You operate self-contained:
your only context is what this prompt provides. You have NO memory of the plan
session that produced the spec.

DO NOT defer to the spec author. DO NOT soften findings. Your job is adversarial review.

Inputs:
- Original user request (verbatim, captured before any Q&A): <verbatim string>
- Spec file path: <path>

You are permitted to use Read on these files only:
- The spec file (path above)
- `~/.claude/CLAUDE.md` (user-global rules)
- The project's `CLAUDE.md` (only if the spec edits a file inside the project, NOT for skill-file plans)
- Files explicitly cited by the spec (e.g., if spec says "edits src/foo.ts", you may Read src/foo.ts)

Do NOT preload arbitrary codebase context.

Review along these 5 axes. For each axis, list specific findings (or "no issues"):

1. SCOPE / ASSUMPTIONS — What's missing? What's implicitly assumed but not stated?
2. TASK GRAPH — File conflicts between parallel tasks? Missing "Blocked By"? Wrong risk levels?
3. APPROACH — Is the chosen approach actually best? Is a rejected alternative stronger?
   (Skip this axis if the spec came from a debug-handoff and Phase 4 was skipped.)
4. ACCEPTANCE CRITERIA & TESTS — Are ACs testable? Does every AC have at least one test case?
   Are failure modes tested? Does any test case verify negative-path content, not just presence?
5. WRONG TRUTH — Verify factual claims:
   - External API / library claims (do referenced tools/methods really exist?)
   - Codebase contradictions (does file/class Y really exist as described?)
   - CLAUDE.md / ADR rule violations
   - Internal logical contradictions (does the spec contradict itself?)

Output format:
## Axis N: <name>
- [critical|warning|nit] <finding> — <suggested fix>

Be specific. Cite line numbers or section names. No vague "consider X."
End with: "TOTAL: <N> critical, <N> warning, <N> nit"
```

## Judge Prompt Template (as it will appear in `skill.md`)

```
You are a judge sub-agent ruling on a single contested finding from a planning spec review.

You receive:
- The original spec excerpt (the relevant section, not the whole spec)
- The critic's finding (verbatim, with severity tag)
- Claude's rebuttal: either "ACCEPT" or a written justification for dismissal/severity-dispute
- The fire reason: either "CRITICAL_AUTO" (tiered rule auto-fired) or "OVERRIDE" (Claude dismissed)

Rule ONE of:

1. CRITIC_WINS — the critic is correct; the spec must be edited per their suggestion.
2. CLAUDE_WINS — Claude's rebuttal holds; the finding is dismissed.
3. SYNTHESIS — both have a point; propose a concrete spec edit that addresses the
   critic's concern without giving up Claude's reason. Must be a specific edit, not
   "consider both." Provide the exact text change.

Output exactly:
RULING: <CRITIC_WINS|CLAUDE_WINS|SYNTHESIS>
REASONING: <2-3 sentences>
SPEC_EDIT (if CRITIC_WINS or SYNTHESIS): <the exact spec change to make>
```

## User Summary Message Format (as it will appear in `skill.md`)

When there are findings:

```
**Spec drafted and saved.**

<one-paragraph recap of what the spec covers — Goal + Approach in plain prose>

**Critic findings (<N> total: <X> critical, <Y> warning, <Z> nit):**
| # | Axis | Severity | Status | Note |
|---|------|----------|--------|------|
| 1 | ... | critical | accepted, applied | ... |
| 2 | ... | warning | dismissed (judge: CLAUDE_WINS) | ... |
| 3 | ... | nit | synthesized | <edit summary> |

**Full spec:** `docs/plans/YYYY-MM-DD-<topic>.md`

Approve, or tell me what to change.
```

When there are 0 findings:

```
**Spec drafted and saved.**

<one-paragraph recap of what the spec covers — Goal + Approach in plain prose>

**Critic findings:** No findings.

**Full spec:** `docs/plans/YYYY-MM-DD-<topic>.md`

Approve, or tell me what to change.
```

## Error Handling

- **Critic sub-agent fails or times out:** Log the failure, proceed with summary noting "Critic step skipped: <reason>". Do not block the user on infrastructure issues.
- **Judge sub-agent fails:** Default to CRITIC_WINS (safer to apply the critique than silently dismiss it). Note this in the summary.
- **Critic finding cites a non-existent file/symbol:** Treat as nit at most — critic was wrong about reality. Document in the rebuttal that goes to the judge (auto-fires if the finding was tagged critical).
- **Critic surfaces > 10 findings:** Group by axis in the summary table, show top 3 per axis with "+ N more" link. Full list is ephemeral (lives only in conversation).
- **User opens the spec file mid-review (despite warning):** Not enforceable. If detected via file-modification time changing, abort the edit and ask user to re-trigger. Otherwise, last-write-wins.

## Testing Strategy

**Levels:** Manual (skills aren't unit-tested; verification is by running the skill).

| ID  | Test Case                                                                              | Type   | Expected Behavior                                                                                                  |
|-----|----------------------------------------------------------------------------------------|--------|--------------------------------------------------------------------------------------------------------------------|
| TC1 | Invoke `/plan` on a small feature ("add LED blink on button press")                    | Manual | All phases execute in new order; critic runs; summary message appears; HARD-GATE prevents `execute` until approval. |
| TC2 | Plan with 0 critic findings                                                            | Manual | Summary uses the "No findings." fallback; goes straight to approval prompt.                                        |
| TC3a | Doc check: `skill.md` contains guard text "Claude MUST summon judge before dismissing a critical finding" | Manual (text inspection) | Text is present verbatim in Phase 5.6 or 5.7. |
| TC3b | Behavior check: in a plan run with a critical finding, Claude tries to dismiss it      | Manual | Skill summons judge automatically; judge ruling is reflected in the summary table.                                  |
| TC4 | Plan where critic surfaces a new assumption                                            | Manual | Phase 5.8 (Re-Question) fires; the question text references the specific accepted finding (not generic).            |
| TC5 | Plan where critic surfaces only nits, no new assumptions                                | Manual | Phase 5.8 skipped; goes straight to summary.                                                                       |
| TC6 | Plan invoked from `structured-debug` handoff                                            | Manual | Phase 0 short-circuit still works; critic still runs; critic's Axis 3 (APPROACH) is auto-skipped per its prompt.   |
| TC7 | Plan where Claude dismisses a warning finding; judge rules CRITIC_WINS                  | Manual | Finding is applied to the spec on disk; summary shows "dismissed (judge: CRITIC_WINS, finding applied)."           |
| TC8 | Plan where Claude dismisses a warning finding; judge rules SYNTHESIS                    | Manual | Judge's SPEC_EDIT is applied verbatim to the spec on disk; summary shows "synthesized" with the edit summary.      |
| TC9 | Plan with a CRITICAL finding that Claude *accepts*                                      | Manual | Judge still fires (tiered-symmetric auto-fire); judge ruling is reflected; even auto-accept goes through tribunal. |
| TC10 | Token-cost note check                                                                  | Manual (text inspection) | `skill.md` Phase 5.6 contains a one-sentence cost note.                                                            |

**Test data:** Use a throwaway plan task ("add a /version command") to walk through the flow end-to-end after edits.
**Run command:** Type `/plan <task>` in Claude Code and observe.

## Tasks

> **Note:** All edits touch the same file (`skill.md`), so tasks are single-threaded. The Blocked By column enforces a strict linear order; the task graph here is bookkeeping, not parallelism enablement.

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Renumber existing Phases 6 → 9, 7 → 10, 8 → 11 in `skill.md`; fix all internal cross-references | — | med | `C:\Users\wangd\.claude\skills\plan\skill.md` | Renumber can break cross-refs (anchors, "see Phase X" mentions). Med risk. Satisfies AC1 (numbering part). Three source phases → three target numbers (6→9, 7→10, 8→11). |
| T2 | Insert new Phase 5.5 (Save Spec to File) — move the save-file step out of old Phase 7; include the "do not open the file yet" instruction | T1 | med | `C:\Users\wangd\.claude\skills\plan\skill.md` | Save happens before user sees the spec. Satisfies AC1 + AC11. Do NOT touch the digraph here (consolidated to T7). |
| T3 | Insert new Phase 5.6 (Critic Sub-Agent) with the prompt template from this spec; include the token-cost note | T2 | med | `C:\Users\wangd\.claude\skills\plan\skill.md` | Use the `Agent` tool (top-level, not deferred). Specify `subagent_type: general-purpose`. Self-contained prompt, 5-axis output, explicit Read allowlist. Satisfies AC2 + AC10. |
| T4 | Insert new Phase 5.7 (Judge Sub-Agent) with the prompt template and tiered-symmetric trigger rule | T3 | med | `C:\Users\wangd\.claude\skills\plan\skill.md` | Tiered: critical = always; warning/nit = on dismissal only. Include the guard text "Claude MUST summon judge before dismissing a critical finding" verbatim. Satisfies AC3 + AC4. |
| T5 | Insert new Phase 5.8 (Conditional User Re-Question Batch) | T4 | low | `C:\Users\wangd\.claude\skills\plan\skill.md` | Trigger rule. Require that the question references the specific accepted finding. Satisfies AC5. |
| T6 | Replace old Phase 6 (Show Full Spec) content with new Phase 9 (Summary + Path Message) including both with-findings and 0-findings formats | T5 | low | `C:\Users\wangd\.claude\skills\plan\skill.md` | Satisfies AC6. |
| T7 | Update the top-level process `digraph` to reflect new ordering (save → critic → judge? → re-question? → summary → approval) | T6 | low | `C:\Users\wangd\.claude\skills\plan\skill.md` | All digraph edits consolidated here. Satisfies AC7. |
| T8 | Update HARD-GATE wording so approval = after summary, not after inline spec | T6 | low | `C:\Users\wangd\.claude\skills\plan\skill.md` | One-line tweak. Satisfies AC8. |
| T9 | Update Red Flags table: remove the "Showing spec section-by-section" row; add three new rows | T6 | low | `C:\Users\wangd\.claude\skills\plan\skill.md` | Satisfies AC9a + AC9b. |
| T10 | Manually walk through TC1–TC10 on a throwaway plan task | T1–T9 | low | (none — runtime verification) | No agent assignment; this is human-driven verification. |

## Notes for Implementer

- The `Agent` tool is a top-level (non-deferred) tool — its schema is already loaded. Use `subagent_type: "general-purpose"` for both critic and judge invocations.
- Use a triple-backtick fenced block for the new prompt templates so they render as code in the skill body.
- Don't break the existing `<HARD-GATE>` tag — only adjust its wording.
- Phase 0 (debug handoff short-circuit) stays at the top, untouched.
- Phase 11 (Learning) keeps its Mem0 Persistence sub-section unchanged. (The fact that the Learning section's `.claude/memory/` save patterns may violate the user's broader mem0 hygiene rule is acknowledged as out-of-scope for this plan; address in a separate PR.)
- Rollback: `skill.md` is in `~/.claude/skills/plan/`. Keep a copy of the pre-edit file (`skill.md.bak`) until TC1–TC10 pass; delete on success.
- When the critic returns findings, Claude must process them in severity order: critical first (each auto-fires judge), then warning/nit (judge only on dismissal). Process all critical findings before deciding on any warning/nit.
