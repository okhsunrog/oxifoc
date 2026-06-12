# Documentation map

One-type-per-file rule: history lives in git and in `archive/`; working
docs do not accumulate "done".

| File / folder | Type | Update rule |
|---|---|---|
| [architecture.md](architecture.md) | how it works NOW | edited together with the code; no history |
| [safety.md](safety.md) | failsafe layers design | living design doc |
| [bench-protocol.md](bench-protocol.md) | hardware bench session: tests, commands, analysis recipes | updated per session; results feed TODO/decisions |
| [TODO.md](TODO.md) | OPEN work only | done items are deleted (history = git log + archive), never marked `[x]` |
| [decisions.md](decisions.md) | decisions + why | append-only, 2–5 lines per decision + a pointer |
| [flash-size.md](flash-size.md) | flash budget: numbers, rules, measured reserves | numbers refreshed via `just size`; the measurement history table stays |
| [register-access.md](register-access.md) | rc_w0/rc_w1 access patterns | reference |
| [perf-bench-2026-06-11.md](perf-bench-2026-06-11.md) | performance measurements | data; new benches → a new file |
| [hw/](hw/) | board/chip facts + PDFs | hardware references |
| [notes/](notes/) | research / RFCs / reference comparisons | every file starts with a Status header; landed parts shrink to one-line pointers (details → decisions.md/architecture.md), open parts stay |
| [archive/](archive/) | frozen documents | never edited |

Where things go:

- Fixed a bug / landed a feature → strike it from TODO.md (delete), adjust
  architecture.md if needed; if the decision is non-obvious — a line in
  decisions.md.
- Decided NOT to do something → decisions.md (won't-fix with rationale).
- Studied reference code / drafted a plan → notes/ with a Status header
  (`open` / `partially landed` / `landed, remainder: ...`).
- A "borrow someday" idea → [notes/borrow-list.md](notes/borrow-list.md).
