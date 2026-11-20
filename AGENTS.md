# AGENTS.md

This repository's agent instructions live in [`CLAUDE.md`](./CLAUDE.md). It is the single source of truth for every coding agent (Claude Code, Codex, Gemini, Cursor, Copilot and others): branch policy, the build and verification gates that match CI, the workspace layout, hardware-safety rules, and the current handoff document.

Read `CLAUDE.md` in full before changing anything. In short:

- Work on `master`; commit and push only when asked; never force-push.
- Before claiming work is done, run the host gates listed in `CLAUDE.md` (`cargo fmt`, `cargo clippy -D warnings`, `cargo test`, `cargo deny check`, `cargo xtask verify-invariants`, `cargo xtask verify-pure-rust`).
- This is a pure-Rust repository: do not add Python, shell, C or assembly source files. `cargo xtask verify-pure-rust` rejects them.
- Never flash, OTA-deploy, or touch the serial port or the LAN device without explicit user confirmation.
- The current state of the migration and the next ordered steps are in `docs/superpowers/plans/2026-11-20-wifi-sta-wpa2-data-plane-handoff-plan.md`.
