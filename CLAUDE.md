# CLAUDE.md

This file provides the Claude Code entry points for this repository. Project
rules are intentionally maintained in shared, tool-neutral files to prevent
different AI clients from using conflicting build or debug procedures.

## Project Instructions

Read and follow `AGENTS.md` for repository architecture, build commands, coding
conventions, and safety constraints.

The root `Makefile` and `make.bat` are authoritative for build modes, target
selection, flashing, and RTT startup. If documentation and these scripts differ,
inspect the scripts and correct the documentation rather than guessing.

## MCU Debugging Skill

Before debugging firmware, read and follow:

`.agents/skills/aitrace/SKILL.md`

Use `.\tools\aitrace.exe` as the default AI debugging entry point. Begin with
passive RTT, waveform, or USB CDC observation. CPU halt and GDB operations require
explicit user confirmation because they interrupt real-time motor control.

`.claude/skills/aitrace-skill.md` is only a compatibility pointer for older
Claude configurations; debugging rules must not be duplicated there.
