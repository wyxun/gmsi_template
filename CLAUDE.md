# CLAUDE.md

This file is the Claude Code entry point for this repository. Shared project
rules are maintained in tool-neutral files to avoid conflicting instructions.

## Project Instructions

Read and follow `AGENTS.md` for architecture, build commands, coding conventions,
and safety constraints.

The root `Makefile` and `make.bat` are authoritative for build modes, target
selection, flashing, and RTT startup.

## MCU Debugging

Before debugging firmware, read and follow:

`.agents/skills/aitrace/SKILL.md`

Use `.\tools\aitrace.exe` as the default AI debugging entry point. Begin with
passive RTT, waveform, or USB CDC observation. CPU halt and GDB operations require
explicit user confirmation because they interrupt real-time control.

`.claude/skills/aitrace-skill.md` is a compatibility pointer only.
