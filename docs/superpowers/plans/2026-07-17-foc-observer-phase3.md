# Universal FOC Observer Phase 3 Implementation Plan

**Goal:** Add multi-instance Hall, SMO, and NLFO angle/speed observers plus a
safe runtime selector for open-loop/Hall to SMO or NLFO transitions.

**Rules:** Algorithms own no hardware and no global mutable state. Inputs are
normalized alpha/beta current and voltage. Every output carries validity and
confidence. Runtime switching requires target validity, confidence, speed
hysteresis, stable samples, bounded angle error, and a finite blend window.

## Tasks

1. Add common observer input/output and context interface types.
2. Implement Hall decoding with invalid-code and illegal-transition handling.
3. Implement bounded SMO and NLFO states with precomputed fixed-point gains.
4. Implement observer selection, qualification, cancellation, and blending.
5. Add float/fixed host tests and AT32F413 dual-backend builds.

SguanFOC v3.1.0 is used as an algorithm reference. This implementation fixes
unsafe Hall indexing and uses `(Ld + Lq) / 2` for NLFO average inductance.
