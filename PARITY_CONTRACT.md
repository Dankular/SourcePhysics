# Source/Havok-to-Stride Parity Contract

This contract governs every implementation and test in `port`.

## Non-negotiable rules

1. The research document and audited reference tree are behavioral evidence only; production code is an original implementation.
2. No numeric value may be added without `source_value`, `source_unit`, `evidence`, `rationale`, and—where the engines differ—a named calibration test.
3. Jolt `CharacterVirtual` or any generic controller must not replace Source movement policy.
4. Source movement ordering is authoritative: friction, gravity split, acceleration, base velocity, movement bumps, clipping, stepping, and ground categorization must be tested in order.
5. Source Z-up and Jolt Y-up conversions must be explicit at boundaries.
6. Gameplay friction, surface friction, and rigid-body contact friction remain separate systems.
7. A workstream is complete only when it has deterministic tests and a reference-course recording format.
8. Unknown behavior is recorded as `MEASURE`, never filled with a guessed default.

## Required evidence labels

`EXACT_SOURCE`, `EXACT_JOLT`, `DERIVED`, `TITLE_DEPENDENT`, `ENGINE_MISMATCH`, `MEASURE`, `DESIGN_CHOICE`.

## Required test output

Every fixture records input/buttons, view angles, position, orientation, linear/angular velocity, ground state/body, water/crouch/ladder state, contacts/normals/fractions, impulses, and fixed-tick index. Comparisons report RMS error, maximum error, timing error, and pass/fail thresholds.
