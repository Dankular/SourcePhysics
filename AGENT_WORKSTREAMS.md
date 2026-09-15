# Ten parity workstreams

These are deliberately isolated ownership areas. Each workstream must follow `PARITY_CONTRACT.md` and submit changes through the shared fixture/test format.

| ID | Workstream | Deliverable | Primary references |
|---:|---|---|---|
| 1 | Evidence manifest | Machine-readable Source/Jolt property manifest with evidence labels | `references/SOURCE_HAVOK_TO_JOLT_PROPERTY_RESEARCH (1).md`, `movevars_shared.cpp` |
| 2 | Source movement | Complete fixed-tick motor: all movement modes, ducking, water, ladders, stepping, clipping, ground state | `gamemovement.cpp`, `gamemovement.h`, title movement overrides |
| 3 | Jolt binding bridge | Verified native shape creation, box casts, narrow-phase queries, manifold/contact metadata | JoltPhysicsSharp API and native `joltc` exports |
| 4 | Collision model | Contents, collision groups, object/broad-phase layers, triggers, ladders, filtering | `vphysics_interface*.h`, trace/filter code |
| 5 | Surfaces/materials | Surface registry, combine laws, friction/restitution, speed/jump/climb factors | `physics_material.*`, surface parser and research manifest |
| 6 | Rigid bodies/weapons | Mass, inertia, damping, drag, CCD, projectile and weapon interaction behavior | VPhysics object APIs, weapon projectile code, body properties |
| 7 | Stride integration | Fixed physics system, entity transforms, interpolation, animation/root motion, camera state | Stride entity/script APIs and movement state |
| 8 | Networking | Input/state serialization, prediction, replay, correction, deterministic ordering | User command state and project replication layer |
| 9 | Reference harness | Course fixtures, per-tick recordings, Source-vs-Stride comparator, golden baselines | Research measurement programme |
| 10 | Parity QA/performance | Differential tests, edge-case matrix, profiling, regression gates, discrepancy reports | All workstreams; no new behavior ownership |

## Integration order

`1 -> 3 -> 4 -> 2 -> 5 -> 6 -> 7 -> 9 -> 8 -> 10`

No later stream may paper over an earlier semantic mismatch. Workstream 10 rejects changes that only improve a single fixture while regressing another.

## Initial test matrix

- Acceleration, braking, reverse, diagonal movement, air-strafing.
- Jump apex/time/landing, ramps from 0–60 degrees, surf ramps, two-plane creases.
- 1–18 Source-unit stairs, 19-unit failure, corners, ledges, internal triangle edges.
- Crouch tunnel, ceiling obstruction, duck transition, duck-jump.
- Ladders, feet/waist/eyes water levels, water jump, moving/rotating platforms.
- Static/dynamic/kinematic props, player pushing, prop pushing player, crushing.
- Weapon projectile travel, impact, bounce, penetration, trigger hits, CCD.
- Sleep/wake, stacks, constraints, save/restore, correction/replay, fixed-tick load.
