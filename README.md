# SourcePhysics.Jolt

This is an original C# semantic port boundary for Source movement on Jolt. It does not translate or redistribute Source/Havok implementation. The motor preserves the observable ordering and limits: fixed-tick friction, ground/air acceleration, air wish-speed cap, jump transition, gravity, four bumps, five clip planes, standable normal `normal.y > 0.7`, and explicit ground state.

`SourceMovementMotor` intentionally depends on `ISourceMovementQueries`, not `CharacterVirtual.Update()`. The Jolt adapter must implement that interface with a box shape cast / narrow-phase query, body point velocity, contact settings and contact event forwarding. This keeps Source movement authoritative and prevents Jolt's controller policy from silently changing parity.

The current profile is the multiplayer Source baseline from the research manifest. Title-specific overrides must be explicit profiles, not hidden constants.

See [PARITY_GAPS.md](PARITY_GAPS.md) for the evidence gates that must pass before claiming complete title parity.
