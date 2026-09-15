using System.Numerics;

namespace SourcePhysics;

/// Source movement policy. Jolt supplies queries; it does not own player movement.
public sealed class SourceMovementMotor
{
    private readonly SourceMovementProfile profile;
    private readonly ISourceMovementQueries queries;
    private readonly Vector3[] planes;
    private bool wasJumpDown;
    public MovementState State { get; private set; }
    public SourceMovementProfile Profile => profile;
    /// Optional title-specific player/prop impulse law. Returning null leaves the contact unchanged.
    public Func<MovementState, MovementContact, Vector3?>? ContactImpulsePolicy { get; set; }

    public void LoadState(in MovementState state)
    {
        wasJumpDown = state.PreviousJumpDown;
        State = state with
        {
            ViewHeightSourceUnits = state.ViewHeightSourceUnits > 0f ? state.ViewHeightSourceUnits : profile.StandingEyeSourceUnits
        };
    }

    public SourceMovementMotor(SourceMovementProfile profile, ISourceMovementQueries queries, Vector3 position)
    {
        this.profile = profile; this.queries = queries; planes = new Vector3[profile.MaxClipPlanes];
        State = new(position, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false,
            ViewHeightSourceUnits: profile.StandingEyeSourceUnits);
    }

    public void Tick(in SourceInput input, float dt)
    {
        if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
        var wasWaterJumping = State.WaterJumpTime > 0f;
        var waterLevel = queries.GetWaterLevel(State.Position, State.Ducking);
        var waterBaseVelocity = queries.GetWaterBaseVelocity(State.Position, waterLevel, State.Ducking);
        var state = State with { Jumped = false, MoveType = input.MoveType,
            WaterJumpTime = MathF.Max(0f, State.WaterJumpTime - dt),
            WaterLevel = waterLevel,
            BaseVelocity = waterLevel != SourceWaterLevel.Dry ? waterBaseVelocity : State.BaseVelocity };
        // PlayerMove categorizes the existing hull before Duck() decides
        // whether a ground transition may begin. Re-categorize when Duck()
        // actually changes the active hull, matching FinishDuck/FinishUnDuck.
        Categorize(ref state);
        var duckingBeforeUpdate = state.Ducking;
        state = UpdateDuck(state, input.IsDown(Buttons.Duck), dt);
        if (state.Ducking != duckingBeforeUpdate)
            Categorize(ref state);
        if (state.WaterLevel == SourceWaterLevel.Dry && state.GroundBodyId >= 0)
            state = state with { BaseVelocity = queries.GetBodyPointVelocity(state.GroundBodyId, state.Position) };
        if (wasWaterJumping)
        {
            // FullWalkMove calls WaterJump then TryPlayerMove for the whole
            // tick, even when this tick consumes the remaining timer. Gravity
            // is not applied during this state; WaterJump only replaces the
            // horizontal velocity with the stored water-jump velocity.
            var waterJumpBase = state.BaseVelocity;
            state = state with { Velocity = state.Velocity + waterJumpBase };
            Move(ref state, dt);
            state = state with { Velocity = state.Velocity - waterJumpBase };
            Categorize(ref state);
            state = Clamp(state);
            wasJumpDown = input.IsDown(Buttons.Jump);
            State = state with { PreviousJumpDown = wasJumpDown };
            return;
        }
        switch (input.MoveType)
        {
            case SourceMoveType.None: state = state with { Velocity = Vector3.Zero }; break;
            case SourceMoveType.Observer: Observer(ref state, input, dt); break;
            case SourceMoveType.Noclip: NoClip(ref state, input, dt, profile.NoclipSpeedFactor, profile.NoclipAcceleration); break;
            case SourceMoveType.Fly: Fly(ref state, input, dt, false); break;
            case SourceMoveType.FlyGravity: Fly(ref state, input, dt, true); break;
            case SourceMoveType.Ladder: Ladder(ref state, input, dt); break;
            default: Walk(ref state, input, dt); break;
        }
        state = Clamp(state);
        if (input.MoveType is not (SourceMoveType.Fly or SourceMoveType.FlyGravity or SourceMoveType.Noclip or SourceMoveType.Observer))
            Categorize(ref state);
        else if (input.MoveType is SourceMoveType.Noclip or SourceMoveType.Observer)
            state = state with { Ground = GroundState.Airborne, GroundBodyId = -1 };
        wasJumpDown = input.IsDown(Buttons.Jump); State = state with { PreviousJumpDown = wasJumpDown };
    }

    private MovementState UpdateDuck(MovementState state, bool wantsDuck, float dt)
    {
        var heightDelta = SourceUnits.ToMeters(
            profile.StandingHalfExtentsSourceUnits.Y - profile.CrouchedHalfExtentsSourceUnits.Y);
        var ducking = state.Ducking;
        var transitioningUp = state.DuckTransitioningUp;
        var transitionTime = MathF.Max(0f, state.DuckTransitionTimeSeconds);
        var position = state.Position;

        if (wantsDuck)
        {
            if (ducking && transitioningUp)
            {
                var upFraction = Math.Clamp(transitionTime / profile.DuckUpTransitionSeconds, 0f, 1f);
                transitionTime = profile.DuckDownTransitionSeconds * (1f - upFraction);
                transitioningUp = false;
            }
            else if (!ducking)
            {
                if (transitioningUp)
                {
                    // Source inverts the remaining unduck timer if duck is
                    // pressed again during an unduck transition.
                    var upFraction = Math.Clamp(transitionTime / profile.DuckUpTransitionSeconds, 0f, 1f);
                    transitionTime = profile.DuckDownTransitionSeconds * (1f - upFraction);
                    transitioningUp = false;
                }

                // Source FinishDuck changes the hull immediately in air, but
                // on ground the standing hull remains active until the duck
                // transition completes.
                if (state.Ground != GroundState.Grounded)
                {
                    ducking = true;
                    transitionTime = 0f;
                    position += Vector3.UnitY * heightDelta;
                }
            }
        }
        else if (!ducking)
        {
            // Releasing during a ground duck transition reverses from the
            // current fraction; the hull is still standing throughout.
            if (!transitioningUp && transitionTime > 0f)
            {
                var downFraction = Math.Clamp(transitionTime / profile.DuckDownTransitionSeconds, 0f, 1f);
                transitionTime = profile.DuckUpTransitionSeconds * (1f - downFraction);
                transitioningUp = true;
            }
        }
        else
        {
            var standPosition = state.Ground == GroundState.Grounded
                ? state.Position : state.Position - Vector3.UnitY * heightDelta;
            if (queries.IsEmpty(standPosition, false))
            {
                if (state.Ground != GroundState.Grounded)
                {
                    // Source CanUnduck finishes immediately while airborne;
                    // the delayed hull transition is a grounded-only path.
                    ducking = false;
                    transitioningUp = false;
                    transitionTime = 0f;
                    position = standPosition;
                }
                else if (!transitioningUp)
                {
                    transitioningUp = true;
                    transitionTime = 0f;
                }
                // Keep the crouched hull active until FinishUnDuck. Only the
                // view transition begins at this point.
            }
            else
            {
                transitionTime = profile.DuckDownTransitionSeconds;
                transitioningUp = false;
            }
        }

        var duration = transitioningUp ? profile.DuckUpTransitionSeconds : profile.DuckDownTransitionSeconds;
        transitionTime = MathF.Min(duration, transitionTime + dt);
        var fraction = SmoothStep(Math.Clamp(transitionTime / duration, 0f, 1f));
        var view = transitioningUp
            ? profile.DuckEyeSourceUnits + (profile.StandingEyeSourceUnits - profile.DuckEyeSourceUnits) * fraction
            : profile.StandingEyeSourceUnits - (profile.StandingEyeSourceUnits - profile.DuckEyeSourceUnits) * fraction;
        if (ducking && transitioningUp && transitionTime >= duration)
        {
            if (state.Ground != GroundState.Grounded)
                position -= Vector3.UnitY * heightDelta;
            ducking = false;
            transitioningUp = false;
            transitionTime = 0f;
            view = profile.StandingEyeSourceUnits;
        }
        if (!ducking && !transitioningUp && transitionTime >= duration)
        {
            ducking = true;
            view = profile.DuckEyeSourceUnits;
        }
        if (ducking && !transitioningUp && transitionTime >= duration)
            view = profile.DuckEyeSourceUnits;
        return state with { Position = position, Ducking = ducking, ViewHeightSourceUnits = view,
            DuckTransitionTimeSeconds = transitionTime, DuckTransitioningUp = transitioningUp };
    }

    private static float SmoothStep(float value) => value * value * (3f - 2f * value);

    private void Walk(ref MovementState s, in SourceInput input, float dt)
    {
        if (s.WaterLevel >= SourceWaterLevel.Waist)
        {
            var forward = Forward(input.ViewYawRadians);
            if (input.IsDown(Buttons.Jump) && !wasJumpDown && queries.TryWaterJump(s.Position, forward, out var waterJumpVelocity, out var duration))
            {
                s = s with { Velocity = waterJumpVelocity, WaterJumpTime = duration,
                    Ground = GroundState.Airborne, GroundBodyId = -1, Jumped = true };
                Move(ref s, dt);
                return;
            }
            Water(ref s, input, dt);
            FinalizeWalk(ref s);
            return;
        }
        StartGravity(ref s, dt); if (s.Ground == GroundState.Grounded) Friction(ref s, dt);
        if (input.IsDown(Buttons.Jump) && !wasJumpDown && s.Ground == GroundState.Grounded)
        { s = s with { Velocity = new(s.Velocity.X, profile.JumpSpeed * s.SurfaceJumpFactor, s.Velocity.Z), Ground = GroundState.Airborne, GroundBodyId = -1, Jumped = true }; Move(ref s, dt); FinishGravity(ref s, dt); FinalizeWalk(ref s); return; }
        var wish = Wish(input, s.Ducking, false, s.SurfaceMaxSpeedFactor);
        if (s.Ground == GroundState.Grounded)
        {
            s = s with { Velocity = new(s.Velocity.X, 0, s.Velocity.Z) }; Accelerate(ref s, wish.Direction, wish.Speed, profile.GroundAcceleration, dt, false);
            var baseVelocity = s.GroundBodyId >= 0 ? s.BaseVelocity : Vector3.Zero;
            var start = s.Position; var original = s with { Velocity = s.Velocity + baseVelocity, BaseVelocity = baseVelocity };
            s = original; Move(ref s, dt); var direct = s; var directDistance = Vector3.DistanceSquared(start, s.Position);
            s = original with { Position = start }; Step(ref s, dt);
            if (Vector3.DistanceSquared(start, s.Position) < directDistance) s = direct;
            s = s with { Velocity = s.Velocity - baseVelocity, BaseVelocity = baseVelocity }; StayOnGround(ref s);
        }
        else
        {
            Accelerate(ref s, wish.Direction, wish.Speed, profile.AirAcceleration, dt, true);
            var baseVelocity = s.BaseVelocity;
            s = s with { Velocity = s.Velocity + baseVelocity };
            Move(ref s, dt);
            s = s with { Velocity = s.Velocity - baseVelocity };
        }
        FinishGravity(ref s, dt);
        FinalizeWalk(ref s);
    }

    private void FinalizeWalk(ref MovementState state)
    {
        Categorize(ref state);
        if (state.Ground == GroundState.Grounded)
            state = state with { Velocity = new Vector3(state.Velocity.X, 0f, state.Velocity.Z) };
    }

    private void Water(ref MovementState s, in SourceInput input, float dt)
    {
        // WaterMove in gamemovement.cpp builds a velocity-valued wish vector,
        // applies the jump/idle drift rules, caps it, then scales wishspeed by
        // sv_waterfriction's companion 0.8 factor. Its acceleration add-speed
        // term uses scalar post-friction speed, not a velocity projection.
        var forward = Forward(input.ViewPitchRadians, input.ViewYawRadians);
        var right = Right(input.ViewYawRadians);
        var maxSpeed = profile.MaxSpeed * MathF.Max(0f, s.SurfaceMaxSpeedFactor) *
            (s.Ducking ? profile.CrouchSpeedScale : 1f);
        var forwardMove = input.Move.Y * maxSpeed;
        var sideMove = input.Move.X * maxSpeed;
        var wishVelocity = forward * forwardMove + right * sideMove;
        if (input.IsDown(Buttons.Jump))
            wishVelocity += Vector3.UnitY * maxSpeed;
        else if (MathF.Abs(forwardMove) < 1e-6f && MathF.Abs(sideMove) < 1e-6f && MathF.Abs(input.UpMove) < 1e-6f)
            wishVelocity += Vector3.UnitY * -SourceUnits.ToMeters(60f);
        else
        {
            // WaterMove exaggerates upward movement while looking upward.
            // Source clamps only this derived component, before adding
            // m_flUpMove.
            var upwardMovement = Math.Clamp(forwardMove * forward.Y * 2f, 0f, maxSpeed);
            wishVelocity += Vector3.UnitY * (input.UpMove * maxSpeed + upwardMovement);
        }
        var wishSpeed = wishVelocity.Length();
        var wishDirection = wishSpeed > 1e-6f ? wishVelocity / wishSpeed : Vector3.Zero;
        if (wishSpeed > maxSpeed) wishSpeed = maxSpeed;
        wishSpeed *= profile.WaterWishSpeedScale;

        var speed = s.Velocity.Length();
        var newSpeed = MathF.Max(0f, speed - dt * profile.WaterFriction * s.SurfaceFriction * speed);
        if (newSpeed < SourceUnits.ToMeters(0.1f)) newSpeed = 0f;
        var velocity = speed > 0f ? s.Velocity * (newSpeed / speed) : Vector3.Zero;
        var add = wishSpeed - newSpeed;
        if (wishSpeed >= SourceUnits.ToMeters(0.1f) && add > 0f)
        {
            var accelerationSpeed = MathF.Min(add,
                profile.WaterAcceleration * wishSpeed * dt * s.SurfaceFriction);
            velocity += wishDirection * accelerationSpeed;
        }
        var baseVelocity = s.BaseVelocity;
        s = s with { Velocity = velocity + baseVelocity, BaseVelocity = baseVelocity, Ground = GroundState.Airborne };
        Move(ref s, dt);
        s = s with { Velocity = s.Velocity - baseVelocity, BaseVelocity = baseVelocity };
    }

    private void Step(ref MovementState s, float dt)
    {
        var start = s.Position; var velocity = s.Velocity;
        var up = queries.SweepPlayer(start, start + Vector3.UnitY * profile.StepHeight, s.Ducking);
        if (up.StartSolid || up.AllSolid) { s = s with { Position = start, Velocity = velocity }; return; }
        s = s with { Position = up.Position }; Move(ref s, dt);
        var down = queries.SweepPlayer(s.Position, s.Position - Vector3.UnitY * profile.StepHeight, s.Ducking);
        if (down.Fraction < 1 && down.Normal.Y >= profile.StandableNormalZ) s = s with { Position = down.Position };
        else s = s with { Position = start, Velocity = velocity };
    }

    private void Ladder(ref MovementState s, in SourceInput input, float dt)
    {
        var forward = Forward(input.ViewYawRadians);
        if (!queries.TryLadder(s.Position, forward, out var normal, out var bodyId)) { s = s with { MoveType = SourceMoveType.Walk }; Walk(ref s, input with { MoveType = SourceMoveType.Walk }, dt); return; }
        if (input.IsDown(Buttons.Jump))
        {
            s = s with { MoveType = SourceMoveType.Walk,
                Velocity = normal * SourceUnits.ToMeters(profile.LadderJumpSpeedSourceUnitsPerSecond),
                Ground = GroundState.Airborne, GroundBodyId = -1 };
            Move(ref s, dt);
            return;
        }
        var climbSpeed = SourceUnits.ToMeters(profile.LadderSpeedSourceUnitsPerSecond);
        var velocity = Right(input.ViewYawRadians) * (input.Move.X * climbSpeed) +
            Vector3.UnitY * ((input.Move.Y + input.UpMove) * climbSpeed);
        var perpendicular = Vector3.Normalize(Vector3.Cross(Vector3.UnitY, normal));
        var normalComponent = Vector3.Dot(velocity, normal);
        var cross = normal * normalComponent;
        var lateral = velocity - cross;
        var ladderUp = Vector3.Normalize(Vector3.Cross(normal, perpendicular));
        velocity = lateral - ladderUp * normalComponent;
        if (s.Ground == GroundState.Grounded && normalComponent > 0f) velocity += Vector3.UnitY * climbSpeed;
        var baseVelocity = s.BaseVelocity;
        s = s with { Velocity = velocity + baseVelocity, Ground = GroundState.Airborne, GroundBodyId = bodyId };
        Move(ref s, dt);
        s = s with { Velocity = s.Velocity - baseVelocity };
    }

    private void Fly(ref MovementState s, in SourceInput input, float dt, bool applyGravity)
    {
        var f = Forward(input.ViewPitchRadians, input.ViewYawRadians);
        var r = Right(input.ViewYawRadians);
        var wish = f * input.Move.Y + r * input.Move.X + Vector3.UnitY * input.UpMove;
        var magnitude = wish.Length();
        var wishDirection = magnitude > 1e-6f ? wish / magnitude : Vector3.Zero;
        var wishSpeed = MathF.Min(profile.MaxSpeed, magnitude * profile.MaxSpeed);
        Accelerate(ref s, wishDirection, wishSpeed, profile.GroundAcceleration, dt, false);
        if (s.Velocity.Y > 0f)
            s = s with { Ground = GroundState.Airborne, GroundBodyId = -1 };
        if (s.Ground == GroundState.Grounded && s.BaseVelocity == Vector3.Zero && s.Velocity == Vector3.Zero)
            return;
        // FullTossMove calls CheckVelocity before applying gravity and before
        // PushEntity. Clamping only after the sweep would move the body farther
        // than Source for an over-limit restored/projectile-like velocity.
        s = Clamp(s);
        if (applyGravity) Gravity(ref s, dt, 1f);
        s = Clamp(s);
        var baseVelocity = s.BaseVelocity;
        s = s with { Velocity = s.Velocity + baseVelocity };
        var move = s.Velocity * dt;
        s = s with { Velocity = s.Velocity - baseVelocity };
        TossMove(ref s, move);
    }

    private void TossMove(ref MovementState s, Vector3 move)
    {
        // FullTossMove uses one PushEntity call, not TryPlayerMove's four
        // bumps. Its default fly collision policy clips once and stops a
        // slow/ground contact; it does not step or slide along a plane set.
        var start = s.Position;
        var hit = queries.SweepPlayer(start, start + move, s.Ducking);
        if (hit.AllSolid)
        {
            s = s with { Velocity = Vector3.Zero, Ground = GroundState.Stuck };
            return;
        }
        if (hit.Fraction >= 1f)
        {
            s = s with { Position = start + move };
            return;
        }

        s = s with { Position = hit.Position };
        if (hit.Normal.Y > profile.StandableNormalZ)
        {
            // Source's default fly collision resolution stops ground contact
            // unless the body is explicitly using FLY_BOUNCE. This port has
            // no implicit bounce mode, so the default is authoritative.
            s = s with { Ground = GroundState.Grounded, GroundNormal = hit.Normal,
                GroundBodyId = hit.BodyId, Velocity = Vector3.Zero };
            return;
        }
        var clipped = Clip(s.Velocity, hit.Normal, 1f);
        s = s with { Velocity = clipped };
    }

    private void NoClip(ref MovementState s, in SourceInput input, float dt, float speedFactor, float acceleration)
    {
        var f = Forward(input.ViewPitchRadians, input.ViewYawRadians); var r = Right(input.ViewYawRadians);
        var factor = MathF.Max(0f, speedFactor);
        var maxSpeed = factor * profile.MaxSpeed;
        if (input.IsDown(Buttons.Speed)) factor *= 0.5f;
        var rawWish = f * input.Move.Y + r * input.Move.X + Vector3.UnitY * input.UpMove;
        var rawWishMagnitude = rawWish.Length();
        var wishDirection = rawWishMagnitude > 1e-6f ? rawWish / rawWishMagnitude : Vector3.Zero;
        var wishSpeed = rawWishMagnitude * factor * profile.MaxSpeed;
        if (wishSpeed > maxSpeed) wishSpeed = maxSpeed;
        var velocity = s.Velocity;
        var currentAlongWish = Vector3.Dot(velocity, wishDirection);
        var addSpeed = wishSpeed - currentAlongWish;
        if (addSpeed > 0f && wishSpeed > 0f)
        {
            var accelerationSpeed = MathF.Min(addSpeed, acceleration * dt * wishSpeed * s.SurfaceFriction);
            velocity += accelerationSpeed * wishDirection;
        }
        var currentSpeed = velocity.Length();
        if (currentSpeed >= 1f / 39.37007874f)
        {
            var control = MathF.Max(currentSpeed, maxSpeed / 4f);
            var drop = control * profile.GroundFriction * s.SurfaceFriction * dt;
            var newSpeed = MathF.Max(0f, currentSpeed - drop);
            velocity *= newSpeed / currentSpeed;
        }
        else velocity = Vector3.Zero;
        s = s with { Velocity = velocity, Position = s.Position + velocity * dt, Ground = GroundState.Airborne, GroundBodyId = -1 };
    }

    private void Observer(ref MovementState s, in SourceInput input, float dt)
    {
        // FullObserverMove has three distinct responsibilities in Source:
        // target following, non-moving cinematic modes, and roaming. Keeping
        // these branches explicit prevents observer state from becoming an
        // accidental alias for noclip.
        if (input.ObserverMode is SourceObserverMode.InEye or SourceObserverMode.Chase)
        {
            if (queries.TryGetObserverTarget(out var target))
            {
                s = s with { Position = target.Position, Velocity = target.Velocity,
                    Ground = GroundState.Airborne, GroundBodyId = -1, Ducking = false };
            }
            return;
        }

        if (input.ObserverMode is not SourceObserverMode.Roaming)
        {
            // Source fixed/death/freeze cameras return without changing the
            // movement command state; the presentation layer owns their view.
            return;
        }

        if (input.ObserverNoClip)
        {
            NoClip(ref s, input, dt, profile.ObserverSpeedFactor, profile.ObserverAcceleration);
            return;
        }

        // Source's clipped roaming path is FullObserverMove, not
        // FullNoClipMove: it uses a view-space wish velocity, ordinary
        // observer friction, and TryPlayerMove collision clipping.
        var forward = Forward(input.ViewPitchRadians, input.ViewYawRadians);
        var right = Right(input.ViewYawRadians);
        var factor = profile.ObserverSpeedFactor;
        if (input.IsDown(Buttons.Speed)) factor *= 0.5f;
        var wishVelocity = forward * (input.Move.Y * factor) + right * (input.Move.X * factor) +
            Vector3.UnitY * input.UpMove;
        var wishSpeed = wishVelocity.Length();
        var maxSpeed = profile.MaxSpeed;
        if (wishSpeed > maxSpeed)
        {
            wishVelocity *= maxSpeed / wishSpeed;
            wishSpeed = maxSpeed;
        }
        var wishDirection = wishSpeed > 1e-6f ? wishVelocity / wishSpeed : Vector3.Zero;
        Accelerate(ref s, wishDirection, wishSpeed, profile.ObserverAcceleration, dt, false);

        var speed = s.Velocity.Length();
        if (speed < SourceUnits.ToMeters(1f))
            s = s with { Velocity = Vector3.Zero };
        else
        {
            var control = MathF.Max(speed, maxSpeed / 4f);
            var drop = control * profile.GroundFriction * dt;
            var newSpeed = MathF.Max(0f, speed - drop);
            s = s with { Velocity = s.Velocity * (newSpeed / speed) };
        }
        Move(ref s, dt);
        s = s with { Ground = GroundState.Airborne, GroundBodyId = -1 };
    }

    private void Friction(ref MovementState s, float dt)
    {
        var speed = s.Velocity.Length(); if (speed < SourceUnits.ToMeters(profile.MinimumFrictionSpeedSourceUnitsPerSecond)) return;
        var drop = MathF.Max(speed, profile.StopSpeed) * profile.GroundFriction * s.SurfaceFriction * dt;
        s = s with { Velocity = s.Velocity * MathF.Max(0, speed - drop) / speed };
    }

    private void Accelerate(ref MovementState s, Vector3 direction, float wishSpeed, float acceleration, float dt, bool air)
    {
        if (wishSpeed <= 0 || direction.LengthSquared() < 1e-8f) return;
        var cap = air ? MathF.Min(wishSpeed, SourceUnits.ToMeters(profile.AirWishSpeedCapSourceUnitsPerSecond)) : wishSpeed;
        var add = cap - Vector3.Dot(s.Velocity, direction); if (add <= 0) return;
        var amount = MathF.Min(add, acceleration * (air ? wishSpeed : cap) * dt * s.SurfaceFriction);
        s = s with { Velocity = s.Velocity + direction * amount };
    }

    private (Vector3 Direction, float Speed) Wish(in SourceInput input, bool crouched, bool water, float surfaceSpeedFactor)
    {
        var f = Forward(input.ViewYawRadians); var r = Right(input.ViewYawRadians); var wish = f * input.Move.Y + r * input.Move.X;
        if (water) wish += Vector3.UnitY * input.UpMove; var magnitude = wish.Length(); if (magnitude < 1e-6f) return (Vector3.Zero, 0);
        var directionalScale = input.Move.Y < 0f ? profile.BackwardSpeedScale : 1f;
        return (wish / magnitude, profile.MaxSpeed * MathF.Min(1, magnitude) * directionalScale * MathF.Max(0f, surfaceSpeedFactor) *
            (crouched ? profile.CrouchSpeedScale : 1f));
    }

    private void Move(ref MovementState s, float dt)
    {
        Array.Clear(planes);
        var timeLeft = dt;
        var original = s.Velocity;
        var primal = s.Velocity;
        var planeCount = 0;
        var allFraction = 0f;
        for (var bump = 0; bump < profile.MaxBumps && timeLeft > 0; bump++)
        {
            if (s.Velocity.LengthSquared() < 1e-12f) break; var hit = queries.SweepPlayer(s.Position, s.Position + s.Velocity * timeLeft, s.Ducking);
            if (hit.AllSolid)
            {
                // Source TryPlayerMove only gives the trapped/all-solid path
                // special treatment. A start-solid trace still participates in
                // the normal plane clipping and reversal rules below.
                s = s with { Velocity = Vector3.Zero, Ground = GroundState.Stuck };
                return;
            }
            allFraction += hit.Fraction;
            if (hit.Fraction > 0f && hit.Fraction < 1f)
            {
                s = s with { Position = hit.Position };
                original = s.Velocity;
                planeCount = 0;
                Array.Clear(planes);
            }
            if (hit.Fraction >= 1f)
            {
                // A clear trace already reports its end position. Advancing
                // from hit.Position again would move twice for every clear
                // Source sweep.
                s = s with { Position = hit.Position };
                // Source re-traces the final position with a stationary hull
                // after a supposedly clear sweep. This catches terrain and
                // triangle-edge precision cases that would otherwise leave a
                // player embedded at the end of the move.
                if (!queries.IsEmpty(s.Position, s.Ducking))
                    s = s with { Velocity = Vector3.Zero };
                break;
            }
            if (hit.BodyId >= 0 && ContactImpulsePolicy is not null)
            {
                var impulse = ContactImpulsePolicy(s, hit);
                if (impulse is { } value && IsFinite(value)) queries.ApplyCharacterImpulse(hit.BodyId, hit.Position, value);
            }
            timeLeft -= timeLeft * Math.Clamp(hit.Fraction, 0f, 1f);
            if (planeCount >= profile.MaxClipPlanes)
            {
                s = s with { Velocity = Vector3.Zero };
                break;
            }
            planes[planeCount++] = hit.Normal;

            if (planeCount == 1 && s.MoveType == SourceMoveType.Walk && s.Ground == GroundState.Airborne)
            {
                var overbounce = hit.Normal.Y > profile.StandableNormalZ
                    ? 1f
                    : 1f + profile.BounceMultiplier * (1f - s.SurfaceFriction);
                var reflected = Clip(original, hit.Normal, overbounce);
                s = s with { Velocity = reflected };
                original = reflected;
                continue;
            }

            var found = false;
            var candidate = Vector3.Zero;
            for (var i = 0; i < planeCount; i++)
            {
                candidate = Clip(original, planes[i], 1f);
                var valid = true;
                for (var j = 0; j < planeCount; j++)
                {
                    if (j != i && Vector3.Dot(candidate, planes[j]) < 0f)
                    {
                        valid = false;
                        break;
                    }
                }
                if (valid) { found = true; break; }
            }
            if (!found)
            {
                if (planeCount != 2) candidate = Vector3.Zero;
                else
                {
                    var crease = Vector3.Cross(planes[0], planes[1]);
                    candidate = crease.LengthSquared() < 1e-8f
                        ? Vector3.Zero
                        : Vector3.Normalize(crease) * Vector3.Dot(Vector3.Normalize(crease), original);
                }
            }
            if (Vector3.Dot(candidate, primal) <= 0f) candidate = Vector3.Zero;
            s = s with { Velocity = candidate };
        }
        if (allFraction == 0f) s = s with { Velocity = Vector3.Zero };
    }

    private static Vector3 Clip(Vector3 velocity, Vector3 normal, float overbounce) { var output = velocity - normal * Vector3.Dot(velocity, normal) * overbounce; var adjust = Vector3.Dot(output, normal); return adjust < 0 ? output - normal * adjust : output; }
    private static Vector3 Crease(Vector3 velocity, Vector3[] planes, int last)
    {
        for (var i = 0; i <= last; i++) for (var j = i + 1; j <= last; j++) { var crease = Vector3.Cross(planes[i], planes[j]); if (crease.LengthSquared() < 1e-8f) continue; crease = Vector3.Normalize(crease); var result = crease * Vector3.Dot(velocity, crease); for (var k = 0; k <= last; k++) if (Vector3.Dot(result, planes[k]) < 0) return Vector3.Zero; return result; }
        return Vector3.Zero;
    }

    private void Categorize(ref MovementState s)
    {
        // CGameMovement::CategorizePosition resets this every recategorization.
        // The player friction is not the raw contact coefficient: Source reads
        // surfacedata_t::physics.friction, scales it by 1.25 to align player
        // and VPhysics feel, then caps it at one.
        s = s with { SurfaceFriction = 1f, SurfaceMaxSpeedFactor = 1f, SurfaceJumpFactor = 1f };
        var groundVelocity = s.GroundBodyId >= 0 ? queries.GetBodyPointVelocity(s.GroundBodyId, s.Position) : Vector3.Zero;
        if (s.MoveType != SourceMoveType.Ladder && s.Velocity.Y - groundVelocity.Y > SourceUnits.ToMeters(profile.GroundCategorizationUpwardSpeedSourceUnitsPerSecond)) { s = s with { Ground = GroundState.Airborne, GroundBodyId = -1 }; return; }
        if (s.MoveType == SourceMoveType.Ladder) { s = s with { Ground = GroundState.Airborne }; return; }
        var hit = queries.SweepPlayer(s.Position, s.Position - Vector3.UnitY * SourceUnits.ToMeters(2), s.Ducking);
        if (!hit.StartSolid && hit.Fraction < 1 && hit.Normal.Y >= profile.StandableNormalZ)
        {
            var surface = queries.GetSurface(hit.SurfaceId);
            s = s with { Ground = GroundState.Grounded, GroundNormal = hit.Normal, GroundBodyId = hit.BodyId,
                SurfaceFriction = Math.Clamp(surface.Friction * 1.25f, 0f, 1f), SurfaceMaxSpeedFactor = surface.MaxSpeedFactor,
                SurfaceJumpFactor = surface.JumpFactor };
        }
        else if (s.Ground != GroundState.Stuck)
        {
            // Source leaves a small amount of friction while moving upward on
            // a non-ground surface, except for noclip. This affects the next
            // acceleration/friction pass and is observable in jump/edge cases.
            var upwardFriction = s.Velocity.Y > 0f && s.MoveType != SourceMoveType.Noclip ? 0.25f : 1f;
            s = s with { Ground = GroundState.Airborne, GroundBodyId = -1, SurfaceFriction = upwardFriction };
        }
    }

    private void StayOnGround(ref MovementState s) { var hit = queries.SweepPlayer(s.Position + Vector3.UnitY * SourceUnits.ToMeters(2), s.Position - Vector3.UnitY * profile.StepHeight, s.Ducking); if (!hit.StartSolid && hit.Fraction < 1 && hit.Normal.Y >= profile.StandableNormalZ) s = s with { Position = hit.Position }; }
    private void StartGravity(ref MovementState s, float dt)
    {
        if (s.WaterJumpTime > 0f) return;
        var baseVelocity = s.BaseVelocity;
        s = s with
        {
            // Source StartGravity applies half gravity and the moving-ground
            // vertical velocity for this frame, then keeps only horizontal
            // base velocity for WalkMove/AirMove.
            Velocity = s.Velocity - Vector3.UnitY * profile.Gravity * dt * 0.5f +
                Vector3.UnitY * baseVelocity.Y * dt,
            BaseVelocity = new Vector3(baseVelocity.X, 0f, baseVelocity.Z)
        };
    }

    private void FinishGravity(ref MovementState s, float dt)
    {
        if (s.WaterJumpTime <= 0f)
            s = s with { Velocity = s.Velocity - Vector3.UnitY * profile.Gravity * dt * 0.5f };
    }

    private void Gravity(ref MovementState s, float dt, float fraction)
    {
        if (s.WaterJumpTime <= 0)
            s = s with { Velocity = s.Velocity - Vector3.UnitY * profile.Gravity * dt * fraction };
    }
    private MovementState Clamp(MovementState s) => s with { Velocity = new(Math.Clamp(s.Velocity.X, -profile.MaxVelocity, profile.MaxVelocity), Math.Clamp(s.Velocity.Y, -profile.MaxVelocity, profile.MaxVelocity), Math.Clamp(s.Velocity.Z, -profile.MaxVelocity, profile.MaxVelocity)) };
    private static bool IsFinite(Vector3 value) => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);
    private static Vector3 Forward(float yaw) => new(MathF.Cos(yaw), 0, MathF.Sin(yaw));
    private static Vector3 Right(float yaw) => new(-MathF.Sin(yaw), 0, MathF.Cos(yaw));
    private static Vector3 Forward(float pitch, float yaw)
    {
        var cosPitch = MathF.Cos(pitch);
        return new(cosPitch * MathF.Cos(yaw), -MathF.Sin(pitch), cosPitch * MathF.Sin(yaw));
    }
}
