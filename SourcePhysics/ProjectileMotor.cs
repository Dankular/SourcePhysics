using System.Numerics;

namespace SourcePhysics;

public readonly record struct ProjectileHit(Vector3 Position, Vector3 Normal, int BodyId, float Restitution,
    int SurfaceId = 0, float ThicknessInches = 0f, float Fraction = 1f);
public readonly record struct ProjectileState(Vector3 Position, Vector3 Velocity, bool Active, int Bounces,
    int Penetrations = 0, float PenetrationPowerRemaining = 0f);

public interface IProjectileQueries
{
    bool Sweep(Vector3 start, Vector3 end, out ProjectileHit hit);
}

public interface IProjectilePenetrationQueries
{
    bool TryPenetrate(Vector3 entryPosition, in ProjectileHit entryHit, Vector3 incomingVelocity,
        float availablePower, out Vector3 exitPosition, out Vector3 exitVelocity, out float consumedPower);
}

public enum SourceProjectileCollisionMode
{
    Generic,
    CounterStrikeGrenade
}

public sealed record SourceProjectileProfile
{
    public float GravitySourceUnitsPerSecondSquared { get; init; } = 800f;
    public float GravityScale { get; init; } = 1f;
    public float Restitution { get; init; } = 0f;
    public int MaximumBounces { get; init; } = 0;
    public float PenetrationPower { get; init; }
    public int MaximumPenetrations { get; init; }
    public float MaximumVelocitySourceUnitsPerSecond { get; init; } = 3500f;
    public bool ContinuousCollision { get; init; } = true;
    public SourceProjectileCollisionMode CollisionMode { get; init; } = SourceProjectileCollisionMode.Generic;
}

public sealed class SourceProjectileMotor
{
    private readonly SourceProjectileProfile profile;
    private readonly IProjectileQueries queries;
    public ProjectileState State { get; private set; }
    /// Title-owned body classification used only by the Source custom
    /// Counter-Strike grenade collision law.
    public Func<int, bool>? IsPlayerBody { get; set; }
    /// Returns true when the title's breakable impact dispatch destroyed the
    /// contacted breakable. A destroyed breakable uses Source's 0.4 velocity
    /// continuation instead of grenade bounce resolution.
    public Func<ProjectileHit, bool>? BreakableImpactDestroyed { get; set; }

    public SourceProjectileMotor(SourceProjectileProfile profile, IProjectileQueries queries, Vector3 position, Vector3 velocity)
    {
        this.profile = profile; this.queries = queries;
        State = new(position, velocity, true, 0);
    }

    public ProjectileHit? Tick(float dt)
    {
        if (!State.Active || dt <= 0f) return null;
        var gravity = Vector3.UnitY * -SourceUnits.ToMeters(profile.GravitySourceUnitsPerSecondSquared) * profile.GravityScale;
        var velocity = State.Velocity + gravity * dt;
        var maxVelocity = SourceUnits.ToMeters(profile.MaximumVelocitySourceUnitsPerSecond);
        velocity = new(Math.Clamp(velocity.X, -maxVelocity, maxVelocity), Math.Clamp(velocity.Y, -maxVelocity, maxVelocity), Math.Clamp(velocity.Z, -maxVelocity, maxVelocity));
        var end = State.Position + velocity * dt;
        if (!queries.Sweep(State.Position, end, out var hit)) { State = State with { Position = end, Velocity = velocity }; return null; }
        var availablePenetrationPower = State.Penetrations == 0 && State.PenetrationPowerRemaining <= 0f
            ? profile.PenetrationPower
            : State.PenetrationPowerRemaining;
        if (availablePenetrationPower > 0f && State.Penetrations < profile.MaximumPenetrations &&
            queries is IProjectilePenetrationQueries penetrator &&
            penetrator.TryPenetrate(hit.Position, in hit, velocity, availablePenetrationPower,
                out var exitPosition, out var exitVelocity, out var consumedPower))
        {
            var remainingPower = MathF.Max(0f, availablePenetrationPower - consumedPower);
            State = new(exitPosition, exitVelocity, remainingPower > 0f, State.Bounces,
                State.Penetrations + 1, remainingPower);
            return hit;
        }
        if (profile.CollisionMode == SourceProjectileCollisionMode.CounterStrikeGrenade)
        {
            ResolveCounterStrikeGrenadeCollision(hit, velocity, dt);
            return hit;
        }
        if (State.Bounces < profile.MaximumBounces)
        {
            var reflected = velocity - 2f * Vector3.Dot(velocity, hit.Normal) * hit.Normal;
            State = new(hit.Position, reflected * Math.Clamp(hit.Restitution * profile.Restitution, 0f, 1f), true, State.Bounces + 1);
        }
        else State = new(hit.Position, Vector3.Zero, false, State.Bounces);
        return hit;
    }

    private void ResolveCounterStrikeGrenadeCollision(in ProjectileHit hit, Vector3 velocity, float dt)
    {
        if (BreakableImpactDestroyed?.Invoke(hit) == true)
        {
            State = State with { Position = hit.Position, Velocity = velocity * 0.4f };
            return;
        }
        // CBaseCSGrenadeProjectile::ResolveFlyCollisionCustom treats the
        // surface as perfectly elastic, except for player contacts, then
        // clamps the projectile elasticity to [0, .9].
        var surfaceElasticity = IsPlayerBody?.Invoke(hit.BodyId) == true ? 0.3f : 1f;
        var totalElasticity = Math.Clamp(profile.Restitution * surfaceElasticity, 0f, 0.9f);
        var clipped = velocity - hit.Normal * Vector3.Dot(velocity, hit.Normal) * 2f;
        var reflected = clipped * totalElasticity;
        var stopSpeed = SourceUnits.ToMeters(30f);
        var speedSquared = reflected.LengthSquared();
        if (speedSquared < stopSpeed * stopSpeed || State.Bounces >= profile.MaximumBounces)
        {
            State = State with { Position = hit.Position, Velocity = Vector3.Zero, Active = false };
            return;
        }

        var position = hit.Position;
        if (hit.Normal.Y > 0.7f)
        {
            // The Source custom floor path pushes the remaining fraction of
            // the frame after a high-speed bounce.
            position += reflected * ((1f - Math.Clamp(hit.Fraction, 0f, 1f)) * dt * 0.9f);
        }
        State = State with { Position = position, Velocity = reflected, Bounces = State.Bounces + 1 };
    }
}
