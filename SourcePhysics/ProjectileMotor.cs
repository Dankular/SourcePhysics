using System.Numerics;

namespace SourcePhysics;

public readonly record struct ProjectileHit(Vector3 Position, Vector3 Normal, int BodyId, float Restitution,
    int SurfaceId = 0, float ThicknessInches = 0f);
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
}

public sealed class SourceProjectileMotor
{
    private readonly SourceProjectileProfile profile;
    private readonly IProjectileQueries queries;
    public ProjectileState State { get; private set; }

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
            penetrator.TryPenetrate(State.Position, in hit, velocity, availablePenetrationPower,
                out var exitPosition, out var exitVelocity, out var consumedPower))
        {
            var remainingPower = MathF.Max(0f, availablePenetrationPower - consumedPower);
            State = new(exitPosition, exitVelocity, remainingPower > 0f, State.Bounces,
                State.Penetrations + 1, remainingPower);
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
}
