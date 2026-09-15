using System.Numerics;

namespace SourcePhysics;

public static class SourceUnits
{
    public const float InchesToMeters = 0.0254f;
    public const float MetersToInches = 39.37007874015748f;

    public static float ToMeters(float sourceUnits) => sourceUnits * InchesToMeters;
    public static System.Numerics.Vector3 ToMeters(System.Numerics.Vector3 sourceUnits) => sourceUnits * InchesToMeters;
    public static float ToSource(float meters) => meters * MetersToInches;
    public static System.Numerics.Vector3 ToSource(System.Numerics.Vector3 meters) => meters * MetersToInches;
}

public sealed record SourceMovementProfile
{
    public float GravitySourceUnitsPerSecondSquared { get; init; } = 800f;
    public float StopSpeedSourceUnitsPerSecond { get; init; } = 100f;
    public float MaxSpeedSourceUnitsPerSecond { get; init; } = 320f;
    public float GroundAcceleration { get; init; } = 10f;
    public float AirAcceleration { get; init; } = 10f;
    public float WaterAcceleration { get; init; } = 10f;
    public float WaterFriction { get; init; } = 1f;
    public float GroundFriction { get; init; } = 4f;
    public float BounceMultiplier { get; init; } = 0f;
    public float StepHeightSourceUnits { get; init; } = 18f;
    public float MaxVelocitySourceUnitsPerSecond { get; init; } = 3500f;
    public float BackwardSpeedScale { get; init; } = 0.6f;
    public float StandableNormalZ { get; init; } = 0.7f;
    public float CollisionEpsilonSourceUnits { get; init; } = 0.03125f;
    public int MaxBumps { get; init; } = 4;
    public int MaxClipPlanes { get; init; } = 5;
    public float JumpSpeedSourceUnitsPerSecond { get; init; } = 268.32816f;
    public float JumpTimingSeconds { get; init; } = 0.510f;
    public float JumpHeightSourceUnits { get; init; } = 21f;
    public float WaterViewDistanceSourceUnits { get; init; } = 12f;
    public float StandingEyeSourceUnits { get; init; } = 64f;
    public float DuckEyeSourceUnits { get; init; } = 28f;
    public float DuckTransitionSeconds { get; init; } = 1f;
    public float DuckDownTransitionSeconds { get; init; } = 0.4f;
    public float DuckUpTransitionSeconds { get; init; } = 0.2f;
    public float LadderPerpendicularDamping { get; init; } = 0.2f;
    public float LadderSpeedSourceUnitsPerSecond { get; init; } = 200f;
    public float LadderJumpSpeedSourceUnitsPerSecond { get; init; } = 270f;
    public float NoclipSpeedFactor { get; init; } = 5f;
    public float ObserverSpeedFactor { get; init; } = 3f;
    public float NoclipAcceleration { get; init; } = 5f;
    public float ObserverAcceleration { get; init; } = 5f;
    public float WaterWishSpeedScale { get; init; } = 0.8f;
    public float CrouchSpeedScale { get; init; } = 1f / 3f;
    public float AirWishSpeedCapSourceUnitsPerSecond { get; init; } = 30f;
    public float MinimumFrictionSpeedSourceUnitsPerSecond { get; init; } = 0.1f;
    public float GroundCategorizationUpwardSpeedSourceUnitsPerSecond { get; init; } = 140f;
    public float LadderFacingDotThreshold { get; init; } = -0.707f;
    /// <summary>Jolt convex-edge recovery distance; calibrate against Source trace fixtures.</summary>
    public float QueryRecoveryDistanceSourceUnits { get; init; } = 0.08f;
    public Vector3 StandingHalfExtentsSourceUnits { get; init; } = new(16f, 36f, 16f);
    public Vector3 CrouchedHalfExtentsSourceUnits { get; init; } = new(16f, 18f, 16f);

    public float Gravity => SourceUnits.ToMeters(GravitySourceUnitsPerSecondSquared);
    public float StopSpeed => SourceUnits.ToMeters(StopSpeedSourceUnitsPerSecond);
    public float MaxSpeed => SourceUnits.ToMeters(MaxSpeedSourceUnitsPerSecond);
    public float StepHeight => SourceUnits.ToMeters(StepHeightSourceUnits);
    public float MaxVelocity => SourceUnits.ToMeters(MaxVelocitySourceUnitsPerSecond);
    public float JumpSpeed => SourceUnits.ToMeters(JumpSpeedSourceUnitsPerSecond);
    public float JumpHeight => SourceUnits.ToMeters(JumpHeightSourceUnits);
    public float WaterViewDistance => SourceUnits.ToMeters(WaterViewDistanceSourceUnits);
    public float CollisionEpsilon => SourceUnits.ToMeters(CollisionEpsilonSourceUnits);
    public Vector3 StandingHalfExtents => SourceUnits.ToMeters(StandingHalfExtentsSourceUnits);
    public Vector3 CrouchedHalfExtents => SourceUnits.ToMeters(CrouchedHalfExtentsSourceUnits);

    public void Validate()
    {
        var scalars = new[]
        {
            GravitySourceUnitsPerSecondSquared, StopSpeedSourceUnitsPerSecond, MaxSpeedSourceUnitsPerSecond,
            GroundAcceleration, AirAcceleration, WaterAcceleration, WaterFriction, GroundFriction,
            BounceMultiplier, StepHeightSourceUnits, MaxVelocitySourceUnitsPerSecond, BackwardSpeedScale,
            StandableNormalZ, CollisionEpsilonSourceUnits, JumpSpeedSourceUnitsPerSecond, JumpTimingSeconds,
            JumpHeightSourceUnits, WaterViewDistanceSourceUnits, StandingEyeSourceUnits, DuckEyeSourceUnits,
            DuckTransitionSeconds, DuckDownTransitionSeconds, DuckUpTransitionSeconds,
            LadderPerpendicularDamping, LadderSpeedSourceUnitsPerSecond,
            LadderJumpSpeedSourceUnitsPerSecond, NoclipSpeedFactor, ObserverSpeedFactor,
            NoclipAcceleration, ObserverAcceleration,
            WaterWishSpeedScale, CrouchSpeedScale, AirWishSpeedCapSourceUnitsPerSecond,
            MinimumFrictionSpeedSourceUnitsPerSecond, GroundCategorizationUpwardSpeedSourceUnitsPerSecond,
            LadderFacingDotThreshold, QueryRecoveryDistanceSourceUnits
        };
        if (scalars.Any(value => !float.IsFinite(value)))
            throw new InvalidDataException("Movement profile contains a non-finite value.");

        if (GravitySourceUnitsPerSecondSquared < 0f || StopSpeedSourceUnitsPerSecond < 0f ||
            MaxSpeedSourceUnitsPerSecond < 0f || MaxVelocitySourceUnitsPerSecond < 0f ||
            StepHeightSourceUnits < 0f || CollisionEpsilonSourceUnits < 0f || JumpSpeedSourceUnitsPerSecond < 0f ||
            JumpTimingSeconds < 0f || JumpHeightSourceUnits < 0f || WaterViewDistanceSourceUnits < 0f ||
            StandingEyeSourceUnits < 0f || DuckEyeSourceUnits < 0f || DuckTransitionSeconds < 0f ||
            DuckDownTransitionSeconds <= 0f || DuckUpTransitionSeconds <= 0f ||
            QueryRecoveryDistanceSourceUnits < 0f || WaterFriction < 0f || GroundFriction < 0f ||
            WaterAcceleration < 0f || GroundAcceleration < 0f || AirAcceleration < 0f ||
            LadderSpeedSourceUnitsPerSecond < 0f || LadderJumpSpeedSourceUnitsPerSecond < 0f ||
            MaxBumps < 1 || MaxClipPlanes < 1)
            throw new InvalidDataException("Movement profile contains an invalid negative or zero-required value.");

        if (!float.IsFinite(StandingHalfExtentsSourceUnits.X) || !float.IsFinite(StandingHalfExtentsSourceUnits.Y) || !float.IsFinite(StandingHalfExtentsSourceUnits.Z) ||
            !float.IsFinite(CrouchedHalfExtentsSourceUnits.X) || !float.IsFinite(CrouchedHalfExtentsSourceUnits.Y) || !float.IsFinite(CrouchedHalfExtentsSourceUnits.Z) ||
            StandingHalfExtentsSourceUnits.X <= 0f || StandingHalfExtentsSourceUnits.Y <= 0f || StandingHalfExtentsSourceUnits.Z <= 0f ||
            CrouchedHalfExtentsSourceUnits.X <= 0f || CrouchedHalfExtentsSourceUnits.Y <= 0f || CrouchedHalfExtentsSourceUnits.Z <= 0f)
            throw new InvalidDataException("Movement profile contains invalid character extents.");
    }
}
