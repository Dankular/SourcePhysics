using System.Numerics;

namespace SourcePhysics;

/// Exact Source obstacle-pushaway policy from obstacle_pushaway.cpp.
/// Outputs remain in Source force / command units; conversion to a Jolt impulse
/// belongs at the physics boundary and must carry its own calibration evidence.
public sealed record SourcePushawayProfile
{
    public float PropForce { get; init; } = 30000f;
    public float MinimumPlayerSpeedSourceUnitsPerSecond { get; init; } = 75f;
    public float MaximumPropForce { get; init; } = 1000f;
    public float PlayerForce { get; init; } = 200000f;
    public float MaximumPlayerForce { get; init; } = 10000f;
    public float MinimumPushMassKilograms { get; init; } = 10f;
    public float MaximumPushMassKilograms { get; init; } = 30f;
    public float MaximumPushawayDistanceSourceUnits { get; init; } = 5f;
    public float RotatingDoorForceScale { get; init; } = 0.25f;

    public void Validate()
    {
        var values = new[]
        {
            PropForce, MinimumPlayerSpeedSourceUnitsPerSecond, MaximumPropForce, PlayerForce,
            MaximumPlayerForce, MinimumPushMassKilograms, MaximumPushMassKilograms,
            MaximumPushawayDistanceSourceUnits, RotatingDoorForceScale
        };
        if (values.Any(value => !float.IsFinite(value) || value < 0f))
            throw new InvalidDataException("Pushaway profile contains an invalid value.");
        if (MaximumPushawayDistanceSourceUnits == 0f || MaximumPushMassKilograms == 0f ||
            MinimumPushMassKilograms > MaximumPushMassKilograms)
            throw new InvalidDataException("Pushaway profile has invalid required limits.");
    }
}

public static class SourcePushawayPolicy
{
    /// Mirrors PerformObstaclePushaway: horizontal center-to-center direction,
    /// inverse distance with a minimum distance of one Source unit, and a cap.
    public static Vector3 ComputeObstacleForce(SourcePushawayProfile profile, Vector3 propCenter,
        Vector3 playerCenter, float playerSpeedSourceUnitsPerSecond, float propMassKilograms,
        bool multiplayerSolid, bool rotatingDoor)
    {
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        if (multiplayerSolid && playerSpeedSourceUnitsPerSecond < profile.MinimumPlayerSpeedSourceUnitsPerSecond)
            return Vector3.Zero;

        var push = propCenter - playerCenter;
        push.Y = 0f;
        var distance = push.Length();
        if (distance > 1e-6f) push /= distance;
        else return Vector3.Zero;
        distance = MathF.Max(distance, 1f);
        var force = MathF.Min(profile.PropForce / distance, profile.MaximumPropForce);
        return push * force * (rotatingDoor ? profile.RotatingDoorForceScale : 1f);
    }

    /// Mirrors AvoidPushawayProps. The returned vector is the command-space
    /// contribution before Source adds it to forwardmove/sidemove.
    public static Vector3 ComputePlayerCommandPush(SourcePushawayProfile profile,
        Vector3 nearestPlayerPoint, Vector3 nearestPropPoint, Vector3 playerCenter,
        Vector3 propCenter, float propMassKilograms, bool nearestPropPointInsidePlayerBounds,
        bool rotatingDoor)
    {
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        var push = nearestPlayerPoint - nearestPropPoint;
        var distance = push.Length();
        if (distance > 1e-6f) push /= distance;
        if (distance > profile.MaximumPushawayDistanceSourceUnits && !nearestPropPointInsidePlayerBounds)
            return Vector3.Zero;

        if (push.LengthSquared() < 1e-12f)
        {
            push = playerCenter - nearestPropPoint;
            distance = push.Length();
            if (distance > 1e-6f) push /= distance;
        }
        if (push.LengthSquared() < 1e-12f)
        {
            push = playerCenter - propCenter;
            distance = push.Length();
            if (distance > 1e-6f) push /= distance;
        }
        distance = MathF.Max(distance, 1f);
        var mass = Math.Clamp(propMassKilograms, profile.MinimumPushMassKilograms, profile.MaximumPushMassKilograms);
        mass = MathF.Max(mass, 0f) / profile.MaximumPushMassKilograms;
        var force = MathF.Min(profile.PlayerForce / distance * mass, profile.MaximumPlayerForce);
        if (rotatingDoor) force *= profile.RotatingDoorForceScale;
        return push * force;
    }
}
