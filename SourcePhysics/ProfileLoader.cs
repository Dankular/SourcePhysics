using System.Text.Json;
using System.Numerics;

namespace SourcePhysics;

public static class SourceProfileLoader
{
    public static SourcePushawayProfile LoadPushaway(string json)
    {
        using var document = JsonDocument.Parse(json);
        var pushaway = document.RootElement.GetProperty("pushaway");
        var profile = new SourcePushawayProfile
        {
            PropForce = Value(pushaway, "propForce"),
            MinimumPlayerSpeedSourceUnitsPerSecond = Value(pushaway, "minimumPlayerSpeedSourceUnitsPerSecond"),
            MaximumPropForce = Value(pushaway, "maximumPropForce"),
            PlayerForce = Value(pushaway, "playerForce"),
            MaximumPlayerForce = Value(pushaway, "maximumPlayerForce"),
            MinimumPushMassKilograms = Value(pushaway, "minimumPushMassKilograms"),
            MaximumPushMassKilograms = Value(pushaway, "maximumPushMassKilograms"),
            MaximumPushawayDistanceSourceUnits = Value(pushaway, "maximumPushawayDistanceSourceUnits"),
            RotatingDoorForceScale = Value(pushaway, "rotatingDoorForceScale")
        };
        profile.Validate();
        return profile;
    }

    public static SourceMovementProfile LoadMovement(string json)
    {
        using var document = JsonDocument.Parse(json);
        var movement = document.RootElement.GetProperty("movement");
        var character = document.RootElement.TryGetProperty("character", out var characterElement)
            ? characterElement : default;
        var profile = new SourceMovementProfile
        {
            GravitySourceUnitsPerSecondSquared = Value(movement, "gravitySourceUnitsPerSecondSquared"),
            StopSpeedSourceUnitsPerSecond = Value(movement, "stopSpeedSourceUnitsPerSecond"),
            MaxSpeedSourceUnitsPerSecond = Value(movement, "maxSpeedSourceUnitsPerSecond"),
            BounceMultiplier = OptionalValue(movement, "bounceMultiplier", 0f),
            MaxVelocitySourceUnitsPerSecond = OptionalValue(movement, "maxVelocitySourceUnitsPerSecond", 3500f),
            GroundAcceleration = Value(movement, "groundAcceleration"),
            AirAcceleration = Value(movement, "airAcceleration"),
            WaterAcceleration = Value(movement, "waterAcceleration"),
            GroundFriction = Value(movement, "groundFriction"),
            WaterFriction = Value(movement, "waterFriction"),
            BackwardSpeedScale = Value(movement, "backwardSpeedScale"),
            JumpSpeedSourceUnitsPerSecond = Value(movement, "jumpSpeedSourceUnitsPerSecond"),
            JumpTimingSeconds = OptionalValue(movement, "jumpTimingSeconds", 0.510f),
            JumpHeightSourceUnits = OptionalValue(movement, "jumpHeightSourceUnits", 21f),
            WaterViewDistanceSourceUnits = OptionalValue(movement, "waterViewDistanceSourceUnits", 12f),
            DuckTransitionSeconds = Value(movement, "duckTransitionSeconds"),
            DuckDownTransitionSeconds = OptionalValue(movement, "duckDownTransitionSeconds", 0.4f),
            DuckUpTransitionSeconds = OptionalValue(movement, "duckUpTransitionSeconds", 0.2f),
            StepHeightSourceUnits = Value(movement, "stepHeightSourceUnits"),
            StandableNormalZ = Value(movement, "standableNormalZ"),
            CollisionEpsilonSourceUnits = Value(movement, "collisionEpsilonSourceUnits"),
            MaxBumps = (int)Value(movement, "maxBumps"),
            MaxClipPlanes = (int)Value(movement, "maxClipPlanes")
            ,NoclipSpeedFactor = OptionalValue(movement, "noclipSpeedFactor", 5f)
            ,ObserverSpeedFactor = OptionalValue(movement, "observerSpeedFactor", 3f)
            ,NoclipAcceleration = OptionalValue(movement, "noclipAcceleration", 5f)
            ,ObserverAcceleration = OptionalValue(movement, "observerAcceleration", 5f)
            ,WaterWishSpeedScale = OptionalValue(movement, "waterWishSpeedScale", 0.8f)
            ,CrouchSpeedScale = OptionalValue(movement, "crouchSpeedScale", 1f / 3f)
            ,AirWishSpeedCapSourceUnitsPerSecond = OptionalValue(movement, "airWishSpeedCapSourceUnitsPerSecond", 30f)
            ,MinimumFrictionSpeedSourceUnitsPerSecond = OptionalValue(movement, "minimumFrictionSpeedSourceUnitsPerSecond", 0.1f)
            ,GroundCategorizationUpwardSpeedSourceUnitsPerSecond = OptionalValue(movement, "groundCategorizationUpwardSpeedSourceUnitsPerSecond", 140f)
            ,LadderFacingDotThreshold = OptionalValue(movement, "ladderFacingDotThreshold", -0.707f)
            ,LadderSpeedSourceUnitsPerSecond = OptionalValue(movement, "ladderSpeedSourceUnitsPerSecond", 200f)
            ,LadderJumpSpeedSourceUnitsPerSecond = OptionalValue(movement, "ladderJumpSpeedSourceUnitsPerSecond", 270f)
            ,QueryRecoveryDistanceSourceUnits = OptionalValue(movement, "queryRecoveryDistanceSourceUnits", 0.08f)
            ,StandingEyeSourceUnits = OptionalValue(character, "standingEyeSourceUnits", 64f)
            ,DuckEyeSourceUnits = OptionalValue(character, "crouchedEyeSourceUnits", 28f)
            ,StandingHalfExtentsSourceUnits = OptionalVector3(character, "standingHalfExtentsSourceUnits", new(16f, 36f, 16f))
            ,CrouchedHalfExtentsSourceUnits = OptionalVector3(character, "crouchedHalfExtentsSourceUnits", new(16f, 18f, 16f))
            ,LadderPerpendicularDamping = OptionalValue(movement, "ladderPerpendicularDamping", 0.2f)
        };
        profile.Validate();
        return profile;
    }

    private static float Value(JsonElement parent, string name) => parent.GetProperty(name).GetProperty("value").GetSingle();
    private static float OptionalValue(JsonElement parent, string name, float fallback)
    {
        if (!parent.TryGetProperty(name, out var property)) return fallback;
        return property.ValueKind == JsonValueKind.Number
            ? property.GetSingle()
            : property.GetProperty("value").GetSingle();
    }

    private static Vector3 OptionalVector3(JsonElement parent, string name, Vector3 fallback)
    {
        if (parent.ValueKind == JsonValueKind.Undefined || !parent.TryGetProperty(name, out var property) ||
            property.ValueKind != JsonValueKind.Array || property.GetArrayLength() != 3)
            return fallback;
        var values = property.EnumerateArray().Select(value => value.GetSingle()).ToArray();
        return new(values[0], values[1], values[2]);
    }
}
