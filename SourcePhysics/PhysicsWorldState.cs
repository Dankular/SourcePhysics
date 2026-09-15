using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

public readonly record struct SourcePhysicsBodySnapshot(uint BodyId, SourceRigidBodyState State);

/// Deterministic correction payload for the Jolt bodies owned by one Source physics host.
public sealed record SourcePhysicsWorldState(int Tick, SourcePhysicsBodySnapshot[] Bodies)
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };

    public string ToJson() => JsonSerializer.Serialize(this, JsonOptions);

    public static SourcePhysicsWorldState FromJson(string json) =>
        JsonSerializer.Deserialize<SourcePhysicsWorldState>(json, JsonOptions)
        ?? throw new InvalidDataException("Invalid physics world state.");
}

public static class SourcePhysicsStateMath
{
    public static bool NearlyEqual(in SourcePhysicsBodySnapshot expected, in SourcePhysicsBodySnapshot actual,
        float positionTolerance, float velocityTolerance, float angularVelocityTolerance, float rotationTolerance = 0f)
    {
        return expected.BodyId == actual.BodyId &&
            Vector3.Distance(expected.State.Position, actual.State.Position) <= positionTolerance &&
            Vector3.Distance(expected.State.LinearVelocity, actual.State.LinearVelocity) <= velocityTolerance &&
            Vector3.Distance(expected.State.AngularVelocity, actual.State.AngularVelocity) <= angularVelocityTolerance &&
            1f - MathF.Min(1f, MathF.Abs(Quaternion.Dot(expected.State.Rotation, actual.State.Rotation))) <= rotationTolerance &&
            expected.State.Active == actual.State.Active &&
            expected.State.JoltMotionType == actual.State.JoltMotionType &&
            expected.State.GravityFactor == actual.State.GravityFactor &&
            expected.State.Friction == actual.State.Friction &&
            expected.State.Restitution == actual.State.Restitution &&
            expected.State.ContentsMask == actual.State.ContentsMask &&
            expected.State.CollisionGroup == actual.State.CollisionGroup &&
            expected.State.SolidFlags == actual.State.SolidFlags &&
            expected.State.UserData == actual.State.UserData;
    }
}

public sealed record PhysicsWorldComparison(
    bool TickMismatch,
    int MissingBodies,
    int ExtraBodies,
    float PositionMaximum,
    float RotationMaximum,
    float LinearVelocityMaximum,
    float AngularVelocityMaximum,
    IReadOnlyList<string> Errors, float PositionRms = 0f, float RotationRms = 0f,
    float LinearVelocityRms = 0f, float AngularVelocityRms = 0f, int ComparedBodyCount = 0)
{
    public bool Passes(float positionTolerance, float rotationTolerance, float velocityTolerance,
        float angularVelocityTolerance) => !TickMismatch && MissingBodies == 0 && ExtraBodies == 0 &&
        PositionMaximum <= positionTolerance && RotationMaximum <= rotationTolerance &&
        LinearVelocityMaximum <= velocityTolerance && AngularVelocityMaximum <= angularVelocityTolerance &&
        !Errors.Any(error => !IsNumericError(error));

    private static bool IsNumericError(string error) => error.StartsWith("position:", StringComparison.Ordinal) ||
        error.StartsWith("rotation:", StringComparison.Ordinal) ||
        error.StartsWith("linear-velocity:", StringComparison.Ordinal) ||
        error.StartsWith("angular-velocity:", StringComparison.Ordinal);
}

public static class PhysicsWorldComparator
{
    public static PhysicsWorldComparison Compare(SourcePhysicsWorldState expected, SourcePhysicsWorldState actual)
    {
        var expectedById = expected.Bodies.ToDictionary(body => body.BodyId);
        var actualById = actual.Bodies.ToDictionary(body => body.BodyId);
        var errors = new List<string>();
        var missing = 0;
        var extra = 0;
        var positionMaximum = 0f;
        var rotationMaximum = 0f;
        var linearMaximum = 0f;
        var angularMaximum = 0f;
        var positionSum = 0f;
        var rotationSum = 0f;
        var linearSum = 0f;
        var angularSum = 0f;
        var comparedBodyCount = 0;

        foreach (var expectedBody in expectedById)
        {
            if (!actualById.TryGetValue(expectedBody.Key, out var actualBody))
            {
                missing++;
                errors.Add($"missing-body:{expectedBody.Key}");
                continue;
            }
            var expectedState = expectedBody.Value.State;
            var actualState = actualBody.State;
            var positionError = Vector3.Distance(expectedState.Position, actualState.Position);
            var rotationError = 1f - MathF.Min(1f, MathF.Abs(Quaternion.Dot(expectedState.Rotation, actualState.Rotation)));
            var linearError = Vector3.Distance(expectedState.LinearVelocity, actualState.LinearVelocity);
            var angularError = Vector3.Distance(expectedState.AngularVelocity, actualState.AngularVelocity);
            positionSum += positionError * positionError;
            rotationSum += rotationError * rotationError;
            linearSum += linearError * linearError;
            angularSum += angularError * angularError;
            comparedBodyCount++;
            positionMaximum = MathF.Max(positionMaximum, positionError);
            rotationMaximum = MathF.Max(rotationMaximum, rotationError);
            linearMaximum = MathF.Max(linearMaximum, linearError);
            angularMaximum = MathF.Max(angularMaximum, angularError);
            if (expectedState.Active != actualState.Active) errors.Add($"active:{expectedBody.Key}");
            if (expectedState.JoltMotionType != actualState.JoltMotionType) errors.Add($"motion-type:{expectedBody.Key}");
            if (expectedState.GravityFactor != actualState.GravityFactor) errors.Add($"gravity-factor:{expectedBody.Key}");
            if (expectedState.Friction != actualState.Friction) errors.Add($"friction:{expectedBody.Key}");
            if (expectedState.Restitution != actualState.Restitution) errors.Add($"restitution:{expectedBody.Key}");
            if (expectedState.ContentsMask != actualState.ContentsMask) errors.Add($"contents:{expectedBody.Key}");
            if (expectedState.CollisionGroup != actualState.CollisionGroup) errors.Add($"collision-group:{expectedBody.Key}");
            if (expectedState.SolidFlags != actualState.SolidFlags) errors.Add($"solid-flags:{expectedBody.Key}");
            if (expectedState.UserData != actualState.UserData) errors.Add($"user-data:{expectedBody.Key}");
            if (expectedState.Position != actualState.Position) errors.Add($"position:{expectedBody.Key}");
            if (expectedState.Rotation != actualState.Rotation) errors.Add($"rotation:{expectedBody.Key}");
            if (expectedState.LinearVelocity != actualState.LinearVelocity) errors.Add($"linear-velocity:{expectedBody.Key}");
            if (expectedState.AngularVelocity != actualState.AngularVelocity) errors.Add($"angular-velocity:{expectedBody.Key}");
        }
        foreach (var actualBody in actualById.Keys)
        {
            if (!expectedById.ContainsKey(actualBody))
            {
                extra++;
                errors.Add($"extra-body:{actualBody}");
            }
        }
        var divisor = Math.Max(1, comparedBodyCount);
        return new PhysicsWorldComparison(expected.Tick != actual.Tick, missing, extra, positionMaximum, rotationMaximum,
            linearMaximum, angularMaximum, errors, MathF.Sqrt(positionSum / divisor),
            MathF.Sqrt(rotationSum / divisor), MathF.Sqrt(linearSum / divisor),
            MathF.Sqrt(angularSum / divisor), comparedBodyCount);
    }
}
