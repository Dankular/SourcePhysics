using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

public readonly record struct MovementFrame(
    int Tick,
    Vector3 Position,
    Vector3 Velocity,
    GroundState Ground,
    Vector3 GroundNormal,
    int GroundBodyId,
    bool Ducking,
    bool Jumped,
    SourceWaterLevel WaterLevel,
    SourceMoveType MoveType,
    Buttons Buttons = Buttons.None,
    Vector2 Move = default,
    float UpMove = 0f,
    float ViewYawRadians = 0f,
    float ViewPitchRadians = 0f,
    Quaternion Orientation = default,
    Vector3 AngularVelocity = default,
    MovementContact[]? Contacts = null,
    Vector3[]? Impulses = null,
    float ViewHeightSourceUnits = 64f,
    float SurfaceFriction = 1f,
    float SurfaceMaxSpeedFactor = 1f,
    float SurfaceJumpFactor = 1f,
    float WaterJumpTime = 0f,
    Vector3 BaseVelocity = default,
    float FallVelocity = 0f,
    bool PreviousJumpDown = false,
    float DuckTransitionTimeSeconds = 0f,
    bool DuckTransitioningUp = false);

public sealed class MovementRecording
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    public string Profile { get; init; } = "source-multiplayer-baseline";
    public float FixedStepSeconds { get; init; } = 1f / 66f;
    public List<MovementFrame> Frames { get; init; } = new();

    public void Validate()
    {
        if (!float.IsFinite(FixedStepSeconds) || FixedStepSeconds <= 0f)
            throw new InvalidDataException("Movement recording fixed-step duration must be finite and positive.");
        var previousTick = -1;
        foreach (var frame in Frames)
        {
            if (frame.Tick < 0 || frame.Tick <= previousTick)
                throw new InvalidDataException("Movement recording frame ticks must be non-negative and strictly increasing.");
            previousTick = frame.Tick;
            ValidateFinite(frame.Position, "position");
            ValidateFinite(frame.Velocity, "velocity");
            ValidateFinite(frame.GroundNormal, "ground normal");
            ValidateFinite(frame.Move, "move input");
            if (!float.IsFinite(frame.UpMove) || !float.IsFinite(frame.ViewYawRadians) || !float.IsFinite(frame.ViewPitchRadians) ||
                !float.IsFinite(frame.ViewHeightSourceUnits) || !float.IsFinite(frame.DuckTransitionTimeSeconds) || !float.IsFinite(frame.SurfaceFriction) ||
                !float.IsFinite(frame.SurfaceMaxSpeedFactor) || !float.IsFinite(frame.SurfaceJumpFactor) ||
                !float.IsFinite(frame.WaterJumpTime) || !float.IsFinite(frame.FallVelocity))
                throw new InvalidDataException($"Movement recording frame {frame.Tick} contains a non-finite scalar.");
        }
    }

    public void Capture(int tick, MovementState state) => Capture(tick, state, default);
    public void Capture(int tick, MovementState state, SourceInput input, Quaternion orientation = default,
        Vector3 angularVelocity = default, IReadOnlyList<MovementContact>? contacts = null,
        IReadOnlyList<Vector3>? impulses = null)
    {
        if (orientation == default) orientation = Quaternion.Identity;
        Frames.Add(new(tick, state.Position, state.Velocity, state.Ground, state.GroundNormal, state.GroundBodyId,
            state.Ducking, state.Jumped, state.WaterLevel, state.MoveType, input.Buttons, input.Move, input.UpMove,
            input.ViewYawRadians, input.ViewPitchRadians, orientation,
            angularVelocity, contacts?.ToArray(), impulses?.ToArray(), state.ViewHeightSourceUnits,
            state.SurfaceFriction, state.SurfaceMaxSpeedFactor, state.SurfaceJumpFactor,
            state.WaterJumpTime, state.BaseVelocity, state.FallVelocity, state.PreviousJumpDown,
            state.DuckTransitionTimeSeconds, state.DuckTransitioningUp));
    }
    public string ToJson()
    {
        Validate();
        return JsonSerializer.Serialize(this, JsonOptions);
    }

    public static MovementRecording FromJson(string json)
    {
        var recording = JsonSerializer.Deserialize<MovementRecording>(json, JsonOptions)
            ?? throw new InvalidDataException("Invalid movement recording.");
        recording.Validate();
        return recording;
    }

    private static void ValidateFinite(Vector3 value, string field)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new InvalidDataException($"Movement recording {field} must be finite.");
    }

    private static void ValidateFinite(Vector2 value, string field)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y))
            throw new InvalidDataException($"Movement recording {field} must be finite.");
    }
}

public readonly record struct ParityError(int Tick, float PositionError, float VelocityError, string Field);

public sealed record ParityComparison(float PositionRms, float PositionMaximum, float VelocityRms, float VelocityMaximum,
    IReadOnlyList<ParityError> Errors, float OrientationMaximum = 0f, float AngularVelocityMaximum = 0f,
    int TimingMismatchCount = 0, float GroundNormalMaximum = 0f)
{
    public bool Passes(float positionTolerance, float velocityTolerance, float orientationTolerance = 0f,
        float angularVelocityTolerance = 0f, float groundNormalTolerance = 0f) => TimingMismatchCount == 0 && PositionMaximum <= positionTolerance &&
        VelocityMaximum <= velocityTolerance && OrientationMaximum <= orientationTolerance &&
        AngularVelocityMaximum <= angularVelocityTolerance && GroundNormalMaximum <= groundNormalTolerance &&
        !Errors.Any(error => error.Field is not ("position" or "pose" or "ground-normal"));
}

public static class ParityComparator
{
    public static ParityComparison Compare(IReadOnlyList<MovementFrame> expected, IReadOnlyList<MovementFrame> actual)
    {
        var count = Math.Min(expected.Count, actual.Count);
        var errors = new List<ParityError>();
        var positionSum = 0f; var velocitySum = 0f; var positionMaximum = 0f; var velocityMaximum = 0f;
        var orientationMaximum = 0f; var angularVelocityMaximum = 0f; var groundNormalMaximum = 0f;
        var timingMismatchCount = Math.Abs(expected.Count - actual.Count);
        for (var i = 0; i < count; i++)
        {
            if (expected[i].Tick != actual[i].Tick)
            {
                timingMismatchCount++;
                errors.Add(new(expected[i].Tick, 0f, 0f, "tick"));
            }
            var position = Vector3.Distance(expected[i].Position, actual[i].Position);
            var velocity = Vector3.Distance(expected[i].Velocity, actual[i].Velocity);
            var orientation = Quaternion.Dot(expected[i].Orientation, actual[i].Orientation);
            var orientationError = 1f - MathF.Min(1f, MathF.Abs(orientation));
            var angularVelocity = Vector3.Distance(expected[i].AngularVelocity, actual[i].AngularVelocity);
            var groundNormal = Vector3.Distance(expected[i].GroundNormal, actual[i].GroundNormal);
            positionSum += position * position; velocitySum += velocity * velocity;
            positionMaximum = MathF.Max(positionMaximum, position); velocityMaximum = MathF.Max(velocityMaximum, velocity);
            orientationMaximum = MathF.Max(orientationMaximum, orientationError); angularVelocityMaximum = MathF.Max(angularVelocityMaximum, angularVelocity);
            groundNormalMaximum = MathF.Max(groundNormalMaximum, groundNormal);
            if (position > 0f) errors.Add(new(expected[i].Tick, position, velocity, "position"));
            if (orientationError > 0f || angularVelocity > 0f) errors.Add(new(expected[i].Tick, orientationError, angularVelocity, "pose"));
            if (expected[i].Ground != actual[i].Ground) errors.Add(new(expected[i].Tick, position, velocity, "ground"));
            if (expected[i].WaterLevel != actual[i].WaterLevel) errors.Add(new(expected[i].Tick, position, velocity, "water"));
            if (expected[i].Buttons != actual[i].Buttons) errors.Add(new(expected[i].Tick, position, velocity, "buttons"));
            if (expected[i].Move != actual[i].Move) errors.Add(new(expected[i].Tick, position, velocity, "move-input"));
            if (expected[i].UpMove != actual[i].UpMove) errors.Add(new(expected[i].Tick, position, velocity, "up-move-input"));
            if (expected[i].ViewYawRadians != actual[i].ViewYawRadians ||
                expected[i].ViewPitchRadians != actual[i].ViewPitchRadians)
                errors.Add(new(expected[i].Tick, position, velocity, "view-angle-input"));
            if (expected[i].Ducking != actual[i].Ducking) errors.Add(new(expected[i].Tick, position, velocity, "ducking"));
            if (expected[i].Jumped != actual[i].Jumped) errors.Add(new(expected[i].Tick, position, velocity, "jumped"));
            if (expected[i].MoveType != actual[i].MoveType) errors.Add(new(expected[i].Tick, position, velocity, "move-type"));
            if (expected[i].GroundBodyId != actual[i].GroundBodyId) errors.Add(new(expected[i].Tick, position, velocity, "ground-body"));
            if (groundNormal > 0f) errors.Add(new(expected[i].Tick, groundNormal, velocity, "ground-normal"));
            if (expected[i].BaseVelocity != actual[i].BaseVelocity) errors.Add(new(expected[i].Tick, position, velocity, "base-velocity"));
            if (expected[i].WaterJumpTime != actual[i].WaterJumpTime) errors.Add(new(expected[i].Tick, position, velocity, "water-jump"));
            if (expected[i].FallVelocity != actual[i].FallVelocity) errors.Add(new(expected[i].Tick, position, velocity, "fall-velocity"));
            if (expected[i].PreviousJumpDown != actual[i].PreviousJumpDown) errors.Add(new(expected[i].Tick, position, velocity, "previous-jump"));
            if (expected[i].DuckTransitionTimeSeconds != actual[i].DuckTransitionTimeSeconds ||
                expected[i].DuckTransitioningUp != actual[i].DuckTransitioningUp)
                errors.Add(new(expected[i].Tick, position, velocity, "duck-transition"));
            if (expected[i].ViewHeightSourceUnits != actual[i].ViewHeightSourceUnits) errors.Add(new(expected[i].Tick, position, velocity, "view-height"));
            if (expected[i].SurfaceFriction != actual[i].SurfaceFriction ||
                expected[i].SurfaceMaxSpeedFactor != actual[i].SurfaceMaxSpeedFactor ||
                expected[i].SurfaceJumpFactor != actual[i].SurfaceJumpFactor)
                errors.Add(new(expected[i].Tick, position, velocity, "surface-state"));
            CompareContacts(expected[i], actual[i], position, velocity, errors);
            CompareImpulses(expected[i], actual[i], position, velocity, errors);
        }
        var divisor = Math.Max(1, count);
        return new(MathF.Sqrt(positionSum / divisor), positionMaximum, MathF.Sqrt(velocitySum / divisor), velocityMaximum, errors,
            orientationMaximum, angularVelocityMaximum, timingMismatchCount, groundNormalMaximum);
    }

    private static void CompareContacts(in MovementFrame expected, in MovementFrame actual,
        float positionError, float velocityError, List<ParityError> errors)
    {
        var expectedContacts = expected.Contacts ?? Array.Empty<MovementContact>();
        var actualContacts = actual.Contacts ?? Array.Empty<MovementContact>();
        if (expectedContacts.Length != actualContacts.Length)
        {
            errors.Add(new(expected.Tick, positionError, velocityError, "contacts-count"));
            return;
        }
        for (var index = 0; index < expectedContacts.Length; index++)
        {
            if (expectedContacts[index] != actualContacts[index])
            {
                errors.Add(new(expected.Tick, positionError, velocityError, $"contact-{index}"));
                return;
            }
        }
    }

    private static void CompareImpulses(in MovementFrame expected, in MovementFrame actual,
        float positionError, float velocityError, List<ParityError> errors)
    {
        var expectedImpulses = expected.Impulses ?? Array.Empty<Vector3>();
        var actualImpulses = actual.Impulses ?? Array.Empty<Vector3>();
        if (expectedImpulses.Length != actualImpulses.Length)
        {
            errors.Add(new(expected.Tick, positionError, velocityError, "impulses-count"));
            return;
        }
        for (var index = 0; index < expectedImpulses.Length; index++)
        {
            if (expectedImpulses[index] != actualImpulses[index])
            {
                errors.Add(new(expected.Tick, positionError, velocityError, $"impulse-{index}"));
                return;
            }
        }
    }
}
