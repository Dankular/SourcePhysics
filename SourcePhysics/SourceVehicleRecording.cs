using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

/// A complete vehicle observation at one authoritative fixed tick. The frame
/// contains only contract data; it does not infer native Jolt solver state.
public sealed record SourceVehicleTickFrame(
    int Tick,
    SourceVehicleControl Control,
    SourceVehicleOperatingState OperatingState,
    SourceVehicleWheelContact[] Contacts,
    SourceVehicleWheelSkidSample[] SkidSamples);

/// Fixed-tick vehicle differential artifact. A title adapter can capture the
/// Source and Stride observations into the same schema and compare them later.
public sealed class SourceVehicleRecording
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };

    public string Profile { get; init; } = "source-vehicle";
    public float FixedStepSeconds { get; init; } = 1f / 66f;
    public List<SourceVehicleTickFrame> Frames { get; init; } = new();

    public void Capture(int tick, SourceVehicleControl control,
        SourceVehicleOperatingState operatingState,
        IReadOnlyList<SourceVehicleWheelContact> contacts,
        IReadOnlyList<SourceVehicleWheelSkidSample> skidSamples)
    {
        ArgumentNullException.ThrowIfNull(control);
        ArgumentNullException.ThrowIfNull(operatingState);
        ArgumentNullException.ThrowIfNull(contacts);
        ArgumentNullException.ThrowIfNull(skidSamples);
        Frames.Add(new(tick, control, operatingState, contacts.ToArray(), skidSamples.ToArray()));
    }

    public void Validate()
    {
        if (!float.IsFinite(FixedStepSeconds) || FixedStepSeconds <= 0f)
            throw new InvalidDataException("Vehicle recording fixed-step duration must be finite and positive.");
        var previousTick = -1;
        foreach (var frame in Frames)
        {
            if (frame.Tick < 0 || frame.Tick <= previousTick)
                throw new InvalidDataException("Vehicle recording ticks must be non-negative and strictly increasing.");
            previousTick = frame.Tick;
            ValidateFinite(frame.OperatingState.SpeedSourceUnitsPerSecond, "speed");
            ValidateFinite(frame.OperatingState.EngineRpm, "engine-rpm");
            ValidateFinite(frame.OperatingState.BoostDelaySeconds, "boost-delay");
            ValidateFinite(frame.OperatingState.SkidSpeedSourceUnitsPerSecond, "skid-speed");
            ValidateFinite(frame.OperatingState.SteeringAngleDegrees, "steering-angle");
            foreach (var contact in frame.Contacts)
            {
                ValidateVector(contact.ContactPointMeters, "contact-point");
                ValidateVector(contact.ContactNormal, "contact-normal");
                ValidateFinite(contact.SuspensionLengthMeters, "suspension-length");
                ValidateFinite(contact.SurfaceFriction, "surface-friction");
            }
            foreach (var skid in frame.SkidSamples)
                ValidateVector(skid.ContactPointVelocitySourceUnitsPerSecond, "contact-point-velocity");
        }
    }

    public string ToJson()
    {
        Validate();
        return JsonSerializer.Serialize(this, JsonOptions);
    }

    public static SourceVehicleRecording FromJson(string json)
    {
        var recording = JsonSerializer.Deserialize<SourceVehicleRecording>(json, JsonOptions)
            ?? throw new InvalidDataException("Invalid vehicle recording.");
        recording.Validate();
        return recording;
    }

    private static void ValidateFinite(float value, string field)
    {
        if (!float.IsFinite(value)) throw new InvalidDataException($"Vehicle recording {field} must be finite.");
    }

    private static void ValidateVector(Vector3 value, string field)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new InvalidDataException($"Vehicle recording {field} must be finite.");
    }
}

public readonly record struct SourceVehicleParityError(int Tick, int Wheel, string Field);

public sealed record SourceVehicleParityComparison(
    float NumericMaximum,
    float NumericRms,
    IReadOnlyList<SourceVehicleParityError> Errors,
    int TimingMismatchCount = 0)
{
    public bool Passes(float numericTolerance) =>
        TimingMismatchCount == 0 && NumericMaximum <= numericTolerance && Errors.Count == 0;
}

public static class SourceVehicleRecordingComparator
{
    public static SourceVehicleParityComparison Compare(
        SourceVehicleRecording expected, SourceVehicleRecording actual,
        float numericTolerance = 0f)
    {
        ArgumentNullException.ThrowIfNull(expected);
        ArgumentNullException.ThrowIfNull(actual);
        expected.Validate();
        actual.Validate();
        var errors = new List<SourceVehicleParityError>();
        var timing = Math.Abs(expected.Frames.Count - actual.Frames.Count);
        if (expected.Frames.Count != actual.Frames.Count)
            errors.Add(new(-1, -1, "frame-count"));
        var sum = 0f;
        var samples = 0;
        var maximum = 0f;
        var count = Math.Min(expected.Frames.Count, actual.Frames.Count);
        for (var index = 0; index < count; index++)
        {
            var left = expected.Frames[index];
            var right = actual.Frames[index];
            if (left.Tick != right.Tick)
            {
                timing++;
                errors.Add(new(left.Tick, -1, "tick"));
            }
            CompareControl(left.Tick, left.Control, right.Control, errors);
            CompareOperating(left.Tick, left.OperatingState, right.OperatingState, errors);
            CompareNumeric(left.Tick, -1, left.OperatingState.SpeedSourceUnitsPerSecond,
                right.OperatingState.SpeedSourceUnitsPerSecond, "speed", numericTolerance,
                ref maximum, ref sum, ref samples, errors);
            CompareNumeric(left.Tick, -1, left.OperatingState.EngineRpm,
                right.OperatingState.EngineRpm, "engine-rpm", numericTolerance,
                ref maximum, ref sum, ref samples, errors);
            if (left.Contacts.Length != right.Contacts.Length)
                errors.Add(new(left.Tick, -1, "contact-count"));
            var wheelCount = Math.Min(left.Contacts.Length, right.Contacts.Length);
            for (var wheel = 0; wheel < wheelCount; wheel++)
                CompareContact(left.Tick, wheel, left.Contacts[wheel], right.Contacts[wheel],
                    numericTolerance, ref maximum, ref sum, ref samples, errors);
            if (left.SkidSamples.Length != right.SkidSamples.Length)
                errors.Add(new(left.Tick, -1, "skid-sample-count"));
            var skidCount = Math.Min(left.SkidSamples.Length, right.SkidSamples.Length);
            for (var wheel = 0; wheel < skidCount; wheel++)
            {
                var a = left.SkidSamples[wheel];
                var b = right.SkidSamples[wheel];
                if (a.InContact != b.InContact) errors.Add(new(left.Tick, wheel, "skid-contact"));
                if (a.SurfaceId != b.SurfaceId) errors.Add(new(left.Tick, wheel, "skid-surface"));
                CompareVector(left.Tick, wheel, a.ContactPointVelocitySourceUnitsPerSecond,
                    b.ContactPointVelocitySourceUnitsPerSecond, "skid-velocity", numericTolerance,
                    ref maximum, ref sum, ref samples, errors);
            }
        }
        return new(maximum, MathF.Sqrt(sum / Math.Max(1, samples)), errors, timing);
    }

    private static void CompareControl(int tick, SourceVehicleControl a, SourceVehicleControl b,
        List<SourceVehicleParityError> errors)
    {
        if (a.Throttle != b.Throttle) errors.Add(new(tick, -1, "throttle"));
        if (a.Steering != b.Steering) errors.Add(new(tick, -1, "steering"));
        if (a.Brake != b.Brake) errors.Add(new(tick, -1, "brake"));
        if (a.Boost != b.Boost) errors.Add(new(tick, -1, "boost"));
        if (a.Handbrake != b.Handbrake || a.HandbrakeLeft != b.HandbrakeLeft ||
            a.HandbrakeRight != b.HandbrakeRight || a.BrakePedal != b.BrakePedal ||
            a.HasBrakePedal != b.HasBrakePedal || a.AnalogSteering != b.AnalogSteering)
            errors.Add(new(tick, -1, "control-flags"));
    }

    private static void CompareOperating(int tick, SourceVehicleOperatingState a,
        SourceVehicleOperatingState b, List<SourceVehicleParityError> errors)
    {
        if (a.Gear != b.Gear) errors.Add(new(tick, -1, "gear"));
        if (a.BoostTimeLeftMilliseconds != b.BoostTimeLeftMilliseconds)
            errors.Add(new(tick, -1, "boost-time"));
        if (a.SkidSurfaceId != b.SkidSurfaceId || a.WheelsNotInContact != b.WheelsNotInContact ||
            a.WheelsInContact != b.WheelsInContact || a.IsTorqueBoosting != b.IsTorqueBoosting)
            errors.Add(new(tick, -1, "operating-state"));
    }

    private static void CompareContact(int tick, int wheel, SourceVehicleWheelContact a,
        SourceVehicleWheelContact b, float tolerance, ref float maximum, ref float sum,
        ref int samples, List<SourceVehicleParityError> errors)
    {
        if (a.InContact != b.InContact) errors.Add(new(tick, wheel, "contact-state"));
        if (a.SurfaceId != b.SurfaceId) errors.Add(new(tick, wheel, "surface"));
        if (a.BodyId != b.BodyId) errors.Add(new(tick, wheel, "body"));
        CompareVector(tick, wheel, a.ContactPointMeters, b.ContactPointMeters, "contact-point",
            tolerance, ref maximum, ref sum, ref samples, errors);
        CompareVector(tick, wheel, a.ContactNormal, b.ContactNormal, "contact-normal",
            tolerance, ref maximum, ref sum, ref samples, errors);
        CompareNumeric(tick, wheel, a.SuspensionLengthMeters, b.SuspensionLengthMeters,
            "suspension-length", tolerance, ref maximum, ref sum, ref samples, errors);
        CompareNumeric(tick, wheel, a.SurfaceFriction, b.SurfaceFriction, "surface-friction",
            tolerance, ref maximum, ref sum, ref samples, errors);
        CompareVector(tick, wheel, a.SurfaceVelocityMetersPerSecond,
            b.SurfaceVelocityMetersPerSecond, "surface-velocity", tolerance,
            ref maximum, ref sum, ref samples, errors);
    }

    private static void CompareVector(int tick, int wheel, Vector3 a, Vector3 b, string field,
        float tolerance, ref float maximum, ref float sum, ref int samples,
        List<SourceVehicleParityError> errors)
    {
        CompareNumeric(tick, wheel, Vector3.Distance(a, b), 0f, field, tolerance,
            ref maximum, ref sum, ref samples, errors);
    }

    private static void CompareNumeric(int tick, int wheel, float a, float b, string field,
        float tolerance, ref float maximum, ref float sum, ref int samples,
        List<SourceVehicleParityError> errors)
    {
        var error = MathF.Abs(a - b);
        maximum = MathF.Max(maximum, error);
        sum += error * error;
        samples++;
        if (error > tolerance) errors.Add(new(tick, wheel, field));
    }
}
