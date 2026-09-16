using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

public readonly record struct SourcePhysicsContactFrame(
    uint BodyA,
    uint BodyB,
    Vector3 ContactPoint,
    Vector3 Normal,
    float PenetrationDepth,
    bool Persisted,
    bool Removed);

public readonly record struct SourcePhysicsCollisionFrame(
    uint BodyA,
    uint BodyB,
    Vector3 ContactPoint,
    Vector3 Normal,
    float PenetrationDepth);

public readonly record struct SourcePhysicsImpulseFrame(
    uint BodyId,
    Vector3 Impulse,
    Vector3 WorldPoint,
    bool AtPoint);

public sealed record SourcePhysicsTickFrame(
    int Tick,
    SourcePhysicsBodySnapshot[] Bodies,
    SourcePhysicsContactFrame[] Contacts,
    SourcePhysicsImpulseFrame[] Impulses,
    SourcePhysicsCollisionFrame[]? Collisions = null);

/// Fixed-tick rigid-body differential recording. Contact frames describe the
/// Jolt event stream; impulse frames describe explicit gameplay impulses sent
/// through the bridge. Native solver internals are intentionally not inferred.
public sealed class SourcePhysicsRecording : IDisposable
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    private readonly JoltPhysicsHost host;
    private readonly List<SourcePhysicsContactFrame> pendingContacts = new();
    private readonly List<SourcePhysicsCollisionFrame> pendingCollisions = new();
    private readonly List<SourcePhysicsImpulseFrame> pendingImpulses = new();
    private bool disposed;

    public string Profile { get; init; } = "source-rigid-body";
    public List<SourcePhysicsTickFrame> Frames { get; } = new();

    public SourcePhysicsRecording(JoltPhysicsHost host)
    {
        this.host = host ?? throw new ArgumentNullException(nameof(host));
        host.Contacts.ContactAdded += OnContactAdded;
        host.Contacts.ContactPersisted += OnContactPersisted;
        host.Contacts.ContactRemoved += OnContactRemoved;
        host.Contacts.CollisionStarted += OnCollisionStarted;
        host.ImpulseApplied += OnImpulseApplied;
    }

    public void Capture(int tick)
    {
        ThrowIfDisposed();
        Frames.Add(new SourcePhysicsTickFrame(tick, host.CaptureState(tick).Bodies,
            pendingContacts.ToArray(), pendingImpulses.ToArray(), pendingCollisions.ToArray()));
        pendingContacts.Clear();
        pendingCollisions.Clear();
        pendingImpulses.Clear();
    }

    public string ToJson() => JsonSerializer.Serialize(this, JsonOptions);

    public void Dispose()
    {
        if (disposed) return;
        host.Contacts.ContactAdded -= OnContactAdded;
        host.Contacts.ContactPersisted -= OnContactPersisted;
        host.Contacts.ContactRemoved -= OnContactRemoved;
        host.Contacts.CollisionStarted -= OnCollisionStarted;
        host.ImpulseApplied -= OnImpulseApplied;
        disposed = true;
    }

    private void OnContactAdded(SourceContactEvent value) => pendingContacts.Add(new(value.BodyA, value.BodyB,
        value.ContactPoint, value.Normal, value.PenetrationDepth, false, false));

    private void OnContactPersisted(SourceContactEvent value) => pendingContacts.Add(new(value.BodyA, value.BodyB,
        value.ContactPoint, value.Normal, value.PenetrationDepth, true, false));

    private void OnContactRemoved(SourceContactRemovedEvent value) => pendingContacts.Add(new(value.BodyA, value.BodyB,
        default, default, 0f, false, true));

    private void OnCollisionStarted(SourceCollisionEvent value) => pendingCollisions.Add(new(value.BodyA, value.BodyB,
        value.ContactPoint, value.Normal, value.PenetrationDepth));

    private void OnImpulseApplied(SourcePhysicsImpulseEvent value) => pendingImpulses.Add(
        new(value.BodyId, value.Impulse, value.WorldPoint, value.AtPoint));

    private void ThrowIfDisposed()
    {
        if (disposed) throw new ObjectDisposedException(nameof(SourcePhysicsRecording));
    }

}

/// Serialized rigid-body recording artifact. It is intentionally detached
/// from a live Jolt host and can be compared or replayed by a caller.
public sealed record SourcePhysicsRecordingArtifact(
    string Profile,
    List<SourcePhysicsTickFrame> Frames)
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };

    public string ToJson() => JsonSerializer.Serialize(this, JsonOptions);

    public static SourcePhysicsRecordingArtifact FromJson(string json) =>
        JsonSerializer.Deserialize<SourcePhysicsRecordingArtifact>(json, JsonOptions)
        ?? throw new InvalidDataException("Invalid physics recording artifact.");
}

public sealed record SourcePhysicsRecordingComparison(
    int TimingMismatchCount,
    float PositionMaximum,
    float RotationMaximum,
    float LinearVelocityMaximum,
    float AngularVelocityMaximum,
    IReadOnlyList<string> Errors, float PositionRms = 0f, float RotationRms = 0f,
    float LinearVelocityRms = 0f, float AngularVelocityRms = 0f)
{
    public bool Passes(float positionTolerance, float rotationTolerance,
        float velocityTolerance, float angularVelocityTolerance) =>
        TimingMismatchCount == 0 && PositionMaximum <= positionTolerance &&
        RotationMaximum <= rotationTolerance && LinearVelocityMaximum <= velocityTolerance &&
        AngularVelocityMaximum <= angularVelocityTolerance && Errors.Count == 0;
}

public static class SourcePhysicsRecordingComparator
{
    public static SourcePhysicsRecordingComparison Compare(
        SourcePhysicsRecordingArtifact expected, SourcePhysicsRecordingArtifact actual)
    {
        ArgumentNullException.ThrowIfNull(expected);
        ArgumentNullException.ThrowIfNull(actual);
        var errors = new List<string>();
        var timing = expected.Frames.Count == actual.Frames.Count ? 0 : Math.Abs(expected.Frames.Count - actual.Frames.Count);
        var positionMaximum = 0f;
        var rotationMaximum = 0f;
        var linearMaximum = 0f;
        var angularMaximum = 0f;
        var positionSum = 0f;
        var rotationSum = 0f;
        var linearSum = 0f;
        var angularSum = 0f;
        var comparedBodyCount = 0;
        var count = Math.Min(expected.Frames.Count, actual.Frames.Count);
        for (var index = 0; index < count; index++)
        {
            var left = expected.Frames[index];
            var right = actual.Frames[index];
            if (left.Tick != right.Tick) { timing++; errors.Add($"tick:{left.Tick}"); }
            var world = PhysicsWorldComparator.Compare(
                new SourcePhysicsWorldState(left.Tick, left.Bodies),
                new SourcePhysicsWorldState(right.Tick, right.Bodies));
            timing += world.TickMismatch ? 1 : 0;
            positionMaximum = MathF.Max(positionMaximum, world.PositionMaximum);
            rotationMaximum = MathF.Max(rotationMaximum, world.RotationMaximum);
            linearMaximum = MathF.Max(linearMaximum, world.LinearVelocityMaximum);
            angularMaximum = MathF.Max(angularMaximum, world.AngularVelocityMaximum);
            positionSum += world.PositionRms * world.PositionRms * world.ComparedBodyCount;
            rotationSum += world.RotationRms * world.RotationRms * world.ComparedBodyCount;
            linearSum += world.LinearVelocityRms * world.LinearVelocityRms * world.ComparedBodyCount;
            angularSum += world.AngularVelocityRms * world.AngularVelocityRms * world.ComparedBodyCount;
            comparedBodyCount += world.ComparedBodyCount;
            errors.AddRange(world.Errors.Select(error => $"frame:{left.Tick}:{error}"));
            if (!left.Contacts.SequenceEqual(right.Contacts)) errors.Add($"contacts:{left.Tick}");
            if (!left.Impulses.SequenceEqual(right.Impulses)) errors.Add($"impulses:{left.Tick}");
            if (!((left.Collisions ?? Array.Empty<SourcePhysicsCollisionFrame>()).SequenceEqual(
                    right.Collisions ?? Array.Empty<SourcePhysicsCollisionFrame>())))
                errors.Add($"collisions:{left.Tick}");
        }
        var divisor = Math.Max(1, comparedBodyCount);
        return new(timing, positionMaximum, rotationMaximum, linearMaximum, angularMaximum, errors,
            MathF.Sqrt(positionSum / divisor), MathF.Sqrt(rotationSum / divisor),
            MathF.Sqrt(linearSum / divisor), MathF.Sqrt(angularSum / divisor));
    }
}

public readonly record struct SourcePhysicsImpulseEvent(
    uint BodyId,
    Vector3 Impulse,
    Vector3 WorldPoint,
    bool AtPoint);
