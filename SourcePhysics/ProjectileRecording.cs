using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

public readonly record struct ProjectileFrame(int Tick, ProjectileState State, ProjectileHit? Hit);

public sealed class ProjectileRecording
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    public string Profile { get; init; } = "source-projectile";
    public List<ProjectileFrame> Frames { get; init; } = new();

    public void Capture(int tick, ProjectileState state, ProjectileHit? hit) => Frames.Add(new(tick, state, hit));
    public string ToJson() => JsonSerializer.Serialize(this, JsonOptions);
    public static ProjectileRecording FromJson(string json) =>
        JsonSerializer.Deserialize<ProjectileRecording>(json, JsonOptions)
        ?? throw new InvalidDataException("Invalid projectile recording.");
}

public readonly record struct ProjectileParityError(int Tick, string Field);

public sealed record ProjectileParityComparison(float PositionMaximum, float VelocityMaximum,
    IReadOnlyList<ProjectileParityError> Errors, float PositionRms = 0f, float VelocityRms = 0f,
    int TimingMismatchCount = 0)
{
    public bool Passes(float positionTolerance, float velocityTolerance) =>
        TimingMismatchCount == 0 && PositionMaximum <= positionTolerance &&
        VelocityMaximum <= velocityTolerance && Errors.Count == 0;
}

public static class ProjectileParityComparator
{
    public static ProjectileParityComparison Compare(IReadOnlyList<ProjectileFrame> expected,
        IReadOnlyList<ProjectileFrame> actual, float positionTolerance = 0f, float velocityTolerance = 0f)
    {
        var errors = new List<ProjectileParityError>();
        var positionMaximum = 0f;
        var velocityMaximum = 0f;
        var positionSum = 0f;
        var velocitySum = 0f;
        var timingMismatchCount = Math.Abs(expected.Count - actual.Count);
        if (expected.Count != actual.Count) errors.Add(new(-1, "frame-count"));
        var count = Math.Min(expected.Count, actual.Count);
        for (var index = 0; index < count; index++)
        {
            var left = expected[index];
            var right = actual[index];
            if (left.Tick != right.Tick)
            {
                timingMismatchCount++;
                errors.Add(new(left.Tick, "tick"));
            }
            var positionError = Vector3.Distance(left.State.Position, right.State.Position);
            var velocityError = Vector3.Distance(left.State.Velocity, right.State.Velocity);
            positionSum += positionError * positionError;
            velocitySum += velocityError * velocityError;
            positionMaximum = MathF.Max(positionMaximum, positionError);
            velocityMaximum = MathF.Max(velocityMaximum, velocityError);
            if (positionError > positionTolerance) errors.Add(new(left.Tick, "position"));
            if (velocityError > velocityTolerance) errors.Add(new(left.Tick, "velocity"));
            if (left.State.Active != right.State.Active) errors.Add(new(left.Tick, "active"));
            if (left.State.Bounces != right.State.Bounces) errors.Add(new(left.Tick, "bounces"));
            if (left.State.Penetrations != right.State.Penetrations) errors.Add(new(left.Tick, "penetrations"));
            if (left.State.PenetrationPowerRemaining != right.State.PenetrationPowerRemaining)
                errors.Add(new(left.Tick, "penetration-power"));
            CompareHits(left.Tick, left.Hit, right.Hit, errors);
        }
        var divisor = Math.Max(1, count);
        return new(positionMaximum, velocityMaximum, errors,
            MathF.Sqrt(positionSum / divisor), MathF.Sqrt(velocitySum / divisor), timingMismatchCount);
    }

    private static void CompareHits(int tick, ProjectileHit? expected, ProjectileHit? actual,
        List<ProjectileParityError> errors)
    {
        if (expected.HasValue != actual.HasValue) { errors.Add(new(tick, "hit")); return; }
        if (!expected.HasValue) return;
        var left = expected.Value;
        var right = actual!.Value;
        if (left.BodyId != right.BodyId) errors.Add(new(tick, "hit-body"));
        if (left.SurfaceId != right.SurfaceId) errors.Add(new(tick, "hit-surface"));
        if (left.Restitution != right.Restitution) errors.Add(new(tick, "hit-restitution"));
        if (left.ThicknessInches != right.ThicknessInches) errors.Add(new(tick, "hit-thickness"));
        if (left.Position != right.Position) errors.Add(new(tick, "hit-position"));
        if (left.Normal != right.Normal) errors.Add(new(tick, "hit-normal"));
    }
}
