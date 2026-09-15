using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

public readonly record struct WeaponShotFrame(
    int Tick,
    int ShotIndex,
    int RandomSeed,
    Vector3 Origin,
    Vector3 Direction,
    bool Hit,
    HitscanHit HitData,
    int AmmoType = 0,
    int PlayerDamage = 0,
    float AppliedDamage = 0f,
    int DamageType = 0,
    SourceFireBulletsFlags Flags = SourceFireBulletsFlags.None,
    float DamageForceScale = 1f,
    bool PrimaryAttack = true,
    int TracerFrequency = 4,
    bool HitWater = false,
    bool SuppressSurfaceImpact = false,
    bool DamageSuppressed = false,
    Vector3 DamageForce = default,
    Vector3 TracerDestination = default);

public sealed class WeaponRecording
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    public string Profile { get; init; } = "source-hitscan";
    public List<WeaponShotFrame> Frames { get; init; } = new();

    public void Capture(int tick, int shotIndex, int randomSeed, Vector3 origin, Vector3 direction,
        bool hit, HitscanHit hitData) => Frames.Add(new(tick, shotIndex, randomSeed, origin, direction, hit, hitData));

    public void Capture(int tick, int shotIndex, int randomSeed, Vector3 origin, Vector3 direction,
        bool hit, HitscanHit hitData, in SourceFireBulletsInfo info, SourceFireBulletsImpact? impact)
    {
        Frames.Add(new(tick, shotIndex, randomSeed, origin, direction, hit, hitData,
            info.AmmoType, info.PlayerDamage, impact?.Damage ?? 0f,
            impact?.DamageType ?? info.DamageType, info.Flags,
            info.DamageForceScale, info.PrimaryAttack, info.TracerFrequency,
            impact?.HitWater ?? false, impact?.SuppressSurfaceImpact ?? false,
            impact?.DamageSuppressed ?? false, impact?.DamageForce ?? default,
            impact?.TracerDestination ?? (hit ? hitData.Position : origin + direction * info.DistanceMeters)));
    }

    public string ToJson() => JsonSerializer.Serialize(this, JsonOptions);
    public static WeaponRecording FromJson(string json) =>
        JsonSerializer.Deserialize<WeaponRecording>(json, JsonOptions)
        ?? throw new InvalidDataException("Invalid weapon recording.");
}

public readonly record struct WeaponParityError(int Tick, int ShotIndex, string Field);

public sealed record WeaponParityComparison(float DirectionMaximum, float FractionMaximum,
    IReadOnlyList<WeaponParityError> Errors, float DirectionRms = 0f, float FractionRms = 0f,
    int TimingMismatchCount = 0)
{
    public bool Passes(float directionTolerance, float fractionTolerance) =>
        TimingMismatchCount == 0 && DirectionMaximum <= directionTolerance &&
        FractionMaximum <= fractionTolerance && Errors.Count == 0;
}

public static class WeaponParityComparator
{
    public static WeaponParityComparison Compare(IReadOnlyList<WeaponShotFrame> expected,
        IReadOnlyList<WeaponShotFrame> actual, float directionTolerance = 0f, float fractionTolerance = 0f)
    {
        var errors = new List<WeaponParityError>();
        var directionMaximum = 0f;
        var fractionMaximum = 0f;
        var directionSum = 0f;
        var fractionSum = 0f;
        var timingMismatchCount = Math.Abs(expected.Count - actual.Count);
        if (expected.Count != actual.Count)
            errors.Add(new(-1, -1, "shot-count"));

        var count = Math.Min(expected.Count, actual.Count);
        for (var index = 0; index < count; index++)
        {
            var left = expected[index];
            var right = actual[index];
            if (left.Tick != right.Tick)
            {
                timingMismatchCount++;
                errors.Add(new(left.Tick, left.ShotIndex, "tick"));
            }
            if (left.ShotIndex != right.ShotIndex) errors.Add(new(left.Tick, left.ShotIndex, "shot-index"));
            if (left.RandomSeed != right.RandomSeed) errors.Add(new(left.Tick, left.ShotIndex, "random-seed"));
            if (left.Hit != right.Hit) errors.Add(new(left.Tick, left.ShotIndex, "hit"));
            if (left.Origin != right.Origin) errors.Add(new(left.Tick, left.ShotIndex, "origin"));
            if (left.AmmoType != right.AmmoType) errors.Add(new(left.Tick, left.ShotIndex, "ammo-type"));
            if (left.PlayerDamage != right.PlayerDamage) errors.Add(new(left.Tick, left.ShotIndex, "player-damage"));
            if (left.AppliedDamage != right.AppliedDamage) errors.Add(new(left.Tick, left.ShotIndex, "applied-damage"));
            if (left.DamageType != right.DamageType) errors.Add(new(left.Tick, left.ShotIndex, "damage-type"));
            if (left.Flags != right.Flags) errors.Add(new(left.Tick, left.ShotIndex, "flags"));
            if (left.DamageForceScale != right.DamageForceScale) errors.Add(new(left.Tick, left.ShotIndex, "damage-force-scale"));
            if (left.PrimaryAttack != right.PrimaryAttack) errors.Add(new(left.Tick, left.ShotIndex, "primary-attack"));
            if (left.TracerFrequency != right.TracerFrequency) errors.Add(new(left.Tick, left.ShotIndex, "tracer-frequency"));
            if (left.HitWater != right.HitWater) errors.Add(new(left.Tick, left.ShotIndex, "hit-water"));
            if (left.SuppressSurfaceImpact != right.SuppressSurfaceImpact) errors.Add(new(left.Tick, left.ShotIndex, "surface-impact"));
            if (left.DamageSuppressed != right.DamageSuppressed) errors.Add(new(left.Tick, left.ShotIndex, "damage-suppressed"));
            if (left.DamageForce != right.DamageForce) errors.Add(new(left.Tick, left.ShotIndex, "damage-force"));
            if (left.TracerDestination != right.TracerDestination) errors.Add(new(left.Tick, left.ShotIndex, "tracer-destination"));

            var directionError = Vector3.Distance(left.Direction, right.Direction);
            directionSum += directionError * directionError;
            directionMaximum = MathF.Max(directionMaximum, directionError);
            if (directionError > directionTolerance) errors.Add(new(left.Tick, left.ShotIndex, "direction"));

            if (left.Hit && right.Hit)
            {
                var fractionError = MathF.Abs(left.HitData.Fraction - right.HitData.Fraction);
                fractionSum += fractionError * fractionError;
                fractionMaximum = MathF.Max(fractionMaximum, fractionError);
                if (fractionError > fractionTolerance) errors.Add(new(left.Tick, left.ShotIndex, "fraction"));
                if (left.HitData.BodyId != right.HitData.BodyId) errors.Add(new(left.Tick, left.ShotIndex, "body"));
                if (left.HitData.SurfaceId != right.HitData.SurfaceId) errors.Add(new(left.Tick, left.ShotIndex, "surface"));
                if (left.HitData.HitGroup != right.HitData.HitGroup) errors.Add(new(left.Tick, left.ShotIndex, "hit-group"));
                if (left.HitData.Hitbox != right.HitData.Hitbox) errors.Add(new(left.Tick, left.ShotIndex, "hitbox"));
                if (left.HitData.PhysicsBone != right.HitData.PhysicsBone) errors.Add(new(left.Tick, left.ShotIndex, "physics-bone"));
                if (left.HitData.Contents != right.HitData.Contents) errors.Add(new(left.Tick, left.ShotIndex, "contents"));
            }
        }
        var divisor = Math.Max(1, count);
        return new(directionMaximum, fractionMaximum, errors,
            MathF.Sqrt(directionSum / divisor), MathF.Sqrt(fractionSum / divisor), timingMismatchCount);
    }
}
