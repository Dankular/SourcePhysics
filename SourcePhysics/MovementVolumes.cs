using System.Numerics;

namespace SourcePhysics;

public readonly record struct SourceAabb(Vector3 Min, Vector3 Max)
{
    public bool Contains(Vector3 point) => point.X >= Min.X && point.X <= Max.X && point.Y >= Min.Y && point.Y <= Max.Y && point.Z >= Min.Z && point.Z <= Max.Z;
}

public readonly record struct SourceWaterVolume(SourceAabb Bounds, float SurfaceHeight,
    SourceWaterLevel Type = SourceWaterLevel.Waist, SourceWaterCurrent Current = SourceWaterCurrent.None);
public readonly record struct SourceWaterJumpVolume(SourceAabb Bounds, Vector3 VelocitySourceUnitsPerSecond, float DurationSeconds);
public readonly record struct SourceLadderVolume(SourceAabb Bounds, Vector3 Normal, int BodyId = -1);

public sealed class SourceMovementVolumes
{
    private readonly float ladderFacingDotThreshold;
    private readonly List<SourceWaterVolume> water = new();
    private readonly List<SourceWaterJumpVolume> waterJumps = new();
    private readonly List<SourceLadderVolume> ladders = new();

    public SourceMovementVolumes(float ladderFacingDotThreshold = -0.707f) => this.ladderFacingDotThreshold = ladderFacingDotThreshold;

    public void AddWater(SourceWaterVolume volume) => water.Add(volume);
    public void AddWaterJump(SourceWaterJumpVolume volume)
    {
        if (volume.DurationSeconds <= 0f) throw new ArgumentOutOfRangeException(nameof(volume));
        waterJumps.Add(volume);
    }
    public void AddLadder(SourceLadderVolume volume) => ladders.Add(volume);

    public SourceWaterLevel GetWaterLevel(Vector3 position, bool crouched)
    {
        foreach (var volume in water)
        {
            if (!volume.Bounds.Contains(position)) continue;
            var depth = volume.SurfaceHeight - position.Y;
            if (depth >= 1.6256f) return SourceWaterLevel.Eyes;
            if (depth >= (crouched ? 0.7112f : 0.9144f)) return SourceWaterLevel.Waist;
            if (depth > 0f) return SourceWaterLevel.Feet;
        }
        return SourceWaterLevel.Dry;
    }

    public Vector3 GetWaterBaseVelocity(Vector3 position, SourceWaterLevel waterLevel)
    {
        if (waterLevel == SourceWaterLevel.Dry) return Vector3.Zero;
        foreach (var volume in water)
        {
            if (!volume.Bounds.Contains(position)) continue;
            var direction = Vector3.Zero;
            if (volume.Current.HasFlag(SourceWaterCurrent.Current0)) direction.X += 1f;
            if (volume.Current.HasFlag(SourceWaterCurrent.Current90)) direction.Z += 1f;
            if (volume.Current.HasFlag(SourceWaterCurrent.Current180)) direction.X -= 1f;
            if (volume.Current.HasFlag(SourceWaterCurrent.Current270)) direction.Z -= 1f;
            if (volume.Current.HasFlag(SourceWaterCurrent.CurrentUp)) direction.Y += 1f;
            if (volume.Current.HasFlag(SourceWaterCurrent.CurrentDown)) direction.Y -= 1f;
            return SourceUnits.ToMeters(direction * (50f * (int)waterLevel));
        }
        return Vector3.Zero;
    }

    public bool TryLadder(Vector3 position, Vector3 direction, out Vector3 normal, out int bodyId)
    {
        foreach (var ladder in ladders)
        {
            if (!ladder.Bounds.Contains(position)) continue;
            if (Vector3.Dot(direction, -ladder.Normal) < ladderFacingDotThreshold) continue;
            normal = ladder.Normal; bodyId = ladder.BodyId; return true;
        }
        normal = default; bodyId = -1; return false;
    }

    public bool TryWaterJump(Vector3 position, Vector3 direction, out Vector3 velocity, out float durationSeconds)
    {
        foreach (var volume in waterJumps)
        {
            if (!volume.Bounds.Contains(position)) continue;
            velocity = SourceUnits.ToMeters(volume.VelocitySourceUnitsPerSecond);
            durationSeconds = volume.DurationSeconds;
            return true;
        }
        velocity = default;
        durationSeconds = 0f;
        return false;
    }
}
