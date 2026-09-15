using System.Numerics;

namespace SourcePhysics;

/// <summary>
/// Port of Source's CShotManipulator::ApplySpread. The caller supplies the
/// title's RandomFloat(-1, 1) stream so prediction/server seed semantics remain
/// outside the physics bridge.
/// </summary>
public sealed class SourceShotManipulator
{
    private readonly Vector3 shotDirection;
    private readonly Vector3 right;
    private readonly Vector3 up;

    public SourceShotManipulator(Vector3 forward)
    {
        if (!float.IsFinite(forward.X) || !float.IsFinite(forward.Y) || !float.IsFinite(forward.Z) ||
            forward.LengthSquared() < 1e-12f)
            throw new ArgumentException("Shot direction must be finite and non-zero.", nameof(forward));
        shotDirection = Vector3.Normalize(forward);
        var referenceUp = MathF.Abs(Vector3.Dot(shotDirection, Vector3.UnitY)) > 0.999f
            ? Vector3.UnitZ : Vector3.UnitY;
        right = Vector3.Normalize(Vector3.Cross(referenceUp, shotDirection));
        up = Vector3.Normalize(Vector3.Cross(shotDirection, right));
    }

    public Vector3 ShotDirection => shotDirection;
    public Vector3 Right => right;
    public Vector3 Up => up;

    public Vector3 ApplySpread(Vector3 spread, float bias,
        float shotBiasMin, float shotBiasMax, Func<float, float, float> randomFloat)
    {
        ArgumentNullException.ThrowIfNull(randomFloat);
        if (!float.IsFinite(spread.X) || !float.IsFinite(spread.Y) || !float.IsFinite(spread.Z) ||
            !float.IsFinite(bias) || !float.IsFinite(shotBiasMin) || !float.IsFinite(shotBiasMax))
            throw new ArgumentException("Spread and bias inputs must be finite.");

        bias = Math.Clamp(bias, 0f, 1f);
        var shotBias = (shotBiasMax - shotBiasMin) * bias + shotBiasMin;
        var flatness = MathF.Abs(shotBias) * 0.5f;
        float x;
        float y;
        float radiusSquared;
        do
        {
            x = randomFloat(-1f, 1f) * flatness + randomFloat(-1f, 1f) * (1f - flatness);
            y = randomFloat(-1f, 1f) * flatness + randomFloat(-1f, 1f) * (1f - flatness);
            if (shotBias < 0f)
            {
                x = x >= 0f ? 1f - x : -1f - x;
                y = y >= 0f ? 1f - y : -1f - y;
            }
            radiusSquared = x * x + y * y;
        } while (radiusSquared > 1f);

        return shotDirection + x * spread.X * right + y * spread.Y * up;
    }
}
