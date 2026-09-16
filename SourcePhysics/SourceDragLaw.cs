using System.Numerics;

namespace SourcePhysics;

/// <summary>Exact Source VPhysics air-drag integration for box collision objects.</summary>
public readonly record struct SourceDragBasis(
    Vector3 Linear,
    Vector3 Angular,
    float LinearCoefficient,
    float AngularCoefficient);

public static class SourceDragLaw
{
    public const float AirDensity = 2f;

    public static SourceDragBasis CreateBoxBasis(Vector3 halfExtentsMeters, float massKg, float inertiaScale,
        float linearCoefficient, float angularCoefficient)
    {
        if (halfExtentsMeters.X <= 0f || halfExtentsMeters.Y <= 0f || halfExtentsMeters.Z <= 0f)
            throw new ArgumentOutOfRangeException(nameof(halfExtentsMeters));
        if (massKg <= 0f) throw new ArgumentOutOfRangeException(nameof(massKg));
        if (inertiaScale <= 0f) throw new ArgumentOutOfRangeException(nameof(inertiaScale));
        if (linearCoefficient < 0f) throw new ArgumentOutOfRangeException(nameof(linearCoefficient));
        if (angularCoefficient < 0f) throw new ArgumentOutOfRangeException(nameof(angularCoefficient));

        var size = halfExtentsMeters * 2f;
        var linear = new Vector3(size.Y * size.Z, size.X * size.Z, size.X * size.Y) / massKg;
        var inertia = new Vector3(
            massKg * (halfExtentsMeters.Y * halfExtentsMeters.Y + halfExtentsMeters.Z * halfExtentsMeters.Z) / 3f,
            massKg * (halfExtentsMeters.X * halfExtentsMeters.X + halfExtentsMeters.Z * halfExtentsMeters.Z) / 3f,
            massKg * (halfExtentsMeters.X * halfExtentsMeters.X + halfExtentsMeters.Y * halfExtentsMeters.Y) / 3f) * inertiaScale;
        var inverseInertia = new Vector3(1f / inertia.X, 1f / inertia.Y, 1f / inertia.Z);
        var h = halfExtentsMeters;
        var angular = new Vector3(
            AngularIntegral(inverseInertia.X, h.X, h.Y, h.Z) + AngularIntegral(inverseInertia.X, h.X, h.Z, h.Y),
            AngularIntegral(inverseInertia.Y, h.Y, h.X, h.Z) + AngularIntegral(inverseInertia.Y, h.Y, h.Z, h.X),
            AngularIntegral(inverseInertia.Z, h.Z, h.X, h.Y) + AngularIntegral(inverseInertia.Z, h.Z, h.Y, h.X));
        return new(linear, angular, linearCoefficient, angularCoefficient);
    }

    public static void Apply(ref Vector3 worldLinearVelocity, ref Vector3 worldAngularVelocity,
        Quaternion worldRotation, in SourceDragBasis basis, float stepSeconds)
    {
        if (stepSeconds <= 0f) throw new ArgumentOutOfRangeException(nameof(stepSeconds));
        var inverseRotation = Quaternion.Conjugate(Quaternion.Normalize(worldRotation));
        var localLinear = Vector3.Transform(worldLinearVelocity, inverseRotation);
        var localAngular = Vector3.Transform(worldAngularVelocity, inverseRotation);

        var linearDrag = -0.5f * basis.LinearCoefficient * (
            MathF.Abs(localLinear.X * basis.Linear.X) +
            MathF.Abs(localLinear.Y * basis.Linear.Y) +
            MathF.Abs(localLinear.Z * basis.Linear.Z)) * AirDensity * stepSeconds;
        localLinear *= 1f + MathF.Max(-1f, linearDrag);

        var angularDrag = -basis.AngularCoefficient * (
            MathF.Abs(localAngular.X * basis.Angular.X) +
            MathF.Abs(localAngular.Y * basis.Angular.Y) +
            MathF.Abs(localAngular.Z * basis.Angular.Z)) * AirDensity * stepSeconds;
        localAngular *= 1f + MathF.Max(-1f, angularDrag);

        worldLinearVelocity = Vector3.Transform(localLinear, worldRotation);
        worldAngularVelocity = Vector3.Transform(localAngular, worldRotation);
    }

    private static float AngularIntegral(float inverseInertia, float l, float w, float h) =>
        inverseInertia * ((1f / 3f) * w * w * l * l * l + 0.5f * w * w * w * w * l + l * w * w * h * h);
}
