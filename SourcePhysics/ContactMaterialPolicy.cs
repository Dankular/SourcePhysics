namespace SourcePhysics;

/// <summary>
/// Defines the rigid-body contact material combination law.
/// The default is only the Jolt-compatible fallback; a title-specific Source/IVP
/// law must be supplied when reference evidence identifies a different rule.
/// </summary>
public sealed class SourceContactMaterialPolicy
{
    public Func<SourceSurface, SourceSurface, float> CombineFriction { get; init; } =
        static (first, second) => MathF.Sqrt(first.Friction * second.Friction);

    public Func<SourceSurface, SourceSurface, float> CombineRestitution { get; init; } =
        static (first, second) => MathF.Max(first.Elasticity, second.Elasticity);

    public float GetFriction(SourceSurface first, SourceSurface second)
    {
        var value = CombineFriction(first, second);
        return ValidateAndClamp(value, nameof(CombineFriction));
    }

    public float GetRestitution(SourceSurface first, SourceSurface second)
    {
        var value = CombineRestitution(first, second);
        return ValidateAndClamp(value, nameof(CombineRestitution));
    }

    // physics_material.cpp clamps both IVP material-manager results to [0, 1]
    // before handing them to the solver. Keep title-specific combine functions
    // injectable, but preserve that Source post-combination contract.
    private static float ValidateAndClamp(float value, string policyName)
    {
        if (!float.IsFinite(value) || value < 0f)
            throw new InvalidOperationException($"{policyName} returned an invalid contact material value: {value}.");
        return MathF.Min(value, 1f);
    }
}
