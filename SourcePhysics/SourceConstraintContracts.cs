using System.Numerics;

namespace SourcePhysics;

public enum SourceConstraintType
{
    Fixed,
    Hinge,
    Slider,
    BallSocket,
    Pulley,
    Length,
    Ragdoll
}

public sealed record SourceConstraintAxisLimit
{
    public float MinimumRotation { get; init; }
    public float MaximumRotation { get; init; }
    public float AngularVelocity { get; init; }
    public float Torque { get; init; }

    public void Validate()
    {
        if (!float.IsFinite(MinimumRotation) || !float.IsFinite(MaximumRotation) ||
            !float.IsFinite(AngularVelocity) || !float.IsFinite(Torque) ||
            MinimumRotation > MaximumRotation || Torque < 0f)
            throw new InvalidDataException("Constraint axis limit is invalid.");
    }
}

/// Source/Havok constraint asset data. This is deliberately a data contract;
/// runtime creation is kept separate until the installed Jolt binding's
/// add/remove/dispose lifecycle is proven safe.
public sealed record SourceConstraintProfile
{
    public SourceConstraintType Type { get; init; }
    public int ConstraintGroup { get; init; }
    public bool AllowCollision { get; init; }
    public Vector3 LocalAnchorA { get; init; }
    public Vector3 LocalAnchorB { get; init; }
    public Vector3 LocalAxisA { get; init; } = Vector3.UnitY;
    public Vector3 LocalAxisB { get; init; } = Vector3.UnitY;
    public float MinimumLimit { get; init; }
    public float MaximumLimit { get; init; }
    public bool EnableMotor { get; init; }
    public float MotorTargetVelocity { get; init; }
    public float MotorMaximumForce { get; init; }
    public float BreakForce { get; init; }
    public float BreakTorque { get; init; }
    /// Source constraint_breakableparams_t fields.
    public float BreakStrength { get; init; } = 1f;
    public float BodyMassScaleA { get; init; } = 1f;
    public float BodyMassScaleB { get; init; } = 1f;
    public bool IsActive { get; init; } = true;
    /// Source constraint_groupparams_t fields.
    public int AdditionalIterations { get; init; }
    public int MinimumErrorTicks { get; init; } = 15;
    public float ErrorToleranceSourceUnits { get; init; } = 3f;
    /// Hinge/slider axis policy, and the three Source ragdoll axes.
    public float AxisAngularVelocity { get; init; }
    public float AxisTorque { get; init; }
    public bool OnlyAngularLimits { get; init; }
    public bool UseClockwiseRotations { get; init; }
    public IReadOnlyList<SourceConstraintAxisLimit> RagdollAxes { get; init; } = Array.Empty<SourceConstraintAxisLimit>();

    public void Validate()
    {
        if (!Enum.IsDefined(Type))
            throw new InvalidDataException("Constraint type is not defined by the Source contract.");
        ValidateVector(LocalAnchorA, "constraint anchor A");
        ValidateVector(LocalAnchorB, "constraint anchor B");
        ValidateVector(LocalAxisA, "constraint axis A");
        ValidateVector(LocalAxisB, "constraint axis B");
        if (LocalAxisA.LengthSquared() <= 0f || LocalAxisB.LengthSquared() <= 0f)
            throw new InvalidDataException("Constraint axes must be non-zero.");
        if (!float.IsFinite(MinimumLimit) || !float.IsFinite(MaximumLimit) || MinimumLimit > MaximumLimit)
            throw new InvalidDataException("Constraint limits must be finite and ordered.");
        if (!float.IsFinite(MotorTargetVelocity) || !float.IsFinite(MotorMaximumForce) ||
            !float.IsFinite(BreakForce) || !float.IsFinite(BreakTorque))
            throw new InvalidDataException("Constraint motor and break values must be finite.");
        if (MotorMaximumForce < 0f || BreakForce < 0f || BreakTorque < 0f)
            throw new InvalidDataException("Constraint motor and break values cannot be negative.");
        if (!float.IsFinite(BreakStrength) || BreakStrength < 0f || BreakStrength > 1f ||
            !float.IsFinite(BodyMassScaleA) || !float.IsFinite(BodyMassScaleB) ||
            BodyMassScaleA < 0f || BodyMassScaleB < 0f)
            throw new InvalidDataException("Constraint breakable parameters are invalid.");
        if (AdditionalIterations < 0 || MinimumErrorTicks < 0 ||
            !float.IsFinite(ErrorToleranceSourceUnits) || ErrorToleranceSourceUnits < 0f ||
            !float.IsFinite(AxisAngularVelocity) || !float.IsFinite(AxisTorque) || AxisTorque < 0f)
            throw new InvalidDataException("Constraint group or axis parameters are invalid.");
        if (Type == SourceConstraintType.Ragdoll && RagdollAxes.Count != 3)
            throw new InvalidDataException("Source ragdoll constraints require three axis limits.");
        foreach (var axis in RagdollAxes)
        {
            ArgumentNullException.ThrowIfNull(axis);
            axis.Validate();
        }
    }

    private static void ValidateVector(Vector3 value, string name)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new InvalidDataException($"{name} must be finite.");
    }
}

public readonly record struct SourceConstraintState(
    bool Active,
    bool Broken,
    float LinearErrorMeters,
    float AngularErrorRadians,
    float AppliedForce,
    float AppliedTorque)
{
    public void Validate()
    {
        if (!float.IsFinite(LinearErrorMeters) || !float.IsFinite(AngularErrorRadians) ||
            !float.IsFinite(AppliedForce) || !float.IsFinite(AppliedTorque) ||
            LinearErrorMeters < 0f || AngularErrorRadians < 0f ||
            AppliedForce < 0f || AppliedTorque < 0f)
            throw new InvalidDataException("Constraint state values must be finite and non-negative.");
    }
}
