namespace SourcePhysics;

public sealed record SourceRigidBodyProfile
{
    public float MassKg { get; init; } = 1f;
    public System.Numerics.Vector3 CenterOfMassOffsetMeters { get; init; }
    public float InertiaScale { get; init; } = 1f;
    public float LinearDampingPerSecond { get; init; } = 0.05f;
    public float AngularDampingPerSecond { get; init; } = 0.05f;
    public bool EnableDrag { get; init; }
    public float DragCoefficientPerSecond { get; init; }
    public float RollingDragCoefficientPerSecond { get; init; }
    public float GravityFactor { get; init; } = 1f;
    public float MaxLinearVelocityMetersPerSecond { get; init; } = 88.9f;
    public float MaxAngularVelocityRadiansPerSecond { get; init; } = 47.1239f;
    public bool ContinuousCollision { get; init; }
    public bool AllowSleep { get; init; } = true;
    public float Friction { get; init; } = 0.8f;
    public float Restitution { get; init; } = 0.001f;
    /// Source CPhysicsObject::GetBuoyancyRatio.  This scales the liquid
    /// medium density for this object's buoyancy solve; it does not alter
    /// fluid-touch damping.
    public float BuoyancyRatio { get; init; } = 1f;
    /// Source CALLBACK_DO_FLUID_SIMULATION.  When disabled, the object is
    /// still eligible for touch bookkeeping, but receives no buoyant force.
    public bool FluidSimulationEnabled { get; init; } = true;
    public SourceContents ContentsMask { get; init; } = SourceContents.Solid;
    /// Source collision-property solid flags. TriggerTouchDebris is retained below as a
    /// convenient compatibility alias for authored trigger profiles.
    public SourceSolidFlags SolidFlags { get; init; }
    /// Optional Source collision group. None leaves the host's object-layer policy in control.
    public SourceCollisionGroup CollisionGroup { get; init; }
    /// Source FSOLID_TRIGGER_TOUCH_DEBRIS: only authored trigger bodies with this flag
    /// receive trigger events from debris bodies.
    public bool TriggerTouchesDebris { get; init; }
    /// Source VPhysics game-data/user-data identity carried by the body.
    public ulong UserData { get; init; }

    public void Validate()
    {
        const SourceSolidFlags supportedSolidFlags = SourceSolidFlags.NotSolid |
            SourceSolidFlags.Trigger | SourceSolidFlags.TriggerTouchDebris;
        if (!Enum.IsDefined(CollisionGroup) || (SolidFlags & ~supportedSolidFlags) != 0)
            throw new InvalidDataException("Rigid-body profile contains an unknown Source collision state.");
        var scalars = new[]
        {
            MassKg, InertiaScale, LinearDampingPerSecond, AngularDampingPerSecond,
            DragCoefficientPerSecond, RollingDragCoefficientPerSecond, GravityFactor,
            MaxLinearVelocityMetersPerSecond, MaxAngularVelocityRadiansPerSecond,
            Friction, Restitution, BuoyancyRatio
        };
        if (scalars.Any(value => !float.IsFinite(value)))
            throw new InvalidDataException("Rigid-body profile contains a non-finite value.");
        if (MassKg <= 0f || InertiaScale <= 0f || LinearDampingPerSecond < 0f ||
            AngularDampingPerSecond < 0f || DragCoefficientPerSecond < 0f ||
            RollingDragCoefficientPerSecond < 0f || GravityFactor < 0f ||
            MaxLinearVelocityMetersPerSecond <= 0f || MaxAngularVelocityRadiansPerSecond <= 0f ||
            Friction < 0f || Restitution < 0f)
            throw new InvalidDataException("Rigid-body profile contains an invalid negative or zero-required value.");
        if (BuoyancyRatio < 0f)
            throw new InvalidDataException("Rigid-body buoyancy ratio cannot be negative.");
        if (!float.IsFinite(CenterOfMassOffsetMeters.X) || !float.IsFinite(CenterOfMassOffsetMeters.Y) || !float.IsFinite(CenterOfMassOffsetMeters.Z))
            throw new InvalidDataException("Rigid-body center-of-mass offset must be finite.");
    }
}
