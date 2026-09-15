namespace SourcePhysics;

public sealed record JoltSolverProfile
{
    public float Baumgarte { get; init; } = 0.2f;
    public float SpeculativeContactDistanceMeters { get; init; } = 0.02f;
    public float PenetrationSlopMeters { get; init; } = 0.02f;
    public float ManifoldToleranceMeters { get; init; } = 0.001f;
    public float MaximumPenetrationCorrectionMeters { get; init; } = 0.2f;
    public uint VelocitySolverSteps { get; init; } = 10;
    public uint PositionSolverSteps { get; init; } = 2;
    public float MinimumRestitutionVelocityMetersPerSecond { get; init; } = 1f;
    public float SleepDelaySeconds { get; init; } = 0.5f;
    public float SleepPointVelocityMetersPerSecond { get; init; } = 0.03f;
    public bool DeterministicSimulation { get; init; } = true;
    public bool ConstraintWarmStart { get; init; } = true;
    public bool EnhancedInternalEdgeRemoval { get; init; } = true;
}
