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

    public void Validate()
    {
        var nonNegative = new[]
        {
            SpeculativeContactDistanceMeters, PenetrationSlopMeters,
            ManifoldToleranceMeters, MaximumPenetrationCorrectionMeters,
            MinimumRestitutionVelocityMetersPerSecond, SleepDelaySeconds,
            SleepPointVelocityMetersPerSecond
        };
        if (nonNegative.Any(value => !float.IsFinite(value) || value < 0f))
            throw new InvalidDataException("Jolt solver profile contains an invalid scalar.");
        if (!float.IsFinite(Baumgarte) || Baumgarte < 0f || Baumgarte > 1f)
            throw new InvalidDataException("Jolt Baumgarte must be in [0, 1].");
        if (VelocitySolverSteps == 0 || PositionSolverSteps == 0)
            throw new InvalidDataException("Jolt solver step counts must be positive.");
    }
}
