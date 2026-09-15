namespace SourcePhysics;

public sealed record SourcePhysicsPerformanceBudget
{
    public double MaximumStepMilliseconds { get; init; }
    public uint MaximumActiveBodies { get; init; }
    public int MaximumCollisionSteps { get; init; }
    public int MaximumIntegrationSubSteps { get; init; }

    public void Validate()
    {
        if (!double.IsFinite(MaximumStepMilliseconds) || MaximumStepMilliseconds <= 0d ||
            MaximumActiveBodies == 0 || MaximumCollisionSteps < 1 || MaximumIntegrationSubSteps < 1)
            throw new ArgumentOutOfRangeException(nameof(SourcePhysicsPerformanceBudget));
    }
}

public readonly record struct SourcePhysicsPerformanceSample(int Tick, PhysicsStepMetrics Metrics);

public sealed record SourcePhysicsPerformanceReport(
    int SampleCount,
    double MaximumElapsedMilliseconds,
    uint MaximumActiveBodies,
    IReadOnlyList<string> Violations)
{
    public bool Passes => Violations.Count == 0;
}

public sealed class SourcePhysicsPerformanceRecorder
{
    private readonly List<SourcePhysicsPerformanceSample> samples = new();
    public IReadOnlyList<SourcePhysicsPerformanceSample> Samples => samples;

    public void Capture(int tick, in PhysicsStepMetrics metrics)
    {
        if (tick < 0) throw new ArgumentOutOfRangeException(nameof(tick));
        if (!double.IsFinite(metrics.ElapsedMilliseconds) || metrics.ElapsedMilliseconds < 0d)
            throw new ArgumentOutOfRangeException(nameof(metrics));
        samples.Add(new(tick, metrics));
    }

    public SourcePhysicsPerformanceReport Evaluate(SourcePhysicsPerformanceBudget budget)
    {
        ArgumentNullException.ThrowIfNull(budget);
        budget.Validate();
        var violations = new List<string>();
        var maximumElapsed = 0d;
        var maximumBodies = 0u;
        foreach (var sample in samples)
        {
            maximumElapsed = Math.Max(maximumElapsed, sample.Metrics.ElapsedMilliseconds);
            maximumBodies = Math.Max(maximumBodies, sample.Metrics.ActiveBodyCount);
            if (sample.Metrics.ElapsedMilliseconds > budget.MaximumStepMilliseconds)
                violations.Add($"step-time:{sample.Tick}");
            if (sample.Metrics.ActiveBodyCount > budget.MaximumActiveBodies)
                violations.Add($"active-bodies:{sample.Tick}");
            if (sample.Metrics.CollisionSteps > budget.MaximumCollisionSteps)
                violations.Add($"collision-steps:{sample.Tick}");
            if (sample.Metrics.IntegrationSubSteps > budget.MaximumIntegrationSubSteps)
                violations.Add($"integration-substeps:{sample.Tick}");
        }
        return new(samples.Count, maximumElapsed, maximumBodies, violations);
    }
}
