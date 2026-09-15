namespace SourcePhysics;

/// Deterministic accumulator used by the Stride owner and by replay tests.
public sealed class SourceFixedStepClock
{
    public float FixedStepSeconds { get; }
    public int Tick { get; private set; }
    public double RemainderSeconds => accumulator;
    private double accumulator;

    public SourceFixedStepClock(float fixedStepSeconds = 1f / 66f)
    {
        if (!float.IsFinite(fixedStepSeconds) || fixedStepSeconds <= 0f)
            throw new ArgumentOutOfRangeException(nameof(fixedStepSeconds));
        FixedStepSeconds = fixedStepSeconds;
    }

    public int Advance(float elapsedSeconds, Action<int, float> fixedTick)
    {
        ArgumentNullException.ThrowIfNull(fixedTick);
        if (!float.IsFinite(elapsedSeconds) || elapsedSeconds < 0f)
            throw new ArgumentOutOfRangeException(nameof(elapsedSeconds));
        accumulator += elapsedSeconds;
        var steps = 0;
        // The inputs are floats but the accumulator is double. Allow the
        // boundary comparison to absorb the representational error that can
        // otherwise turn two exact source-frame portions into a missed tick.
        var boundaryTolerance = Math.Max(1e-7, FixedStepSeconds * 1e-6);
        while (accumulator + boundaryTolerance >= FixedStepSeconds)
        {
            fixedTick(Tick++, FixedStepSeconds);
            accumulator -= FixedStepSeconds;
            if (accumulator < 0d && accumulator > -boundaryTolerance)
                accumulator = 0d;
            steps++;
        }
        return steps;
    }
}
