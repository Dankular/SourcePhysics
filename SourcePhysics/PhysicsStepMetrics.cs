namespace SourcePhysics;

public readonly record struct PhysicsStepMetrics(
    int CollisionSteps,
    int IntegrationSubSteps,
    float SimulatedSeconds,
    double ElapsedMilliseconds,
    uint ActiveBodyCount);
