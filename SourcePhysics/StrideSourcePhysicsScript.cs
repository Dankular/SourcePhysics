using Stride.Engine;

namespace SourcePhysics;

/// <summary>Scene-level Stride owner for the authoritative fixed physics tick.</summary>
public sealed class StrideSourcePhysicsScript : SyncScript
{
    public float FixedStepSeconds { get; set; } = 1f / 66f;
    public int CollisionSteps { get; set; } = 1;
    public int IntegrationSubSteps { get; set; } = 1;
    public SourceMovementProfile MovementProfile { get; set; } = new();
    public JoltSolverProfile SolverProfile { get; set; } = new();
    public bool RecordPhysics { get; set; }
    public SourcePhysicsPerformanceBudget? PerformanceBudget { get; set; }

    public JoltPhysicsHost Host { get; private set; } = null!;
    public int SimulationTick => clock?.Tick ?? 0;
    public SourcePhysicsRecording? PhysicsRecording { get; private set; }
    public SourcePhysicsPerformanceRecorder? PerformanceRecording { get; private set; }
    public event Action<int, float>? FixedTick;
    /// Emitted after Jolt integration and contact flushing for the tick.
    public event Action<int, float>? FixedTickCompleted;
    private SourceFixedStepClock? clock;
    private bool started;
    private readonly List<Action<float>> fixedTickParticipants = new();

    public override void Start() => EnsureStarted();

    /// <summary>Initializes the host once, allowing dependent Stride scripts to be started in any scene order.</summary>
    public void EnsureStarted()
    {
        if (started) return;
        if (!float.IsFinite(FixedStepSeconds) || FixedStepSeconds <= 0f)
            throw new InvalidOperationException("FixedStepSeconds must be finite and positive.");
        if (CollisionSteps < 1) throw new InvalidOperationException("CollisionSteps must be positive.");
        if (IntegrationSubSteps < 1) throw new InvalidOperationException("IntegrationSubSteps must be positive.");
        Host = new JoltPhysicsHost(MovementProfile, FixedStepSeconds, SolverProfile);
        Host.Initialize();
        clock = new SourceFixedStepClock(FixedStepSeconds);
        if (RecordPhysics) PhysicsRecording = new SourcePhysicsRecording(Host);
        if (PerformanceBudget is not null)
        {
            PerformanceBudget.Validate();
            PerformanceRecording = new SourcePhysicsPerformanceRecorder();
        }
        started = true;
    }

    public void EnablePhysicsRecording()
    {
        EnsureStarted();
        PhysicsRecording ??= new SourcePhysicsRecording(Host);
    }

    public override void Cancel()
    {
        if (!started) return;
        PhysicsRecording?.Dispose();
        PhysicsRecording = null;
        PerformanceRecording = null;
        Host.Dispose();
        clock = null;
        // Do not retain callbacks that may capture components from the
        // disposed scene/world. A subsequent EnsureStarted begins a fresh
        // authoritative lifecycle and requires fresh registrations.
        fixedTickParticipants.Clear();
        FixedTick = null;
        FixedTickCompleted = null;
        started = false;
    }

    public override void Update() => Advance((float)Game.UpdateTime.WarpElapsed.TotalSeconds);

    public void RegisterFixedTick(Action<float> callback)
    {
        ArgumentNullException.ThrowIfNull(callback);
        if (!fixedTickParticipants.Contains(callback)) fixedTickParticipants.Add(callback);
    }

    public void UnregisterFixedTick(Action<float> callback) => fixedTickParticipants.Remove(callback);

    public void Advance(float elapsedSeconds)
    {
        var fixedClock = clock ?? throw new InvalidOperationException("Start the Stride physics script before advancing it.");
        fixedClock.Advance(elapsedSeconds, (tick, dt) =>
        {
            FixedTick?.Invoke(tick, dt);
            foreach (var participant in fixedTickParticipants.ToArray()) participant(dt);
            Host.Step(CollisionSteps, IntegrationSubSteps);
            PhysicsRecording?.Capture(tick);
            if (PerformanceRecording is not null)
                PerformanceRecording.Capture(tick, Host.LastStepMetrics);
            FixedTickCompleted?.Invoke(tick, dt);
        });
    }
}
