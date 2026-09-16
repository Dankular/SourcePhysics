using Stride.Engine;

namespace SourcePhysics;

/// Stride integration boundary for post-simulation vehicle contact capture.
/// Vehicle forces and operating-state policy remain title-owned callbacks;
/// this component only samples Jolt contacts and records the observation.
public sealed class JoltVehicleContactRecorder : SyncScript
{
    public StrideSourcePhysicsScript PhysicsSystem { get; set; } = null!;
    public JoltRigidBody? VehicleBody { get; set; }
    public SourceVehicleProfile Profile { get; set; } = new();
    public float[] RayLengthsSourceUnits { get; set; } = Array.Empty<float>();
    public Func<SourceVehicleControl>? ControlProvider { get; set; }
    public Func<SourceVehicleOperatingState>? OperatingStateProvider { get; set; }
    public bool RecordVehicle { get; set; } = true;
    public SourceVehicleRecording Recording { get; } = new();
    public IReadOnlyList<SourceVehicleWheelContact> CurrentContacts { get; private set; } = Array.Empty<SourceVehicleWheelContact>();
    public IReadOnlyList<SourceVehicleWheelSkidSample> CurrentSkidSamples { get; private set; } = Array.Empty<SourceVehicleWheelSkidSample>();

    private JoltVehicleWheelQueries? queries;

    public override void Start()
    {
        if (PhysicsSystem is null) throw new InvalidOperationException("Assign PhysicsSystem before starting JoltVehicleContactRecorder.");
        PhysicsSystem.EnsureStarted();
        Profile.Validate();
        if (RayLengthsSourceUnits.Length != Profile.WheelCount)
            throw new InvalidDataException("Vehicle ray-length count must match the vehicle wheel count.");
        if (RayLengthsSourceUnits.Any(length => !float.IsFinite(length) || length <= 0f))
            throw new InvalidDataException("Vehicle ray lengths must be finite and positive.");
        if (RecordVehicle && (ControlProvider is null || OperatingStateProvider is null))
            throw new InvalidOperationException("Vehicle recording requires authored control and operating-state providers.");
        Recording.FixedStepSeconds = PhysicsSystem.FixedStepSeconds;
        PhysicsSystem.FixedTickCompleted += OnFixedTickCompleted;
    }

    public override void Update() { }

    public override void Cancel()
    {
        if (PhysicsSystem is not null) PhysicsSystem.FixedTickCompleted -= OnFixedTickCompleted;
        queries = null;
        CurrentContacts = Array.Empty<SourceVehicleWheelContact>();
        CurrentSkidSamples = Array.Empty<SourceVehicleWheelSkidSample>();
    }

    private void OnFixedTickCompleted(int tick, float _)
    {
        if (VehicleBody is null || !VehicleBody.BodyId.IsValid)
            return;
        queries ??= new JoltVehicleWheelQueries(PhysicsSystem.Host, VehicleBody.BodyId, Profile);
        CurrentContacts = queries.Trace(RayLengthsSourceUnits);
        CurrentSkidSamples = queries.BuildSkidSamples(CurrentContacts);
        if (!RecordVehicle) return;
        Recording.Capture(tick, ControlProvider!(), OperatingStateProvider!(), CurrentContacts, CurrentSkidSamples);
    }
}
