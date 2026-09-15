using Stride.Core.Mathematics;
using Stride.Engine;
using System.Numerics;

namespace SourcePhysics;

/// Stride entity component that owns one Source movement motor and synchronizes its world transform.
public sealed class JoltCharacterMovement : SyncScript
{
    public StrideSourcePhysicsScript PhysicsSystem { get; set; } = null!;
    public SourceInput Command { get; set; }
    /// Optional fixed-tick command source. The callback is sampled exactly once per physics tick.
    /// Leaving it null preserves the directly assigned Command property for deterministic replays.
    public Func<SourceInput>? CommandProvider { get; set; }
    public bool RecordMovement { get; set; }
    public MovementRecording Recording { get; } = new();
    /// Optional entity override. If unset, the scene host profile is used for
    /// both the Source motor and its Jolt query shapes.
    public SourceMovementProfile? Profile { get; set; }
    public event Action<MovementState>? StateUpdated;
    /// Emitted after each authoritative fixed tick for presentation systems.
    public event Action<SourceCharacterPresentationFrame>? PresentationUpdated;

    public SourceMovementMotor Motor { get; private set; } = null!;
    public MovementState CurrentState => Motor.State;
    public float ViewHeightMeters => SourceUnits.ToMeters(Motor.State.ViewHeightSourceUnits);
    public static System.Numerics.Quaternion OrientationForCommand(SourceInput command) =>
        System.Numerics.Quaternion.CreateFromYawPitchRoll(command.ViewYawRadians, command.ViewPitchRadians, 0f);
    private JoltMovementQueries queries = null!;
    private SourceInput sampledCommand;

    public override void Start()
    {
        if (PhysicsSystem is null) throw new InvalidOperationException("Assign PhysicsSystem before starting JoltCharacterMovement.");
        PhysicsSystem.EnsureStarted();
        var movementProfile = Profile ?? PhysicsSystem.Host.MovementProfile;
        queries = new JoltMovementQueries(PhysicsSystem.Host, movementProfile);
        sampledCommand = Command;
        var position = ToNumerics(Entity.Transform.WorldMatrix.TranslationVector);
        Motor = new SourceMovementMotor(movementProfile, queries, position);
        PhysicsSystem.FixedTick += FixedTick;
    }

    public override void Update()
    {
        var yaw = OrientationForCommand(sampledCommand);
        StrideTransformSync.SetWorldPose(Entity.Transform, Motor.State.Position, yaw);
    }

    public override void Cancel()
    {
        if (PhysicsSystem is not null) PhysicsSystem.FixedTick -= FixedTick;
        queries?.Dispose();
    }

    private void FixedTick(int tick, float dt)
    {
        var command = CommandProvider?.Invoke() ?? Command;
        sampledCommand = command;
        Motor.Tick(command, dt);
        if (RecordMovement) Recording.Capture(tick, Motor.State, command);
        StateUpdated?.Invoke(Motor.State);
        var authoritativeOrientation = OrientationForCommand(command);
        var presentation = new SourceCharacterPresentationFrame(tick, command, Motor.State,
            Motor.State.Position, authoritativeOrientation, ViewHeightMeters);
        presentation.Validate();
        PresentationUpdated?.Invoke(presentation);
    }

    private float ProfileTick => PhysicsSystem.FixedStepSeconds;
    private static System.Numerics.Vector3 ToNumerics(Stride.Core.Mathematics.Vector3 value) => new(value.X, value.Y, value.Z);
    private static System.Numerics.Quaternion ToNumerics(Stride.Core.Mathematics.Quaternion value) => new(value.X, value.Y, value.Z, value.W);
}
