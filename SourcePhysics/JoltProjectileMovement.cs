using Stride.Engine;
using System.Numerics;

namespace SourcePhysics;

/// <summary>
/// Stride fixed-tick owner for a Source projectile motor.
/// The motor owns gravity, bounce and penetration state; Jolt only supplies the
/// continuous sweep query through <see cref="JoltProjectileQueries"/>.
/// </summary>
public sealed class JoltProjectileMovement : SyncScript
{
    public StrideSourcePhysicsScript PhysicsSystem { get; set; } = null!;
    public SourceProjectileProfile Profile { get; set; } = new();
    public Vector3 InitialVelocityMetersPerSecond { get; set; }
    public float SweepHalfExtentSourceUnits { get; set; } = 1f;
    public bool StartImmediately { get; set; } = true;
    public ProjectileRecording? Recording { get; set; }
    public int RecordingTick { get; set; }
    /// Title-specific penetration law. Arguments are the hit metadata and remaining power;
    /// the return value is the power consumed by the authored surface.
    public Func<ProjectileHit, float, float>? PenetrationCostByHit { get; set; }

    public event Action<ProjectileHit>? Hit;
    public SourceProjectileMotor Motor { get; private set; } = null!;
    public ProjectileState State => Motor.State;

    private JoltProjectileQueries queries = null!;

    public override void Start()
    {
        if (PhysicsSystem is null) throw new InvalidOperationException("Assign PhysicsSystem before starting JoltProjectileMovement.");
        PhysicsSystem.EnsureStarted();
        queries = new JoltProjectileQueries(PhysicsSystem.Host, SweepHalfExtentSourceUnits,
            penetrationCost: PenetrationCostByHit);
        var position = Entity.Transform.WorldMatrix.TranslationVector;
        Motor = new SourceProjectileMotor(Profile, queries,
            new Vector3(position.X, position.Y, position.Z), InitialVelocityMetersPerSecond);
        if (StartImmediately)
        {
            PhysicsSystem.FixedTick -= FixedTick;
            PhysicsSystem.FixedTick += FixedTick;
        }
    }

    public void Launch(Vector3 positionMeters, Vector3 velocityMetersPerSecond)
    {
        if (Motor is null) throw new InvalidOperationException("The projectile must be started before launching.");
        Motor = new SourceProjectileMotor(Profile, queries, positionMeters, velocityMetersPerSecond);
        PhysicsSystem.FixedTick -= FixedTick;
        PhysicsSystem.FixedTick += FixedTick;
    }

    public override void Update()
    {
        if (Motor is null) return;
        var position = Motor.State.Position;
        Entity.Transform.Position = new Stride.Core.Mathematics.Vector3(position.X, position.Y, position.Z);
    }

    public override void Cancel()
    {
        if (PhysicsSystem is not null) PhysicsSystem.FixedTick -= FixedTick;
        queries?.Dispose();
    }

    private void FixedTick(int tick, float dt)
    {
        var hit = Motor.Tick(dt);
        Recording?.Capture(tick + RecordingTick, Motor.State, hit);
        if (hit.HasValue) Hit?.Invoke(hit.Value);
        if (!Motor.State.Active) PhysicsSystem.FixedTick -= FixedTick;
    }
}
