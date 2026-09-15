using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

/// Optional Jolt CharacterBase probe. The SourceMovementMotor remains authoritative;
/// this object exposes Jolt contact/ground diagnostics for comparison and tuning.
public sealed class JoltCharacterProbe : IDisposable
{
    private readonly CharacterVirtual character;
    private readonly BoxShape shape;
    private readonly JoltPhysicsHost host;

    public CharacterVirtual Character => character;
    public GroundState Ground => character.GroundState switch
    {
        JoltPhysicsSharp.GroundState.OnGround => SourcePhysics.GroundState.Grounded,
        JoltPhysicsSharp.GroundState.NotSupported => SourcePhysics.GroundState.Airborne,
        _ => SourcePhysics.GroundState.Airborne
    };
    public Vector3 GroundNormal => character.GroundNormal;
    public Vector3 GroundVelocity => character.GroundVelocity;
    public uint GroundBodyId => character.GroundBodyId;

    public JoltCharacterProbe(JoltPhysicsHost host, Vector3 position, SourceMovementProfile profile)
    {
        this.host = host;
        shape = new BoxShape(profile.StandingHalfExtents, 0.001f);
        var settings = new CharacterVirtualSettings
        {
            Shape = shape,
            Up = Vector3.UnitY,
            MaxSlopeAngle = MathF.Acos(profile.StandableNormalZ),
            EnhancedInternalEdgeRemoval = true,
            CollisionTolerance = profile.CollisionEpsilon,
            MaxCollisionIterations = (uint)profile.MaxBumps,
            MaxNumHits = 256
        };
        RVector3 precisePosition = position;
        character = new CharacterVirtual(settings, precisePosition, Quaternion.Identity, 0, host.System);
    }

    public void Update(float dt, Vector3 desiredVelocity, ObjectLayer layer)
    {
        character.LinearVelocity = desiredVelocity;
        character.Update(dt, in layer, host.System, null!, null!);
    }

    public void Dispose() { character.Dispose(); shape.Dispose(); }
}
