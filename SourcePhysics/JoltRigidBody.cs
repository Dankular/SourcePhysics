using JoltPhysicsSharp;
using Stride.Core.Mathematics;
using Stride.Engine;
using NumericsVector3 = System.Numerics.Vector3;
using NumericsQuaternion = System.Numerics.Quaternion;
using NumericsMatrix4x4 = System.Numerics.Matrix4x4;

namespace SourcePhysics;

public sealed class JoltRigidBody : SyncScript
{
    public StrideSourcePhysicsScript PhysicsSystem { get; set; } = null!;
    public SourceObjectLayer Layer { get; set; } = SourceObjectLayer.Dynamic;
    public MotionType MotionType { get; set; } = MotionType.Dynamic;
    public NumericsVector3 HalfExtentMeters { get; set; } = NumericsVector3.One * 0.5f;
    public SourceRigidBodyProfile Profile { get; set; } = new();
    public int SurfaceId { get; set; }

    public BodyID BodyId { get; private set; }

    /// Converts Jolt's center-of-mass pose back to the authored Stride entity
    /// origin. OffsetCenterOfMassShape stores the authored offset in body
    /// space, so writing COM position directly would shift the entity.
    public static NumericsVector3 EntityOriginFromCenterOfMass(
        NumericsVector3 centerOfMassPosition, NumericsQuaternion bodyRotation,
        NumericsVector3 centerOfMassOffset) =>
        centerOfMassPosition - NumericsVector3.Transform(centerOfMassOffset, bodyRotation);

    public void AddForce(NumericsVector3 force) => PhysicsSystem.Host.AddForce(BodyId, force);
    public void AddTorque(NumericsVector3 torque) => PhysicsSystem.Host.AddTorque(BodyId, torque);
    public void AddImpulse(NumericsVector3 impulse) => PhysicsSystem.Host.AddImpulse(BodyId, impulse);
    public void AddImpulseAtPoint(NumericsVector3 impulse, NumericsVector3 worldPoint) =>
        PhysicsSystem.Host.AddImpulseAtPoint(BodyId, impulse, worldPoint);
    public void MoveKinematic(NumericsVector3 targetPosition, NumericsQuaternion targetRotation, float deltaSeconds) =>
        PhysicsSystem.Host.MoveKinematic(BodyId, targetPosition, targetRotation, deltaSeconds);
    public void Activate() => PhysicsSystem.Host.ActivateBody(BodyId);
    public void Deactivate() => PhysicsSystem.Host.DeactivateBody(BodyId);
    public void ResetSleepTimer() => PhysicsSystem.Host.ResetSleepTimer(BodyId);
    public void SetMotionType(MotionType motionType, bool activate = true) =>
        PhysicsSystem.Host.SetMotionType(BodyId, motionType, activate);
    public void SetLinearVelocity(NumericsVector3 velocity) => PhysicsSystem.Host.SetLinearVelocity(BodyId, velocity);
    public void SetAngularVelocity(NumericsVector3 velocity) => PhysicsSystem.Host.SetAngularVelocity(BodyId, velocity);
    public void SetGravityFactor(float gravityFactor) => PhysicsSystem.Host.SetGravityFactor(BodyId, gravityFactor);
    public void SetFriction(float friction) => PhysicsSystem.Host.SetFriction(BodyId, friction);
    public void SetRestitution(float restitution) => PhysicsSystem.Host.SetRestitution(BodyId, restitution);
    public void SetCollisionGroup(SourceCollisionGroup group) => PhysicsSystem.Host.SetBodyCollisionGroup(BodyId, group);
    public SourceCollisionGroup GetCollisionGroup() => PhysicsSystem.Host.GetBodyCollisionGroup(BodyId);
    public void SetSolidFlags(SourceSolidFlags flags) => PhysicsSystem.Host.SetBodySolidFlags(BodyId, flags);
    public SourceSolidFlags GetSolidFlags() => PhysicsSystem.Host.GetBodySolidFlags(BodyId);
    public bool ApplyBuoyancyImpulse(NumericsVector3 surfacePosition, NumericsVector3 surfaceNormal,
        float buoyancy, float linearDrag, float angularDrag, NumericsVector3 fluidVelocity, float deltaSeconds) =>
        PhysicsSystem.Host.ApplyBuoyancyImpulse(BodyId, surfacePosition, surfaceNormal,
            buoyancy, linearDrag, angularDrag, fluidVelocity, deltaSeconds);
    public void ApplySourceFluidTouchDamping(NumericsVector3 surfaceNormal, float fluidDensity,
        float linearDrag, float angularDrag, float simulationTimestep) =>
        PhysicsSystem.Host.ApplySourceFluidTouchDamping(BodyId, surfaceNormal, fluidDensity,
            linearDrag, angularDrag, simulationTimestep);

    public override void Start()
    {
        if (PhysicsSystem is null) throw new InvalidOperationException("Assign PhysicsSystem before starting JoltRigidBody.");
        PhysicsSystem.EnsureStarted();
        var position = Entity.Transform.WorldMatrix.TranslationVector;
        var scale = Entity.Transform.Scale;
        var halfExtents = new NumericsVector3(HalfExtentMeters.X * scale.X, HalfExtentMeters.Y * scale.Y, HalfExtentMeters.Z * scale.Z);
        var strideRotation = Entity.Transform.Rotation;
        var rotation = new NumericsQuaternion(strideRotation.X, strideRotation.Y, strideRotation.Z, strideRotation.W);
        BodyId = PhysicsSystem.Host.CreateBoxBody(halfExtents, new(position.X, position.Y, position.Z), rotation,
            MotionType, Layer, Profile, SurfaceId);
    }

    public override void Update()
    {
        if (!BodyId.IsValid) return;
        var transform = PhysicsSystem.Host.Bodies.GetRCenterOfMassTransform(BodyId);
        var position = (NumericsVector3)transform.Translation;
        var rotation = NumericsQuaternion.CreateFromRotationMatrix(new NumericsMatrix4x4(
            transform.M11, transform.M12, transform.M13, 0,
            transform.M21, transform.M22, transform.M23, 0,
            transform.M31, transform.M32, transform.M33, 0,
            0, 0, 0, 1));
        var entityPosition = EntityOriginFromCenterOfMass(position, rotation, Profile.CenterOfMassOffsetMeters);
        StrideTransformSync.SetWorldPose(Entity.Transform, entityPosition, rotation);
    }

    public override void Cancel()
    {
        if (PhysicsSystem is not null) PhysicsSystem.Host.DestroyBody(BodyId);
        BodyId = BodyID.Invalid;
    }
}
