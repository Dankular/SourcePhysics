using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

public readonly record struct SourceRigidBodyState(
    Vector3 Position,
    Quaternion Rotation,
    Vector3 LinearVelocity,
    Vector3 AngularVelocity,
    bool Active,
    MotionType JoltMotionType = MotionType.Dynamic,
    float GravityFactor = 1f,
    float Friction = 0.8f,
    float Restitution = 0.001f,
    SourceContents ContentsMask = SourceContents.Solid,
    ulong UserData = 0,
    SourceCollisionGroup CollisionGroup = SourceCollisionGroup.None,
    SourceSolidFlags SolidFlags = SourceSolidFlags.None,
    SourceCallbackFlags CallbackFlags = SourceCallbackFlags.Default);

public static class JoltRigidBodyState
{
    public static SourceRigidBodyState Capture(JoltPhysicsHost host, BodyID bodyId)
    {
        EnsureBody(host, bodyId);
        var transform = host.Bodies.GetRCenterOfMassTransform(bodyId);
        var matrix = new Matrix4x4(
            transform.M11, transform.M12, transform.M13, 0,
            transform.M21, transform.M22, transform.M23, 0,
            transform.M31, transform.M32, transform.M33, 0,
            0, 0, 0, 1);
        return new((Vector3)transform.Translation,
            Quaternion.CreateFromRotationMatrix(matrix), host.Bodies.GetLinearVelocity(bodyId),
            host.Bodies.GetAngularVelocity(bodyId), host.Bodies.IsActive(bodyId),
            host.Bodies.GetMotionType(bodyId), host.Bodies.GetGravityFactor(bodyId),
            host.Bodies.GetFriction(bodyId), host.Bodies.GetRestitution(bodyId), host.GetBodyContents(bodyId),
            host.Bodies.GetUserData(bodyId), host.GetBodyCollisionGroup(bodyId), host.GetBodySolidFlags(bodyId),
            host.GetBodyCallbackFlags(bodyId));
    }

    public static void Restore(JoltPhysicsHost host, BodyID bodyId, in SourceRigidBodyState state)
    {
        EnsureBody(host, bodyId);
        RVector3 position = state.Position;
        var rotation = state.Rotation;
        var linearVelocity = state.LinearVelocity;
        var angularVelocity = state.AngularVelocity;
        if (host.Bodies.GetMotionType(bodyId) != state.JoltMotionType)
            host.Bodies.SetMotionType(in bodyId, state.JoltMotionType,
                state.Active ? Activation.Activate : Activation.DontActivate);
        host.Bodies.SetGravityFactor(in bodyId, state.GravityFactor);
        host.Bodies.SetFriction(bodyId, state.Friction);
        host.Bodies.SetRestitution(bodyId, state.Restitution);
        host.Bodies.SetUserData(bodyId, state.UserData);
        host.SetBodyCollisionGroup(bodyId, state.CollisionGroup);
        host.SetBodySolidFlags(bodyId, state.SolidFlags);
        host.SetBodyCallbackFlags(bodyId, state.CallbackFlags);

        host.Bodies.SetRPositionAndRotation(in bodyId, in position, in rotation,
            state.Active ? Activation.Activate : Activation.DontActivate);
        host.Bodies.SetLinearVelocity(in bodyId, in linearVelocity);
        host.Bodies.SetAngularVelocity(in bodyId, in angularVelocity);
        if (state.Active) host.Bodies.ActivateBody(in bodyId);
        else host.Bodies.DeactivateBody(in bodyId);
    }

    private static void EnsureBody(JoltPhysicsHost host, BodyID bodyId)
    {
        if (!bodyId.IsValid || !host.Bodies.IsAdded(bodyId)) throw new ArgumentException("Body is not in the Jolt system.", nameof(bodyId));
    }
}
