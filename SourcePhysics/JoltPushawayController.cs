using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

/// Applies the exact Source obstacle-pushaway impulse through the Source-to-Jolt
/// unit boundary. The override remains injectable for title-specific coordinate
/// or asset boundaries.
public sealed class JoltPushawayController : IDisposable
{
    private readonly JoltPhysicsHost host;
    private readonly SourcePushawayProfile profile;
    private readonly Func<Vector3, Vector3> sourceForceToJoltImpulse;
    private readonly Func<IReadOnlyList<BodyID>> propBodies;
    private readonly Func<BodyID, bool> isRotatingDoor;
    private readonly Action<float> preStep;
    private Vector3 playerCenter;
    private float playerSpeedSourceUnitsPerSecond;
    private bool playerActive;
    private bool registered;

    public JoltPushawayController(JoltPhysicsHost host, SourcePushawayProfile profile,
        Func<Vector3, Vector3>? sourceForceToJoltImpulse,
        Func<IReadOnlyList<BodyID>> propBodies,
        Func<BodyID, bool>? isRotatingDoor = null)
    {
        this.host = host ?? throw new ArgumentNullException(nameof(host));
        this.profile = profile ?? throw new ArgumentNullException(nameof(profile));
        this.profile.Validate();
        this.sourceForceToJoltImpulse = sourceForceToJoltImpulse ?? SourcePhysicsImpulseConversion.ToJolt;
        this.propBodies = propBodies ?? throw new ArgumentNullException(nameof(propBodies));
        this.isRotatingDoor = isRotatingDoor ?? (static _ => false);
        preStep = ApplyAtFixedStep;
    }

    public void SetPlayerState(Vector3 center, float speedSourceUnitsPerSecond, bool active = true)
    {
        if (!IsFinite(center) || !float.IsFinite(speedSourceUnitsPerSecond) || speedSourceUnitsPerSecond < 0f)
            throw new ArgumentOutOfRangeException(nameof(speedSourceUnitsPerSecond));
        playerCenter = center;
        playerSpeedSourceUnitsPerSecond = speedSourceUnitsPerSecond;
        playerActive = active;
    }

    public void RegisterFixedStep()
    {
        if (registered) return;
        host.RegisterPreStepController(preStep);
        registered = true;
    }

    public void UnregisterFixedStep()
    {
        if (!registered) return;
        host.UnregisterPreStepController(preStep);
        registered = false;
    }

    private void ApplyAtFixedStep(float deltaSeconds)
    {
        if (!playerActive || !float.IsFinite(deltaSeconds) || deltaSeconds <= 0f) return;
        foreach (var bodyId in propBodies())
        {
            if (!host.TryGetBodyMass(bodyId, out var mass)) continue;
            var propCenter = (Vector3)host.Bodies.GetRCenterOfMassPosition(bodyId);
            var sourceForce = SourcePushawayPolicy.ComputeObstacleForce(profile, propCenter, playerCenter,
                playerSpeedSourceUnitsPerSecond, mass, multiplayerSolid: true, isRotatingDoor(bodyId));
            if (sourceForce.LengthSquared() < 1e-12f) continue;
            var impulse = sourceForceToJoltImpulse(sourceForce);
            if (!IsFinite(impulse)) throw new InvalidOperationException("Source force conversion returned a non-finite Jolt impulse.");
            host.AddImpulseAtPoint(bodyId, impulse, playerCenter);
        }
    }

    public void Dispose() => UnregisterFixedStep();

    private static bool IsFinite(Vector3 value) => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);
}
