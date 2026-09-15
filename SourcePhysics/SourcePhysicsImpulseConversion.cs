using System.Numerics;

namespace SourcePhysics;

/// Source vphysics_interface.h defines ApplyForceOffset's vector as an impulse
/// in kg-in/s. The port's vectors are already expressed in its explicit Y-up
/// boundary, so only the exact inch-to-meter scale is required for Jolt.
public static class SourcePhysicsImpulseConversion
{
    public static Vector3 ToJolt(Vector3 sourceImpulseKgInPerSecond) =>
        SourceUnits.ToMeters(sourceImpulseKgInPerSecond);
}
