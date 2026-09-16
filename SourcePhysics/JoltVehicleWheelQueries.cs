using System.Numerics;
using JoltPhysicsSharp;

namespace SourcePhysics;

/// Source raycast-vehicle wheel query bridge. Source constructs each trace at
/// raytraceCenterOffset +/- raytraceOffset in vehicle space; this class keeps
/// that ordering and requires the authored ray length for every wheel.
public sealed class JoltVehicleWheelQueries
{
    private readonly JoltPhysicsHost host;
    private readonly BodyID vehicleBody;
    private readonly SourceVehicleProfile profile;
    private readonly JoltHitscanQueries queries;

    public JoltVehicleWheelQueries(JoltPhysicsHost host, BodyID vehicleBody, SourceVehicleProfile profile)
    {
        ArgumentNullException.ThrowIfNull(host);
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        if (!vehicleBody.IsValid) throw new ArgumentException("Vehicle body must be valid.", nameof(vehicleBody));
        this.host = host;
        this.vehicleBody = vehicleBody;
        this.profile = profile;
        queries = new JoltHitscanQueries(host, contentsMask: SourceContents.MaskPlayerSolid,
            queryCollisionGroup: SourceCollisionGroup.Vehicle,
            ignoredBodyId: unchecked((int)vehicleBody.ID));
    }

    public IReadOnlyList<SourceVehicleWheelContact> Trace(IReadOnlyList<float> rayLengthsSourceUnits)
    {
        ArgumentNullException.ThrowIfNull(rayLengthsSourceUnits);
        if (rayLengthsSourceUnits.Count != profile.WheelCount)
            throw new ArgumentException("Wheel ray-length count must match the vehicle wheel count.", nameof(rayLengthsSourceUnits));
        var transform = (Matrix4x4)host.Bodies.GetRCenterOfMassTransform(vehicleBody);
        var vehicleDown = Vector3.TransformNormal(-Vector3.UnitY, transform);
        if (vehicleDown.LengthSquared() < 1e-12f)
            throw new InvalidOperationException("Vehicle transform produced an invalid wheel ray direction.");
        vehicleDown = Vector3.Normalize(vehicleDown);
        var contacts = new SourceVehicleWheelContact[profile.WheelCount];
        var wheelIndex = 0;
        foreach (var axle in profile.Axles)
        {
            for (var wheel = 0; wheel < profile.WheelsPerAxle; wheel++, wheelIndex++)
            {
                var rayLength = rayLengthsSourceUnits[wheelIndex];
                if (!float.IsFinite(rayLength) || rayLength <= 0f)
                    throw new ArgumentOutOfRangeException(nameof(rayLengthsSourceUnits), "Wheel ray lengths must be finite and positive.");
                var local = axle.RaytraceCenterOffsetSourceUnits;
                if (wheel == 1) local += axle.RaytraceOffsetSourceUnits;
                else if (profile.WheelsPerAxle == 2) local -= axle.RaytraceOffsetSourceUnits;
                var start = Vector3.Transform(SourceUnits.ToMeters(local), transform);
                var lengthMeters = SourceUnits.ToMeters(rayLength);
                if (queries.Cast(start, vehicleDown, lengthMeters, out var hit))
                {
                    var surface = host.Surfaces.Get(hit.SurfaceId);
                    var bodyId = hit.BodyId;
                    contacts[wheelIndex] = new(true, hit.Position, hit.Normal,
                        hit.Fraction * lengthMeters, hit.SurfaceId, surface.Friction, bodyId,
                        GetBodyPointVelocity(bodyId, hit.Position));
                }
                else
                {
                    contacts[wheelIndex] = default;
                }
            }
        }
        return contacts;
    }

    /// Returns the contacted body's world point velocity in Jolt metres per
    /// second. The vehicle controller can convert it explicitly to Source
    /// units and apply the title's relative-contact law.
    public Vector3 GetContactBodyPointVelocity(in SourceVehicleWheelContact contact)
    {
        if (!contact.InContact || contact.BodyId < 0)
            return Vector3.Zero;
        var bodyId = new BodyID(unchecked((uint)contact.BodyId));
        if (!host.Bodies.IsAdded(bodyId))
            return Vector3.Zero;
        return GetBodyPointVelocity(contact.BodyId, contact.ContactPointMeters);
    }

    private Vector3 GetBodyPointVelocity(int bodyIdValue, Vector3 contactPoint)
    {
        var bodyId = new BodyID(unchecked((uint)bodyIdValue));
        if (!host.Bodies.IsAdded(bodyId))
            return Vector3.Zero;
        host.Bodies.GetPointVelocity(in bodyId, in contactPoint, out var velocity);
        if (velocity.LengthSquared() < 1e-12f)
        {
            var angular = host.Bodies.GetAngularVelocity(bodyId);
            var center = (Vector3)host.Bodies.GetRCenterOfMassPosition(bodyId);
            velocity = host.Bodies.GetLinearVelocity(bodyId) +
                Vector3.Cross(angular, contactPoint - center);
        }
        return velocity;
    }
}
