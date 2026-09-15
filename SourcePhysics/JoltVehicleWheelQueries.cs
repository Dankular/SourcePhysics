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
                if (queries.Cast(start, -Vector3.UnitY, lengthMeters, out var hit))
                {
                    var surface = host.Surfaces.Get(hit.SurfaceId);
                    contacts[wheelIndex] = new(true, hit.Position, hit.Normal,
                        hit.Fraction * lengthMeters, hit.SurfaceId, surface.Friction);
                }
                else
                {
                    contacts[wheelIndex] = default;
                }
            }
        }
        return contacts;
    }
}
