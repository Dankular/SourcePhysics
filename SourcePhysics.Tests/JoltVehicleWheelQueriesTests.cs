using System.Numerics;
using JoltPhysicsSharp;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class JoltVehicleWheelQueriesTests
{
    [Fact]
    public void WheelTraceUsesSourceRaytraceOffsetsAndReturnsSurfaceContact()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(5f, 0.1f, 5f), new(0f, -0.1f, 0f), MotionType.Static,
            SourceObjectLayer.World);
        var vehicleBody = host.CreateBoxBody(new(0.5f, 0.5f, 0.5f), new(0f, 2f, 0f), MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { MassKg = 100f });
        var profile = new SourceVehicleProfile
        {
            AxleCount = 1,
            WheelsPerAxle = 2,
            Axles = new[]
            {
                new SourceVehicleAxleProfile
                {
                    RaytraceCenterOffsetSourceUnits = Vector3.Zero,
                    RaytraceOffsetSourceUnits = new(10f, 0f, 0f),
                    Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 10f }
                }
            }
        };
        var queries = new JoltVehicleWheelQueries(host, vehicleBody, profile);

        var contacts = queries.Trace(new[] { 120f, 120f });

        Assert.Equal(2, contacts.Count);
        Assert.All(contacts, contact =>
        {
            Assert.True(contact.InContact);
            Assert.Equal(Vector3.UnitY, contact.ContactNormal);
            Assert.Equal(0, contact.SurfaceId);
            Assert.True(contact.SuspensionLengthMeters > 0f);
        });
    }
}
