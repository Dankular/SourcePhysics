using System.Numerics;
using JoltPhysicsSharp;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class StaticMeshCookerTests
{
    [Fact]
    public void ZeroTolerancePreservesVertexAndTriangleOrder()
    {
        var vertices = new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0) };
        var triangles = new[] { Triangle(0, 1, 2) };
        var profile = new SourceStaticMeshProfile { TriangleSurfaceIds = new[] { 17 }, ContentsMask = SourceContents.Solid | SourceContents.Hitbox };

        var cooked = SourceStaticMeshCooker.Cook(vertices, triangles, profile);

        Assert.Equal(vertices, cooked.Vertices);
        Assert.Equal(triangles, cooked.Triangles);
        Assert.Equal(new[] { 17 }, cooked.Profile.TriangleSurfaceIds);
        Assert.Equal(profile.ContentsMask, cooked.Profile.ContentsMask);
    }

    [Fact]
    public void CookingPreservesAuthoredTriggerContentsAndCollisionState()
    {
        var vertices = new[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0) };
        var profile = new SourceStaticMeshProfile
        {
            ContentsMask = SourceContents.Water,
            CallbackFlags = SourceCallbackFlags.Default & ~SourceCallbackFlags.DoFluidSimulation,
            SolidFlags = SourceSolidFlags.NotSolid | SourceSolidFlags.Trigger,
            CollisionGroup = SourceCollisionGroup.DebrisTrigger,
            TriggerTouchesDebris = true,
            UserData = 91
        };

        var cooked = SourceStaticMeshCooker.Cook(vertices, new[] { Triangle(0, 1, 2) }, profile);

        Assert.Equal(profile.ContentsMask, cooked.Profile.ContentsMask);
        Assert.Equal(profile.CallbackFlags, cooked.Profile.CallbackFlags);
        Assert.Equal(profile.SolidFlags, cooked.Profile.SolidFlags);
        Assert.Equal(profile.CollisionGroup, cooked.Profile.CollisionGroup);
        Assert.Equal(profile.TriggerTouchesDebris, cooked.Profile.TriggerTouchesDebris);
        Assert.Equal(profile.UserData, cooked.Profile.UserData);
    }

    [Fact]
    public void WeldingIsDeterministicAndRemapsSurfaceIds()
    {
        var vertices = new[]
        {
            new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0),
            new Vector3(0.001f, 0, 0), new Vector3(1, 0.001f, 0), new Vector3(0, 1.001f, 0)
        };
        var triangles = new[] { Triangle(0, 1, 2), Triangle(3, 4, 5) };
        var profile = new SourceStaticMeshProfile { TriangleSurfaceIds = new[] { 4, 9 }, UserData = 55 };
        var options = new SourceMeshCookOptions { WeldToleranceSourceUnits = 0.01f };

        var first = SourceStaticMeshCooker.Cook(vertices, triangles, profile, options);
        var second = SourceStaticMeshCooker.Cook(vertices, triangles, profile, options);

        Assert.Equal(first.Vertices, second.Vertices);
        Assert.Equal(first.Triangles, second.Triangles);
        Assert.Equal(new[] { 4, 9 }, first.Profile.TriangleSurfaceIds);
        Assert.Equal((ulong)55, first.Profile.UserData);
    }

    [Fact]
    public void DegenerateRemovalIsExplicitAndRetainsRemainingSurfaceMapping()
    {
        var vertices = new[] { Vector3.Zero, Vector3.UnitX, Vector3.UnitY };
        var triangles = new[] { Triangle(0, 1, 1), Triangle(0, 1, 2) };
        var profile = new SourceStaticMeshProfile { TriangleSurfaceIds = new[] { 11, 22 } };

        var cooked = SourceStaticMeshCooker.Cook(vertices, triangles, profile,
            new SourceMeshCookOptions { RemoveDegenerateTriangles = true });

        Assert.Single(cooked.Triangles);
        Assert.Equal(new[] { 22 }, cooked.Profile.TriangleSurfaceIds);
    }

    [Fact]
    public void ProfileValidationRejectsInvalidAuthoredPhysicsValues()
    {
        Assert.Throws<InvalidDataException>(() => new SourceStaticMeshProfile { MaxTrianglesPerLeaf = 0 }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceStaticMeshProfile { Friction = -1f }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceRigidBodyProfile { MassKg = 0f }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceRigidBodyProfile { LinearDampingPerSecond = -1f }.Validate());
    }

    private static IndexedTriangle Triangle(uint a, uint b, uint c) => new(in a, in b, in c, 3, 77);
}
