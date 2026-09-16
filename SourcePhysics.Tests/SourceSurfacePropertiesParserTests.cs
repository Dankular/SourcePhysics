using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class SourceSurfacePropertiesParserTests
{
    [Fact]
    public void ParserPreservesSourcePhysicsGameAudioAndInheritanceFields()
    {
        var registry = SourceSurfaceRegistry.FromSurfaceProperties(
            "surfaceproperties {\n" +
            "  \"default\" { \"friction\" \"0.8\" \"gamematerial\" \"C\" }\n" +
            "  \"metal\" { \"base\" \"default\" \"elasticity\" \"0.12\" \"density\" \"7800\" \"climbable\" \"1\" \"bulletimpact\" \"Metal.Bullet\" \"stepleft\" \"Metal.StepLeft\" \"audioHardMinVelocity\" \"4.5\" }\n" +
            "}");

        var metal = registry.Surfaces[1];
        Assert.Equal("metal", metal.Name);
        Assert.Equal(0.8f, metal.Friction);
        Assert.Equal(0.12f, metal.Elasticity);
        Assert.Equal(7800f, metal.DensityKgPerM3);
        Assert.True(metal.Climbable);
        Assert.Equal('C', metal.GameMaterial);
        Assert.Equal("Metal.Bullet", metal.BulletImpactSound);
        Assert.Equal("Metal.StepLeft", metal.StepLeftSound);
        Assert.Equal(4.5f, metal.HardVelocityThreshold);
        Assert.Equal(metal, registry.Get(1));
        Assert.Equal("default", registry.Fallback.Name);
    }

    [Fact]
    public void ParserRejectsUnknownKeysAndCircularBases()
    {
        Assert.Throws<InvalidDataException>(() => SourceSurfaceRegistry.FromSurfaceProperties(
            "surfaceproperties { \"bad\" { \"unknown\" \"1\" } }"));
        Assert.Throws<InvalidDataException>(() => SourceSurfaceRegistry.FromSurfaceProperties(
            "surfaceproperties { \"a\" { \"base\" \"b\" } \"b\" { \"base\" \"a\" } }"));
    }

    [Fact]
    public void ParserRejectsNestedSurfaceBlocksInsteadOfDiscardingThem()
    {
        Assert.Throws<InvalidDataException>(() => SourceSurfaceRegistry.FromSurfaceProperties(
            "surfaceproperties { \"metal\" { \"friction\" \"0.5\" \"unhandled\" { \"value\" \"1\" } } }"));
    }
}
