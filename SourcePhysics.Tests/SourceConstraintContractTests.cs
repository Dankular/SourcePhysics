using System.Numerics;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class SourceConstraintContractTests
{
    [Fact]
    public void ConstraintProfilePreservesAnchorsAxesLimitsMotorAndBreakData()
    {
        var profile = new SourceConstraintProfile
        {
            Type = SourceConstraintType.Hinge,
            ConstraintGroup = 4,
            LocalAnchorA = new Vector3(1f, 2f, 3f),
            LocalAnchorB = new Vector3(-1f, 0f, 2f),
            LocalAxisA = Vector3.UnitX,
            LocalAxisB = Vector3.UnitX,
            MinimumLimit = -1f,
            MaximumLimit = 2f,
            EnableMotor = true,
            MotorTargetVelocity = 3f,
            MotorMaximumForce = 10f,
            BreakForce = 20f,
            BreakTorque = 30f,
            BreakStrength = 0.75f,
            BodyMassScaleA = 0.5f,
            BodyMassScaleB = 2f,
            AdditionalIterations = 2,
            MinimumErrorTicks = 15,
            ErrorToleranceSourceUnits = 3f,
            AxisAngularVelocity = 4f,
            AxisTorque = 8f
        };

        profile.Validate();
        Assert.Equal(SourceConstraintType.Hinge, profile.Type);
        Assert.Equal(4, profile.ConstraintGroup);
        Assert.Equal(2f, profile.MaximumLimit);
        Assert.Equal(0.75f, profile.BreakStrength);
        Assert.Equal(2, profile.AdditionalIterations);

        var runtimeGroup = profile.ToRuntimeGroupParameters();
        Assert.Equal(2, runtimeGroup.AdditionalIterations);
        Assert.Equal(15, runtimeGroup.MinimumErrorTicks);
        Assert.Equal(SourceUnits.ToMeters(3f), runtimeGroup.ErrorToleranceMeters);
    }

    [Fact]
    public void ConstraintProfileRejectsInvalidAxisLimitsAndBreakValues()
    {
        Assert.Throws<InvalidDataException>(() => new SourceConstraintProfile
        {
            LocalAxisA = Vector3.Zero
        }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceConstraintProfile
        {
            MinimumLimit = 2f, MaximumLimit = 1f
        }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceConstraintProfile
        {
            BreakTorque = -1f
        }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceConstraintProfile
        {
            BreakStrength = 2f
        }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceConstraintProfile
        {
            Type = SourceConstraintType.Ragdoll
        }.Validate());
        Assert.Throws<InvalidDataException>(() => new SourceConstraintProfile
        {
            RagdollAxes = new[] { new SourceConstraintAxisLimit { MinimumRotation = 1f, MaximumRotation = 0f } }
        }.Validate());
    }

    [Fact]
    public void ConstraintStateRejectsNegativeErrorsAndLoads()
    {
        var state = new SourceConstraintState(true, false, 0.01f, 0.02f, 4f, 5f);
        state.Validate();
        Assert.Throws<InvalidDataException>(() => new SourceConstraintState(
            true, false, -1f, 0f, 0f, 0f).Validate());
    }
}
