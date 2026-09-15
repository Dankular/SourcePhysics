using System.Numerics;
using SourcePhysics;
using Stride.Engine;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class PresentationContractTests
{
    [Fact]
    public void PresentationFrameCarriesAuthoritativePhysicsAndViewState()
    {
        var state = new MovementState(new(1, 2, 3), new(4, 5, 6), GroundState.Grounded,
            Vector3.UnitY, 7, 0.8f, false, false, ViewHeightSourceUnits: 28f);
        var frame = new SourceCharacterPresentationFrame(12,
            new(new(0.25f, 1f), Buttons.Forward, 0.5f), state,
            state.Position, Quaternion.Identity, SourceUnits.ToMeters(28f));

        frame.Validate();

        Assert.Equal(12, frame.Tick);
        Assert.Equal(28f, state.ViewHeightSourceUnits);
        Assert.Equal(SourceUnits.ToMeters(28f), frame.ViewHeightMeters);
    }

    [Fact]
    public void PresentationFrameRejectsNonFiniteValues()
    {
        var frame = new SourceCharacterPresentationFrame(0, default,
            new MovementState(new(float.NaN, 0, 0), Vector3.Zero, GroundState.Airborne,
                Vector3.UnitY, -1, 1f, false, false), Vector3.Zero, Quaternion.Identity, 1f);

        Assert.Throws<InvalidDataException>(frame.Validate);
    }

    [Fact]
    public void CharacterOrientationUsesTheAuthoritativeCommandYaw()
    {
        var orientation = JoltCharacterMovement.OrientationForCommand(new SourceInput(default, Buttons.None, MathF.PI / 2f));
        var forward = Vector3.Transform(Vector3.UnitZ, orientation);

        Assert.Equal(Vector3.UnitX.X, forward.X, 5);
        Assert.Equal(Vector3.UnitX.Y, forward.Y, 5);
        Assert.Equal(Vector3.UnitX.Z, forward.Z, 5);
    }

    [Fact]
    public void CharacterOrientationIncludesAuthoritativeCommandPitch()
    {
        var orientation = JoltCharacterMovement.OrientationForCommand(
            new SourceInput(default, Buttons.None, ViewPitchRadians: MathF.PI / 2f));
        var forward = Vector3.Transform(Vector3.UnitZ, orientation);

        Assert.Equal(0f, forward.X, 5);
        Assert.Equal(-1f, forward.Y, 5);
        Assert.Equal(0f, forward.Z, 5);
    }

    [Fact]
    public void StrideWorldPoseSyncWritesParentRelativeTransform()
    {
        var parentEntity = new Entity();
        var childEntity = new Entity();
        var parent = parentEntity.Transform;
        parent.Position = new Stride.Core.Mathematics.Vector3(10f, 0f, 0f);
        var parentRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI * 0.5f);
        parent.Rotation = new Stride.Core.Mathematics.Quaternion(
            parentRotation.X, parentRotation.Y, parentRotation.Z, parentRotation.W);
        var child = childEntity.Transform;
        child.Parent = parent;
        var worldPosition = new Vector3(10f, 0f, 2f);
        var worldRotation = Quaternion.Identity;

        StrideTransformSync.SetWorldPose(child, worldPosition, worldRotation);
        child.UpdateWorldMatrix();

        var actualPosition = child.WorldMatrix.TranslationVector;
        Assert.Equal(worldPosition.X, actualPosition.X, 5);
        Assert.Equal(worldPosition.Y, actualPosition.Y, 5);
        Assert.Equal(worldPosition.Z, actualPosition.Z, 5);
    }
}
