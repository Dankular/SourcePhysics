using System.Numerics;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class MovementRecordingValidationTests
{
    [Fact]
    public void RecordingRejectsDuplicateOrReorderedTicks()
    {
        var recording = new MovementRecording();
        recording.Capture(2, default);
        recording.Capture(1, default);

        Assert.Throws<InvalidDataException>(() => recording.Validate());
        Assert.Throws<InvalidDataException>(() => recording.ToJson());
    }

    [Fact]
    public void RecordingRejectsNonFiniteStateAndStepValues()
    {
        var recording = new MovementRecording { FixedStepSeconds = 0f };
        Assert.Throws<InvalidDataException>(() => recording.Validate());

        recording = new MovementRecording();
        recording.Capture(0, new MovementState(new Vector3(float.NaN, 0, 0), Vector3.Zero,
            GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false));
        Assert.Throws<InvalidDataException>(() => recording.Validate());
    }
}
