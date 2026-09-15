using System.Numerics;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class MovementModeCourseTests
{
    [Fact]
    public void ReferenceCourseExecutesEverySourceMovementDispatchMode()
    {
        var modes = Enum.GetValues<SourceMoveType>();
        foreach (var mode in modes)
        {
            var motor = new SourceMovementMotor(new SourceMovementProfile(), new OpenCourseQueries(), Vector3.Zero);
            var course = new SourceReferenceCourse
            {
                Name = $"mode-{mode}",
                Commands = new()
                {
                    new SourceCommand(0, new SourceInput(new Vector2(1f, 0f), Buttons.None,
                        MoveType: mode)),
                    new SourceCommand(1, new SourceInput(new Vector2(0f, 1f), Buttons.None,
                        ViewYawRadians: 0.25f, MoveType: mode))
                }
            };

            var recording = course.Execute(motor);
            recording.Validate();

            Assert.Equal(2, recording.Frames.Count);
            Assert.All(recording.Frames, frame =>
            {
                Assert.True(float.IsFinite(frame.Position.X));
                Assert.True(float.IsFinite(frame.Velocity.X));
                Assert.True(float.IsFinite(frame.Velocity.Y));
                Assert.True(float.IsFinite(frame.Velocity.Z));
            });
            Assert.Equal(mode, recording.Frames[^1].MoveType);
            Assert.Equal(Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.25f),
                recording.Frames[^1].Orientation);
        }
    }

    private sealed class OpenCourseQueries : ISourceMovementQueries
    {
        public MovementContact SweepPlayer(Vector3 start, Vector3 end, bool crouched) =>
            new(end, Vector3.UnitY, 1f, -1, 1f, 0f);
        public bool IsEmpty(Vector3 position, bool crouched) => true;
        public Vector3 GetBodyPointVelocity(int bodyId, Vector3 worldPoint) => Vector3.Zero;
        public void ApplyCharacterImpulse(int bodyId, Vector3 point, Vector3 impulse) { }
        public SourceWaterLevel GetWaterLevel(Vector3 position, bool crouched) => SourceWaterLevel.Dry;
        public bool TryLadder(Vector3 position, Vector3 direction, out Vector3 normal, out int bodyId)
        {
            normal = Vector3.UnitX;
            bodyId = 1;
            return true;
        }
    }
}
