using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class ReferenceCourseValidationTests
{
    [Fact]
    public void CourseRequiresStrictlyIncreasingValidatedTicks()
    {
        var course = new SourceReferenceCourse
        {
            Commands = new()
            {
                new SourceCommand(2, default),
                new SourceCommand(2, default)
            }
        };

        Assert.Throws<InvalidDataException>(() => course.Validate());
        Assert.Throws<InvalidDataException>(() => course.ToJson());
    }

    [Fact]
    public void CourseRejectsInvalidFixedStepAndCommandInput()
    {
        Assert.Throws<InvalidDataException>(() => new SourceReferenceCourse { FixedStepSeconds = 0f }.Validate());
        Assert.Throws<ArgumentOutOfRangeException>(() => new SourceReferenceCourse
        {
            Commands = new() { new SourceCommand(-1, default) }
        }.Validate());
    }

    [Fact]
    public void CourseRoundTripValidatesTheDeserializedStream()
    {
        var course = new SourceReferenceCourse
        {
            Name = "validation",
            Commands = new()
            {
                new SourceCommand(0, default),
                new SourceCommand(1, default)
            }
        };

        var restored = SourceReferenceCourse.FromJson(course.ToJson());

        Assert.Equal(new[] { 0, 1 }, restored.Commands.Select(command => command.Tick));
    }
}
