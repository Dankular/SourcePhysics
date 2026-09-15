using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

public sealed class SourceReferenceCourse
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    public string Name { get; init; } = "unnamed";
    public float FixedStepSeconds { get; init; } = 1f / 66f;
    public List<SourceCommand> Commands { get; init; } = new();

    public void Validate()
    {
        if (!float.IsFinite(FixedStepSeconds) || FixedStepSeconds <= 0f)
            throw new InvalidDataException("Reference course fixed-step duration must be finite and positive.");
        var previousTick = -1;
        foreach (var command in Commands)
        {
            command.Validate();
            if (command.Tick <= previousTick)
                throw new InvalidDataException("Reference course command ticks must be strictly increasing.");
            previousTick = command.Tick;
        }
    }

    public MovementRecording Execute(SourceMovementMotor motor)
    {
        ArgumentNullException.ThrowIfNull(motor);
        Validate();
        var recording = new MovementRecording { Profile = Name, FixedStepSeconds = FixedStepSeconds };
        for (var index = 0; index < Commands.Count; index++)
        {
            var command = Commands[index];
            motor.Tick(command.Input, FixedStepSeconds);
            var orientation = Quaternion.CreateFromYawPitchRoll(command.Input.ViewYawRadians, command.Input.ViewPitchRadians, 0f);
            recording.Capture(command.Tick, motor.State, command.Input, orientation);
        }
        recording.Validate();
        return recording;
    }

    public string ToJson()
    {
        Validate();
        return JsonSerializer.Serialize(this, JsonOptions);
    }

    public static SourceReferenceCourse FromJson(string json)
    {
        var course = JsonSerializer.Deserialize<SourceReferenceCourse>(json, JsonOptions)
            ?? throw new InvalidDataException("Invalid reference course.");
        course.Validate();
        return course;
    }
}
