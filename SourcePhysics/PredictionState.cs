using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

/// A single client input command. Sequence is transport metadata; Tick remains
/// the simulation ordering key used by the Source motor.
public readonly record struct SourceCommand(int Tick, SourceInput Input, int Sequence = 0)
{
    public void Validate()
    {
        if (Tick < 0) throw new ArgumentOutOfRangeException(nameof(Tick));
        if (Sequence < 0) throw new ArgumentOutOfRangeException(nameof(Sequence));
        if (!float.IsFinite(Input.Move.X) || !float.IsFinite(Input.Move.Y) ||
            !float.IsFinite(Input.ViewYawRadians) || !float.IsFinite(Input.ViewPitchRadians) || !float.IsFinite(Input.UpMove))
            throw new ArgumentException("Command input must contain finite values.", nameof(Input));
    }
}

public readonly record struct SourceSnapshot(int Tick, MovementState State, SourcePhysicsWorldState? WorldState = null)
{
    public void Validate()
    {
        if (Tick < 0) throw new ArgumentOutOfRangeException(nameof(Tick));
        if (WorldState is not null && WorldState.Tick != Tick)
            throw new ArgumentException("World correction tick must match the movement snapshot tick.", nameof(WorldState));
    }
}

/// Server-authoritative movement/world correction. LastProcessedSequence is
/// transport acknowledgement only; replay still follows authoritative ticks.
public readonly record struct SourceAuthoritativeCorrection(int Tick, int LastProcessedSequence,
    MovementState State, SourcePhysicsWorldState? WorldState = null)
{
    public SourceSnapshot Snapshot => new(Tick, State, WorldState);

    public void Validate()
    {
        if (Tick < 0) throw new ArgumentOutOfRangeException(nameof(Tick));
        if (LastProcessedSequence < 0) throw new ArgumentOutOfRangeException(nameof(LastProcessedSequence));
        Snapshot.Validate();
    }
}

public sealed class SourcePredictionBuffer
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    private readonly int capacity;
    private readonly SortedDictionary<int, SourceCommand> commands = new();
    private readonly SortedDictionary<int, SourceSnapshot> snapshots = new();

    public SourcePredictionBuffer(int capacity = 256)
    {
        if (capacity < 1) throw new ArgumentOutOfRangeException(nameof(capacity));
        this.capacity = capacity;
    }

    public void AddCommand(SourceCommand command) { command.Validate(); commands[command.Tick] = command; Trim(commands); }
    public void AddSnapshot(SourceSnapshot snapshot) { snapshot.Validate(); snapshots[snapshot.Tick] = snapshot; Trim(snapshots); }
    public bool TryGetCommand(int tick, out SourceCommand command) => commands.TryGetValue(tick, out command);
    public bool TryGetSnapshot(int tick, out SourceSnapshot snapshot) => snapshots.TryGetValue(tick, out snapshot);

    public bool ReplayFrom(SourceMovementMotor motor, int snapshotTick, float fixedStepSeconds, out MovementState state)
    {
        if (!TryGetSnapshot(snapshotTick, out var snapshot))
        {
            state = default;
            return false;
        }
        motor.LoadState(snapshot.State);
        foreach (var command in CommandsFrom(snapshotTick + 1)) motor.Tick(command.Input, fixedStepSeconds);
        state = motor.State;
        return true;
    }

    public bool ReplayFrom(JoltPhysicsHost host, SourceMovementMotor motor, int snapshotTick,
        float fixedStepSeconds, int collisionSteps, int integrationSubSteps, out MovementState state)
    {
        if (!TryGetSnapshot(snapshotTick, out var snapshot))
        {
            state = default;
            return false;
        }
        if (snapshot.WorldState is not null) host.RestoreState(snapshot.WorldState);
        motor.LoadState(snapshot.State);
        foreach (var command in CommandsFrom(snapshotTick + 1))
        {
            motor.Tick(command.Input, fixedStepSeconds);
            host.Step(collisionSteps, integrationSubSteps);
        }
        state = motor.State;
        return true;
    }

    public bool ApplyCorrection(SourceMovementMotor motor, in SourceAuthoritativeCorrection correction,
        float fixedStepSeconds, out MovementState state)
    {
        correction.Validate();
        AddSnapshot(correction.Snapshot);
        return ReplayFrom(motor, correction.Tick, fixedStepSeconds, out state);
    }

    public bool ApplyCorrection(JoltPhysicsHost host, SourceMovementMotor motor,
        in SourceAuthoritativeCorrection correction, float fixedStepSeconds, int collisionSteps,
        int integrationSubSteps, out MovementState state)
    {
        correction.Validate();
        AddSnapshot(correction.Snapshot);
        return ReplayFrom(host, motor, correction.Tick, fixedStepSeconds, collisionSteps, integrationSubSteps, out state);
    }

    public string ToJson()
    {
        return JsonSerializer.Serialize(new PredictionPacket(commands.Values.ToArray(), snapshots.Values.ToArray()), JsonOptions);
    }

    public static SourcePredictionBuffer FromJson(string json, int capacity = 256)
    {
        var packet = JsonSerializer.Deserialize<PredictionPacket>(json, JsonOptions)
            ?? throw new InvalidDataException("Invalid prediction packet.");
        var buffer = new SourcePredictionBuffer(capacity);
        foreach (var command in packet.Commands) buffer.AddCommand(command);
        foreach (var snapshot in packet.Snapshots) buffer.AddSnapshot(snapshot);
        return buffer;
    }

    public IEnumerable<SourceCommand> CommandsFrom(int tick) => commands.Where(pair => pair.Key >= tick).Select(pair => pair.Value);

    private void Trim<T>(SortedDictionary<int, T> values)
    {
        while (values.Count > capacity) values.Remove(values.Keys.First());
    }

    private sealed record PredictionPacket(SourceCommand[] Commands, SourceSnapshot[] Snapshots);
}
