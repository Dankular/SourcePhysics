namespace SourcePhysics;

/// Server-authoritative fixed-tick session. Transport only delivers commands;
/// this class owns ordering, simulation advancement and correction production.
public sealed class SourceAuthoritativePhysicsSession
{
    private readonly JoltPhysicsHost host;
    private readonly SourceMovementMotor motor;
    private readonly SourceFixedStepClock clock;
    private readonly Func<int, SourceInput> missingCommand;
    private readonly bool includeWorldState;
    private readonly SortedDictionary<int, SourceCommand> pending = new();
    private int highestAcceptedSequence = -1;
    private int lastProcessedSequence = -1;

    public int SimulationTick => clock.Tick;
    public int LastProcessedSequence => lastProcessedSequence;
    public event Action<SourceAuthoritativeCorrection>? CorrectionProduced;

    public SourceAuthoritativePhysicsSession(JoltPhysicsHost host, SourceMovementMotor motor,
        Func<int, SourceInput> missingCommand, bool includeWorldState = true)
    {
        this.host = host ?? throw new ArgumentNullException(nameof(host));
        this.motor = motor ?? throw new ArgumentNullException(nameof(motor));
        this.missingCommand = missingCommand ?? throw new ArgumentNullException(nameof(missingCommand));
        this.includeWorldState = includeWorldState;
        clock = new SourceFixedStepClock(host.FixedStepSeconds);
    }

    public bool AcceptCommand(SourceCommand command)
    {
        command.Validate();
        if (command.Sequence <= highestAcceptedSequence || command.Tick < clock.Tick)
            return false;
        if (pending.ContainsKey(command.Tick)) return false;
        pending.Add(command.Tick, command);
        highestAcceptedSequence = command.Sequence;
        return true;
    }

    public int Advance(float elapsedSeconds)
    {
        return clock.Advance(elapsedSeconds, SimulateTick);
    }

    public int ProcessUdpCommands(SourceUdpTransport transport)
    {
        ArgumentNullException.ThrowIfNull(transport);
        var accepted = 0;
        while (transport.TryReceiveCommand(out _, out var command))
            if (AcceptCommand(command)) accepted++;
        return accepted;
    }

    public int SendProducedCorrection(SourceUdpTransport transport, SourceAuthoritativeCorrection correction,
        int currentNetworkTick)
    {
        ArgumentNullException.ThrowIfNull(transport);
        return transport.SendCorrection(correction, currentNetworkTick, correction.LastProcessedSequence).Sequence;
    }

    private void SimulateTick(int tick, float deltaSeconds)
    {
        var hasCommand = pending.Remove(tick, out var command);
        var input = hasCommand ? command.Input : missingCommand(tick);
        if (hasCommand)
            lastProcessedSequence = command.Sequence;
        motor.Tick(input, deltaSeconds);
        host.Step();
        var world = includeWorldState ? host.CaptureState(tick) : null;
        var correction = new SourceAuthoritativeCorrection(tick, lastProcessedSequence, motor.State, world);
        correction.Validate();
        CorrectionProduced?.Invoke(correction);
    }
}
