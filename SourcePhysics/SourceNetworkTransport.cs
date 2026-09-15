using System.Collections.Generic;
using System.Text.Json;

namespace SourcePhysics;

public enum SourceNetworkEndpoint
{
    Client,
    Server
}

public enum SourceNetworkConnectionState
{
    Disconnected,
    Connected,
    TimingOut,
    TimedOut
}

public readonly record struct SourceNetworkTimeoutProfile(
    float ConnectionProblemSeconds = 15f,
    float SignOnTimeoutSeconds = 120f)
{
    public void Validate()
    {
        if (!float.IsFinite(ConnectionProblemSeconds) || !float.IsFinite(SignOnTimeoutSeconds) ||
            ConnectionProblemSeconds <= 0f || SignOnTimeoutSeconds < ConnectionProblemSeconds)
            throw new ArgumentOutOfRangeException(nameof(SourceNetworkTimeoutProfile));
    }
}

public sealed class SourceNetworkConnectionTracker
{
    private readonly SourceNetworkTimeoutProfile profile;
    private readonly float fixedStepSeconds;
    private int lastReceivedTick;
    private bool hasReceivedPacket;

    public SourceNetworkConnectionState State { get; private set; } = SourceNetworkConnectionState.Disconnected;
    public int LastReceivedTick => lastReceivedTick;
    public float TimeSinceLastReceivedSeconds(int currentTick) =>
        hasReceivedPacket ? MathF.Max(0f, (currentTick - lastReceivedTick) * fixedStepSeconds) : float.PositiveInfinity;

    public SourceNetworkConnectionTracker(float fixedStepSeconds,
        SourceNetworkTimeoutProfile profile = default)
    {
        if (!float.IsFinite(fixedStepSeconds) || fixedStepSeconds <= 0f)
            throw new ArgumentOutOfRangeException(nameof(fixedStepSeconds));
        profile.Validate();
        this.fixedStepSeconds = fixedStepSeconds;
        this.profile = profile;
    }

    public void Connect(int currentTick)
    {
        if (currentTick < 0) throw new ArgumentOutOfRangeException(nameof(currentTick));
        lastReceivedTick = currentTick;
        hasReceivedPacket = false;
        State = SourceNetworkConnectionState.Connected;
    }

    public void ObservePacket(int currentTick)
    {
        if (currentTick < 0) throw new ArgumentOutOfRangeException(nameof(currentTick));
        if (State == SourceNetworkConnectionState.Disconnected)
            Connect(currentTick);
        lastReceivedTick = currentTick;
        hasReceivedPacket = true;
        State = SourceNetworkConnectionState.Connected;
    }

    public SourceNetworkConnectionState Advance(int currentTick)
    {
        if (currentTick < 0) throw new ArgumentOutOfRangeException(nameof(currentTick));
        var elapsed = TimeSinceLastReceivedSeconds(currentTick);
        if (elapsed >= profile.SignOnTimeoutSeconds) State = SourceNetworkConnectionState.TimedOut;
        else if (elapsed >= profile.ConnectionProblemSeconds && State == SourceNetworkConnectionState.Connected)
            State = SourceNetworkConnectionState.TimingOut;
        return State;
    }

    public void Disconnect() => State = SourceNetworkConnectionState.Disconnected;
}

public readonly record struct SourceNetworkImpairment(
    float LossProbability = 0f,
    int MinimumLatencyTicks = 0,
    int MaximumLatencyTicks = 0,
    float DuplicateProbability = 0f,
    float ReorderProbability = 0f,
    int Seed = 1)
{
    public void Validate()
    {
        if (!float.IsFinite(LossProbability) || LossProbability is < 0f or > 1f ||
            !float.IsFinite(DuplicateProbability) || DuplicateProbability is < 0f or > 1f ||
            !float.IsFinite(ReorderProbability) || ReorderProbability is < 0f or > 1f ||
            MinimumLatencyTicks < 0 || MaximumLatencyTicks < MinimumLatencyTicks)
            throw new ArgumentOutOfRangeException(nameof(SourceNetworkImpairment));
    }
}

public readonly record struct SourceNetworkPacket(
    SourceNetworkEndpoint From,
    SourceNetworkEndpoint To,
    int Sequence,
    int AcknowledgedSequence,
    bool Reliable,
    int SentTick,
    byte[] Payload)
{
    public const int MaximumDatagramPayload = 1400; // Source NET_MAX_DATAGRAM_PAYLOAD.

    public void Validate()
    {
        if (Sequence < 0 || AcknowledgedSequence < -1 || SentTick < 0 || Payload is null ||
            Payload.Length > MaximumDatagramPayload)
            throw new InvalidDataException("Invalid Source network packet.");
    }
}

public sealed class SourceImpairedTransport
{
    private readonly SourceNetworkImpairment impairment;
    private readonly Random random;
    private readonly List<PendingPacket> pending = new();
    private readonly Dictionary<SourceNetworkEndpoint, int> nextSequence = new()
    {
        [SourceNetworkEndpoint.Client] = 0,
        [SourceNetworkEndpoint.Server] = 0
    };
    private readonly Dictionary<(SourceNetworkEndpoint From, int Sequence), SourceNetworkPacket> reliablePending = new();

    public SourceImpairedTransport(SourceNetworkImpairment impairment = default)
    {
        impairment.Validate();
        this.impairment = impairment;
        random = new Random(impairment.Seed);
    }

    public SourceNetworkPacket Send(SourceNetworkEndpoint from, ReadOnlySpan<byte> payload, bool reliable,
        int currentTick, int acknowledgedSequence = -1)
    {
        if (currentTick < 0) throw new ArgumentOutOfRangeException(nameof(currentTick));
        if (payload.Length > SourceNetworkPacket.MaximumDatagramPayload)
            throw new ArgumentException("Payload exceeds Source datagram limit.", nameof(payload));
        var to = from == SourceNetworkEndpoint.Client ? SourceNetworkEndpoint.Server : SourceNetworkEndpoint.Client;
        var packet = new SourceNetworkPacket(from, to, nextSequence[from]++, acknowledgedSequence, reliable,
            currentTick, payload.ToArray());
        packet.Validate();
        if (reliable) reliablePending[(from, packet.Sequence)] = packet;
        Enqueue(packet, currentTick);
        return packet;
    }

    public void Acknowledge(SourceNetworkEndpoint from, int sequence)
    {
        reliablePending.Remove((from, sequence));
    }

    public int RetransmitUnacknowledged(int currentTick, int retryAfterTicks = 1)
    {
        if (currentTick < 0 || retryAfterTicks < 1) throw new ArgumentOutOfRangeException();
        var resent = 0;
        foreach (var packet in reliablePending.Values.ToArray())
        {
            if (currentTick - packet.SentTick < retryAfterTicks) continue;
            var retransmit = packet with { SentTick = currentTick };
            reliablePending[(packet.From, packet.Sequence)] = retransmit;
            Enqueue(retransmit, currentTick);
            resent++;
        }
        return resent;
    }

    public IReadOnlyList<SourceNetworkPacket> Receive(SourceNetworkEndpoint receiver, int currentTick)
    {
        if (currentTick < 0) throw new ArgumentOutOfRangeException(nameof(currentTick));
        var ready = pending.Where(item => item.Packet.To == receiver && item.DeliveryTick <= currentTick)
            .OrderBy(item => item.DeliveryTick).ThenBy(item => item.Order).ToArray();
        foreach (var item in ready) pending.Remove(item);
        return ready.Select(item => item.Packet).ToArray();
    }

    public int UnacknowledgedReliableCount => reliablePending.Count;

    private void Enqueue(SourceNetworkPacket packet, int currentTick)
    {
        if (random.NextDouble() < impairment.LossProbability) return;
        var latency = impairment.MinimumLatencyTicks == impairment.MaximumLatencyTicks
            ? impairment.MinimumLatencyTicks
            : random.Next(impairment.MinimumLatencyTicks, impairment.MaximumLatencyTicks + 1);
        var order = pending.Count;
        if (random.NextDouble() < impairment.ReorderProbability) order = -order - 1;
        pending.Add(new(packet, currentTick + latency, order));
        if (random.NextDouble() < impairment.DuplicateProbability)
            pending.Add(new(packet, currentTick + latency, order - 1));
    }

    private readonly record struct PendingPacket(SourceNetworkPacket Packet, int DeliveryTick, int Order);
}

public static class SourceNetworkSerialization
{
    private static readonly JsonSerializerOptions Options = new() { IncludeFields = true };

    public static byte[] SerializeCommand(SourceCommand command)
    {
        command.Validate();
        return Serialize(command);
    }

    public static SourceCommand DeserializeCommand(ReadOnlySpan<byte> payload) =>
        Deserialize<SourceCommand>(payload);

    public static byte[] SerializeCorrection(SourceAuthoritativeCorrection correction)
    {
        correction.Validate();
        return Serialize(correction);
    }

    public static SourceAuthoritativeCorrection DeserializeCorrection(ReadOnlySpan<byte> payload) =>
        Deserialize<SourceAuthoritativeCorrection>(payload);

    private static byte[] Serialize<T>(T value) => JsonSerializer.SerializeToUtf8Bytes(value, Options);

    private static T Deserialize<T>(ReadOnlySpan<byte> payload) =>
        JsonSerializer.Deserialize<T>(payload, Options)
        ?? throw new InvalidDataException($"Invalid {typeof(T).Name} payload.");
}
