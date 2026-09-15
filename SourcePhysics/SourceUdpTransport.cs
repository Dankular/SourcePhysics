using System.Net;
using System.Net.Sockets;
using System.Text.Json;
using System.Buffers.Binary;

namespace SourcePhysics;

/// Thin UDP boundary for SourceNetworkPacket. Reliability, retransmission and
/// authority remain above this layer, matching Source's separate netchannel and
/// simulation responsibilities.
public sealed class SourceUdpTransport : IDisposable
{
    private const ushort WireMagic = 0x5350;
    private const byte WireVersion = 1;
    private const int WireHeaderBytes = 20;
    private static readonly JsonSerializerOptions Options = new() { IncludeFields = true };
    private readonly UdpClient socket;
    private IPEndPoint remote;
    private int nextSequence;
    private bool disposed;

    public IPEndPoint LocalEndPoint => (IPEndPoint)socket.Client.LocalEndPoint!;

    public SourceUdpTransport(IPEndPoint local, IPEndPoint remote)
    {
        this.remote = remote ?? throw new ArgumentNullException(nameof(remote));
        socket = new UdpClient(local ?? throw new ArgumentNullException(nameof(local)));
        socket.Client.ReceiveTimeout = 1;
    }

    public void SetRemote(IPEndPoint remote) => this.remote = remote ?? throw new ArgumentNullException(nameof(remote));

    public SourceNetworkPacket Send(SourceNetworkEndpoint from, ReadOnlySpan<byte> payload,
        bool reliable, int currentTick, int acknowledgedSequence = -1)
    {
        EnsureNotDisposed();
        if (payload.Length > SourceNetworkPacket.MaximumDatagramPayload)
            throw new ArgumentException("Payload exceeds Source datagram limit.", nameof(payload));
        var to = from == SourceNetworkEndpoint.Client ? SourceNetworkEndpoint.Server : SourceNetworkEndpoint.Client;
        var packet = new SourceNetworkPacket(from, to, nextSequence++, acknowledgedSequence, reliable,
            currentTick, payload.ToArray());
        packet.Validate();
        var bytes = SerializeWire(packet);
        socket.Send(bytes, bytes.Length, remote);
        return packet;
    }

    public bool TryReceive(out SourceNetworkPacket packet)
    {
        EnsureNotDisposed();
        packet = default;
        if (socket.Available <= 0) return false;
        IPEndPoint sender = new(IPAddress.Any, 0);
        var bytes = socket.Receive(ref sender);
        packet = bytes.Length >= WireHeaderBytes &&
            BinaryPrimitives.ReadUInt16LittleEndian(bytes.AsSpan(0, 2)) == WireMagic
            ? DeserializeWire(bytes)
            : JsonSerializer.Deserialize<SourceNetworkPacket>(bytes, Options);
        packet.Validate();
        return true;
    }

    public bool TryReceiveCommand(out SourceNetworkPacket packet, out SourceCommand command)
    {
        if (!TryReceive(out packet))
        {
            command = default;
            return false;
        }
        command = SourceNetworkSerialization.DeserializeCommand(packet.Payload);
        return true;
    }

    public bool TryReceiveCorrection(out SourceNetworkPacket packet, out SourceAuthoritativeCorrection correction)
    {
        if (!TryReceive(out packet))
        {
            correction = default;
            return false;
        }
        correction = SourceNetworkSerialization.DeserializeCorrection(packet.Payload);
        return true;
    }

    public SourceNetworkPacket SendCommand(SourceCommand command, int currentTick, int acknowledgedSequence = -1,
        bool reliable = true) => Send(SourceNetworkEndpoint.Client,
            SourceNetworkSerialization.SerializeCommand(command), reliable, currentTick, acknowledgedSequence);

    public SourceNetworkPacket SendCorrection(SourceAuthoritativeCorrection correction, int currentTick,
        int acknowledgedSequence = -1, bool reliable = true) => Send(SourceNetworkEndpoint.Server,
            SourceNetworkSerialization.SerializeCorrection(correction), reliable, currentTick, acknowledgedSequence);

    public void Dispose()
    {
        if (disposed) return;
        disposed = true;
        socket.Dispose();
    }

    private void EnsureNotDisposed()
    {
        if (disposed) throw new ObjectDisposedException(nameof(SourceUdpTransport));
    }

    private static byte[] SerializeWire(in SourceNetworkPacket packet)
    {
        if (packet.Payload.Length > SourceNetworkPacket.MaximumDatagramPayload - WireHeaderBytes)
            throw new InvalidDataException("Serialized Source packet exceeds the datagram limit.");
        var bytes = new byte[WireHeaderBytes + packet.Payload.Length];
        var span = bytes.AsSpan();
        BinaryPrimitives.WriteUInt16LittleEndian(span, WireMagic);
        span[2] = WireVersion;
        span[3] = (byte)packet.From;
        span[4] = (byte)packet.To;
        span[5] = packet.Reliable ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteInt32LittleEndian(span[6..], packet.Sequence);
        BinaryPrimitives.WriteInt32LittleEndian(span[10..], packet.AcknowledgedSequence);
        BinaryPrimitives.WriteInt32LittleEndian(span[14..], packet.SentTick);
        BinaryPrimitives.WriteUInt16LittleEndian(span[18..], checked((ushort)packet.Payload.Length));
        packet.Payload.CopyTo(span[WireHeaderBytes..]);
        return bytes;
    }

    private static SourceNetworkPacket DeserializeWire(ReadOnlySpan<byte> bytes)
    {
        if (bytes[2] != WireVersion)
            throw new InvalidDataException("Unsupported Source UDP wire version.");
        var payloadLength = BinaryPrimitives.ReadUInt16LittleEndian(bytes[18..]);
        if (bytes.Length != WireHeaderBytes + payloadLength)
            throw new InvalidDataException("Invalid Source UDP wire payload length.");
        return new((SourceNetworkEndpoint)bytes[3], (SourceNetworkEndpoint)bytes[4],
            BinaryPrimitives.ReadInt32LittleEndian(bytes[6..]),
            BinaryPrimitives.ReadInt32LittleEndian(bytes[10..]), bytes[5] != 0,
            BinaryPrimitives.ReadInt32LittleEndian(bytes[14..]),
            bytes[WireHeaderBytes..].ToArray());
    }
}
