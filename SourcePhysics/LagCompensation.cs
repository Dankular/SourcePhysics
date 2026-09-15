namespace SourcePhysics;

/// Bounded authoritative physics history used by server-side historical traces.
/// It stores only already-captured state; it does not invent interpolation or
/// a latency policy.
public sealed class SourceLagCompensationHistory
{
    private readonly int capacity;
    private readonly SortedDictionary<int, SourcePhysicsWorldState> states = new();

    public SourceLagCompensationHistory(int capacity = 128)
    {
        if (capacity < 1) throw new ArgumentOutOfRangeException(nameof(capacity));
        this.capacity = capacity;
    }

    public void Record(SourcePhysicsWorldState state)
    {
        if (state.Tick < 0) throw new ArgumentOutOfRangeException(nameof(state));
        var copy = state with { Bodies = state.Bodies.ToArray() };
        states[state.Tick] = copy;
        while (states.Count > capacity) states.Remove(states.Keys.First());
    }

    public void Record(JoltPhysicsHost host, int tick)
    {
        ArgumentNullException.ThrowIfNull(host);
        Record(host.CaptureState(tick));
    }

    public bool TryGet(int tick, out SourcePhysicsWorldState state) => states.TryGetValue(tick, out state!);

    public bool TryBeginRewind(JoltPhysicsHost host, int tick, out IDisposable rewind)
    {
        ArgumentNullException.ThrowIfNull(host);
        if (!states.TryGetValue(tick, out var historical))
        {
            rewind = NullRewind.Instance;
            return false;
        }
        var live = host.CaptureState();
        host.RestoreState(historical);
        rewind = new RewindScope(host, live);
        return true;
    }

    private sealed class RewindScope(JoltPhysicsHost host, SourcePhysicsWorldState live) : IDisposable
    {
        private bool restored;

        public void Dispose()
        {
            if (restored) return;
            restored = true;
            host.RestoreState(live);
        }
    }

    private sealed class NullRewind : IDisposable
    {
        public static NullRewind Instance { get; } = new();
        public void Dispose() { }
    }
}
