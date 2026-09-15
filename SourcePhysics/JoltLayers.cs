using JoltPhysicsSharp;

namespace SourcePhysics;

internal sealed class SourceCollisionLayers : IDisposable
{
    public BroadPhaseLayerInterfaceTable BroadPhase { get; }
    public ObjectLayerPairFilterTable PairFilter { get; }
    public ObjectVsBroadPhaseLayerFilterTable ObjectVsBroadPhase { get; }

    public SourceCollisionLayers(SourceCollisionPolicy policy)
    {
        var count = (uint)Enum.GetValues<SourceObjectLayer>().Length;
        BroadPhase = new BroadPhaseLayerInterfaceTable(2, count);
        for (var layer = 0u; layer < count; layer++)
            BroadPhase.MapObjectToBroadPhaseLayer(new ObjectLayer((ushort)layer), new BroadPhaseLayer((byte)(layer <= 1 ? 0 : 1)));
        PairFilter = new ObjectLayerPairFilterTable(count);
        for (var a = 0; a < (int)count; a++)
        for (var b = 0; b < (int)count; b++)
        {
            PairFilter.EnableCollision(new ObjectLayer((ushort)a), new ObjectLayer((ushort)b));
            var sensorPair = (SourceObjectLayer)a == SourceObjectLayer.Trigger || (SourceObjectLayer)b == SourceObjectLayer.Trigger;
            var triggerPair = (SourceObjectLayer)a == SourceObjectLayer.Trigger &&
                (SourceObjectLayer)b == SourceObjectLayer.Trigger;
            // Source trigger volumes do not touch other trigger volumes, but
            // sensor-vs-solid pairs remain enabled for trigger events without
            // making the pair physically solid.
            if (!policy.CanCollide((SourceObjectLayer)a, (SourceObjectLayer)b) &&
                (!sensorPair || triggerPair))
                PairFilter.DisableCollision(new ObjectLayer((ushort)a), new ObjectLayer((ushort)b));
        }
        ObjectVsBroadPhase = new ObjectVsBroadPhaseLayerFilterTable(BroadPhase, 2, PairFilter, count);
    }

    public void Dispose()
    {
        ObjectVsBroadPhase.Dispose(); PairFilter.Dispose(); BroadPhase.Dispose();
    }
}
