namespace SourcePhysics;

public enum SourceObjectLayer : byte { World = 0, Static = 1, Dynamic = 2, Player = 3, Trigger = 4, Debris = 5, NonSolid = 6 }

[Flags]
public enum SourceSolidFlags : ushort
{
    None = 0,
    NotSolid = 0x0004,
    Trigger = 0x0008,
    TriggerTouchDebris = 0x0200
}

public static class SourceCollisionRules
{
    public static bool CanCollide(SourceObjectLayer a, SourceObjectLayer b)
    {
        if (a == SourceObjectLayer.NonSolid || b == SourceObjectLayer.NonSolid) return false;
        if (a == SourceObjectLayer.Trigger || b == SourceObjectLayer.Trigger) return false;
        if (a == SourceObjectLayer.Debris && b == SourceObjectLayer.Debris) return false;
        return true;
    }
}
