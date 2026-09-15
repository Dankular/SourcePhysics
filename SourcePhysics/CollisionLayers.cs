namespace SourcePhysics;

public enum SourceObjectLayer : byte { World = 0, Static = 1, Dynamic = 2, Player = 3, Trigger = 4, Debris = 5 }

public static class SourceCollisionRules
{
    public static bool CanCollide(SourceObjectLayer a, SourceObjectLayer b)
    {
        if (a == SourceObjectLayer.Trigger || b == SourceObjectLayer.Trigger) return false;
        if (a == SourceObjectLayer.Debris && b == SourceObjectLayer.Debris) return false;
        return true;
    }
}
