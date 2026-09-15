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

public enum SourceCollisionGroup : byte
{
    None = 0,
    Debris,
    DebrisTrigger,
    InteractiveDebris,
    Interactive,
    Player,
    BreakableGlass,
    Vehicle,
    PlayerMovement,
    Npc,
    InVehicle,
    Weapon,
    VehicleClip,
    Projectile,
    DoorBlocker,
    PassableDoor,
    Dissolving,
    PushAway,
    NpcActor,
    NpcScripted
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

    /// Port of the referenced base CGameRules::ShouldCollide matrix. The caller
    /// supplies ordered Source collision groups; this method performs the same
    /// low/high ordering used by Source before applying the rules.
    public static bool ShouldCollide(SourceCollisionGroup first, SourceCollisionGroup second)
    {
        var a = (int)first;
        var b = (int)second;
        if (a > b) (a, b) = (b, a);
        var low = (SourceCollisionGroup)a;
        var high = (SourceCollisionGroup)b;

        if ((low == SourceCollisionGroup.Player || low == SourceCollisionGroup.PlayerMovement) &&
            high == SourceCollisionGroup.PushAway) return false;
        if (low == SourceCollisionGroup.Debris && high == SourceCollisionGroup.PushAway) return true;
        if (low == SourceCollisionGroup.InVehicle || high == SourceCollisionGroup.InVehicle) return false;
        if (high == SourceCollisionGroup.DoorBlocker && low != SourceCollisionGroup.Npc) return false;
        if (low == SourceCollisionGroup.Player && high == SourceCollisionGroup.PassableDoor) return false;
        if (low == SourceCollisionGroup.Debris || low == SourceCollisionGroup.DebrisTrigger) return false;
        if (low == SourceCollisionGroup.Dissolving || high == SourceCollisionGroup.Dissolving)
            return low == SourceCollisionGroup.None;
        if (low == SourceCollisionGroup.InteractiveDebris && high == SourceCollisionGroup.InteractiveDebris) return false;
        if (low == SourceCollisionGroup.InteractiveDebris &&
            (high == SourceCollisionGroup.Player || high == SourceCollisionGroup.PlayerMovement)) return false;
        if (low == SourceCollisionGroup.BreakableGlass && high == SourceCollisionGroup.BreakableGlass) return false;
        if (high == SourceCollisionGroup.Interactive && low != SourceCollisionGroup.None) return false;
        if (high == SourceCollisionGroup.Projectile &&
            (low == SourceCollisionGroup.Debris || low == SourceCollisionGroup.Weapon || low == SourceCollisionGroup.Projectile)) return false;
        if (high == SourceCollisionGroup.Weapon &&
            (low == SourceCollisionGroup.Vehicle || low == SourceCollisionGroup.Player || low == SourceCollisionGroup.Npc)) return false;
        if (low == SourceCollisionGroup.VehicleClip || high == SourceCollisionGroup.VehicleClip)
            return low == SourceCollisionGroup.Vehicle;
        return true;
    }
}
