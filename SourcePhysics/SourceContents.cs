namespace SourcePhysics;

[Flags]
public enum SourceContents : uint
{
    Empty = 0,
    Solid = 0x00000001,
    Window = 0x00000002,
    Grate = 0x00000008,
    Water = 0x00000020,
    Moveable = 0x00004000,
    Monster = 0x02000000,
    Debris = 0x04000000,
    Hitbox = 0x40000000,
    MaskPlayerSolid = Solid | Moveable | Window | Monster | Grate,
    MaskShot = Solid | Moveable | Monster | Window | Debris | Hitbox
}
