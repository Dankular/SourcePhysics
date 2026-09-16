namespace SourcePhysics;

[Flags]
public enum SourceCallbackFlags : uint
{
    GlobalCollision = 0x0001,
    GlobalFriction = 0x0002,
    GlobalTouch = 0x0004,
    GlobalTouchStatic = 0x0008,
    ShadowCollision = 0x0010,
    GlobalCollideStatic = 0x0020,
    IsVehicleWheel = 0x0040,
    FluidTouch = 0x0100,
    NeverDeleted = 0x0200,
    MarkedForDelete = 0x0400,
    EnablingCollision = 0x0800,
    DoFluidSimulation = 0x1000,
    IsPlayerController = 0x2000,
    CheckCollisionDisable = 0x4000,
    MarkedForTest = 0x8000,

    Default = GlobalCollision | GlobalFriction | FluidTouch | GlobalTouch |
        GlobalCollideStatic | DoFluidSimulation
}
