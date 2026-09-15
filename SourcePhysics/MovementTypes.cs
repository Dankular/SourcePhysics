using System.Numerics;

namespace SourcePhysics;

[Flags]
public enum Buttons { None = 0, Jump = 1, Duck = 2, Forward = 4, Back = 8, Use = 16, Speed = 32 }

public enum SourceMoveType { Walk, Fly, FlyGravity, Ladder, Noclip, Observer, None }
public enum SourceWaterLevel { Dry, Feet, Waist, Eyes }
public enum SourceObserverMode { None, DeathCam, FreezeCam, Fixed, InEye, Chase, Roaming }

[Flags]
public enum SourceWaterCurrent
{
    None = 0,
    Current0 = 1,
    Current90 = 2,
    Current180 = 4,
    Current270 = 8,
    CurrentUp = 16,
    CurrentDown = 32
}

public readonly record struct SourceInput(Vector2 Move, Buttons Buttons, float ViewYawRadians = 0f,
    float UpMove = 0f, SourceMoveType MoveType = SourceMoveType.Walk, float ViewPitchRadians = 0f,
    SourceObserverMode ObserverMode = SourceObserverMode.Roaming, bool ObserverNoClip = true)
{
    public bool IsDown(Buttons button) => (Buttons & button) != 0;
}

public enum GroundState { Airborne, Grounded, Stuck }

public readonly record struct MovementState(
    Vector3 Position,
    Vector3 Velocity,
    GroundState Ground,
    Vector3 GroundNormal,
    int GroundBodyId,
    float SurfaceFriction,
    bool Ducking,
    bool Jumped,
    SourceMoveType MoveType = SourceMoveType.Walk,
    SourceWaterLevel WaterLevel = SourceWaterLevel.Dry,
    float WaterJumpTime = 0f,
    Vector3 BaseVelocity = default,
    float FallVelocity = 0f,
    float ViewHeightSourceUnits = 64f,
    float SurfaceMaxSpeedFactor = 1f,
    float SurfaceJumpFactor = 1f,
    bool PreviousJumpDown = false,
    float DuckTransitionTimeSeconds = 0f,
    bool DuckTransitioningUp = false);

public readonly record struct MovementContact(Vector3 Position, Vector3 Normal, float Fraction, int BodyId, float Friction, float Restitution,
    bool StartSolid = false, bool AllSolid = false, int SurfaceId = 0, float PenetrationDepth = 0f);

public interface ISourceMovementQueries
{
    MovementContact SweepPlayer(Vector3 start, Vector3 end, bool crouched);
    bool IsEmpty(Vector3 position, bool crouched);
    Vector3 GetBodyPointVelocity(int bodyId, Vector3 worldPoint);
    void ApplyCharacterImpulse(int bodyId, Vector3 point, Vector3 impulse);
    SourceSurface GetSurface(int surfaceId) => new("fallback");
    bool TryWaterJump(Vector3 position, Vector3 direction, out Vector3 velocity, out float durationSeconds)
    {
        velocity = default;
        durationSeconds = 0f;
        return false;
    }
    SourceWaterLevel GetWaterLevel(Vector3 position, bool crouched);
    /// Source CheckWater adds 50 * water-level in the authored current
    /// direction to the player's base velocity. Providers that have no
    /// current return zero.
    Vector3 GetWaterBaseVelocity(Vector3 position, SourceWaterLevel waterLevel) => Vector3.Zero;
    bool TryGetObserverTarget(out MovementState target)
    {
        target = default;
        return false;
    }
    bool TryLadder(Vector3 position, Vector3 direction, out Vector3 normal, out int bodyId);
}
