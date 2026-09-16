using System.Numerics;

namespace SourcePhysics;

public enum SourceHitGroup
{
    Generic = 0,
    Head = 1,
    Chest = 2,
    Stomach = 3,
    LeftArm = 4,
    RightArm = 5,
    LeftLeg = 6,
    RightLeg = 7,
    Gear = 10
}

[Flags]
public enum SourceFireBulletsFlags
{
    None = 0,
    FirstShotAccurate = 0x1,
    DontHitUnderwater = 0x2,
    AllowWaterSurfaceImpacts = 0x4,
    TemporaryDangerSound = 0x8
}

[Flags]
public enum SourceAmmoFlags
{
    None = 0,
    ForceDropIfCarried = 0x1,
    InterpretPlayerDamageAsDamageToPlayer = 0x2
}

public enum SourceShotTraceShape
{
    Ray,
    PlayerAlternatingHull
}

/// The AmmoDef fields consumed by the shared Source FireBullets path. The
/// title supplies these values; the physics bridge never invents an ammo
/// table from an integer ammo index.
public readonly record struct SourceAmmoDefinition(
    int DamageType,
    SourceAmmoFlags Flags = SourceAmmoFlags.None,
    int PlayerDamage = 0);

/// Direct representation of the Source FireBulletsInfo_t fields used by the shared fire path.
public readonly record struct SourceFireBulletsInfo(
    int Shots,
    Vector3 OriginMeters,
    Vector3 Direction,
    Vector3 Spread,
    float DistanceMeters,
    int AmmoType,
    int TracerFrequency = 4,
    float Damage = 0f,
    int PlayerDamage = 0,
    SourceFireBulletsFlags Flags = SourceFireBulletsFlags.None,
    float DamageForceScale = 1f,
    bool PrimaryAttack = true,
    int DamageType = 0);

public readonly record struct SourceHitMetadata(
    SourceHitGroup HitGroup = SourceHitGroup.Generic,
    int Hitbox = -1,
    int PhysicsBone = -1,
    SourceContents Contents = SourceContents.Solid);

public readonly record struct SourceFireBulletsImpact(
    int ShotIndex,
    HitscanHit Hit,
    SourceHitMetadata Metadata,
    float Damage,
    int AmmoType,
    int DamageType,
    SourceFireBulletsFlags Flags,
    float DamageForceScale,
    bool IsPlayerTarget,
    bool IsTracer,
    bool PrimaryAttack = true,
    bool HitWater = false,
    bool SuppressSurfaceImpact = false,
    bool DamageSuppressed = false,
    Vector3 DamageForce = default,
    Vector3 TracerDestination = default);

public readonly record struct SourceGlassPenetrationResult(
    bool PassedThrough,
    HitscanHit Entry,
    HitscanHit Exit,
    HitscanHit? ContinuationHit);
