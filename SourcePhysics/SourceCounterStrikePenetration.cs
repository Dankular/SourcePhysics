namespace SourcePhysics;

public enum SourceBulletMaterial
{
    Default,
    Metal,
    Dirt,
    Concrete,
    Grate,
    Vent,
    Tile,
    Computer,
    Wood
}

public readonly record struct SourceBulletTypeParameters(float PenetrationPower, float PenetrationDistance,
    float PhysicsForceImpulse = 0f);

public readonly record struct SourcePenetrationState(
    float Damage,
    float PenetrationPower,
    float CurrentDistance,
    int PenetrationsRemaining);

public readonly record struct SourcePenetrationResult(
    SourcePenetrationState State,
    float PenetrationModifier,
    float DamageModifier,
    bool HitGrate,
    bool Success);

public readonly record struct SourceCounterStrikeImpact(HitscanHit Hit, bool Exit, float Damage);

public sealed record SourceCounterStrikeTraceResult(
    IReadOnlyList<SourceCounterStrikeImpact> Impacts,
    SourcePenetrationState State,
    bool StoppedByPenetration);

/// Pure port of the Counter-Strike FireBullet penetration tables and state transition.
/// Geometry tracing and entity damage dispatch remain outside this policy.
public static class SourceCounterStrikePenetration
{
    public static SourceBulletTypeParameters GetBulletTypeParameters(string ammoTypeName) => ammoTypeName switch
    {
        "BULLET_PLAYER_50AE" => new(30f, 1000f, 2400f),
        "BULLET_PLAYER_762MM" => new(39f, 5000f, 2400f),
        "BULLET_PLAYER_556MM" => new(35f, 4000f, 2400f),
        "BULLET_PLAYER_556MM_BOX" => new(35f, 4000f, 2400f),
        "BULLET_PLAYER_338MAG" => new(45f, 8000f, 2800f),
        "BULLET_PLAYER_9MM" => new(21f, 800f, 2000f),
        "BULLET_PLAYER_BUCKSHOT" => new(0f, 0f, 600f),
        "BULLET_PLAYER_45ACP" => new(15f, 500f, 2100f),
        "BULLET_PLAYER_357SIG" => new(25f, 800f, 2000f),
        "BULLET_PLAYER_57MM" => new(30f, 2000f, 2000f),
        _ => throw new ArgumentOutOfRangeException(nameof(ammoTypeName), "Unknown Source bullet type name.")
    };

    public static (float PenetrationModifier, float DamageModifier) GetMaterialParameters(SourceBulletMaterial material) => material switch
    {
        SourceBulletMaterial.Metal => (0.5f, 0.3f),
        SourceBulletMaterial.Dirt => (0.5f, 0.3f),
        SourceBulletMaterial.Concrete => (0.4f, 0.25f),
        SourceBulletMaterial.Grate => (1f, 0.99f),
        SourceBulletMaterial.Vent => (0.5f, 0.45f),
        SourceBulletMaterial.Tile => (0.65f, 0.3f),
        SourceBulletMaterial.Computer => (0.4f, 0.45f),
        SourceBulletMaterial.Wood => (1f, 0.6f),
        _ => (1f, 0.5f)
    };

    public static SourcePenetrationResult TryPenetrate(in SourcePenetrationState state,
        SourceBulletMaterial enterMaterial, SourceBulletMaterial exitMaterial,
        float thicknessInches, bool enterIsGrate, float rangeModifier)
    {
        if (!float.IsFinite(thicknessInches) || thicknessInches < 0f)
            throw new ArgumentOutOfRangeException(nameof(thicknessInches));
        if (!float.IsFinite(rangeModifier) || rangeModifier < 0f)
            throw new ArgumentOutOfRangeException(nameof(rangeModifier));
        if (state.PenetrationsRemaining == 0 || state.PenetrationPower <= 0f)
            return new(state, 0f, 0f, enterIsGrate, false);

        var (penetrationModifier, damageModifier) = GetMaterialParameters(enterMaterial);
        var hitGrate = enterIsGrate;
        if (hitGrate) (penetrationModifier, damageModifier) = (1f, 0.99f);
        if (!hitGrate && enterMaterial == exitMaterial &&
            (enterMaterial == SourceBulletMaterial.Wood || enterMaterial == SourceBulletMaterial.Metal))
            penetrationModifier *= 2f;
        var distance = state.CurrentDistance + thicknessInches;
        var damage = state.Damage * MathF.Pow(rangeModifier, distance / 500f);
        if (thicknessInches > state.PenetrationPower * penetrationModifier)
            return new(state with { Damage = damage, CurrentDistance = distance }, penetrationModifier, damageModifier, hitGrate, false);
        var next = state with
        {
            Damage = damage * damageModifier,
            PenetrationPower = state.PenetrationPower - thicknessInches / penetrationModifier,
            CurrentDistance = distance,
            PenetrationsRemaining = state.PenetrationsRemaining - 1
        };
        return new(next, penetrationModifier, damageModifier, hitGrate, true);
    }
}
