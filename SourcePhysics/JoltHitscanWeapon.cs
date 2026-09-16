using Stride.Engine;
using System.Numerics;

namespace SourcePhysics;

/// Stride integration boundary for Source-style hitscan weapons.
/// Spread, damage, lag compensation and firing cadence remain explicit gameplay policy;
/// this component owns only the Jolt trace and hit notification.
public sealed class JoltHitscanWeapon : SyncScript
{
    public readonly record struct ShotResult(bool Hit, Vector3 Direction, HitscanHit HitData);
    public StrideSourcePhysicsScript PhysicsSystem { get; set; } = null!;
    public bool IncludeSensors { get; set; }
    /// Set by the title when this component represents a player shooter. Source
    /// uses the player-only alternating shotgun hull path in FireBullets.
    public bool ShooterIsPlayer { get; set; }
    /// Source entity identity excluded from every shot trace. -1 means none.
    public int ShooterBodyId { get; set; } = -1;
    public WeaponRecording? Recording { get; set; }
    public int RecordingTick { get; set; }
    public event Action<HitscanHit>? Hit;
    /// Raised for every shot-responsive trigger crossed before the solid hit.
    public event Action<HitscanHit>? TriggerHit;
    /// Caller-owned Source breakable-glass eligibility. A null resolver means
    /// ordinary hitscan behavior and never performs the glass refire path.
    public Func<HitscanHit, bool>? CanPenetrateGlass { get; set; }
    public event Action<HitscanHit>? GlassImpact;
    public Func<HitscanHit, SourceHitMetadata>? HitMetadataResolver { get; set; }
    public Func<HitscanHit, bool>? IsPlayerTarget { get; set; }
    public Func<int, HitscanHit, float>? DamageResolver { get; set; }
    /// Resolves the title AmmoDef::DamageForce value for an ammo index. The
    /// integer index is intentionally not mapped to a guessed ammo table.
    public Func<int, float>? BulletForceResolver { get; set; }
    /// Resolves the title AmmoDef fields consumed by CBaseEntity::FireBullets.
    /// A null resolver leaves the already-resolved fields on SourceFireBulletsInfo
    /// unchanged; no ammo-table defaults are inferred here.
    public Func<int, SourceAmmoDefinition?>? AmmoDefinitionResolver { get; set; }
    /// Title-owned equivalent of Pickup_ForcePlayerToDropThisObject. It is
    /// invoked only after a non-suppressed Source bullet impact whose AmmoDef
    /// carries AMMO_FORCE_DROP_IF_CARRIED.
    public Action<HitscanHit>? ForceDropIfCarried { get; set; }
    /// Resolves a traced body/hitbox to the title's damage target. When set,
    /// FireBullets executes Source TraceAttack/ApplyMultiDamage ordering.
    public Func<HitscanHit, ISourceDamageTarget?>? DamageTargetResolver { get; set; }
    /// Resolves the Source entity identity used by CMultiDamage. When omitted,
    /// the Jolt body ID is used as a conservative fallback.
    public Func<HitscanHit, int>? DamageTargetIdResolver { get; set; }
    /// Optional point-contents callback for shots that begin inside water.
    /// Water volumes are also queried through Jolt when this is not supplied.
    public Func<Vector3, bool>? IsWaterPoint { get; set; }
    /// Optional animated Source studio hitbox catalog. When present, the Jolt
    /// body hit is refined against the authored bone-relative boxes before any
    /// metadata or damage callback runs.
    public SourceHitboxCatalog? HitboxCatalog { get; set; }
    public event Action<SourceFireBulletsImpact>? Impact;
    private static int tracerCount;

    private JoltHitscanQueries? queries;
    private JoltHitscanQueries? waterQueries;
    private JoltHitscanQueries? triggerQueries;

    public override void Start()
    {
        if (PhysicsSystem is null) throw new InvalidOperationException("Assign PhysicsSystem before starting JoltHitscanWeapon.");
        PhysicsSystem.EnsureStarted();
        Initialize(PhysicsSystem.Host);
    }

    public void Initialize(JoltPhysicsHost host)
    {
        queries?.Dispose();
        queries = new JoltHitscanQueries(host, IncludeSensors, ignoredBodyId: ShooterBodyId);
        waterQueries?.Dispose();
        waterQueries = new JoltHitscanQueries(host, includeSensors: true, contentsMask: SourceContents.Water,
            ignoredBodyId: ShooterBodyId);
        triggerQueries?.Dispose();
        triggerQueries = new JoltHitscanQueries(host, includeSensors: true, contentsMask: SourceContents.MaskShot,
            ignoredBodyId: ShooterBodyId);
    }

    public bool Fire(Vector3 originMeters, Vector3 direction, float distanceMeters, out HitscanHit hit)
    {
        if (queries is null) throw new InvalidOperationException("The weapon must be started before firing.");
        var didHit = queries.Cast(originMeters, direction, distanceMeters, out hit);
        EmitTriggerHits(originMeters, direction, didHit ? distanceMeters * hit.Fraction : distanceMeters);
        if (!didHit) return false;
        RefineHitbox(originMeters, direction, distanceMeters, ref hit);
        Hit?.Invoke(hit);
        return true;
    }

    public SourceGlassPenetrationResult FireGlass(Vector3 originMeters, Vector3 direction, float distanceMeters)
    {
        if (queries is null) throw new InvalidOperationException("The weapon must be started before firing.");
        var result = queries.CastSourceGlass(originMeters, direction, distanceMeters,
            CanPenetrateGlass ?? (_ => false));
        if (result.Entry != default) GlassImpact?.Invoke(result.Entry);
        if (result.PassedThrough) GlassImpact?.Invoke(result.Exit);
        return result;
    }

    /// <summary>Fires Source-style spread shots using the caller's authoritative RNG.</summary>
    public IReadOnlyList<ShotResult> FireSpread(Vector3 originMeters, Vector3 direction, float distanceMeters,
        int shots, Vector3 spread, float bias, float shotBiasMin, float shotBiasMax,
        Func<float, float, float> randomFloat, bool firstShotAccurate = false)
    {
        if (queries is null) throw new InvalidOperationException("The weapon must be started before firing.");
        if (shots < 1) throw new ArgumentOutOfRangeException(nameof(shots));
        var manipulator = new SourceShotManipulator(direction);
        var results = new ShotResult[shots];
        for (var shot = 0; shot < shots; shot++)
        {
            var shotDirection = firstShotAccurate && shot == 0 && shots > 1
                ? manipulator.ShotDirection
                : manipulator.ApplySpread(spread, bias, shotBiasMin, shotBiasMax, randomFloat);
            var didHit = queries.Cast(originMeters, shotDirection, distanceMeters, out var hit);
            EmitTriggerHits(originMeters, shotDirection, didHit ? distanceMeters * hit.Fraction : distanceMeters);
            if (didHit) RefineHitbox(originMeters, shotDirection, distanceMeters, ref hit);
            results[shot] = new(didHit, shotDirection, hit);
            if (didHit) Hit?.Invoke(hit);
        }
        return results;
    }

    /// Executes the Source FireBullets field selection and shot/impact ordering. Target damage dispatch
    /// remains an event so the game's entity system can perform TraceAttack/ApplyMultiDamage semantics.
    public IReadOnlyList<ShotResult> FireBullets(in SourceFireBulletsInfo info, int sourceRandomSeed)
    {
        if (info.Shots < 1) throw new ArgumentOutOfRangeException(nameof(info), "Shots must be positive.");
        if (!IsFinite(info.OriginMeters) || !IsFinite(info.Direction) || !IsFinite(info.Spread))
            throw new ArgumentOutOfRangeException(nameof(info), "Origin, direction and spread must be finite.");
        if (!float.IsFinite(info.DistanceMeters) || info.DistanceMeters <= 0f)
            throw new ArgumentOutOfRangeException(nameof(info), "Distance must be finite and positive.");
        if (info.TracerFrequency < 0)
            throw new ArgumentOutOfRangeException(nameof(info), "Tracer frequency cannot be negative.");
        if (info.AdditionalIgnoreBodyId < -1 || info.InflictorBodyId < -1 ||
            info.AttackerBodyId < -1 || info.WeaponBodyId < -1)
            throw new ArgumentOutOfRangeException(nameof(info), "Source body identities must be -1 or non-negative.");
        if (!float.IsFinite(info.Damage) || info.Damage < 0f || info.PlayerDamage < 0)
            throw new ArgumentOutOfRangeException(nameof(info), "Damage must be non-negative and finite.");
        if (!float.IsFinite(info.DamageForceScale) || info.DamageForceScale < 0f)
            throw new ArgumentOutOfRangeException(nameof(info), "Damage force scale must be non-negative and finite.");
        var query = queries ?? throw new InvalidOperationException("The weapon must be started before firing.");
        var ammoDefinition = AmmoDefinitionResolver?.Invoke(info.AmmoType);
        var resolvedPlayerDamage = info.PlayerDamage;
        var resolvedDamageType = ammoDefinition?.DamageType ?? info.DamageType;
        if (ammoDefinition is { } definition && resolvedPlayerDamage == 0 &&
            definition.Flags.HasFlag(SourceAmmoFlags.InterpretPlayerDamageAsDamageToPlayer))
            resolvedPlayerDamage = definition.PlayerDamage;
        if (resolvedPlayerDamage < 0)
            throw new InvalidOperationException("AmmoDefinitionResolver returned a negative player damage.");
        var resolvedInfo = info with { PlayerDamage = resolvedPlayerDamage, DamageType = resolvedDamageType };
        var manipulator = new SourceShotManipulator(info.Direction);
        var results = new ShotResult[info.Shots];
        var multiDamage = new SourceMultiDamageAccumulator();
        for (var shot = 0; shot < results.Length; shot++)
        {
            var tracerIndex = tracerCount++;
            // CBaseEntity::FireBullets calls RandomSeed(iSeed) for every
            // pellet and increments iSeed after the shot. Preserve that
            // sequence instead of sharing one random stream across pellets.
            var shotSeed = unchecked((sourceRandomSeed + shot) & 255);
            var shotDirection = info.Flags.HasFlag(SourceFireBulletsFlags.FirstShotAccurate) && shot == 0 && info.Shots > 1
                ? manipulator.ShotDirection
                : manipulator.ApplySpread(info.Spread, 0f, 0f, 0f,
                    new SourceUniformRandomStream(shotSeed).RandomFloat);
            var useHull = ShooterIsPlayer && info.Shots > 1 && (shot & 1) != 0;
            var traceShape = useHull ? SourceShotTraceShape.PlayerAlternatingHull : SourceShotTraceShape.Ray;
            var additionalIgnore = info.AdditionalIgnoreBodyId >= 0
                ? info.AdditionalIgnoreBodyId : (int?)null;
            HitscanHit hit;
            var didHit = useHull
                ? query.CastHull(info.OriginMeters,
                    info.OriginMeters + shotDirection * info.DistanceMeters,
                    new Vector3(3f), out hit, additionalIgnore)
                : query.Cast(info.OriginMeters, shotDirection, info.DistanceMeters, out hit, additionalIgnore);
            EmitTriggerHits(info.OriginMeters, shotDirection,
                didHit ? info.DistanceMeters * hit.Fraction : info.DistanceMeters, additionalIgnore);
            if (didHit) RefineHitbox(info.OriginMeters, shotDirection, info.DistanceMeters, ref hit);
            results[shot] = new(didHit, shotDirection, hit);
            if (didHit) Hit?.Invoke(hit);
            var result = results[shot];
            if (!result.Hit)
            {
                Recording?.Capture(RecordingTick, shot, shotSeed, info.OriginMeters, result.Direction,
                    false, result.HitData, in resolvedInfo, null, traceShape);
                continue;
            }
            var startedInWater = IsWaterPoint?.Invoke(info.OriginMeters) ?? false;
            var waterImpact = default(HitscanHit);
            var waterHit = waterQueries?.Cast(info.OriginMeters, shotDirection, info.DistanceMeters, out waterImpact,
                additionalIgnore) == true &&
                (startedInWater || waterImpact.Fraction <= result.HitData.Fraction);
            var suppressDamage = waterHit && info.Flags.HasFlag(SourceFireBulletsFlags.DontHitUnderwater);
            var suppressImpact = waterHit && !startedInWater &&
                !info.Flags.HasFlag(SourceFireBulletsFlags.AllowWaterSurfaceImpacts);
            // HandleShotImpactingWater replaces the tracer endpoint with the
            // first water-entry trace when the shot began outside water.
            var tracerDestination = !startedInWater && waterHit
                ? waterImpact.Position
                : result.HitData.Position;
            var metadata = HitMetadataResolver?.Invoke(result.HitData) ?? new SourceHitMetadata(
                result.HitData.HitGroup, result.HitData.Hitbox, result.HitData.PhysicsBone,
                result.HitData.Contents);
            var isPlayer = IsPlayerTarget?.Invoke(result.HitData) ?? false;
            var damage = suppressDamage ? 0f : isPlayer && resolvedPlayerDamage != 0
                ? resolvedPlayerDamage
                : info.Damage != 0f
                    ? info.Damage
                    : DamageResolver?.Invoke(info.AmmoType, result.HitData) ?? 0f;
            if (!float.IsFinite(damage) || damage < 0f)
                throw new InvalidOperationException("DamageResolver returned an invalid damage value.");
            var bulletForce = BulletForceResolver?.Invoke(info.AmmoType) ?? 0f;
            if (!float.IsFinite(bulletForce) || bulletForce < 0f)
                throw new InvalidOperationException("BulletForceResolver returned an invalid force value.");
            var damageForce = suppressDamage ? Vector3.Zero :
                Vector3.Normalize(result.Direction) * bulletForce * info.DamageForceScale;
            var explicitDamage = !suppressDamage && (info.Damage != 0f || (isPlayer && resolvedPlayerDamage != 0));
            var actualDamageType = resolvedDamageType | (explicitDamage
                ? damage > 16f ? 1 << 13 : 1 << 12
                : 0);
            var impact = new SourceFireBulletsImpact(shot, result.HitData, metadata, damage, info.AmmoType,
                actualDamageType, info.Flags, info.DamageForceScale, isPlayer,
                info.TracerFrequency != 0 && tracerIndex % info.TracerFrequency == 0,
                info.PrimaryAttack, waterHit, suppressImpact, suppressDamage, damageForce,
                tracerDestination);
            if (!suppressDamage && DamageTargetResolver?.Invoke(result.HitData) is { } target)
            {
                var hitData = result.HitData;
                var inflictorId = info.InflictorBodyId >= 0 ? info.InflictorBodyId : -1;
                var attackerId = info.AttackerBodyId >= 0 ? info.AttackerBodyId : ShooterBodyId;
                var weaponId = info.WeaponBodyId >= 0 ? info.WeaponBodyId : -1;
                var damageInfo = new SourceDamageInfo(damage, damage, damageForce, result.HitData.Position,
                    info.OriginMeters, actualDamageType, info.AmmoType,
                    InflictorBodyId: inflictorId, AttackerBodyId: attackerId, WeaponBodyId: weaponId);
                var targetId = DamageTargetIdResolver?.Invoke(result.HitData) ?? result.HitData.BodyId;
                multiDamage.DispatchTraceAttack(targetId, target, in damageInfo,
                    result.Direction, in hitData);
            }
            if (!suppressDamage && ammoDefinition is { } resolvedAmmo &&
                resolvedAmmo.Flags.HasFlag(SourceAmmoFlags.ForceDropIfCarried))
                ForceDropIfCarried?.Invoke(result.HitData);
            Impact?.Invoke(impact);
            Recording?.Capture(RecordingTick, shot, shotSeed, info.OriginMeters, result.Direction,
                true, result.HitData, in resolvedInfo, impact, traceShape);
        }
        multiDamage.ApplyMultiDamage();
        if (Recording is not null) RecordingTick++;
        return results;
    }

    /// <summary>
    /// Uses Source's vstdlib random stream. Pass the authoritative prediction
    /// seed; player callers should pass the Source-masked seed when required by
    /// the title's FireBullets path.
    /// </summary>
    public IReadOnlyList<ShotResult> FireSpread(Vector3 originMeters, Vector3 direction, float distanceMeters,
        int shots, Vector3 spread, float bias, float shotBiasMin, float shotBiasMax, int sourceRandomSeed,
        bool firstShotAccurate = false, bool maskSeedToPlayerByte = true)
    {
        if (shots < 1) throw new ArgumentOutOfRangeException(nameof(shots));
        var manipulator = new SourceShotManipulator(direction);
        var results = new ShotResult[shots];
        for (var shot = 0; shot < shots; shot++)
        {
            var seed = unchecked(sourceRandomSeed + shot);
            if (maskSeedToPlayerByte) seed &= 255;
            var random = new SourceUniformRandomStream(seed);
            var shotDirection = firstShotAccurate && shot == 0 && shots > 1
                ? manipulator.ShotDirection
                : manipulator.ApplySpread(spread, bias, shotBiasMin, shotBiasMax, random.RandomFloat);
            var didHit = queries!.Cast(originMeters, shotDirection, distanceMeters, out var hit);
            EmitTriggerHits(originMeters, shotDirection, didHit ? distanceMeters * hit.Fraction : distanceMeters);
            if (didHit) RefineHitbox(originMeters, shotDirection, distanceMeters, ref hit);
            results[shot] = new(didHit, shotDirection, hit);
            if (didHit) Hit?.Invoke(hit);
        }
        if (Recording is not null)
        {
            for (var index = 0; index < results.Length; index++)
            {
                var result = results[index];
                var shotSeed = unchecked(sourceRandomSeed + index);
                if (maskSeedToPlayerByte) shotSeed &= 255;
                Recording.Capture(RecordingTick, index, shotSeed, originMeters, result.Direction,
                    result.Hit, result.HitData);
            }
            RecordingTick++;
        }
        return results;
    }

    public override void Update() { }

    public override void Cancel()
    {
        queries?.Dispose();
        waterQueries?.Dispose();
        triggerQueries?.Dispose();
        queries = null;
        waterQueries = null;
        triggerQueries = null;
    }

    private void RefineHitbox(Vector3 origin, Vector3 direction, float distance, ref HitscanHit hit)
    {
        if (HitboxCatalog is not null) HitboxCatalog.TryResolve(in hit, origin, direction, distance, out hit);
    }

    private void EmitTriggerHits(Vector3 origin, Vector3 direction, float distance,
        int? ignoredBodyIdOverride = null)
    {
        if (triggerQueries is null) return;
        foreach (var trigger in triggerQueries.CastTriggers(origin, direction, distance, ignoredBodyIdOverride))
            TriggerHit?.Invoke(trigger);
    }

    private static bool IsFinite(Vector3 value) =>
        float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);
}
