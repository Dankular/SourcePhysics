using System.Numerics;

namespace SourcePhysics;

/// The portable fields needed by Source's TraceAttack/CMultiDamage path.
/// Damage force is deliberately supplied by the caller after the title's ammo
/// force law has been applied; this layer never invents it.
public readonly record struct SourceDamageInfo(
    float Damage,
    float MaxDamage,
    Vector3 DamageForce,
    Vector3 DamagePosition,
    Vector3 ReportedPosition,
    int DamageType,
    int AmmoType,
    int DamageCustom = 0,
    int PlayerPenetrationCount = 0);

public interface ISourceDamageTarget
{
    void TraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit);
    void TakeDamage(in SourceDamageInfo info);
}

/// Optional Source target hook for TraceAttack implementations that alter the
/// damage payload before it is accumulated by CMultiDamage (armor, hit-group,
/// team and title-specific target rules).
public interface ISourceDamageTargetAdjustment
{
    SourceDamageInfo AdjustTraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit);
}

/// Source's global multi-damage accumulator, expressed without global mutable
/// state. Target changes flush the previous target before starting the next.
public sealed class SourceMultiDamageAccumulator
{
    private int targetId = -1;
    private ISourceDamageTarget? target;
    private SourceDamageInfo accumulated;
    private bool hasDamage;

    public void DispatchTraceAttack(int targetId, ISourceDamageTarget target,
        in SourceDamageInfo info, Vector3 direction, in HitscanHit hit)
    {
        ArgumentNullException.ThrowIfNull(target);
        var adjusted = target is ISourceDamageTargetAdjustment adjustment
            ? adjustment.AdjustTraceAttack(info, direction, hit)
            : info;
        if (!float.IsFinite(adjusted.Damage) || adjusted.Damage < 0f ||
            !float.IsFinite(adjusted.MaxDamage) || adjusted.MaxDamage < 0f)
            throw new InvalidOperationException("TraceAttack adjustment returned invalid damage.");
        target.TraceAttack(adjusted, direction, hit);
        AddMultiDamage(targetId, target, adjusted);
    }

    public void AddMultiDamage(int targetId, ISourceDamageTarget target, in SourceDamageInfo info)
    {
        ArgumentNullException.ThrowIfNull(target);
        if (hasDamage && this.targetId != targetId) ApplyMultiDamage();
        if (!hasDamage)
        {
            this.targetId = targetId;
            this.target = target;
            accumulated = info;
            hasDamage = true;
            return;
        }

        accumulated = accumulated with
        {
            Damage = accumulated.Damage + info.Damage,
            MaxDamage = MathF.Max(accumulated.MaxDamage, info.MaxDamage),
            DamageForce = accumulated.DamageForce + info.DamageForce,
            DamagePosition = info.DamagePosition,
            ReportedPosition = info.ReportedPosition,
            DamageType = accumulated.DamageType | info.DamageType,
            AmmoType = info.AmmoType,
            PlayerPenetrationCount = accumulated.PlayerPenetrationCount == 0
                ? info.PlayerPenetrationCount
                : accumulated.PlayerPenetrationCount
        };
    }

    public void ApplyMultiDamage()
    {
        if (!hasDamage) return;
        ApplyMultiDamageCore(target!);
    }

    private void ApplyMultiDamageCore(ISourceDamageTarget target)
    {
        ArgumentNullException.ThrowIfNull(target);
        if (!hasDamage) return;
        target.TakeDamage(accumulated);
        targetId = -1;
        this.target = null;
        accumulated = default;
        hasDamage = false;
    }

    public void Flush(Func<int, ISourceDamageTarget?> targetResolver)
    {
        ArgumentNullException.ThrowIfNull(targetResolver);
        if (!hasDamage) return;
        var target = targetResolver(targetId) ?? throw new InvalidOperationException($"Damage target {targetId} is unavailable.");
        if (!ReferenceEquals(target, this.target))
            throw new InvalidOperationException($"Damage target {targetId} changed before ApplyMultiDamage.");
        ApplyMultiDamageCore(target);
    }
}
