using System.Numerics;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class SourceDamageAdjustmentTests
{
    [Fact]
    public void TraceAttackAdjustmentFeedsAdjustedPayloadIntoMultiDamage()
    {
        var target = new AdjustingTarget();
        var accumulator = new SourceMultiDamageAccumulator();
        var info = new SourceDamageInfo(40f, 40f, Vector3.UnitZ, Vector3.Zero,
            Vector3.Zero, 1, 2);

        accumulator.DispatchTraceAttack(7, target, info, Vector3.UnitZ, default);
        accumulator.ApplyMultiDamage();

        Assert.Equal(20f, target.TracedDamage);
        Assert.Equal(20f, target.AppliedDamage);
    }

    [Fact]
    public void InvalidTraceAttackAdjustmentIsRejectedBeforeAccumulation()
    {
        var accumulator = new SourceMultiDamageAccumulator();
        Assert.Throws<InvalidOperationException>(() => accumulator.DispatchTraceAttack(
            1, new InvalidAdjustingTarget(),
            new SourceDamageInfo(1f, 1f, default, default, default, 0, 0),
            Vector3.UnitZ, default));
    }

    [Fact]
    public void DirectMultiDamageRejectsInvalidForceAndPenetrationData()
    {
        var accumulator = new SourceMultiDamageAccumulator();
        var invalid = new SourceDamageInfo(1f, 1f, new Vector3(float.NaN, 0f, 0f),
            Vector3.Zero, Vector3.Zero, 0, 0, PlayerPenetrationCount: -1);

        Assert.Throws<InvalidOperationException>(() => accumulator.AddMultiDamage(
            1, new AdjustingTarget(), invalid));
    }

    private sealed class AdjustingTarget : ISourceDamageTarget, ISourceDamageTargetAdjustment
    {
        public float TracedDamage { get; private set; }
        public float AppliedDamage { get; private set; }
        public SourceDamageInfo AdjustTraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit) =>
            info with { Damage = info.Damage * 0.5f, MaxDamage = info.MaxDamage * 0.5f };
        public void TraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit) => TracedDamage = info.Damage;
        public void TakeDamage(in SourceDamageInfo info) => AppliedDamage = info.Damage;
    }

    private sealed class InvalidAdjustingTarget : ISourceDamageTarget, ISourceDamageTargetAdjustment
    {
        public SourceDamageInfo AdjustTraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit) =>
            info with { Damage = float.NaN };
        public void TraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit) { }
        public void TakeDamage(in SourceDamageInfo info) { }
    }
}
