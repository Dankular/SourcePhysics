using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

/// Narrow-phase bridge for projectile-sized swept volumes. The projectile motor
/// remains Source-authoritative; Jolt supplies only the continuous collision query.
public sealed class JoltProjectileQueries : IProjectileQueries, IProjectilePenetrationQueries, IDisposable
{
    private readonly JoltPhysicsHost host;
    private readonly Func<ProjectileHit, float, float>? penetrationCost;
    private readonly SourceContents contentsMask;
    private readonly BoxShape shape;
    private readonly List<ShapeCastResult> hits = new();
    private readonly AllBroadPhaseFilter broadPhaseFilter = new();
    private readonly AllObjectLayerFilter objectLayerFilter = new();
    private readonly ProjectileBodyFilter bodyFilter;

    public JoltProjectileQueries(JoltPhysicsHost host, float radiusSourceUnits = 1f, bool includeSensors = false,
        Func<ProjectileHit, float, float>? penetrationCost = null,
        SourceContents contentsMask = SourceContents.MaskShot)
    {
        if (radiusSourceUnits <= 0f) throw new ArgumentOutOfRangeException(nameof(radiusSourceUnits));
        this.host = host;
        this.penetrationCost = penetrationCost;
        this.contentsMask = contentsMask;
        var radius = SourceUnits.ToMeters(radiusSourceUnits);
        shape = new BoxShape(new Vector3(radius, radius, radius), 0.001f);
        bodyFilter = new ProjectileBodyFilter(host, includeSensors, contentsMask);
    }

    public bool Sweep(Vector3 start, Vector3 end, out ProjectileHit hit)
    {
        hits.Clear();
        var transform = (RMatrix4x4)Matrix4x4.CreateTranslation(start);
        host.NarrowPhase.CastShape(shape, transform, end - start, RVector3.Zero,
            CollisionCollectorType.AllHitSorted, hits, broadPhaseFilter, objectLayerFilter, bodyFilter, null!);
        if (hits.Count == 0)
        {
            hit = default;
            return false;
        }

        var result = hits[0];
        var normal = result.PenetrationAxis.LengthSquared() > 1e-8f
            ? Vector3.Normalize(-result.PenetrationAxis)
            : Vector3.UnitY;
        var surface = host.GetBodySurface(result.BodyID2, result.SubShapeID2, out var surfaceId);
        hit = new(Vector3.Lerp(start, end, result.Fraction), normal,
            unchecked((int)result.BodyID2.ID), surface.Elasticity, surfaceId, surface.ThicknessInches);
        return true;
    }

    public bool TryPenetrate(Vector3 entryPosition, in ProjectileHit entryHit, Vector3 incomingVelocity,
        float availablePower, out Vector3 exitPosition, out Vector3 exitVelocity, out float consumedPower)
    {
        exitPosition = entryPosition;
        exitVelocity = incomingVelocity;
        consumedPower = 0f;
        if (penetrationCost is null || availablePower <= 0f || entryHit.ThicknessInches <= 0f ||
            incomingVelocity.LengthSquared() < 1e-12f)
            return false;

        var requestedCost = penetrationCost(entryHit, availablePower);
        if (!float.IsFinite(requestedCost) || requestedCost <= 0f || requestedCost > availablePower)
            return false;
        var direction = Vector3.Normalize(incomingVelocity);
        exitPosition = entryPosition + direction * SourceUnits.ToMeters(entryHit.ThicknessInches);
        consumedPower = requestedCost;
        return true;
    }

    public void Dispose()
    {
        bodyFilter.Dispose();
        broadPhaseFilter.Dispose();
        objectLayerFilter.Dispose();
        shape.Dispose();
    }

    private sealed class AllBroadPhaseFilter : BroadPhaseLayerFilter
    {
        protected override bool ShouldCollide(BroadPhaseLayer layer) => true;
    }

    private sealed class AllObjectLayerFilter : ObjectLayerFilter
    {
        protected override bool ShouldCollide(ObjectLayer layer) => true;
    }

    private sealed class ProjectileBodyFilter : BodyFilter
    {
        private readonly JoltPhysicsHost host;
        private readonly bool includeSensors;
        private readonly SourceContents contentsMask;

        public ProjectileBodyFilter(JoltPhysicsHost host, bool includeSensors, SourceContents contentsMask)
        {
            this.host = host;
            this.includeSensors = includeSensors;
            this.contentsMask = contentsMask;
        }

        protected override bool ShouldCollide(BodyID bodyId) => (includeSensors || !host.IsSensor(bodyId)) &&
            (host.GetBodyContents(bodyId) & contentsMask) != 0;
        protected override bool ShouldCollideLocked(Body body) => (includeSensors || !host.IsSensor(body.ID)) &&
            (host.GetBodyContents(body.ID) & contentsMask) != 0;
    }
}
