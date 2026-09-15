using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

public readonly record struct HitscanHit(Vector3 Position, Vector3 Normal, int BodyId, int SurfaceId, float Fraction,
    SourceHitGroup HitGroup = SourceHitGroup.Generic, int Hitbox = -1, int PhysicsBone = -1,
    SourceContents Contents = SourceContents.Solid);

/// Direct Source-style ray query backed by Jolt's narrow phase. Damage, penetration,
/// lag compensation and weapon spread remain higher-level policy.
public sealed class JoltHitscanQueries : IDisposable
{
    private readonly JoltPhysicsHost host;
    private readonly bool includeSensors;
    private readonly SourceContents contentsMask;
    private sealed class AllBroadPhaseFilter : BroadPhaseLayerFilter { protected override bool ShouldCollide(BroadPhaseLayer layer) => true; }
    private sealed class AllObjectLayerFilter : ObjectLayerFilter { protected override bool ShouldCollide(ObjectLayer layer) => true; }
    private sealed class AllBodyFilter : BodyFilter
    {
        private readonly JoltPhysicsHost host;
        private readonly bool includeSensors;
        private readonly SourceContents contentsMask;
        private readonly int ignoredBodyId;

        public AllBodyFilter(JoltPhysicsHost host, bool includeSensors, SourceContents contentsMask, int ignoredBodyId)
        {
            this.host = host;
            this.includeSensors = includeSensors;
            this.contentsMask = contentsMask;
            this.ignoredBodyId = ignoredBodyId;
        }

        protected override bool ShouldCollide(BodyID bodyId) => bodyId.ID != unchecked((uint)ignoredBodyId) &&
            (includeSensors || !host.IsSensor(bodyId)) &&
            (host.IsSolidBody(bodyId) || (includeSensors && host.IsSensor(bodyId))) &&
            (host.GetBodyContents(bodyId) & contentsMask) != 0;
        protected override bool ShouldCollideLocked(Body body) => body.ID.ID != unchecked((uint)ignoredBodyId) &&
            (includeSensors || !host.IsSensor(body.ID)) &&
            (host.IsSolidBody(body.ID) || (includeSensors && host.IsSensor(body.ID))) &&
            (host.GetBodyContents(body.ID) & contentsMask) != 0;
    }

    public JoltHitscanQueries(JoltPhysicsHost host, bool includeSensors = false,
        SourceContents contentsMask = SourceContents.MaskShot, int ignoredBodyId = -1)
    {
        this.host = host;
        this.includeSensors = includeSensors;
        this.contentsMask = contentsMask;
        this.ignoredBodyId = ignoredBodyId;
    }

    private readonly int ignoredBodyId;

    public bool Cast(Vector3 start, Vector3 direction, float distance, out HitscanHit hit)
    {
        if (direction.LengthSquared() < 1e-12f || distance <= 0f)
        {
            hit = default;
            return false;
        }

        direction = Vector3.Normalize(direction);
        RVector3 origin = start;
        var rayDirection = direction * distance;
        var result = default(RayCastResult);
        using var broadPhaseFilter = new AllBroadPhaseFilter();
        using var objectLayerFilter = new AllObjectLayerFilter();
        using var bodyFilter = new AllBodyFilter(host, includeSensors, contentsMask, ignoredBodyId);
        if (!host.NarrowPhase.CastRay(in origin, in rayDirection, out result, broadPhaseFilter, objectLayerFilter, bodyFilter))
        {
            hit = default;
            return false;
        }

        hit = ResolveHit(start, direction, distance, result);
        return true;
    }

    /// Returns all ordered ray intersections, preserving Jolt's sub-shape IDs for penetration and
    /// Source surface/material resolution. This is the ray equivalent of Source's repeated trace loop.
    public IReadOnlyList<HitscanHit> CastAll(Vector3 start, Vector3 direction, float distance)
    {
        if (direction.LengthSquared() < 1e-12f || distance <= 0f)
            return Array.Empty<HitscanHit>();
        direction = Vector3.Normalize(direction);
        RVector3 origin = start;
        var rayDirection = direction * distance;
        var results = new List<RayCastResult>();
        using var broadPhaseFilter = new AllBroadPhaseFilter();
        using var objectLayerFilter = new AllObjectLayerFilter();
        using var bodyFilter = new AllBodyFilter(host, includeSensors, contentsMask, ignoredBodyId);
        var settings = new RayCastSettings();
        host.NarrowPhase.CastRay(in origin, in rayDirection, settings, CollisionCollectorType.AllHitSorted,
            results, broadPhaseFilter, objectLayerFilter, bodyFilter);
        return results.Select(result => ResolveHit(start, direction, distance, result)).ToArray();
    }

    /// Returns only sensor bodies intersected by the ray, in Jolt's ordered
    /// hit order. Source FireBullets dispatches shot-responsive triggers before
    /// processing the first solid impact.
    public IReadOnlyList<HitscanHit> CastTriggers(Vector3 start, Vector3 direction, float distance)
    {
        return CastAll(start, direction, distance)
            .Where(hit => host.IsSensor(new BodyID(unchecked((uint)hit.BodyId))))
            .ToArray();
    }

    /// Implements the shared Source breakable-glass path: probe no farther
    /// than the authored 16 Source-unit glass depth, reverse-trace the exit,
    /// then perform one line trace from the exit. Entity/material eligibility
    /// stays with the caller because Source determines it from game data.
    public SourceGlassPenetrationResult CastSourceGlass(
        Vector3 start,
        Vector3 direction,
        float distance,
        Func<HitscanHit, bool> canPenetrate,
        float maximumDepthSourceUnits = 16f)
    {
        ArgumentNullException.ThrowIfNull(canPenetrate);
        if (!float.IsFinite(maximumDepthSourceUnits) || maximumDepthSourceUnits <= 0f)
            throw new ArgumentOutOfRangeException(nameof(maximumDepthSourceUnits));
        if (!Cast(start, direction, distance, out var entry) || !canPenetrate(entry))
            return new(false, entry, default, null);

        var remaining = distance * (1f - entry.Fraction);
        var probeDepth = MathF.Min(remaining, SourceUnits.ToMeters(maximumDepthSourceUnits));
        if (!TryCastExit(entry, Vector3.Normalize(direction), probeDepth, out var exit))
            return new(false, entry, default, null);

        var epsilon = SourceUnits.ToMeters(0.001f);
        var continuationStart = exit.Position + Vector3.Normalize(direction) * epsilon;
        var continuationDistance = distance * (1f - entry.Fraction);
        var continuation = default(HitscanHit);
        var hasContinuation = continuationDistance > epsilon &&
            Cast(continuationStart, direction, continuationDistance, out continuation);
        return new(true, entry, exit, hasContinuation ? continuation : null);
    }

    /// Runs the ordered Jolt intersections through the referenced Counter-Strike entry/exit law.
    /// The caller supplies the authoritative Source material for each hit; no name-based inference is used.
    public SourceCounterStrikeTraceResult CastCounterStrikePenetrating(Vector3 start, Vector3 direction,
        float distance, string ammoTypeName, float damage, float rangeModifier, int penetrationCount,
        Func<HitscanHit, SourceBulletMaterial> materialResolver)
    {
        ArgumentNullException.ThrowIfNull(materialResolver);
        if (!float.IsFinite(damage) || damage < 0f) throw new ArgumentOutOfRangeException(nameof(damage));
        if (!float.IsFinite(rangeModifier) || rangeModifier < 0f) throw new ArgumentOutOfRangeException(nameof(rangeModifier));
        if (penetrationCount < 0) throw new ArgumentOutOfRangeException(nameof(penetrationCount));
        var parameters = SourceCounterStrikePenetration.GetBulletTypeParameters(ammoTypeName);
        var state = new SourcePenetrationState(damage, parameters.PenetrationPower, 0f, penetrationCount);
        if (direction.LengthSquared() < 1e-12f || distance <= 0f)
            return new(Array.Empty<SourceCounterStrikeImpact>(), state, false);
        direction = Vector3.Normalize(direction);
        var impacts = new List<SourceCounterStrikeImpact>();
        var stopped = false;
        var currentStart = start;
        var distanceTravelled = 0f;
        while (distanceTravelled < distance)
        {
            var remaining = distance - distanceTravelled;
            if (!Cast(currentStart, direction, remaining, out var entry)) break;
            var entryDistanceInches = SourceUnits.ToSource(distanceTravelled + entry.Fraction * remaining);
            state = state with
            {
                Damage = state.Damage * MathF.Pow(rangeModifier, entryDistanceInches / 500f),
                CurrentDistance = entryDistanceInches
            };
            impacts.Add(new(entry, false, state.Damage));
            if (state.PenetrationsRemaining <= 0)
            {
                stopped = true;
                break;
            }

            if (!TryCastExit(entry, direction, remaining * (1f - entry.Fraction), out var exit))
            {
                stopped = true;
                break;
            }
            var thicknessInches = SourceUnits.ToSource(Vector3.Distance(entry.Position, exit.Position));
            var penetration = SourceCounterStrikePenetration.TryPenetrate(in state,
                materialResolver(entry), materialResolver(exit), thicknessInches,
                (entry.Contents & SourceContents.Grate) != 0, 1f);
            if (!penetration.Success)
            {
                stopped = true;
                break;
            }
            state = penetration.State;
            impacts.Add(new(exit, true, state.Damage));
            distanceTravelled += Vector3.Distance(currentStart, exit.Position) + SourceUnits.ToMeters(0.001f);
            currentStart = exit.Position + direction * SourceUnits.ToMeters(0.001f);
        }
        return new(impacts, state, stopped);
    }

    private bool TryCastExit(HitscanHit entry, Vector3 direction, float remainingDistance, out HitscanHit exit)
    {
        var epsilon = SourceUnits.ToMeters(0.001f);
        if (remainingDistance <= epsilon)
        {
            exit = default;
            return false;
        }
        var bodyId = new BodyID(unchecked((uint)entry.BodyId));
        var shape = host.Bodies.GetShape(in bodyId);
        if (shape is null)
        {
            exit = default;
            return false;
        }
        var worldTransform = (Matrix4x4)host.Bodies.GetRCenterOfMassTransform(bodyId);
        if (!Matrix4x4.Invert(worldTransform, out var inverseTransform))
        {
            exit = default;
            return false;
        }
        var worldOrigin = entry.Position + direction * (remainingDistance - epsilon);
        var localOrigin = Vector3.Transform(worldOrigin, inverseTransform);
        var localDirection = Vector3.TransformNormal(-direction * (remainingDistance - epsilon), inverseTransform);
        var ray = new Ray(localOrigin, localDirection);
        var results = new List<RayCastResult>();
        var settings = new RayCastSettings();
        shape.CastRay(in ray, in settings, CollisionCollectorType.AllHitSorted, results);
        if (results.Count == 0)
        {
            exit = default;
            return false;
        }
        var result = results[0];
        result.BodyID = bodyId;
        exit = ResolveHit(worldOrigin, -direction, remainingDistance - epsilon, result);
        return true;
    }

    private HitscanHit ResolveHit(Vector3 start, Vector3 direction, float distance, RayCastResult result)
    {
        var bodyId = result.BodyID;
        var rayDirection = direction * distance;
        var point = start + rayDirection * result.Fraction;
        var normal = -direction;
        var lockRead = new BodyLockRead();
        var lockInterface = host.System.BodyLockInterfaceNoLock;
        lockInterface.LockRead(in bodyId, out lockRead);
        if (lockRead.Succeeded)
        {
            var subShape = (SubShapeID)result.subShapeID2;
            normal = lockRead.Body!.GetWorldSpaceSurfaceNormal(subShape, point);
            lockInterface.UnlockRead(in lockRead);
        }

        var surface = host.GetBodySurface(bodyId, (SubShapeID)result.subShapeID2, out var surfaceId);
        _ = surface;
        return new(point, normal, unchecked((int)bodyId.ID), surfaceId, result.Fraction,
            Contents: host.GetBodyContents(bodyId));
    }

    public void Dispose() { }
}
