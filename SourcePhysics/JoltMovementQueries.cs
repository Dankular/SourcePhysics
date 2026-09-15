using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

/// Bridges Source-style player sweeps to Jolt's narrow phase without delegating movement policy.
public sealed class JoltMovementQueries : ISourceMovementQueries, IDisposable
{
    private readonly JoltPhysicsHost host;
    public SourceMovementVolumes Volumes { get; }
    private readonly BoxShape standingShape;
    private readonly BoxShape crouchedShape;
    private readonly List<ShapeCastResult> hits = new();
    private readonly List<CollideShapeResult> overlaps = new();
    private readonly MovementBodyFilter bodyFilter;
    private readonly float queryRecoveryDistance;
    private readonly float standableNormalY;
    private readonly ShapeCastSettings castSettings;

    public JoltMovementQueries(JoltPhysicsHost host, SourceMovementProfile? movementProfile = null)
    {
        this.host = host;
        var profile = movementProfile ?? new SourceMovementProfile();
        Volumes = new SourceMovementVolumes(profile.LadderFacingDotThreshold, profile);
        if (profile.QueryRecoveryDistanceSourceUnits < 0f || !float.IsFinite(profile.QueryRecoveryDistanceSourceUnits))
            throw new ArgumentOutOfRangeException(nameof(movementProfile), "Query recovery distance must be finite and non-negative.");
        queryRecoveryDistance = SourceUnits.ToMeters(profile.QueryRecoveryDistanceSourceUnits);
        standableNormalY = profile.StandableNormalZ;
        standingShape = new BoxShape(profile.StandingHalfExtents, 0.001f);
        crouchedShape = new BoxShape(profile.CrouchedHalfExtents, 0.001f);
        bodyFilter = new MovementBodyFilter(host);
        castSettings = new ShapeCastSettings
        {
            BackFaceModeTriangles = BackFaceMode.CollideWithBackFaces,
            BackFaceModeConvex = BackFaceMode.CollideWithBackFaces
        };
    }

    public MovementContact SweepPlayer(Vector3 start, Vector3 end, bool crouched)
    {
        var requestedEnd = end;
        var shape = crouched ? crouchedShape : standingShape;
        var direction = end - start;
        var centerOffset = crouched ? crouchedShape.HalfExtent.Y : standingShape.HalfExtent.Y;
        RMatrix4x4 transform = Matrix4x4.CreateTranslation(start + Vector3.UnitY * centerOffset);
        RVector3 baseOffset = RVector3.Zero;
        hits.Clear();
        host.NarrowPhase.CastShape(shape, transform, direction, castSettings, baseOffset, CollisionCollectorType.AllHitSorted,
            hits, null!, null!, bodyFilter, null!);
        if (hits.Count == 0 && direction.Y < -1e-5f && queryRecoveryDistance > 0f)
        {
            // Jolt can omit a contact when the cast starts exactly touching a
            // floor, while Source's downward TryTouchGround trace reports it.
            // Lift only downward support casts by the authored recovery
            // distance and preserve the recovered coordinate frame below.
            start += Vector3.UnitY * queryRecoveryDistance;
            end += Vector3.UnitY * queryRecoveryDistance;
            transform = Matrix4x4.CreateTranslation(start + Vector3.UnitY * centerOffset);
            hits.Clear();
            host.NarrowPhase.CastShape(shape, transform, end - start, castSettings, baseOffset,
                CollisionCollectorType.AllHitSorted, hits, null!, null!, bodyFilter, null!);
        }
        if (hits.Count == 0) return new(requestedEnd, Vector3.UnitY, 1f, -1, 1f, 0f);
        var hitIndex = hits.FindIndex(candidate => !host.IsSensor(candidate.BodyID2));
        if (hitIndex < 0) return new(requestedEnd, Vector3.UnitY, 1f, -1, 1f, 0f);
        var hit = hits[hitIndex];
        var touchingThreshold = queryRecoveryDistance;
        if (hit.Fraction <= 0f && hit.PenetrationDepth <= touchingThreshold)
        {
            var initialNormal = hit.PenetrationAxis.LengthSquared() > 1e-8f
                ? Vector3.Normalize(-hit.PenetrationAxis) : Vector3.UnitY;
            // A downward trace that begins exactly on a standable floor is a
            // valid Source support contact, not an embedded convex-edge case.
            // Preserve it so CategorizePosition can keep the player grounded.
            // Horizontal casts still use recovery below; otherwise a touching
            // floor would become a zero-time wall and consume all bumps.
            if (initialNormal.Y >= standableNormalY && Vector3.Dot(direction, initialNormal) < -1e-5f)
            {
                var supportSurface = host.GetBodySurface(hit.BodyID2, hit.SubShapeID2, out var supportSurfaceId);
                return new(start, initialNormal, 0f, unchecked((int)hit.BodyID2.ID),
                    supportSurface.Friction, supportSurface.Elasticity, false, false,
                    supportSurfaceId, hit.PenetrationDepth);
            }
            // A box at exact contact can report a zero-time edge axis from Jolt's
            // convex radius. Move the query origin by one radius and recast so the
            // Source ground categorizer receives the actual supporting plane.
            var recovery = queryRecoveryDistance;
            var recoveredStart = start + Vector3.UnitY * recovery;
            var recoveredEnd = end + Vector3.UnitY * recovery;
            transform = Matrix4x4.CreateTranslation(recoveredStart + Vector3.UnitY * centerOffset);
            hits.Clear();
            host.NarrowPhase.CastShape(shape, transform, recoveredEnd - recoveredStart, castSettings, baseOffset,
                CollisionCollectorType.AllHitSorted, hits, null!, null!, bodyFilter, null!);
            if (hits.Count > 0)
            {
                var recoveredHit = hits[0];
                var recoveredNormal = recoveredHit.PenetrationAxis.LengthSquared() > 1e-8f
                    ? Vector3.Normalize(-recoveredHit.PenetrationAxis) : Vector3.UnitY;
                var recoveredDirection = recoveredEnd - recoveredStart;
                if (recoveredHit.Fraction <= 0f && recoveredHit.PenetrationDepth <= touchingThreshold &&
                    Vector3.Dot(recoveredDirection, recoveredNormal) >= -1e-5f)
                    return new(requestedEnd, Vector3.UnitY, 1f, -1, 1f, 0f);
                hit = recoveredHit;
                start = recoveredStart;
                end = recoveredEnd;
            }
        }
        var normal = hit.PenetrationAxis.LengthSquared() > 1e-8f ? Vector3.Normalize(-hit.PenetrationAxis) : Vector3.UnitY;
        var bodyId = unchecked((int)hit.BodyID2.ID);
        var surface = host.GetBodySurface(hit.BodyID2, hit.SubShapeID2, out var surfaceId);
        var penetrating = hit.Fraction <= 0f && hit.PenetrationDepth > touchingThreshold;
        return new(Vector3.Lerp(start, end, hit.Fraction), normal, hit.Fraction, bodyId, surface.Friction, surface.Elasticity,
            penetrating, penetrating, surfaceId, hit.PenetrationDepth);
    }

    public bool IsEmpty(Vector3 position, bool crouched)
    {
        var shape = crouched ? crouchedShape : standingShape;
        var centerOffset = crouched ? crouchedShape.HalfExtent.Y : standingShape.HalfExtent.Y;
        RMatrix4x4 transform = Matrix4x4.CreateTranslation(position + Vector3.UnitY * centerOffset);
        RVector3 baseOffset = RVector3.Zero;
        overlaps.Clear();
        if (!host.NarrowPhase.CollideShape(shape, Vector3.One, transform, baseOffset,
            CollisionCollectorType.AllHitSorted, overlaps, null!, null!, bodyFilter, null!)) return true;
        return !overlaps.Any(overlap => !host.IsSensor(overlap.BodyID2));
    }

    public Vector3 GetBodyPointVelocity(int bodyId, Vector3 worldPoint)
    {
        if (bodyId < 0) return Vector3.Zero;
        var id = new BodyID(unchecked((uint)bodyId));
        host.Bodies.GetPointVelocity(in id, in worldPoint, out var velocity);
        if (velocity.LengthSquared() < 1e-12f)
        {
            var angular = host.Bodies.GetAngularVelocity(id);
            var center = (Vector3)host.Bodies.GetRCenterOfMassPosition(id);
            velocity = host.Bodies.GetLinearVelocity(id) + Vector3.Cross(angular, worldPoint - center);
        }
        return velocity;
    }

    public void ApplyCharacterImpulse(int bodyId, Vector3 point, Vector3 impulse)
    {
        if (bodyId < 0) return;
        var id = new BodyID(unchecked((uint)bodyId));
        host.AddImpulseAtPoint(id, impulse, point);
    }

    public SourceSurface GetSurface(int surfaceId) => host.Surfaces.Get(surfaceId);
    public bool TryWaterJump(Vector3 position, Vector3 direction, out Vector3 velocity, out float durationSeconds) =>
        Volumes.TryWaterJump(position, direction, out velocity, out durationSeconds);

    public SourceWaterLevel GetWaterLevel(Vector3 position, bool crouched) => Volumes.GetWaterLevel(position, crouched);
    public Vector3 GetWaterBaseVelocity(Vector3 position, SourceWaterLevel waterLevel) =>
        Volumes.GetWaterBaseVelocity(position, waterLevel);
    public Vector3 GetWaterBaseVelocity(Vector3 position, SourceWaterLevel waterLevel, bool crouched) =>
        Volumes.GetWaterBaseVelocity(position, waterLevel, crouched);

    public bool TryLadder(Vector3 position, Vector3 direction, out Vector3 normal, out int bodyId)
    {
        return Volumes.TryLadder(position, direction, out normal, out bodyId);
    }

    public void Dispose()
    {
        bodyFilter.Dispose(); standingShape.Dispose(); crouchedShape.Dispose();
    }

    private sealed class MovementBodyFilter : BodyFilter
    {
        private readonly JoltPhysicsHost host;
        public MovementBodyFilter(JoltPhysicsHost host) => this.host = host;
        protected override bool ShouldCollide(BodyID bodyId) => !host.IsSensor(bodyId) &&
            (host.GetBodyContents(bodyId) & SourceContents.MaskPlayerSolid) != 0;
        protected override bool ShouldCollideLocked(Body body) => !host.IsSensor(body.ID) &&
            (host.GetBodyContents(body.ID) & SourceContents.MaskPlayerSolid) != 0;
    }
}
