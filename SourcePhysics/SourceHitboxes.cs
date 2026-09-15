using System.Numerics;
using System.Text.Json;

namespace SourcePhysics;

/// A Source studio hitbox after its explicit Source-unit to metre conversion.
/// BoneToBodyMeters contains the animated bone transform relative to the
/// physics body; bounds are the original mstudiobbox_t bbmin/bbmax values.
public readonly record struct SourceHitboxDefinition(
    int Hitbox,
    SourceHitGroup HitGroup,
    int PhysicsBone,
    Vector3 BoundsMinMeters,
    Vector3 BoundsMaxMeters,
    Matrix4x4 BoneToBodyMeters)
{
    public static SourceHitboxDefinition FromSourceUnits(int hitbox, SourceHitGroup hitGroup,
        int physicsBone, Vector3 boundsMinSourceUnits, Vector3 boundsMaxSourceUnits,
        Matrix4x4 boneToBodySourceUnits)
    {
        var converted = boneToBodySourceUnits;
        converted.Translation = SourceUnits.ToMeters(converted.Translation);
        return new(hitbox, hitGroup, physicsBone, SourceUnits.ToMeters(boundsMinSourceUnits),
            SourceUnits.ToMeters(boundsMaxSourceUnits), converted);
    }
}

public readonly record struct SourceHitboxBodyManifest(int BodyId, Matrix4x4 BodyToWorld,
    SourceHitboxDefinition[] Definitions);

/// Authored animated hitboxes associated with one Jolt body. The body transform
/// is updated by the animation/pose system at the same fixed tick as tracing.
public sealed class SourceHitboxCatalog
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, IncludeFields = true };
    private sealed record BodyHitboxes(Matrix4x4 BodyToWorld, SourceHitboxDefinition[] Definitions);
    private readonly Dictionary<int, BodyHitboxes> bodies = new();

    public void Register(int bodyId, Matrix4x4 bodyToWorld, IReadOnlyList<SourceHitboxDefinition> definitions)
    {
        ArgumentNullException.ThrowIfNull(definitions);
        if (definitions.Count == 0) throw new ArgumentException("At least one hitbox is required.", nameof(definitions));
        var copy = definitions.ToArray();
        foreach (var definition in copy)
        {
            if (definition.Hitbox < 0 || definition.PhysicsBone < -1)
                throw new ArgumentException("Hitbox and physics-bone indices are invalid.", nameof(definitions));
            if (definition.BoundsMinMeters.X > definition.BoundsMaxMeters.X ||
                definition.BoundsMinMeters.Y > definition.BoundsMaxMeters.Y ||
                definition.BoundsMinMeters.Z > definition.BoundsMaxMeters.Z)
                throw new ArgumentException("Hitbox bounds must be ordered min-to-max.", nameof(definitions));
        }
        bodies[bodyId] = new(bodyToWorld, copy);
    }

    public bool UpdateBodyTransform(int bodyId, Matrix4x4 bodyToWorld)
    {
        if (!bodies.TryGetValue(bodyId, out var body)) return false;
        bodies[bodyId] = body with { BodyToWorld = bodyToWorld };
        return true;
    }

    public bool UpdateHitboxTransform(int bodyId, int hitbox, Matrix4x4 boneToBodyMeters)
    {
        if (!bodies.TryGetValue(bodyId, out var body)) return false;
        var definitions = body.Definitions.ToArray();
        var index = Array.FindIndex(definitions, definition => definition.Hitbox == hitbox);
        if (index < 0) return false;
        definitions[index] = definitions[index] with { BoneToBodyMeters = boneToBodyMeters };
        bodies[bodyId] = body with { Definitions = definitions };
        return true;
    }

    /// Atomically publishes the body transform and all animated hitbox-bone
    /// transforms for one authoritative simulation tick.
    public bool UpdatePose(int bodyId, Matrix4x4 bodyToWorld,
        IReadOnlyDictionary<int, Matrix4x4> hitboxTransforms)
    {
        ArgumentNullException.ThrowIfNull(hitboxTransforms);
        if (!bodies.TryGetValue(bodyId, out var body)) return false;
        foreach (var definition in body.Definitions)
        {
            if (!hitboxTransforms.TryGetValue(definition.Hitbox, out var transform)) return false;
            if (!IsFinite(transform)) throw new ArgumentException("Hitbox pose contains a non-finite transform.", nameof(hitboxTransforms));
        }
        var definitions = body.Definitions
            .Select(definition => definition with { BoneToBodyMeters = hitboxTransforms[definition.Hitbox] })
            .ToArray();
        bodies[bodyId] = new BodyHitboxes(bodyToWorld, definitions);
        return true;
    }

    public bool Remove(int bodyId) => bodies.Remove(bodyId);

    public SourceHitboxBodyManifest[] CaptureManifest() => bodies
        .OrderBy(pair => pair.Key)
        .Select(pair => new SourceHitboxBodyManifest(pair.Key, pair.Value.BodyToWorld,
            pair.Value.Definitions.ToArray()))
        .ToArray();

    public string ToJson() => JsonSerializer.Serialize(CaptureManifest(), JsonOptions);

    public static SourceHitboxCatalog FromJson(string json)
    {
        var manifest = JsonSerializer.Deserialize<SourceHitboxBodyManifest[]>(json, JsonOptions)
            ?? throw new InvalidDataException("Invalid source hitbox manifest.");
        var catalog = new SourceHitboxCatalog();
        foreach (var body in manifest) catalog.Register(body.BodyId, body.BodyToWorld, body.Definitions);
        return catalog;
    }

    public bool TryResolve(in HitscanHit physicsHit, Vector3 start, Vector3 direction, float distance,
        out HitscanHit hit)
    {
        hit = physicsHit;
        if (!bodies.TryGetValue(physicsHit.BodyId, out var body) ||
            direction.LengthSquared() < 1e-12f || !float.IsFinite(distance) || distance <= 0f)
            return false;

        direction = Vector3.Normalize(direction);
        var found = false;
        var bestDistance = distance;
        var bestPosition = default(Vector3);
        var bestNormal = default(Vector3);
        SourceHitboxDefinition bestDefinition = default;
        foreach (var definition in body.Definitions)
        {
            var localToWorld = definition.BoneToBodyMeters * body.BodyToWorld;
            if (!Matrix4x4.Invert(localToWorld, out var worldToLocal)) continue;
            var localStart = Vector3.Transform(start, worldToLocal);
            var localDirection = Vector3.TransformNormal(direction, worldToLocal);
            if (!TryRayBox(localStart, localDirection, definition.BoundsMinMeters,
                    definition.BoundsMaxMeters, out var localDistance, out var localNormal)) continue;

            var localPosition = localStart + localDirection * localDistance;
            var worldPosition = Vector3.Transform(localPosition, localToWorld);
            var worldDistance = Vector3.Dot(worldPosition - start, direction);
            if (!float.IsFinite(worldDistance) || worldDistance < 0f || worldDistance > bestDistance) continue;
            var worldNormal = Vector3.TransformNormal(localNormal, Matrix4x4.Transpose(worldToLocal));
            if (worldNormal.LengthSquared() < 1e-12f) continue;
            found = true;
            bestDistance = worldDistance;
            bestPosition = worldPosition;
            bestNormal = Vector3.Normalize(worldNormal);
            bestDefinition = definition;
        }

        if (!found) return false;
        hit = physicsHit with
        {
            Position = bestPosition,
            Normal = bestNormal,
            Fraction = bestDistance / distance,
            HitGroup = bestDefinition.HitGroup,
            Hitbox = bestDefinition.Hitbox,
            PhysicsBone = bestDefinition.PhysicsBone
        };
        return true;
    }

    private static bool TryRayBox(Vector3 start, Vector3 direction, Vector3 min, Vector3 max,
        out float distance, out Vector3 normal)
    {
        var entry = float.NegativeInfinity;
        var exit = float.PositiveInfinity;
        var entryNormal = default(Vector3);
        var exitNormal = default(Vector3);
        if (!ClipAxis(start.X, direction.X, min.X, max.X, Vector3.UnitX, ref entry, ref exit,
                ref entryNormal, ref exitNormal) ||
            !ClipAxis(start.Y, direction.Y, min.Y, max.Y, Vector3.UnitY, ref entry, ref exit,
                ref entryNormal, ref exitNormal) ||
            !ClipAxis(start.Z, direction.Z, min.Z, max.Z, Vector3.UnitZ, ref entry, ref exit,
                ref entryNormal, ref exitNormal) || exit < 0f)
        {
            distance = 0f;
            normal = default;
            return false;
        }

        if (entry >= 0f)
        {
            distance = entry;
            normal = entryNormal;
        }
        else
        {
            distance = exit;
            normal = exitNormal;
        }
        return float.IsFinite(distance) && distance >= 0f;
    }

    private static bool IsFinite(Matrix4x4 value) =>
        float.IsFinite(value.M11) && float.IsFinite(value.M12) && float.IsFinite(value.M13) && float.IsFinite(value.M14) &&
        float.IsFinite(value.M21) && float.IsFinite(value.M22) && float.IsFinite(value.M23) && float.IsFinite(value.M24) &&
        float.IsFinite(value.M31) && float.IsFinite(value.M32) && float.IsFinite(value.M33) && float.IsFinite(value.M34) &&
        float.IsFinite(value.M41) && float.IsFinite(value.M42) && float.IsFinite(value.M43) && float.IsFinite(value.M44);

    private static bool ClipAxis(float start, float direction, float min, float max, Vector3 axis,
        ref float entry, ref float exit, ref Vector3 entryNormal, ref Vector3 exitNormal)
    {
        if (MathF.Abs(direction) < 1e-12f) return start >= min && start <= max;
        var first = (min - start) / direction;
        var second = (max - start) / direction;
        var firstNormal = -axis;
        var secondNormal = axis;
        if (first > second)
        {
            (first, second) = (second, first);
            (firstNormal, secondNormal) = (secondNormal, firstNormal);
        }
        if (first > entry) { entry = first; entryNormal = firstNormal; }
        if (second < exit) { exit = second; exitNormal = secondNormal; }
        return entry <= exit;
    }
}
