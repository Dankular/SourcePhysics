using System.Numerics;
using JoltPhysicsSharp;

namespace SourcePhysics;

/// <summary>
/// Explicit, deterministic mesh-cooking policy at the Source/Jolt boundary.
/// A zero weld tolerance preserves the imported vertex stream exactly; no
/// implicit cleanup is performed.
/// </summary>
public sealed record SourceMeshCookOptions
{
    public float WeldToleranceSourceUnits { get; init; }
    public bool RemoveDegenerateTriangles { get; init; }
    public float DegenerateAreaToleranceSourceUnits { get; init; }

    public void Validate()
    {
        if (!float.IsFinite(WeldToleranceSourceUnits) || WeldToleranceSourceUnits < 0)
            throw new ArgumentOutOfRangeException(nameof(WeldToleranceSourceUnits));
        if (!float.IsFinite(DegenerateAreaToleranceSourceUnits) || DegenerateAreaToleranceSourceUnits < 0)
            throw new ArgumentOutOfRangeException(nameof(DegenerateAreaToleranceSourceUnits));
    }
}

public sealed record SourceCookedStaticMesh(
    IReadOnlyList<Vector3> Vertices,
    IReadOnlyList<IndexedTriangle> Triangles,
    SourceStaticMeshProfile Profile);

/// <summary>
/// Performs only explicitly requested, order-preserving mesh preparation.
/// Surface IDs are remapped with their triangles and the profile's contents,
/// friction, restitution and user data are retained.
/// </summary>
public static class SourceStaticMeshCooker
{
    public static SourceCookedStaticMesh Cook(
        IReadOnlyList<Vector3> vertices,
        IReadOnlyList<IndexedTriangle> triangles,
        SourceStaticMeshProfile profile,
        SourceMeshCookOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(vertices);
        ArgumentNullException.ThrowIfNull(triangles);
        ArgumentNullException.ThrowIfNull(profile);
        options ??= new SourceMeshCookOptions();
        options.Validate();
        profile.Validate();
        if (vertices.Count == 0) throw new ArgumentException("A mesh requires vertices.", nameof(vertices));
        if (triangles.Count == 0) throw new ArgumentException("A mesh requires triangles.", nameof(triangles));
        if (profile.TriangleSurfaceIds is not null && profile.TriangleSurfaceIds.Length != triangles.Count)
            throw new ArgumentException("TriangleSurfaceIds must contain one surface id per input triangle.", nameof(profile));

        var outputVertices = new List<Vector3>(vertices.Count);
        var remap = new int[vertices.Count];
        var toleranceSquared = options.WeldToleranceSourceUnits * options.WeldToleranceSourceUnits;
        for (var i = 0; i < vertices.Count; i++)
        {
            var vertex = vertices[i];
            if (!float.IsFinite(vertex.X) || !float.IsFinite(vertex.Y) || !float.IsFinite(vertex.Z))
                throw new ArgumentException("Mesh vertices must be finite.", nameof(vertices));
            var mapped = -1;
            if (options.WeldToleranceSourceUnits > 0)
            {
                for (var candidate = 0; candidate < outputVertices.Count; candidate++)
                {
                    if (Vector3.DistanceSquared(vertex, outputVertices[candidate]) <= toleranceSquared)
                    {
                        mapped = candidate;
                        break;
                    }
                }
            }
            if (mapped < 0)
            {
                mapped = outputVertices.Count;
                outputVertices.Add(vertex);
            }
            remap[i] = mapped;
        }

        var outputTriangles = new List<IndexedTriangle>(triangles.Count);
        var outputSurfaceIds = profile.TriangleSurfaceIds is null ? null : new List<int>(triangles.Count);
        var areaToleranceSquared = options.DegenerateAreaToleranceSourceUnits * options.DegenerateAreaToleranceSourceUnits;
        for (var i = 0; i < triangles.Count; i++)
        {
            var source = triangles[i];
            if (source.I1 >= vertices.Count || source.I2 >= vertices.Count || source.I3 >= vertices.Count)
                throw new ArgumentException($"Triangle {i} references a vertex outside the input stream.", nameof(triangles));
            var i1 = (uint)remap[source.I1];
            var i2 = (uint)remap[source.I2];
            var i3 = (uint)remap[source.I3];
            var a = outputVertices[(int)i1];
            var b = outputVertices[(int)i2];
            var c = outputVertices[(int)i3];
            var cross = Vector3.Cross(b - a, c - a);
            var degenerate = i1 == i2 || i1 == i3 || i2 == i3 ||
                              (options.DegenerateAreaToleranceSourceUnits > 0 && cross.LengthSquared() <= areaToleranceSquared);
            if (degenerate && options.RemoveDegenerateTriangles) continue;
            outputTriangles.Add(new IndexedTriangle(i1, i2, i3, source.MaterialIndex, source.UserData));
            outputSurfaceIds?.Add(profile.TriangleSurfaceIds![i]);
        }
        if (outputTriangles.Count == 0)
            throw new InvalidOperationException("Mesh cooking removed every triangle.");

        var outputProfile = profile with
        {
            TriangleSurfaceIds = outputSurfaceIds?.ToArray()
        };
        return new SourceCookedStaticMesh(outputVertices.ToArray(), outputTriangles.ToArray(), outputProfile);
    }
}
