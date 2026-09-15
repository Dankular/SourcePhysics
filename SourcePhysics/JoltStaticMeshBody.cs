using JoltPhysicsSharp;
using Stride.Engine;
using System.Numerics;

namespace SourcePhysics;

/// Stride scene component for an authored static collision mesh.
/// Vertices are supplied in local meters and triangle indices are local to Vertices.
public sealed class JoltStaticMeshBody : SyncScript
{
    public StrideSourcePhysicsScript PhysicsSystem { get; set; } = null!;
    public SourceObjectLayer Layer { get; set; } = SourceObjectLayer.World;
    public Vector3[] VerticesMeters { get; set; } = Array.Empty<Vector3>();
    public IndexedTriangle[] Triangles { get; set; } = Array.Empty<IndexedTriangle>();
    public SourceStaticMeshProfile Profile { get; set; } = new();
    /// Source-authored weld tolerance. Zero preserves the imported mesh exactly.
    public float WeldToleranceSourceUnits { get; set; }
    public bool RemoveDegenerateTriangles { get; set; }
    public float DegenerateAreaToleranceSourceUnits { get; set; }

    public BodyID BodyId { get; private set; }

    public override void Start()
    {
        if (PhysicsSystem is null) throw new InvalidOperationException("Assign PhysicsSystem before starting JoltStaticMeshBody.");
        PhysicsSystem.EnsureStarted();
        if (VerticesMeters.Length == 0 || Triangles.Length == 0)
            throw new InvalidOperationException("JoltStaticMeshBody requires vertices and triangles.");
        var translation = Entity.Transform.WorldMatrix.TranslationVector;
        var scale = Entity.Transform.Scale;
        var scaledVertices = VerticesMeters
            .Select(vertex => new Vector3(vertex.X * scale.X, vertex.Y * scale.Y, vertex.Z * scale.Z))
            .ToArray();
        var strideRotation = Entity.Transform.Rotation;
        var rotation = new Quaternion(strideRotation.X, strideRotation.Y, strideRotation.Z, strideRotation.W);
        var cooked = SourceStaticMeshCooker.Cook(scaledVertices, Triangles, Profile, new SourceMeshCookOptions
        {
            // Component vertices are meters; the cooker contract is Source units.
            WeldToleranceSourceUnits = SourceUnits.ToMeters(WeldToleranceSourceUnits),
            RemoveDegenerateTriangles = RemoveDegenerateTriangles,
            DegenerateAreaToleranceSourceUnits = SourceUnits.ToMeters(DegenerateAreaToleranceSourceUnits)
        });
        BodyId = PhysicsSystem.Host.CreateStaticMeshBody(cooked.Vertices, cooked.Triangles,
            new Vector3(translation.X, translation.Y, translation.Z), rotation, Layer, cooked.Profile);
    }

    public override void Update() { }

    public override void Cancel()
    {
        if (PhysicsSystem is not null) PhysicsSystem.Host.DestroyBody(BodyId);
        BodyId = BodyID.Invalid;
    }
}
