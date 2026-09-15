using System.Text.Json;

namespace SourcePhysics;

public sealed record SourceSurface(string Name, float Friction = 0.8f, float Elasticity = 0.001f,
    float DensityKgPerM3 = 1000f, float Dampening = 0f, float MaxSpeedFactor = 1f,
    float JumpFactor = 1f, bool Climbable = false, float ThicknessInches = 0f,
    float Reflectivity = 0f, float HardnessFactor = 0f, float RoughnessFactor = 0f,
    string ImpactHardSound = "", string ImpactSoftSound = "", string ScrapeRoughSound = "",
    string ScrapeSmoothSound = "", string BulletImpactSound = "", string FootstepSound = "",
    string StepLeftSound = "", string StepRightSound = "", string BreakSound = "",
    string StrainSound = "", string RollingSound = "", char GameMaterial = '\0',
    float HardVelocityThreshold = 0f, float RoughThreshold = 0f, float HardThreshold = 0f);

public sealed class SourceSurfaceRegistry
{
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true, PropertyNameCaseInsensitive = true };
    private readonly Dictionary<int, SourceSurface> surfaces = new();
    public SourceSurface Fallback { get; init; } = new("default", 0.8f, 0.001f);

    public IReadOnlyDictionary<int, SourceSurface> Surfaces => surfaces;

    public void Register(int id, SourceSurface surface)
    {
        if (id < 0) throw new ArgumentOutOfRangeException(nameof(id));
        ArgumentNullException.ThrowIfNull(surface);
        Validate(surface);
        surfaces[id] = surface;
    }

    public bool TryGet(int id, out SourceSurface surface) => surfaces.TryGetValue(id, out surface!);
    public SourceSurface Get(int id) => surfaces.TryGetValue(id, out var value) ? value : Fallback;
    public static float CombineFriction(SourceSurface a, SourceSurface b) => MathF.Sqrt(a.Friction * b.Friction);
    public static float CombineRestitution(SourceSurface a, SourceSurface b) => MathF.Max(a.Elasticity, b.Elasticity);

    public string ToJson() => JsonSerializer.Serialize(new SurfaceManifest(Fallback, surfaces), JsonOptions);

    public static SourceSurfaceRegistry FromJson(string json)
    {
        ArgumentException.ThrowIfNullOrEmpty(json);
        var manifest = JsonSerializer.Deserialize<SurfaceManifest>(json, JsonOptions)
            ?? throw new InvalidDataException("Invalid source surface manifest.");
        var registry = new SourceSurfaceRegistry { Fallback = manifest.Fallback ?? new SourceSurface("default", 0.8f, 0.001f) };
        Validate(registry.Fallback);
        foreach (var entry in manifest.Surfaces ?? new Dictionary<int, SourceSurface>())
            registry.Register(entry.Key, entry.Value);
        return registry;
    }

    public static SourceSurfaceRegistry FromSurfaceProperties(string text) =>
        SourceSurfacePropertiesParser.Parse(text);

    private static void Validate(SourceSurface surface)
    {
        if (!float.IsFinite(surface.Friction) || surface.Friction < 0f ||
            !float.IsFinite(surface.Elasticity) || surface.Elasticity < 0f ||
            !float.IsFinite(surface.DensityKgPerM3) || surface.DensityKgPerM3 <= 0f ||
            !float.IsFinite(surface.Dampening) || surface.Dampening < 0f ||
            !float.IsFinite(surface.MaxSpeedFactor) || surface.MaxSpeedFactor < 0f ||
            !float.IsFinite(surface.JumpFactor) || surface.JumpFactor < 0f ||
            !float.IsFinite(surface.ThicknessInches) || surface.ThicknessInches < 0f ||
            !float.IsFinite(surface.Reflectivity) || surface.Reflectivity < 0f ||
            !float.IsFinite(surface.HardnessFactor) || surface.HardnessFactor < 0f ||
            !float.IsFinite(surface.RoughnessFactor) || surface.RoughnessFactor < 0f ||
            !float.IsFinite(surface.HardVelocityThreshold) || surface.HardVelocityThreshold < 0f ||
            !float.IsFinite(surface.RoughThreshold) || surface.RoughThreshold < 0f ||
            !float.IsFinite(surface.HardThreshold) || surface.HardThreshold < 0f)
            throw new ArgumentException($"Surface '{surface.Name}' contains an invalid physical property.", nameof(surface));
    }

    private sealed record SurfaceManifest(SourceSurface? Fallback, Dictionary<int, SourceSurface>? Surfaces);
}
