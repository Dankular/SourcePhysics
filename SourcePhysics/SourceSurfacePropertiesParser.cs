using System.Globalization;

namespace SourcePhysics;

/// Strict loader for the Source surfaceproperties KeyValues format. Unknown
/// keys are rejected so a title asset cannot silently lose physics semantics.
internal static class SourceSurfacePropertiesParser
{
    private sealed class Block
    {
        public string Name { get; }
        public Dictionary<string, string> Values { get; } = new(StringComparer.OrdinalIgnoreCase);
        public List<Block> Children { get; } = new();
        public Block(string name) => Name = name;
    }

    public static SourceSurfaceRegistry Parse(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        var tokens = Tokenize(text).ToArray();
        var index = 0;
        var roots = new List<Block>();
        while (index < tokens.Length)
        {
            var name = tokens[index++];
            if (index >= tokens.Length || tokens[index++] != "{")
                throw new InvalidDataException($"Surface properties key '{name}' must open a block.");
            roots.Add(ParseBlock(name, tokens, ref index));
        }

        var surfaceBlocks = roots.SelectMany(root => root.Name.Equals("surfaceproperties", StringComparison.OrdinalIgnoreCase)
            ? root.Children : (IEnumerable<Block>)new[] { root }).ToArray();
        if (surfaceBlocks.Any(block => block.Children.Count != 0))
            throw new InvalidDataException("Nested blocks inside a Source surface are unsupported and would discard authored data.");
        var byName = surfaceBlocks.ToDictionary(block => block.Name, StringComparer.OrdinalIgnoreCase);
        var resolved = new Dictionary<string, SourceSurface>(StringComparer.OrdinalIgnoreCase);
        var resolving = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        SourceSurface Resolve(Block block)
        {
            if (resolved.TryGetValue(block.Name, out var existing)) return existing;
            if (!resolving.Add(block.Name)) throw new InvalidDataException($"Circular surface base chain at '{block.Name}'.");
            var surface = new SourceSurface(block.Name);
            if (block.Values.TryGetValue("base", out var baseName))
            {
                if (!byName.TryGetValue(baseName, out var baseBlock))
                    throw new InvalidDataException($"Surface '{block.Name}' references missing base '{baseName}'.");
                surface = Resolve(baseBlock) with { Name = block.Name };
            }
            surface = Apply(surface, block.Values);
            resolving.Remove(block.Name);
            resolved[block.Name] = surface;
            return surface;
        }

        var registry = new SourceSurfaceRegistry();
        var nextId = 0;
        SourceSurface? fallback = null;
        foreach (var block in surfaceBlocks)
        {
            var surface = Resolve(block);
            if (block.Name.Equals("default", StringComparison.OrdinalIgnoreCase)) fallback = surface;
            registry.Register(nextId++, surface);
        }
        if (fallback is not null)
        {
            registry = new SourceSurfaceRegistry { Fallback = fallback };
            nextId = 0;
            foreach (var block in surfaceBlocks) registry.Register(nextId++, Resolve(block));
        }
        return registry;
    }

    private static Block ParseBlock(string name, string[] tokens, ref int index)
    {
        var block = new Block(name);
        while (index < tokens.Length && tokens[index] != "}")
        {
            var key = tokens[index++];
            if (index >= tokens.Length) throw new InvalidDataException($"Unclosed surface block '{name}'.");
            if (tokens[index] == "{")
            {
                index++;
                block.Children.Add(ParseBlock(key, tokens, ref index));
            }
            else
            {
                var value = tokens[index++];
                if (value is "{" or "}") throw new InvalidDataException($"Invalid value for surface key '{key}'.");
                block.Values[key] = value;
            }
        }
        if (index >= tokens.Length || tokens[index++] != "}")
            throw new InvalidDataException($"Unclosed surface block '{name}'.");
        return block;
    }

    private static SourceSurface Apply(SourceSurface surface, IReadOnlyDictionary<string, string> values)
    {
        foreach (var pair in values)
        {
            if (pair.Key.Equals("base", StringComparison.OrdinalIgnoreCase)) continue;
            surface = pair.Key.ToLowerInvariant() switch
            {
                "friction" => surface with { Friction = Float(pair) },
                "elasticity" => surface with { Elasticity = Float(pair) },
                "density" => surface with { DensityKgPerM3 = Float(pair) },
                "dampening" => surface with { Dampening = Float(pair) },
                "thickness" => surface with { ThicknessInches = Float(pair) },
                "maxspeedfactor" => surface with { MaxSpeedFactor = Float(pair) },
                "jumpfactor" => surface with { JumpFactor = Float(pair) },
                "climbable" => surface with { Climbable = Int(pair) != 0 },
                "audioreflectivity" => surface with { Reflectivity = Float(pair) },
                "audiohardnessfactor" => surface with { HardnessFactor = Float(pair) },
                "audioroughnessfactor" => surface with { RoughnessFactor = Float(pair) },
                "audiohardminvelocity" => surface with { HardVelocityThreshold = Float(pair) },
                "scraperoughthreshold" => surface with { RoughThreshold = Float(pair) },
                "impacthardthreshold" => surface with { HardThreshold = Float(pair) },
                "impacthard" => surface with { ImpactHardSound = pair.Value },
                "impactsoft" => surface with { ImpactSoftSound = pair.Value },
                "scraperough" => surface with { ScrapeRoughSound = pair.Value },
                "scrapesmooth" => surface with { ScrapeSmoothSound = pair.Value },
                "bulletimpact" => surface with { BulletImpactSound = pair.Value },
                "stepleft" => surface with { StepLeftSound = pair.Value },
                "stepright" => surface with { StepRightSound = pair.Value },
                "break" => surface with { BreakSound = pair.Value },
                "strain" => surface with { StrainSound = pair.Value },
                "rolling" => surface with { RollingSound = pair.Value },
                "gamematerial" => surface with { GameMaterial = ParseGameMaterial(pair.Value) },
                _ => throw new InvalidDataException($"Unsupported Source surface key '{pair.Key}' in '{surface.Name}'.")
            };
        }
        return surface;
    }

    private static float Float(KeyValuePair<string, string> pair) =>
        float.Parse(pair.Value, NumberStyles.Float, CultureInfo.InvariantCulture);
    private static int Int(KeyValuePair<string, string> pair) => int.Parse(pair.Value, CultureInfo.InvariantCulture);
    private static char ParseGameMaterial(string value) => value.Length == 1 && !char.IsDigit(value[0])
        ? char.ToUpperInvariant(value[0])
        : (char)int.Parse(value, CultureInfo.InvariantCulture);

    private static IEnumerable<string> Tokenize(string text)
    {
        for (var index = 0; index < text.Length;)
        {
            while (index < text.Length && char.IsWhiteSpace(text[index])) index++;
            if (index >= text.Length) yield break;
            if (text[index] == '/' && index + 1 < text.Length && text[index + 1] == '/')
            {
                index += 2;
                while (index < text.Length && text[index] != '\n') index++;
                continue;
            }
            if (text[index] is '{' or '}') { yield return text[index++].ToString(); continue; }
            if (text[index] == '"')
            {
                index++;
                var start = index;
                while (index < text.Length && text[index] != '"') index++;
                if (index >= text.Length) throw new InvalidDataException("Unclosed quoted surface token.");
                yield return text[start..index];
                index++;
                continue;
            }
            var tokenStart = index;
            while (index < text.Length && !char.IsWhiteSpace(text[index]) && text[index] is not ('{' or '}')) index++;
            yield return text[tokenStart..index];
        }
    }
}
