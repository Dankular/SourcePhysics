namespace SourcePhysics;

public sealed record SourceStaticMeshProfile
{
    public float Friction { get; init; } = 0.8f;
    public float Restitution { get; init; } = 0.001f;
    /// Source objectparams_t::pName, retained for authored physics diagnostics.
    public string Name { get; init; } = "";
    /// Source objectparams_t::volume, in cubic Source inches.
    public float VolumeCubicInches { get; init; }
    /// Source objectparams_t::enableCollisions for the static/poly body.
    public bool EnableCollisions { get; init; } = true;
    public float ActiveEdgeCosThresholdAngle { get; init; } = 0.996f;
    public uint MaxTrianglesPerLeaf { get; init; } = 8;
    public int SurfaceId { get; init; }
    public SourceContents ContentsMask { get; init; } = SourceContents.Solid;
    /// Source VPhysics callback flags carried by the authored static object.
    public SourceCallbackFlags CallbackFlags { get; init; } = SourceCallbackFlags.Default;
    /// Source collision-property solid flags.
    public SourceSolidFlags SolidFlags { get; init; }
    /// Optional Source collision group. None leaves the host's object-layer policy in control.
    public SourceCollisionGroup CollisionGroup { get; init; }
    /// Source FSOLID_TRIGGER_TOUCH_DEBRIS: only authored trigger bodies with this flag
    /// receive trigger events from debris bodies.
    public bool TriggerTouchesDebris { get; init; }
    /// Source VPhysics game-data/user-data identity carried by the static body.
    public ulong UserData { get; init; }
    /// <summary>
    /// Optional Source surface index for each triangle, in the same order as the mesh triangle array.
    /// When supplied, Jolt stores this value as per-triangle user data and queries resolve it from the
    /// returned sub-shape id instead of falling back to <see cref="SurfaceId"/>.
    /// </summary>
    public int[]? TriangleSurfaceIds { get; init; }

    public void Validate()
    {
        const SourceSolidFlags supportedSolidFlags = SourceSolidFlags.NotSolid |
            SourceSolidFlags.Trigger | SourceSolidFlags.TriggerTouchDebris;
        const SourceCallbackFlags supportedCallbackFlags = (SourceCallbackFlags)0xFFFF;
        if (!Enum.IsDefined(CollisionGroup) || (CallbackFlags & ~supportedCallbackFlags) != 0 ||
            (SolidFlags & ~supportedSolidFlags) != 0)
            throw new InvalidDataException("Static mesh profile contains an unknown Source collision state.");
        if (!float.IsFinite(Friction) || Friction < 0f)
            throw new InvalidDataException("Static mesh friction must be finite and non-negative.");
        if (!float.IsFinite(Restitution) || Restitution < 0f)
            throw new InvalidDataException("Static mesh restitution must be finite and non-negative.");
        if (!float.IsFinite(VolumeCubicInches) || VolumeCubicInches < 0f)
            throw new InvalidDataException("Static mesh volume must be finite and non-negative.");
        if (!float.IsFinite(ActiveEdgeCosThresholdAngle) || ActiveEdgeCosThresholdAngle < -1f || ActiveEdgeCosThresholdAngle > 1f)
            throw new InvalidDataException("Static mesh active-edge cosine threshold must be in [-1, 1].");
        if (MaxTrianglesPerLeaf == 0)
            throw new InvalidDataException("Static mesh MaxTrianglesPerLeaf must be positive.");
    }
}
