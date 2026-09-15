using JoltPhysicsSharp;
using System.Numerics;

namespace SourcePhysics;

/// Source fluidparams_t and FluidStartTouch/FluidEndTouch bridge. Fluid bodies
/// must be authored as Jolt sensor bodies; no volume or buoyancy defaults are
/// inferred from shape geometry.
public sealed record SourceFluidProfile
{
    public Vector4 SurfacePlane { get; init; }
    public Vector3 CurrentVelocitySourceUnitsPerSecond { get; init; }
    public float DensityKgPerM3 { get; init; }
    public float Damping { get; init; }
    public float TorqueFactor { get; init; }
    public float ViscosityFactor { get; init; }
    public SourceContents Contents { get; init; } = SourceContents.Water;
    public float? BuoyancyForceNewtons { get; init; }

    public Vector3 SurfaceNormal
    {
        get
        {
            var normal = new Vector3(SurfacePlane.X, SurfacePlane.Y, SurfacePlane.Z);
            return normal.LengthSquared() > 1e-12f ? Vector3.Normalize(normal) : Vector3.Zero;
        }
    }

    public void Validate()
    {
        var normal = SurfaceNormal;
        if (!IsFinite(SurfacePlane) || !IsFinite(CurrentVelocitySourceUnitsPerSecond) ||
            normal.LengthSquared() < 0.999f || normal.LengthSquared() > 1.001f ||
            !float.IsFinite(DensityKgPerM3) || DensityKgPerM3 < 0f ||
            !float.IsFinite(Damping) || Damping < 0f || !float.IsFinite(TorqueFactor) || TorqueFactor < 0f ||
            !float.IsFinite(ViscosityFactor) || ViscosityFactor < 0f ||
            (BuoyancyForceNewtons is { } buoyancy && (!float.IsFinite(buoyancy) || buoyancy < 0f)) )
            throw new InvalidDataException("Fluid profile contains an invalid value.");
    }

    private static bool IsFinite(Vector3 value) => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);
    private static bool IsFinite(Vector4 value) => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z) && float.IsFinite(value.W);
}

public sealed class JoltFluidController : IDisposable
{
    private readonly JoltPhysicsHost host;
    private readonly Dictionary<uint, SourceFluidProfile> fluids = new();
    private readonly Dictionary<(uint Fluid, uint Body), SourceFluidContact> active = new();
    private readonly Action<float> preStep;
    private bool registered;

    public int ActiveContactCount => active.Count;

    public JoltFluidController(JoltPhysicsHost host)
    {
        this.host = host ?? throw new ArgumentNullException(nameof(host));
        preStep = ApplyFluidForces;
        host.Contacts.TriggerEntered += OnTriggerEntered;
        host.Contacts.TriggerStayed += OnTriggerStayed;
        host.Contacts.TriggerExited += OnTriggerExited;
    }

    public void RegisterFluidBody(BodyID fluidBody, SourceFluidProfile profile)
    {
        profile.Validate();
        if (!host.IsSensor(fluidBody)) throw new ArgumentException("Fluid body must be a Jolt sensor.", nameof(fluidBody));
        fluids[fluidBody.ID] = profile;
    }

    public void UnregisterFluidBody(BodyID fluidBody)
    {
        fluids.Remove(fluidBody.ID);
        foreach (var key in active.Keys.Where(key => key.Fluid == fluidBody.ID).ToArray()) active.Remove(key);
    }

    public void RegisterFixedStep()
    {
        if (registered) return;
        host.RegisterPreStepController(preStep);
        registered = true;
    }

    public void UnregisterFixedStep()
    {
        if (!registered) return;
        host.UnregisterPreStepController(preStep);
        registered = false;
    }

    private void OnTriggerEntered(SourceTriggerEvent value) => Track(value);
    private void OnTriggerStayed(SourceTriggerEvent value) => Track(value);
    private void Track(SourceTriggerEvent value)
    {
        if (!fluids.ContainsKey(value.TriggerBody)) return;
        var normal = value.Normal.LengthSquared() > 1e-12f ? Vector3.Normalize(value.Normal) : fluids[value.TriggerBody].SurfaceNormal;
        active[(value.TriggerBody, value.OtherBody)] = new(value.ContactPoint, normal);
    }

    private void OnTriggerExited(SourceTriggerRemovedEvent value)
    {
        active.Remove((value.TriggerBody, value.OtherBody));
    }

    private void ApplyFluidForces(float deltaSeconds)
    {
        foreach (var pair in active.ToArray())
        {
            if (!fluids.TryGetValue(pair.Key.Fluid, out var profile) || !host.Bodies.IsAdded(new BodyID(pair.Key.Body)))
                continue;
            var bodyId = new BodyID(pair.Key.Body);
            if (host.Bodies.GetMotionType(bodyId) != MotionType.Dynamic) continue;
            var fluidVelocity = SourceUnits.ToMeters(profile.CurrentVelocitySourceUnitsPerSecond);
            host.ApplySourceFluidTouchDamping(bodyId, pair.Value.Normal, profile.DensityKgPerM3,
                profile.Damping, profile.TorqueFactor, deltaSeconds);
            if (profile.BuoyancyForceNewtons is { } buoyancy)
                host.ApplyBuoyancyImpulse(bodyId, pair.Value.Point, pair.Value.Normal, buoyancy,
                    profile.ViscosityFactor, profile.TorqueFactor, fluidVelocity, deltaSeconds);
        }
    }

    public void Dispose()
    {
        UnregisterFixedStep();
        host.Contacts.TriggerEntered -= OnTriggerEntered;
        host.Contacts.TriggerStayed -= OnTriggerStayed;
        host.Contacts.TriggerExited -= OnTriggerExited;
        active.Clear();
        fluids.Clear();
    }

    private readonly record struct SourceFluidContact(Vector3 Point, Vector3 Normal);
}
