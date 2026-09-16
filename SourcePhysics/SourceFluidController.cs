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
    /// Source physics_fluid.cpp initializes IVP_Template_Buoyancy torque_factor
    /// to 0.01 when the fluid params do not override it.
    public float TorqueFactor { get; init; } = 0.01f;
    /// Source initializes viscosity_factor to zero; the separate IVP
    /// viscosity_input_factor is not silently folded into this Jolt parameter.
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

public readonly record struct SourceFluidSurfaceState(Vector3 Normal, float Distance,
    Vector3 CurrentVelocity);

/// Coordinate conversion used by Source's CLiquidSurfaceDescriptor. Plane
/// distance uses the Source convention dot(normal, position) = distance.
public static class SourceFluidSurfaceMath
{
    public static SourceFluidSurfaceState CaptureLocal(Vector4 worldPlane,
        Vector3 worldCurrentVelocity, Vector3 origin, Quaternion rotation)
    {
        var normalLength = new Vector3(worldPlane.X, worldPlane.Y, worldPlane.Z).Length();
        if (!float.IsFinite(normalLength) || normalLength < 1e-12f)
            throw new ArgumentOutOfRangeException(nameof(worldPlane));
        var worldNormal = new Vector3(worldPlane.X, worldPlane.Y, worldPlane.Z) / normalLength;
        var worldDistance = worldPlane.W / normalLength;
        var inverseRotation = Quaternion.Inverse(rotation);
        return new(
            Vector3.Transform(worldNormal, inverseRotation),
            worldDistance - Vector3.Dot(worldNormal, origin),
            Vector3.Transform(worldCurrentVelocity, inverseRotation));
    }

    public static SourceFluidSurfaceState ToWorld(SourceFluidSurfaceState local,
        Vector3 origin, Quaternion rotation)
    {
        var normal = Vector3.Normalize(Vector3.Transform(local.Normal, rotation));
        return new(normal, local.Distance + Vector3.Dot(normal, origin),
            Vector3.Transform(local.CurrentVelocity, rotation));
    }
}

public sealed class JoltFluidController : IDisposable
{
    private readonly JoltPhysicsHost host;
    private readonly Dictionary<uint, SourceFluidProfile> fluids = new();
    private readonly Dictionary<uint, SourceFluidSurfaceState> localSurfaces = new();
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
        if (!host.Bodies.IsAdded(fluidBody)) throw new ArgumentException("Fluid body is not in the Jolt system.", nameof(fluidBody));
        GetBodyPose(fluidBody, out var origin, out var rotation);
        fluids[fluidBody.ID] = profile;
        localSurfaces[fluidBody.ID] = SourceFluidSurfaceMath.CaptureLocal(
            profile.SurfacePlane, SourceUnits.ToMeters(profile.CurrentVelocitySourceUnitsPerSecond),
            origin, rotation);
    }

    public void UnregisterFluidBody(BodyID fluidBody)
    {
        fluids.Remove(fluidBody.ID);
        localSurfaces.Remove(fluidBody.ID);
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
        var bodyId = new BodyID(value.OtherBody);
        if (!host.IsFluidTouchEnabled(bodyId))
        {
            active.Remove((value.TriggerBody, value.OtherBody));
            return;
        }
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
            if (!localSurfaces.TryGetValue(pair.Key.Fluid, out var localSurface) ||
                !host.Bodies.IsAdded(new BodyID(pair.Key.Fluid)))
                continue;
            var bodyId = new BodyID(pair.Key.Body);
            if (host.Bodies.GetMotionType(bodyId) != MotionType.Dynamic) continue;
            GetBodyPose(new BodyID(pair.Key.Fluid), out var fluidOrigin, out var fluidRotation);
            var surface = SourceFluidSurfaceMath.ToWorld(localSurface, fluidOrigin, fluidRotation);
            var surfacePoint = pair.Value.Point + surface.Normal *
                (surface.Distance - Vector3.Dot(surface.Normal, pair.Value.Point));
            host.ApplySourceFluidTouchDamping(bodyId, surface.Normal, profile.DensityKgPerM3,
                profile.Damping, profile.TorqueFactor, deltaSeconds);
            if (profile.BuoyancyForceNewtons is { } buoyancy && host.IsFluidSimulationEnabled(bodyId))
                host.ApplyBuoyancyImpulse(bodyId, surfacePoint, surface.Normal, buoyancy,
                    profile.ViscosityFactor, profile.TorqueFactor, surface.CurrentVelocity, deltaSeconds,
                    host.GetBodyBuoyancyRatio(bodyId));
        }
    }

    private void GetBodyPose(BodyID bodyId, out Vector3 origin, out Quaternion rotation)
    {
        var transform = host.Bodies.GetRCenterOfMassTransform(bodyId);
        origin = (Vector3)transform.Translation;
        rotation = Quaternion.CreateFromRotationMatrix(new Matrix4x4(
            transform.M11, transform.M12, transform.M13, 0f,
            transform.M21, transform.M22, transform.M23, 0f,
            transform.M31, transform.M32, transform.M33, 0f,
            0f, 0f, 0f, 1f));
    }

    public void Dispose()
    {
        UnregisterFixedStep();
        host.Contacts.TriggerEntered -= OnTriggerEntered;
        host.Contacts.TriggerStayed -= OnTriggerStayed;
        host.Contacts.TriggerExited -= OnTriggerExited;
        active.Clear();
        fluids.Clear();
        localSurfaces.Clear();
    }

    private readonly record struct SourceFluidContact(Vector3 Point, Vector3 Normal);
}
