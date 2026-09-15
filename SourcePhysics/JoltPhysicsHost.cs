using JoltPhysicsSharp;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;

namespace SourcePhysics;

/// <summary>Owns Jolt lifetime and the fixed authoritative simulation clock.</summary>
public sealed partial class JoltPhysicsHost : IDisposable
{
    private static readonly object FoundationGate = new();
    private static int foundationUsers;
    [DllImport("joltc", EntryPoint = "JPH_MeshShape_GetTriangleUserData", CallingConvention = CallingConvention.Cdecl)]
    private static extern uint GetMeshTriangleUserData(nint shape, uint subShapeId);

    private SourceCollisionLayers? collisionLayers;
    private SourceSimShapeFilter? sourceSimShapeFilter;
    private readonly List<BodyID> ownedBodies = new();
    private readonly List<Constraint> ownedConstraints = new();
    private readonly Dictionary<uint, int> bodySurfaces = new();
    private readonly Dictionary<uint, SourceRigidBodyProfile> bodyProfiles = new();
    private readonly Dictionary<uint, SourceContents> bodyContents = new();
    private readonly Dictionary<uint, SourceObjectLayer> bodyLayers = new();
    private readonly Dictionary<uint, SourceCollisionGroup> bodyCollisionGroups = new();
    private readonly HashSet<uint> triggerTouchesDebris = new();
    private readonly HashSet<uint> nonSolidBodies = new();
    private readonly HashSet<uint> perTriangleSurfaceBodies = new();
    private readonly HashSet<uint> sensorBodies = new();
    private readonly List<Action<float>> preStepControllers = new();
    private JobSystemThreadPool? jobSystem;
    private bool initialized;
    private bool ownsFoundationReference;

    private PhysicsSystem? system;
    public PhysicsSystem System => system ?? throw new InvalidOperationException("Initialize the Jolt host first.");
    public BodyInterface Bodies => System.BodyInterface;
    public NarrowPhaseQuery NarrowPhase => System.NarrowPhaseQuery;
    public float FixedStepSeconds { get; }
    public SourceMovementProfile MovementProfile { get; }
    public SourceContactRouter Contacts { get; }
    public SourceSurfaceRegistry Surfaces { get; } = new();
    public SourceContactMaterialPolicy ContactMaterialPolicy { get; }
    public JoltSolverProfile SolverProfile { get; }
    public SourceCollisionPolicy CollisionPolicy { get; }
    public PhysicsStepMetrics LastStepMetrics { get; private set; }
    public event Action<SourcePhysicsImpulseEvent>? ImpulseApplied;

    public JoltPhysicsHost(SourceMovementProfile profile, float fixedStepSeconds = 1f / 66f,
        JoltSolverProfile? solverProfile = null, SourceCollisionPolicy? collisionPolicy = null,
        SourceContactMaterialPolicy? contactMaterialPolicy = null)
    {
        if (fixedStepSeconds <= 0f) throw new ArgumentOutOfRangeException(nameof(fixedStepSeconds));
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        FixedStepSeconds = fixedStepSeconds;
        MovementProfile = profile;
        SolverProfile = solverProfile ?? new JoltSolverProfile();
        CollisionPolicy = collisionPolicy ?? new SourceCollisionPolicy();
        ContactMaterialPolicy = contactMaterialPolicy ?? new SourceContactMaterialPolicy();
        Contacts = new SourceContactRouter(
            (in Body body, SubShapeID subShapeId) => GetBodySurface(in body, subShapeId, out _),
            ContactMaterialPolicy, IsSensor, ShouldTouchTrigger);
        // Gravity is exposed by SourceMovementProfile.Gravity in Jolt's meter units.
        gravity = new Vector3(0f, -profile.Gravity, 0f);
    }

    private readonly Vector3 gravity;

    private static SourceObjectLayer GetEffectiveLayer(SourceObjectLayer requested, SourceSolidFlags flags)
    {
        if ((flags & SourceSolidFlags.Trigger) != 0 || requested == SourceObjectLayer.Trigger)
            return SourceObjectLayer.Trigger;
        return (flags & SourceSolidFlags.NotSolid) != 0 ? SourceObjectLayer.NonSolid : requested;
    }

    private static SourceCollisionGroup GetEffectiveCollisionGroup(SourceObjectLayer layer, SourceCollisionGroup authored)
    {
        if (authored != SourceCollisionGroup.None) return authored;
        return layer switch
        {
            SourceObjectLayer.Player => SourceCollisionGroup.Player,
            SourceObjectLayer.Debris => SourceCollisionGroup.Debris,
            _ => SourceCollisionGroup.None
        };
    }

    public void Initialize(uint maxBodies = 65536, uint numBodyMutexes = 0, uint maxBodyPairs = 65536, uint maxContactConstraints = 10240)
    {
        if (initialized) return;
        lock (FoundationGate)
        {
            if (foundationUsers == 0 && !Foundation.Init(true))
                throw new InvalidOperationException("Jolt Foundation.Init failed.");
            foundationUsers++;
            ownsFoundationReference = true;
        }
        collisionLayers = new SourceCollisionLayers(CollisionPolicy);
        var count = checked((int)maxBodies);
        var settings = new PhysicsSystemSettings
        {
            MaxBodies = count, NumBodyMutexes = checked((int)numBodyMutexes),
            MaxBodyPairs = checked((int)maxBodyPairs), MaxContactConstraints = checked((int)maxContactConstraints),
            BroadPhaseLayerInterface = collisionLayers.BroadPhase,
            ObjectLayerPairFilter = collisionLayers.PairFilter,
            ObjectVsBroadPhaseLayerFilter = collisionLayers.ObjectVsBroadPhase
        };
        system = new PhysicsSystem(settings) { Gravity = gravity };
        sourceSimShapeFilter = new SourceSimShapeFilter(GetBodyCollisionGroup);
        System.SetSimShapeFilter(sourceSimShapeFilter);
        var physicsSettings = System.Settings;
        physicsSettings.Baumgarte = SolverProfile.Baumgarte;
        physicsSettings.SpeculativeContactDistance = SolverProfile.SpeculativeContactDistanceMeters;
        physicsSettings.PenetrationSlop = SolverProfile.PenetrationSlopMeters;
        physicsSettings.ManifoldTolerance = SolverProfile.ManifoldToleranceMeters;
        physicsSettings.MaxPenetrationDistance = SolverProfile.MaximumPenetrationCorrectionMeters;
        physicsSettings.NumVelocitySteps = SolverProfile.VelocitySolverSteps;
        physicsSettings.NumPositionSteps = SolverProfile.PositionSolverSteps;
        physicsSettings.MinVelocityForRestitution = SolverProfile.MinimumRestitutionVelocityMetersPerSecond;
        physicsSettings.TimeBeforeSleep = SolverProfile.SleepDelaySeconds;
        physicsSettings.PointVelocitySleepThreshold = SolverProfile.SleepPointVelocityMetersPerSecond;
        physicsSettings.DeterministicSimulation = SolverProfile.DeterministicSimulation;
        physicsSettings.ConstraintWarmStart = SolverProfile.ConstraintWarmStart;
        physicsSettings.CheckActiveEdges = SolverProfile.EnhancedInternalEdgeRemoval;
        System.Settings = physicsSettings;
        var config = new JobSystemThreadPoolConfig { maxJobs = 1024, maxBarriers = 1024, numThreads = Math.Max(1, Environment.ProcessorCount - 1) };
        jobSystem = new JobSystemThreadPool(in config);
        System.OnContactAdded += Contacts.OnAdded;
        System.OnContactPersisted += Contacts.OnPersisted;
        System.OnContactRemoved += Contacts.OnRemoved;
        System.OptimizeBroadPhase();
        initialized = true;
    }

    public void Step(int collisionSteps = 1, int integrationSubSteps = 1)
    {
        if (!initialized) throw new InvalidOperationException("Initialize the Jolt host before stepping it.");
        if (collisionSteps < 1) throw new ArgumentOutOfRangeException(nameof(collisionSteps));
        if (integrationSubSteps < 1) throw new ArgumentOutOfRangeException(nameof(integrationSubSteps));
        var stopwatch = Stopwatch.StartNew();
        var subStepSeconds = FixedStepSeconds / integrationSubSteps;
        for (var subStep = 0; subStep < integrationSubSteps; subStep++)
        {
            foreach (var controller in preStepControllers.ToArray()) controller(subStepSeconds);
            ApplyConfiguredDrag(subStepSeconds);
            System.Update(subStepSeconds, collisionSteps, jobSystem!);
            Contacts.Flush();
        }
        stopwatch.Stop();
        LastStepMetrics = new PhysicsStepMetrics(collisionSteps, integrationSubSteps,
            FixedStepSeconds, stopwatch.Elapsed.TotalMilliseconds, System.GetNumActiveBodies(BodyType.Rigid));
    }

    public void RegisterPreStepController(Action<float> controller)
    {
        ArgumentNullException.ThrowIfNull(controller);
        if (!preStepControllers.Contains(controller)) preStepControllers.Add(controller);
    }

    public void UnregisterPreStepController(Action<float> controller) => preStepControllers.Remove(controller);

    private void ApplyConfiguredDrag(float stepSeconds)
    {
        foreach (var id in ownedBodies)
        {
            if (!bodyProfiles.TryGetValue(id.ID, out var profile) || !profile.EnableDrag || !Bodies.IsAdded(id)) continue;
            var linearScale = MathF.Exp(-MathF.Max(0f, profile.DragCoefficientPerSecond) * stepSeconds);
            var angularScale = MathF.Exp(-MathF.Max(0f, profile.RollingDragCoefficientPerSecond) * stepSeconds);
            var linear = Bodies.GetLinearVelocity(id) * linearScale;
            var angular = Bodies.GetAngularVelocity(id) * angularScale;
            Bodies.SetLinearVelocity(in id, in linear);
            Bodies.SetAngularVelocity(in id, in angular);
        }
    }

    public BodyID CreateBoxBody(Vector3 halfExtent, Vector3 position, MotionType motionType, SourceObjectLayer layer,
        float friction = 0.8f, float restitution = 0.001f, int surfaceId = 0)
        => CreateBoxBody(halfExtent, position, motionType, layer, new SourceRigidBodyProfile { Friction = friction, Restitution = restitution }, surfaceId);

    public BodyID CreateBoxBody(Vector3 halfExtent, Vector3 position, MotionType motionType, SourceObjectLayer layer,
        SourceRigidBodyProfile profile, int surfaceId = 0)
        => CreateBoxBody(halfExtent, position, Quaternion.Identity, motionType, layer, profile, surfaceId);

    public BodyID CreateBoxBody(Vector3 halfExtent, Vector3 position, Quaternion rotation, MotionType motionType, SourceObjectLayer layer,
        SourceRigidBodyProfile profile, int surfaceId = 0)
    {
        if (!initialized) throw new InvalidOperationException("Initialize the Jolt host before creating bodies.");
        profile.Validate();
        using var boxShape = new BoxShape(halfExtent, 0.001f);
        OffsetCenterOfMassShape? offsetShape = null;
        Shape shape = boxShape;
        if (profile.CenterOfMassOffsetMeters.LengthSquared() > 1e-12f)
        {
            var centerOfMassOffset = profile.CenterOfMassOffsetMeters;
            offsetShape = new OffsetCenterOfMassShape(in centerOfMassOffset, boxShape);
            shape = offsetShape;
        }
        RVector3 precisePosition = position;
        var effectiveLayer = GetEffectiveLayer(layer, profile.SolidFlags);
        var effectiveCollisionGroup = GetEffectiveCollisionGroup(effectiveLayer, profile.CollisionGroup);
        var isTrigger = effectiveLayer == SourceObjectLayer.Trigger;
        using var settings = new BodyCreationSettings(shape, precisePosition, rotation, motionType, new ObjectLayer((ushort)effectiveLayer))
        {
            Friction = profile.Friction,
            Restitution = profile.Restitution,
            LinearDamping = profile.LinearDampingPerSecond,
            AngularDamping = profile.AngularDampingPerSecond,
            GravityFactor = profile.GravityFactor,
            MaxLinearVelocity = profile.MaxLinearVelocityMetersPerSecond,
            MaxAngularVelocity = profile.MaxAngularVelocityRadiansPerSecond,
            InertiaMultiplier = profile.InertiaScale,
            MotionQuality = profile.ContinuousCollision ? MotionQuality.LinearCast : MotionQuality.Discrete,
            AllowSleeping = profile.AllowSleep,
            IsSensor = isTrigger,
            OverrideMassProperties = OverrideMassProperties.MassAndInertiaProvided
        };
        var massProperties = settings.MassPropertiesOverride;
        massProperties.SetMassAndInertiaOfSolidBox(halfExtent * 2f, profile.MassKg);
        settings.MassPropertiesOverride = massProperties;
        var id = Bodies.CreateAndAddBody(settings, motionType == MotionType.Static ? Activation.DontActivate : Activation.Activate);
        offsetShape?.Dispose();
        if (!id.IsValid) throw new InvalidOperationException("Jolt rejected body creation.");
        Bodies.SetFriction(id, profile.Friction);
        Bodies.SetRestitution(id, profile.Restitution);
        Bodies.SetUserData(id, profile.UserData);
        ownedBodies.Add(id);
        bodySurfaces[id.ID] = surfaceId;
        bodyProfiles[id.ID] = profile;
        bodyContents[id.ID] = profile.ContentsMask;
        bodyLayers[id.ID] = effectiveLayer;
        bodyCollisionGroups[id.ID] = effectiveCollisionGroup;
        if ((profile.SolidFlags & SourceSolidFlags.NotSolid) != 0 && !isTrigger)
            nonSolidBodies.Add(id.ID);
        if (isTrigger)
        {
            sensorBodies.Add(id.ID);
            if (profile.TriggerTouchesDebris || (profile.SolidFlags & SourceSolidFlags.TriggerTouchDebris) != 0)
                triggerTouchesDebris.Add(id.ID);
        }
        return id;
    }

    public BodyID CreateStaticMeshBody(IReadOnlyList<Vector3> vertices, IReadOnlyList<IndexedTriangle> triangles,
        Vector3 position, SourceObjectLayer layer = SourceObjectLayer.World, SourceStaticMeshProfile? profile = null)
        => CreateStaticMeshBody(vertices, triangles, position, Quaternion.Identity, layer, profile);

    public BodyID CreateStaticMeshBody(SourceCookedStaticMesh mesh, Vector3 position,
        SourceObjectLayer layer = SourceObjectLayer.World)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        return CreateStaticMeshBody(mesh.Vertices, mesh.Triangles, position, Quaternion.Identity, layer, mesh.Profile);
    }

    public BodyID CreateStaticMeshBody(IReadOnlyList<Vector3> vertices, IReadOnlyList<IndexedTriangle> triangles,
        Vector3 position, Quaternion rotation, SourceObjectLayer layer = SourceObjectLayer.World, SourceStaticMeshProfile? profile = null)
    {
        if (!initialized) throw new InvalidOperationException("Initialize the Jolt host before creating bodies.");
        if (vertices.Count == 0 || triangles.Count == 0) throw new ArgumentException("A static mesh requires vertices and triangles.");
        profile ??= new SourceStaticMeshProfile();
        profile.Validate();
        if (profile.TriangleSurfaceIds is not null && profile.TriangleSurfaceIds.Length != triangles.Count)
            throw new ArgumentException("TriangleSurfaceIds must contain one surface id per triangle.", nameof(profile));
        var authoredTriangles = triangles.ToArray();
        if (profile.TriangleSurfaceIds is not null)
        {
            for (var i = 0; i < authoredTriangles.Length; i++)
            {
                var triangle = authoredTriangles[i];
                authoredTriangles[i] = new IndexedTriangle(triangle.I1, triangle.I2, triangle.I3,
                    triangle.MaterialIndex, unchecked((uint)profile.TriangleSurfaceIds[i]));
            }
        }
        using var settings = new MeshShapeSettings(vertices.ToArray().AsSpan(), authoredTriangles.AsSpan())
        {
            ActiveEdgeCosThresholdAngle = profile.ActiveEdgeCosThresholdAngle,
            MaxTrianglesPerLeaf = profile.MaxTrianglesPerLeaf,
            PerTriangleUserData = profile.TriangleSurfaceIds is not null
        };
        using var shape = settings.Create();
        RVector3 precisePosition = position;
        var effectiveLayer = GetEffectiveLayer(layer, profile.SolidFlags);
        var effectiveCollisionGroup = GetEffectiveCollisionGroup(effectiveLayer, profile.CollisionGroup);
        var isTrigger = effectiveLayer == SourceObjectLayer.Trigger;
        using var bodySettings = new BodyCreationSettings(shape, precisePosition, rotation,
            MotionType.Static, new ObjectLayer((ushort)effectiveLayer))
        {
            Friction = profile.Friction,
            Restitution = profile.Restitution,
            IsSensor = isTrigger
        };
        var id = Bodies.CreateAndAddBody(bodySettings, Activation.DontActivate);
        if (!id.IsValid) throw new InvalidOperationException("Jolt rejected static mesh creation.");
        Bodies.SetFriction(id, profile.Friction);
        Bodies.SetRestitution(id, profile.Restitution);
        Bodies.SetUserData(id, profile.UserData);
        ownedBodies.Add(id);
        bodySurfaces[id.ID] = profile.SurfaceId;
        bodyProfiles[id.ID] = new SourceRigidBodyProfile { Friction = profile.Friction, Restitution = profile.Restitution };
        bodyContents[id.ID] = profile.ContentsMask;
        bodyLayers[id.ID] = effectiveLayer;
        bodyCollisionGroups[id.ID] = effectiveCollisionGroup;
        if ((profile.SolidFlags & SourceSolidFlags.NotSolid) != 0 && !isTrigger)
            nonSolidBodies.Add(id.ID);
        if (profile.TriangleSurfaceIds is not null) perTriangleSurfaceBodies.Add(id.ID);
        if (isTrigger)
        {
            sensorBodies.Add(id.ID);
            if (profile.TriggerTouchesDebris || (profile.SolidFlags & SourceSolidFlags.TriggerTouchDebris) != 0)
                triggerTouchesDebris.Add(id.ID);
        }
        return id;
    }

    public bool IsSensor(BodyID id) => sensorBodies.Contains(id.ID);
    public bool IsSolidBody(BodyID id) => !nonSolidBodies.Contains(id.ID);
    private bool ShouldTouchTrigger(BodyID trigger, BodyID other)
    {
        if (!IsSensor(trigger)) return false;
        if (bodyLayers.TryGetValue(other.ID, out var otherLayer) && otherLayer == SourceObjectLayer.Trigger)
            return false;
        var otherIsDebris = bodyLayers.TryGetValue(other.ID, out otherLayer) && otherLayer == SourceObjectLayer.Debris;
        otherIsDebris |= (GetBodyContents(other) & SourceContents.Debris) != 0;
        return !otherIsDebris || triggerTouchesDebris.Contains(trigger.ID);
    }

    public SourceContents GetBodyContents(BodyID id) => bodyContents.TryGetValue(id.ID, out var contents) ? contents : SourceContents.Solid;
    public SourceCollisionGroup GetBodyCollisionGroup(BodyID id) =>
        bodyCollisionGroups.TryGetValue(id.ID, out var group) ? group : SourceCollisionGroup.None;
    public void SetBodyCollisionGroup(BodyID id, SourceCollisionGroup group)
    {
        EnsureBody(id);
        bodyCollisionGroups[id.ID] = group;
    }
    public void SetBodyContents(BodyID id, SourceContents contents)
    {
        EnsureBody(id);
        bodyContents[id.ID] = contents;
    }

    public SourcePhysicsWorldState CaptureState(int tick = 0)
    {
        if (!initialized) throw new InvalidOperationException("Initialize the Jolt host before capturing state.");
        var snapshots = ownedBodies
            .Where(bodyId => Bodies.IsAdded(bodyId))
            .Select(bodyId => new SourcePhysicsBodySnapshot(bodyId.ID, JoltRigidBodyState.Capture(this, bodyId)))
            .ToArray();
        return new SourcePhysicsWorldState(tick, snapshots);
    }

    public void RestoreState(SourcePhysicsWorldState state)
    {
        if (!initialized) throw new InvalidOperationException("Initialize the Jolt host before restoring state.");
        foreach (var snapshot in state.Bodies)
        {
            var bodyId = new BodyID(snapshot.BodyId);
            if (!Bodies.IsAdded(bodyId))
                throw new InvalidOperationException($"Cannot restore missing Jolt body {snapshot.BodyId}.");
            JoltRigidBodyState.Restore(this, bodyId, snapshot.State);
        }
    }

    public void AddForce(BodyID bodyId, Vector3 force)
    {
        EnsureDynamicBody(bodyId);
        Bodies.AddForce(in bodyId, in force);
    }

    public void AddTorque(BodyID bodyId, Vector3 torque)
    {
        EnsureDynamicBody(bodyId);
        Bodies.AddTorque(in bodyId, in torque);
    }

    public void AddImpulse(BodyID bodyId, Vector3 impulse)
    {
        EnsureDynamicBody(bodyId);
        Bodies.AddImpulse(in bodyId, in impulse);
        ImpulseApplied?.Invoke(new(bodyId.ID, impulse, default, false));
    }

    public void AddImpulseAtPoint(BodyID bodyId, Vector3 impulse, Vector3 worldPoint)
    {
        EnsureDynamicBody(bodyId);
        Bodies.AddImpulse(in bodyId, in impulse, in worldPoint);
        ImpulseApplied?.Invoke(new(bodyId.ID, impulse, worldPoint, true));
    }

    public bool TryGetBodyMass(BodyID bodyId, out float massKilograms)
    {
        massKilograms = 0f;
        if (!bodyId.IsValid || !Bodies.IsAdded(bodyId) || Bodies.GetMotionType(bodyId) != MotionType.Dynamic)
            return false;
        var lockRead = new BodyLockRead();
        var lockInterface = System.BodyLockInterfaceNoLock;
        lockInterface.LockRead(in bodyId, out lockRead);
        if (!lockRead.Succeeded || lockRead.Body is null)
            return false;
        var inverseMass = lockRead.Body.MotionProperties.IsNotNull
            ? lockRead.Body.MotionProperties.InverseMassUnchecked : 0f;
        lockInterface.UnlockRead(in lockRead);
        if (!float.IsFinite(inverseMass) || inverseMass <= 0f)
            return false;
        massKilograms = 1f / inverseMass;
        return float.IsFinite(massKilograms) && massKilograms > 0f;
    }

    public void MoveKinematic(BodyID bodyId, Vector3 targetPosition, Quaternion targetRotation, float deltaSeconds)
    {
        if (!initialized || !bodyId.IsValid || !Bodies.IsAdded(bodyId))
            throw new ArgumentException("Body is not in the Jolt system.", nameof(bodyId));
        if (Bodies.GetMotionType(bodyId) != MotionType.Kinematic)
            throw new InvalidOperationException("MoveKinematic requires a kinematic body.");
        if (deltaSeconds <= 0f) throw new ArgumentOutOfRangeException(nameof(deltaSeconds));
        RVector3 preciseTarget = targetPosition;
        Bodies.MoveKinematic(in bodyId, in preciseTarget, in targetRotation, deltaSeconds);
    }

    public void ActivateBody(BodyID bodyId)
    {
        EnsureBody(bodyId);
        Bodies.ActivateBody(in bodyId);
    }

    public void DeactivateBody(BodyID bodyId)
    {
        EnsureBody(bodyId);
        Bodies.DeactivateBody(in bodyId);
    }

    public void ResetSleepTimer(BodyID bodyId)
    {
        EnsureBody(bodyId);
        Bodies.ResetSleepTimer(in bodyId);
    }

    public void SetMotionType(BodyID bodyId, MotionType motionType, bool activate = true)
    {
        EnsureBody(bodyId);
        Bodies.SetMotionType(in bodyId, motionType, activate ? Activation.Activate : Activation.DontActivate);
    }

    public void SetLinearVelocity(BodyID bodyId, Vector3 velocity)
    {
        EnsureBody(bodyId);
        Bodies.SetLinearVelocity(in bodyId, in velocity);
    }

    public void SetAngularVelocity(BodyID bodyId, Vector3 velocity)
    {
        EnsureBody(bodyId);
        Bodies.SetAngularVelocity(in bodyId, in velocity);
    }

    public void SetGravityFactor(BodyID bodyId, float gravityFactor)
    {
        EnsureBody(bodyId);
        if (!float.IsFinite(gravityFactor)) throw new ArgumentOutOfRangeException(nameof(gravityFactor));
        Bodies.SetGravityFactor(in bodyId, gravityFactor);
    }

    public void SetFriction(BodyID bodyId, float friction)
    {
        EnsureBody(bodyId);
        if (!float.IsFinite(friction) || friction < 0f) throw new ArgumentOutOfRangeException(nameof(friction));
        Bodies.SetFriction(bodyId, friction);
    }

    public void SetRestitution(BodyID bodyId, float restitution)
    {
        EnsureBody(bodyId);
        if (!float.IsFinite(restitution) || restitution < 0f) throw new ArgumentOutOfRangeException(nameof(restitution));
        Bodies.SetRestitution(bodyId, restitution);
    }

    /// <summary>
    /// Applies Jolt's explicit buoyancy impulse primitive. Fluid-volume discovery and Source's
    /// controller parameters remain caller-owned, matching VPhysics' separate fluid controller.
    /// </summary>
    public bool ApplyBuoyancyImpulse(BodyID bodyId, Vector3 surfacePosition, Vector3 surfaceNormal,
        float buoyancy, float linearDrag, float angularDrag, Vector3 fluidVelocity, float deltaSeconds)
    {
        EnsureDynamicBody(bodyId);
        if (!IsFinite(surfaceNormal) || surfaceNormal.LengthSquared() < 1e-12f)
            throw new ArgumentOutOfRangeException(nameof(surfaceNormal));
        if (!float.IsFinite(buoyancy) || buoyancy < 0f) throw new ArgumentOutOfRangeException(nameof(buoyancy));
        if (!float.IsFinite(linearDrag) || linearDrag < 0f) throw new ArgumentOutOfRangeException(nameof(linearDrag));
        if (!float.IsFinite(angularDrag) || angularDrag < 0f) throw new ArgumentOutOfRangeException(nameof(angularDrag));
        if (!IsFinite(fluidVelocity)) throw new ArgumentOutOfRangeException(nameof(fluidVelocity));
        if (!float.IsFinite(deltaSeconds) || deltaSeconds <= 0f) throw new ArgumentOutOfRangeException(nameof(deltaSeconds));
        var gravity = System.Gravity;
        RVector3 preciseSurfacePosition = surfacePosition;
        return Bodies.ApplyBuoyancyImpulse(in bodyId, in preciseSurfacePosition, in surfaceNormal,
            buoyancy, linearDrag, angularDrag, in fluidVelocity, in gravity, deltaSeconds);
    }

    /// Mirrors the referenced VPhysics FluidStartTouch velocity damping. The
    /// fluid controller supplies a unit outward surface normal and density;
    /// this method does not infer either value from geometry.
    public void ApplySourceFluidTouchDamping(BodyID bodyId, Vector3 surfaceNormal, float fluidDensity,
        float linearDrag, float angularDrag, float simulationTimestep)
    {
        EnsureDynamicBody(bodyId);
        if (!IsFinite(surfaceNormal) || surfaceNormal.LengthSquared() < 1e-12f)
            throw new ArgumentOutOfRangeException(nameof(surfaceNormal));
        if (!float.IsFinite(fluidDensity) || fluidDensity < 0f) throw new ArgumentOutOfRangeException(nameof(fluidDensity));
        if (!float.IsFinite(linearDrag) || linearDrag < 0f) throw new ArgumentOutOfRangeException(nameof(linearDrag));
        if (!float.IsFinite(angularDrag) || angularDrag < 0f) throw new ArgumentOutOfRangeException(nameof(angularDrag));
        if (!float.IsFinite(simulationTimestep) || simulationTimestep <= 0f)
            throw new ArgumentOutOfRangeException(nameof(simulationTimestep));

        var velocity = Bodies.GetLinearVelocity(bodyId);
        var speedSquared = velocity.LengthSquared();
        var unitVelocity = speedSquared > 1e-12f ? velocity / MathF.Sqrt(speedSquared) : Vector3.Zero;
        var dragScale = fluidDensity * simulationTimestep;
        var linearScale = Math.Clamp(0.5f * Vector3.Dot(unitVelocity, -surfaceNormal) * linearDrag * dragScale, 0f, 1f);
        var angularScale = Math.Clamp(0.25f * angularDrag * dragScale, 0f, 1f);
        var dampedVelocity = velocity * (1f - linearScale);
        var angularVelocity = Bodies.GetAngularVelocity(bodyId) * (1f - angularScale);
        Bodies.SetLinearVelocity(in bodyId, in dampedVelocity);
        Bodies.SetAngularVelocity(in bodyId, in angularVelocity);
    }

    private static bool IsFinite(Vector3 value) => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

    public SourceSurface GetBodySurface(BodyID id) => Surfaces.Get(bodySurfaces.TryGetValue(id.ID, out var surfaceId) ? surfaceId : 0);
    public SourceSurface GetBodySurface(BodyID id, out int surfaceId)
    {
        surfaceId = bodySurfaces.TryGetValue(id.ID, out var storedId) ? storedId : 0;
        return Surfaces.Get(surfaceId);
    }

    public SourceSurface GetBodySurface(BodyID id, SubShapeID subShapeId, out int surfaceId)
    {
        surfaceId = bodySurfaces.TryGetValue(id.ID, out var storedId) ? storedId : 0;
        if (perTriangleSurfaceBodies.Contains(id.ID))
        {
            var shape = Bodies.GetShape(in id);
            if (shape is not null && shape.SubType == ShapeSubType.Mesh)
                surfaceId = unchecked((int)GetMeshTriangleUserData(shape.Handle, subShapeId.Value));
        }
        return Surfaces.Get(surfaceId);
    }

    // Contact callbacks already provide a locked Body view. Resolve mesh
    // triangle metadata through it instead of reacquiring BodyInterface's
    // lock from inside Jolt's simulation callback.
    internal SourceSurface GetBodySurface(in Body body, SubShapeID subShapeId, out int surfaceId)
    {
        surfaceId = bodySurfaces.TryGetValue(body.ID.ID, out var storedId) ? storedId : 0;
        if (perTriangleSurfaceBodies.Contains(body.ID.ID) && body.Shape is { } shape &&
            shape.SubType == ShapeSubType.Mesh)
            surfaceId = unchecked((int)GetMeshTriangleUserData(shape.Handle, subShapeId.Value));
        return Surfaces.Get(surfaceId);
    }

    public void DestroyBody(BodyID id)
    {
        if (!initialized || !id.IsValid || !Bodies.IsAdded(id)) return;
        foreach (var constraint in ownedConstraints.Where(constraint => ReferencesBody(constraint, id)).ToArray())
            RemoveConstraint(constraint);
        var ownedId = id;
        Bodies.DestroyBody(in ownedId);
        ownedBodies.RemoveAll(candidate => candidate.ID == id.ID);
        bodySurfaces.Remove(id.ID);
        bodyProfiles.Remove(id.ID);
        bodyContents.Remove(id.ID);
        perTriangleSurfaceBodies.Remove(id.ID);
        sensorBodies.Remove(id.ID);
        bodyLayers.Remove(id.ID);
        bodyCollisionGroups.Remove(id.ID);
        triggerTouchesDebris.Remove(id.ID);
        nonSolidBodies.Remove(id.ID);
    }

    /// Adds a native Jolt constraint under the host's lifetime owner. The host
    /// removes it before either referenced body or the native PhysicsSystem is
    /// destroyed, which is required by the installed JoltPhysicsSharp wrapper.
    public void AddConstraint(Constraint constraint)
    {
        ArgumentNullException.ThrowIfNull(constraint);
        if (!initialized) throw new InvalidOperationException("Initialize the Jolt host before adding constraints.");
        if (constraint is TwoBodyConstraint twoBody &&
            (!Bodies.IsAdded(twoBody.Body1.ID) || !Bodies.IsAdded(twoBody.Body2.ID)))
            throw new ArgumentException("Every two-body constraint body must belong to this host.", nameof(constraint));
        if (ownedConstraints.Contains(constraint)) return;
        System.AddConstraint(constraint);
        ownedConstraints.Add(constraint);
    }

    public bool RemoveConstraint(Constraint constraint)
    {
        ArgumentNullException.ThrowIfNull(constraint);
        if (!ownedConstraints.Remove(constraint)) return false;
        if (initialized) System.RemoveConstraint(constraint);
        constraint.Dispose();
        return true;
    }

    public void Dispose()
    {
        if (!initialized) return;
        System.OnContactAdded -= Contacts.OnAdded;
        System.OnContactPersisted -= Contacts.OnPersisted;
        System.OnContactRemoved -= Contacts.OnRemoved;
        // Jolt requires constraints to be detached before their bodies and
        // before the PhysicsSystem native handle is released.
        foreach (var constraint in ownedConstraints.ToArray())
        {
            System.RemoveConstraint(constraint);
            constraint.Dispose();
        }
        ownedConstraints.Clear();
        foreach (var id in ownedBodies)
        {
            var ownedId = id;
            if (Bodies.IsAdded(ownedId)) Bodies.DestroyBody(in ownedId);
        }
        ownedBodies.Clear();
        bodySurfaces.Clear();
        bodyProfiles.Clear();
        bodyContents.Clear();
        perTriangleSurfaceBodies.Clear();
        sensorBodies.Clear();
        bodyLayers.Clear();
        bodyCollisionGroups.Clear();
        triggerTouchesDebris.Clear();
        nonSolidBodies.Clear();
        sourceSimShapeFilter?.Dispose();
        sourceSimShapeFilter = null;
        preStepControllers.Clear();
        // PhysicsSystem owns the native contact/activation listeners and the
        // native world handle. It must be destroyed after its bodies and
        // listeners are detached, but before the Jolt foundation shuts down.
        var nativeSystem = system;
        system = null;
        nativeSystem?.Dispose();
        jobSystem?.Dispose();
        jobSystem = null;
        collisionLayers?.Dispose();
        collisionLayers = null;
        if (ownsFoundationReference)
        {
            lock (FoundationGate)
            {
                if (--foundationUsers == 0) Foundation.Shutdown();
                ownsFoundationReference = false;
            }
        }
        initialized = false;
    }

    private void EnsureDynamicBody(BodyID bodyId)
    {
        if (!initialized || !bodyId.IsValid || !Bodies.IsAdded(bodyId))
            throw new ArgumentException("Body is not in the Jolt system.", nameof(bodyId));
        if (Bodies.GetMotionType(bodyId) == MotionType.Static)
            throw new InvalidOperationException("Forces and impulses require a non-static body.");
    }

    private void EnsureBody(BodyID bodyId)
    {
        if (!initialized || !bodyId.IsValid || !Bodies.IsAdded(bodyId))
            throw new ArgumentException("Body is not in the Jolt system.", nameof(bodyId));
    }

    private static bool ReferencesBody(Constraint constraint, BodyID bodyId) =>
        constraint is TwoBodyConstraint twoBody &&
        (twoBody.Body1.ID.ID == bodyId.ID || twoBody.Body2.ID.ID == bodyId.ID);

}
