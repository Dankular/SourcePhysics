using System.Numerics;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class MovementMotorTests
{
    [Fact]
    public void JoltHostInitializesAndStepsFixedClock()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile(), 1f / 66f);
        host.Initialize(1024, 0, 1024, 256);
        host.Surfaces.Register(7, new SourceSurface("ice", 0.1f, 0.25f));
        var body = host.CreateBoxBody(new Vector3(1f, 0.5f, 1f), new Vector3(0f, -0.5f, 0f), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World, surfaceId: 7);
        using var queries = new JoltMovementQueries(host);
        var hit = queries.SweepPlayer(new Vector3(0f, 2f, 0f), new Vector3(0f, -2f, 0f), false);
        using var probe = new JoltCharacterProbe(host, new Vector3(0f, 2f, 0f), new SourceMovementProfile());
        var playerLayer = new JoltPhysicsSharp.ObjectLayer((ushort)SourceObjectLayer.Player);
        probe.Update(1f / 66f, new Vector3(0f, -1f, 0f), playerLayer);
        host.Step();
        Assert.True(body.IsValid);
        Assert.True(hit.Fraction < 1f);
        Assert.Equal(0.1f, hit.Friction);
        Assert.InRange(hit.Position.Y, -0.001f, 0.001f);
        Assert.Equal(new Vector3(0f, -SourceUnits.ToMeters(new SourceMovementProfile().GravitySourceUnitsPerSecondSquared), 0f), host.System.Gravity);
    }

    [Fact]
    public void FixedStepClockPreservesAuthoritativeTickOrderAndRemainder()
    {
        var clock = new SourceFixedStepClock(0.1f);
        var ticks = new List<(int Tick, float Dt)>();
        Assert.Equal(2, clock.Advance(0.25f, (tick, dt) => ticks.Add((tick, dt))));
        Assert.Equal(new[] { 0, 1 }, ticks.Select(value => value.Tick));
        Assert.All(ticks, value => Assert.Equal(0.1f, value.Dt));
        Assert.InRange(clock.RemainderSeconds, 0.0499, 0.0501);
        Assert.Equal(1, clock.Advance(0.05f, (tick, _) => ticks.Add((tick, 0f))));
        Assert.Equal(3, ticks.Count);
        Assert.Equal(2, ticks[2].Tick);
    }

    [Fact]
    public void FixedStepClockRejectsInvalidInputAndConsumesLongFramesWithoutDrift()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new SourceFixedStepClock(0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SourceFixedStepClock(float.NaN));

        var clock = new SourceFixedStepClock(1f / 66f);
        var ticks = new List<int>();
        Assert.Throws<ArgumentOutOfRangeException>(() => clock.Advance(-1f, (_, _) => { }));
        Assert.Throws<ArgumentOutOfRangeException>(() => clock.Advance(float.NaN, (_, _) => { }));
        Assert.Throws<ArgumentNullException>(() => clock.Advance(0.1f, null!));

        var steps = clock.Advance(1f, (tick, _) => ticks.Add(tick));
        Assert.Equal(66, steps);
        Assert.Equal(Enumerable.Range(0, 66), ticks);
        Assert.InRange(clock.RemainderSeconds, 0d, 1e-7d);
    }

    [Fact]
    public void StridePhysicsHostInitializationIsIdempotentForDependentScripts()
    {
        var script = new StrideSourcePhysicsScript { FixedStepSeconds = 1f / 66f };
        script.EnsureStarted();
        var host = script.Host;
        script.EnsureStarted();
        Assert.Same(host, script.Host);
        Assert.Equal(0, script.SimulationTick);
        script.EnablePhysicsRecording();
        script.Advance(script.FixedStepSeconds);
        Assert.Single(script.PhysicsRecording!.Frames);
        script.Cancel();
    }

    [Fact]
    public void JoltSweepReportsRotatedSlopeNormal()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        host.CreateBoxBody(new Vector3(5f, 0.05f, 5f), new Vector3(5f, 0f, 0f), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 6f), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World, new SourceRigidBodyProfile());
        using var queries = new JoltMovementQueries(host);
        var hit = queries.SweepPlayer(new Vector3(5f, 2f, 0f), new Vector3(5f, -2f, 0f), false);
        Assert.True(hit.Fraction < 1f);
        Assert.InRange(hit.Normal.Y, 0.7f, 1f);
        Assert.InRange(hit.Normal.Length(), 0.999f, 1.001f);
    }

    [Fact]
    public void JoltProjectileQueryReportsContinuousContactAndSurfaceRestitution()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        host.Surfaces.Register(9, new SourceSurface("metal", 0.8f, 0.65f, ThicknessInches: 2.5f));
        host.CreateBoxBody(new Vector3(2, 0.05f, 2), new Vector3(0, 0, 0), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, new SourceRigidBodyProfile(), 9);
        using var queries = new JoltProjectileQueries(host, 1f);
        Assert.True(queries.Sweep(new(0, 2, 0), new(0, -2, 0), out var hit));
        Assert.True(hit.BodyId >= 0);
        Assert.InRange(hit.Normal.Y, 0.99f, 1.01f);
        Assert.Equal(0.65f, hit.Restitution);
        Assert.Equal(9, hit.SurfaceId);
        Assert.Equal(2.5f, hit.ThicknessInches);

        using var penetrating = new JoltProjectileQueries(host, 1f, penetrationCost: (_, _) => 1f);
        Assert.True(penetrating.TryPenetrate(hit.Position, in hit, new(0, -1, 0), 2f,
            out var exit, out var exitVelocity, out var consumed));
        Assert.Equal(1f, consumed);
        Assert.Equal(new Vector3(0, -1, 0), exitVelocity);
        Assert.InRange(Vector3.Distance(exit, hit.Position), SourceUnits.ToMeters(2.49f), SourceUnits.ToMeters(2.51f));
    }

    [Fact]
    public void SourceFluidTouchDampingMatchesReferencedLinearAndAngularTerms()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.SetLinearVelocity(body, new(0f, -4f, 0f));
        host.SetAngularVelocity(body, new(2f, 0f, 0f));

        host.ApplySourceFluidTouchDamping(body, Vector3.UnitY, 2f, 1f, 1f, 0.5f);

        Assert.Equal(new Vector3(0f, -2f, 0f), host.Bodies.GetLinearVelocity(body));
        Assert.Equal(new Vector3(1.5f, 0f, 0f), host.Bodies.GetAngularVelocity(body));
    }

    [Fact]
    public void JoltHitscanReturnsSurfaceNormalAndMaterialIdentity()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        host.Surfaces.Register(12, new SourceSurface("target", 0.5f, 0.2f));
        host.CreateBoxBody(new Vector3(0.05f, 2f, 2f), new Vector3(1f, 0, 0), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, new SourceRigidBodyProfile(), 12);
        var hitscan = new JoltHitscanQueries(host);
        Assert.True(hitscan.Cast(Vector3.Zero, Vector3.UnitX, 5f, out var hit));
        Assert.Equal(12, hit.SurfaceId);
        Assert.InRange(hit.Normal.X, -1.01f, -0.99f);
        Assert.InRange(hit.Fraction, 0f, 1f);
    }

    [Fact]
    public void WeaponQueriesUseExplicitSensorPolicy()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var trigger = host.CreateBoxBody(new(0.05f, 1f, 1f), new(1f, 0, 0), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.Trigger);
        var world = host.CreateBoxBody(new(0.05f, 1f, 1f), new(3f, 0, 0), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);

        var gameplayTrace = new JoltHitscanQueries(host);
        Assert.True(gameplayTrace.Cast(Vector3.Zero, Vector3.UnitX, 5f, out var worldHit));
        Assert.Equal(world.ID, unchecked((uint)worldHit.BodyId));

        var triggerTrace = new JoltHitscanQueries(host, includeSensors: true);
        Assert.True(triggerTrace.Cast(Vector3.Zero, Vector3.UnitX, 5f, out var triggerHit));
        Assert.Equal(trigger.ID, unchecked((uint)triggerHit.BodyId));
    }

    [Fact]
    public void JoltPointVelocityIncludesAngularPlatformMotion()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.Bodies.SetAngularVelocity(body, Vector3.UnitY);
        host.Bodies.ActivateBody(body);
        Assert.InRange(host.Bodies.GetAngularVelocity(body).Y, 0.99f, 1.01f);
        host.Step();
        using var queries = new JoltMovementQueries(host);
        var velocity = queries.GetBodyPointVelocity(unchecked((int)body.ID), new Vector3(1, 0, 0));
        Assert.InRange(velocity.Z, -1.01f, -0.99f);
    }

    [Fact]
    public void RigidBodyForceAndPointImpulseCommandsReachJoltAtFixedStep()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f, LinearDampingPerSecond = 0f, AngularDampingPerSecond = 0f });

        host.AddForce(body, new(10f, 0f, 0f));
        host.AddImpulseAtPoint(body, new(0f, 0f, 1f), new(0f, 1f, 0f));
        host.Step();

        Assert.True(host.Bodies.GetLinearVelocity(body).X > 0f);
        Assert.NotEqual(Vector3.Zero, host.Bodies.GetAngularVelocity(body));
    }

    [Fact]
    public void RigidBodyCenterOfMassOffsetIsAppliedThroughJoltShapeDecorator()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile
            {
                GravityFactor = 0f,
                CenterOfMassOffsetMeters = new Vector3(0.25f, 0f, 0f)
            });

        var center = (Vector3)host.Bodies.GetRCenterOfMassPosition(body);
        Assert.InRange(center.X, 0.249f, 0.251f);
    }

    [Fact]
    public void KinematicBodyUsesFixedStepMoveKinematicBridge()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Kinematic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.MoveKinematic(body, new Vector3(2f, 0f, 0f), Quaternion.Identity, host.FixedStepSeconds);
        host.Step();
        var position = (Vector3)host.Bodies.GetRCenterOfMassPosition(body);
        Assert.InRange(position.X, 1.99f, 2.01f);
    }

    [Fact]
    public void RigidBodySleepWakeCommandsMapToJoltActivationState()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.DeactivateBody(body);
        Assert.False(host.Bodies.IsActive(body));
        host.ResetSleepTimer(body);
        host.ActivateBody(body);
        Assert.True(host.Bodies.IsActive(body));
    }

    [Fact]
    public void RuntimeMotionTypeTransitionEnablesKinematicControllerPath()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.SetMotionType(body, JoltPhysicsSharp.MotionType.Kinematic);
        host.MoveKinematic(body, new Vector3(1f, 0f, 0f), Quaternion.Identity, host.FixedStepSeconds);
        host.Step();
        Assert.Equal(JoltPhysicsSharp.MotionType.Kinematic, host.Bodies.GetMotionType(body));
        Assert.InRange(((Vector3)host.Bodies.GetRCenterOfMassPosition(body)).X, 0.99f, 1.01f);
    }

    [Fact]
    public void PhysicsStepMetricsReportConfiguredSubstepsAndActiveBodies()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.Step(collisionSteps: 2, integrationSubSteps: 3);
        Assert.Equal(2, host.LastStepMetrics.CollisionSteps);
        Assert.Equal(3, host.LastStepMetrics.IntegrationSubSteps);
        Assert.Equal(host.FixedStepSeconds, host.LastStepMetrics.SimulatedSeconds);
        Assert.True(host.LastStepMetrics.ElapsedMilliseconds >= 0d);
        Assert.True(host.LastStepMetrics.ActiveBodyCount >= 1);
    }


    [Fact]
    public void RuntimeRigidBodyPropertySettersReachJolt()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 1f });
        host.SetLinearVelocity(body, new(1f, 2f, 3f));
        host.SetAngularVelocity(body, new(0f, 1f, 0f));
        host.SetGravityFactor(body, 0f);
        host.SetFriction(body, 0.25f);
        host.SetRestitution(body, 0.5f);
        Assert.Equal(new Vector3(1f, 2f, 3f), host.Bodies.GetLinearVelocity(body));
        Assert.Equal(new Vector3(0f, 1f, 0f), host.Bodies.GetAngularVelocity(body));
        Assert.Equal(0f, host.Bodies.GetGravityFactor(body));
        Assert.Equal(0.25f, host.Bodies.GetFriction(body));
        Assert.Equal(0.5f, host.Bodies.GetRestitution(body));
    }

    [Fact]
    public void PhysicsWorldComparatorRejectsRigidBodyCorrectionDivergence()
    {
        var expected = new SourcePhysicsWorldState(4, new[]
        {
            new SourcePhysicsBodySnapshot(7, new SourceRigidBodyState(Vector3.Zero, Quaternion.Identity,
                Vector3.Zero, Vector3.Zero, true))
        });
        var actual = new SourcePhysicsWorldState(4, new[]
        {
            new SourcePhysicsBodySnapshot(7, new SourceRigidBodyState(new(0.1f, 0f, 0f), Quaternion.Identity,
                Vector3.UnitX, Vector3.Zero, true))
        });
        var comparison = PhysicsWorldComparator.Compare(expected, actual);
        Assert.False(comparison.Passes(0.01f, 0.001f, 0.01f, 0.01f));
        Assert.Contains("position:7", comparison.Errors);
        Assert.Contains("linear-velocity:7", comparison.Errors);
    }

    [Fact]
    public void PhysicsWorldComparatorRejectsRigidBodyPropertyDivergence()
    {
        var expected = new SourcePhysicsWorldState(1, new[]
        {
            new SourcePhysicsBodySnapshot(2, new SourceRigidBodyState(Vector3.Zero, Quaternion.Identity,
                Vector3.Zero, Vector3.Zero, true, JoltPhysicsSharp.MotionType.Dynamic, 1f, 0.4f, 0.1f,
                SourceContents.Solid))
        });
        var actual = new SourcePhysicsWorldState(1, new[]
        {
            new SourcePhysicsBodySnapshot(2, new SourceRigidBodyState(Vector3.Zero, Quaternion.Identity,
                Vector3.Zero, Vector3.Zero, true, JoltPhysicsSharp.MotionType.Dynamic, 0f, 0.8f, 0.001f,
                SourceContents.Water))
        });

        var comparison = PhysicsWorldComparator.Compare(expected, actual);

        Assert.False(comparison.Passes(0.001f, 0.001f, 0.001f, 0.001f));
        Assert.Contains("gravity-factor:2", comparison.Errors);
        Assert.Contains("friction:2", comparison.Errors);
        Assert.Contains("contents:2", comparison.Errors);
    }

    [Fact]
    public void PhysicsWorldComparatorUsesNumericTolerancesForBodyDrift()
    {
        var expected = new SourcePhysicsWorldState(1, new[]
        {
            new SourcePhysicsBodySnapshot(3, new SourceRigidBodyState(Vector3.Zero, Quaternion.Identity,
                Vector3.Zero, Vector3.Zero, true))
        });
        var actual = new SourcePhysicsWorldState(1, new[]
        {
            new SourcePhysicsBodySnapshot(3, new SourceRigidBodyState(new(0.0001f, 0f, 0f), Quaternion.Identity,
                new(0.0001f, 0f, 0f), Vector3.Zero, true))
        });

        var comparison = PhysicsWorldComparator.Compare(expected, actual);

        Assert.True(comparison.Passes(0.001f, 0.001f, 0.001f, 0.001f));
        Assert.False(comparison.Passes(0.00001f, 0.001f, 0.001f, 0.001f));
    }

    [Fact]
    public void PreStepControllerRunsOncePerConfiguredIntegrationSubstep()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile(), 1f / 66f);
        host.Initialize(1024, 0, 1024, 256);
        var calls = 0;
        var elapsed = 0f;
        Action<float> controller = dt => { calls++; elapsed += dt; };
        host.RegisterPreStepController(controller);
        host.Step(integrationSubSteps: 4);
        Assert.Equal(4, calls);
        Assert.Equal(host.FixedStepSeconds, elapsed, 6);
        host.UnregisterPreStepController(controller);
    }

    [Fact]
    public void PhysicsWorldStateCapturesRestoresAndSerializesOwnedBodies()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, new(2f, 3f, 4f), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        host.Bodies.SetLinearVelocity(body, new(1f, 2f, 3f));
        var captured = host.CaptureState(17);
        var restored = SourcePhysicsWorldState.FromJson(captured.ToJson());

        host.Bodies.SetRPositionAndRotation(body, new(20f, 20f, 20f), Quaternion.Identity, JoltPhysicsSharp.Activation.Activate);
        host.Bodies.SetLinearVelocity(body, Vector3.Zero);
        host.RestoreState(restored);

        var current = host.CaptureState(17).Bodies.Single(snapshot => snapshot.BodyId == body.ID);
        var expected = restored.Bodies.Single(snapshot => snapshot.BodyId == body.ID);
        Assert.True(SourcePhysicsStateMath.NearlyEqual(expected, current, 0.000001f, 0.000001f, 0.000001f));
    }

    [Fact]
    public void PhysicsRecordingCapturesBodyStateAndExplicitImpulseStream()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f, UserData = 77 });
        host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, new SourceRigidBodyProfile { UserData = 88 });
        using var recording = new SourcePhysicsRecording(host);
        host.AddImpulseAtPoint(body, Vector3.UnitX, new(0f, 1f, 0f));
        recording.Capture(12);

        var artifact = SourcePhysicsRecordingArtifact.FromJson(recording.ToJson());
        var frame = Assert.Single(artifact.Frames);
        Assert.Equal(12, frame.Tick);
        Assert.Equal((ulong)77, frame.Bodies.Single(snapshot => snapshot.State.UserData == 77).State.UserData);
        var impulse = Assert.Single(frame.Impulses);
        Assert.Equal(body.ID, impulse.BodyId);
        Assert.Equal(Vector3.UnitX, impulse.Impulse);
        Assert.Equal(new Vector3(0f, 1f, 0f), impulse.WorldPoint);
        Assert.True(impulse.AtPoint);

        var altered = artifact with
        {
            Frames = new List<SourcePhysicsTickFrame>
            {
                frame with
                {
                    Bodies = frame.Bodies.Select(snapshot => snapshot.State.UserData == 77
                        ? snapshot with { State = snapshot.State with { Position = Vector3.UnitX } }
                        : snapshot).ToArray()
                }
            }
        };
        var comparison = SourcePhysicsRecordingComparator.Compare(artifact, altered);
        Assert.False(comparison.Passes(0.001f, 0.001f, 0.001f, 0.001f));
        Assert.Contains(comparison.Errors, error => error.Contains("position:"));

        host.Step();
        recording.Capture(13);
        var withContacts = SourcePhysicsRecordingArtifact.FromJson(recording.ToJson());
        Assert.NotEmpty(withContacts.Frames[1].Contacts);
    }

    [Fact]
    public void RigidBodyStateRestoresTransformAndVelocities()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, new(2, 3, 4), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f, UserData = 0xCAFE_BEEFul });
        host.Bodies.SetLinearVelocity(body, new(1, 2, 3));
        host.Bodies.SetAngularVelocity(body, new(0, 1, 0));
        var saved = JoltRigidBodyState.Capture(host, body);
        var changed = saved with { Position = new(20, 20, 20), LinearVelocity = Vector3.Zero, AngularVelocity = Vector3.Zero };
        JoltRigidBodyState.Restore(host, body, changed);
        var restored = JoltRigidBodyState.Capture(host, body);
        Assert.Equal(changed.Position, restored.Position);
        Assert.Equal(changed.LinearVelocity, restored.LinearVelocity);
        Assert.Equal(changed.AngularVelocity, restored.AngularVelocity);
        Assert.Equal(0xCAFE_BEEFul, restored.UserData);

        var changedIdentity = changed with { UserData = 0x1234UL };
        JoltRigidBodyState.Restore(host, body, changedIdentity);
        Assert.Equal(0x1234UL, JoltRigidBodyState.Capture(host, body).UserData);
    }

    [Fact]
    public void SourceMotorClimbsAWithinStepHeightJoltStair()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(5, 0.05f, 10), new(0, -0.05f, 0), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        host.CreateBoxBody(new(0.2f, SourceUnits.ToMeters(9f) * 0.5f, 10), new(0.2f, SourceUnits.ToMeters(9f) * 0.5f, 0), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        using var queries = new JoltMovementQueries(host);
        var floorHit = queries.SweepPlayer(new(-1, 0.05f, 0), new(-1, 0.05f - SourceUnits.ToMeters(2), 0), false);
        var touchingHit = queries.SweepPlayer(new(-1, 0, 0), new(-1, -SourceUnits.ToMeters(2), 0), false);
        Assert.False(floorHit.StartSolid);
        Assert.False(touchingHit.StartSolid);
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new(-1, 0.05f, 0));
        for (var tick = 0; tick < 132; tick++) motor.Tick(new SourceInput(new(0, 1), Buttons.None), 1f / 66f);
        Assert.InRange(motor.State.Position.Y, SourceUnits.ToMeters(9f) - 0.002f, SourceUnits.ToMeters(9f) + 0.002f);
        Assert.Equal(GroundState.Grounded, motor.State.Ground);
    }

    [Fact]
    public void JoltContactManifoldRoutesGameplayContactAndUsesSourceCombine()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        host.Surfaces.Register(3, new SourceSurface("stone", 0.25f, 0.4f));
        host.Surfaces.Register(4, new SourceSurface("rubber", 0.81f, 0.1f));
        var floorBody = host.CreateBoxBody(new(2, 0.05f, 2), new(0, -0.05f, 0), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, new SourceRigidBodyProfile(), 3);
        var dynamicBody = host.CreateBoxBody(new(0.25f, 0.25f, 0.25f), new(0, 1, 0), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 1f }, 4);
        var added = 0;
        SourceContactEvent contact = default;
        host.Contacts.ContactAdded += value => { added++; contact = value; };
        host.Contacts.ContactPersisted += value => { added++; contact = value; };
        for (var tick = 0; tick < 90 && added == 0; tick++) host.Step();
        Assert.True(added > 0);
        Assert.True(contact.BodyA == dynamicBody.ID || contact.BodyB == dynamicBody.ID);
        Assert.Equal(floorBody.ID, contact.BodyA == dynamicBody.ID ? contact.BodyB : contact.BodyA);
        Assert.True(float.IsFinite(contact.ContactPoint.X) && float.IsFinite(contact.ContactPoint.Y) &&
            float.IsFinite(contact.ContactPoint.Z));
        Assert.True(float.IsFinite(contact.PenetrationDepth));
        Assert.True(contact.Normal.LengthSquared() > 0.5f);
        Assert.Equal(MathF.Sqrt(0.25f * 0.81f), SourceSurfaceRegistry.CombineFriction(
            host.Surfaces.Get(3), host.Surfaces.Get(4)), 5);
        Assert.Equal(0.4f, SourceSurfaceRegistry.CombineRestitution(host.Surfaces.Get(3), host.Surfaces.Get(4)));
    }

    [Fact]
    public void TriggerLayerGeneratesContactWithoutBlockingDynamicBody()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var trigger = host.CreateBoxBody(new(1, 0.05f, 1), new(0, 0, 0), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.Trigger);
        var dynamicBody = host.CreateBoxBody(new(0.1f, 0.1f, 0.1f), new(0, 0.5f, 0), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic);
        var triggerEvents = 0;
        var sourceTriggerEvents = 0;
        host.Contacts.TriggerEntered += _ => sourceTriggerEvents++;
        host.Contacts.ContactAdded += value =>
        {
            if (value.BodyA == trigger.ID || value.BodyB == trigger.ID) triggerEvents++;
        };
        for (var tick = 0; tick < 45; tick++) host.Step();
        Assert.True(triggerEvents > 0);
        Assert.True(sourceTriggerEvents > 0);
        var position = (Vector3)host.Bodies.GetRCenterOfMassPosition(dynamicBody);
        Assert.True(position.Y < -0.1f);
    }

    [Fact]
    public void AuthoredCollisionPolicyCanDisableAConfiguredLayerPair()
    {
        var policy = new SourceCollisionPolicy();
        policy.SetCollision(SourceObjectLayer.World, SourceObjectLayer.Dynamic, false);
        using var host = new JoltPhysicsHost(new SourceMovementProfile(), collisionPolicy: policy);
        host.Initialize(1024, 0, 1024, 256);
        host.CreateBoxBody(new(2f, 0.05f, 2f), new(0f, -0.05f, 0f), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        var dynamicBody = host.CreateBoxBody(new(0.25f, 0.25f, 0.25f), new(0f, 1f, 0f), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 1f });
        for (var tick = 0; tick < 66; tick++) host.Step();
        var position = (Vector3)host.Bodies.GetRCenterOfMassPosition(dynamicBody);
        Assert.True(position.Y < -0.1f);
    }

    [Fact]
    public void JoltStaticMeshBodyProvidesTriangleCollisionForPlayerSweep()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        var vertices = new[]
        {
            new Vector3(-5, 0, -5), new Vector3(5, 0, -5),
            new Vector3(5, 0, 5), new Vector3(-5, 0, 5)
        };
        var triangles = new[] { Triangle(0, 1, 2), Triangle(0, 2, 3) };
        host.CreateStaticMeshBody(vertices, triangles, Vector3.Zero,
            SourceObjectLayer.World, new SourceStaticMeshProfile { SurfaceId = 6 });
        using var queries = new JoltMovementQueries(host);
        var hit = queries.SweepPlayer(new(0, 2, 0), new(0, -2, 0), false);
        Assert.True(hit.Fraction < 1f);
        Assert.InRange(hit.Normal.Y, 0.99f, 1.01f);
        Assert.Equal(6, hit.SurfaceId);
    }

    [Fact]
    public void JoltStaticMeshBodyPreservesAuthoredPerTriangleSurfaceIds()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.Surfaces.Register(17, new SourceSurface("left", 0.15f, 0.1f));
        host.Surfaces.Register(23, new SourceSurface("right", 0.85f, 0.7f));
        var vertices = new[]
        {
            new Vector3(-5, 0, -5), new Vector3(0, 0, -5), new Vector3(0, 0, 5),
            new Vector3(5, 0, -5), new Vector3(5, 0, 5)
        };
        var triangles = new[] { Triangle(0, 1, 2), Triangle(1, 3, 4) };
        host.CreateStaticMeshBody(vertices, triangles, Vector3.Zero, SourceObjectLayer.World,
            new SourceStaticMeshProfile { SurfaceId = 0, TriangleSurfaceIds = new[] { 17, 23 } });

        using var queries = new JoltMovementQueries(host);
        var leftHit = queries.SweepPlayer(new(-2, 2, 0), new(-2, -2, 0), false);
        var rightHit = queries.SweepPlayer(new(2, 2, 0), new(2, -2, 0), false);

        Assert.Equal(17, leftHit.SurfaceId);
        Assert.Equal(23, rightHit.SurfaceId);
        Assert.Equal(0.15f, leftHit.Friction);
        Assert.Equal(0.85f, rightHit.Friction);
    }

    [Fact]
    public void TriggerStaticMeshIsReportedAsSensorAndDoesNotBlockPlayerSweep()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        var vertices = new[]
        {
            new Vector3(-2, 0, -2), new Vector3(2, 0, -2),
            new Vector3(2, 0, 2), new Vector3(-2, 0, 2)
        };
        var trigger = host.CreateStaticMeshBody(vertices, new[] { Triangle(0, 1, 2), Triangle(0, 2, 3) },
            Vector3.Zero, SourceObjectLayer.Trigger, new SourceStaticMeshProfile { SurfaceId = 9 });
        using var queries = new JoltMovementQueries(host);

        Assert.True(host.IsSensor(trigger));
        var hit = queries.SweepPlayer(new(0, 2, 0), new(0, -2, 0), false);
        Assert.Equal(-1, hit.BodyId);
        Assert.Equal(1f, hit.Fraction);
    }

    [Fact]
    public void HitscanHonorsSourceContentsMaskBeforeReturningAHit()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(0.5f, 0.5f, 0.5f), new(0, 0, 2), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, new SourceRigidBodyProfile { ContentsMask = SourceContents.Water });
        var defaultShot = new JoltHitscanQueries(host);
        var waterTrace = new JoltHitscanQueries(host, contentsMask: SourceContents.Water);
        using var movementQueries = new JoltMovementQueries(host);

        Assert.False(defaultShot.Cast(Vector3.Zero, Vector3.UnitZ, 10f, out _));
        Assert.True(waterTrace.Cast(Vector3.Zero, Vector3.UnitZ, 10f, out var hit));
        Assert.True(hit.Fraction > 0f && hit.Fraction < 1f);
        var movementHit = movementQueries.SweepPlayer(new(0f, 2f, 2f), new(0f, -2f, 2f), false);
        Assert.Equal(-1, movementHit.BodyId);
    }

    [Fact]
    public void FireBulletsPreservesSourceShotOrderingAndPlayerDamageSelection()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 3f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);
        var weapon = new JoltHitscanWeapon();
        weapon.Initialize(host);
        var impacts = new List<SourceFireBulletsImpact>();
        var recording = new WeaponRecording();
        var damageTarget = new DamageTargetProbe();
        weapon.Recording = recording;
        weapon.IsPlayerTarget = _ => true;
        weapon.BulletForceResolver = _ => 100f;
        weapon.DamageTargetResolver = _ => damageTarget;
        weapon.DamageTargetIdResolver = _ => 9001;
        weapon.Impact += impacts.Add;

        var info = new SourceFireBulletsInfo(2, Vector3.Zero, Vector3.UnitZ, Vector3.Zero, 10f, 3,
            Damage: 12f, PlayerDamage: 25, Flags: SourceFireBulletsFlags.FirstShotAccurate,
            DamageForceScale: 0.5f, DamageType: 0x1234);
        var results = weapon.FireBullets(in info, 47);

        Assert.Equal(2, results.Count);
        Assert.Equal(2, impacts.Count);
        Assert.All(impacts, impact => Assert.Equal(25f, impact.Damage));
        Assert.All(impacts, impact => Assert.Equal(3, impact.AmmoType));
        Assert.All(impacts, impact => Assert.Equal(0x3234, impact.DamageType));
        Assert.All(impacts, impact => Assert.Equal(0.5f, impact.DamageForceScale));
        Assert.All(impacts, impact => Assert.True(impact.PrimaryAttack));
        Assert.All(impacts, impact => Assert.Equal(new Vector3(0f, 0f, 50f), impact.DamageForce));
        Assert.Equal(SourceHitGroup.Generic, impacts[0].Metadata.HitGroup);
        Assert.Equal(0, impacts[0].ShotIndex);
        Assert.Equal(1, impacts[1].ShotIndex);
        Assert.Equal(2, recording.Frames.Count);
        Assert.Equal(3, recording.Frames[0].AmmoType);
        Assert.Equal(25f, recording.Frames[0].AppliedDamage);
        Assert.Equal(0x3234, recording.Frames[0].DamageType);
        Assert.Equal(SourceFireBulletsFlags.FirstShotAccurate, recording.Frames[0].Flags);
        Assert.Equal(new Vector3(0f, 0f, 50f), recording.Frames[0].DamageForce);
        Assert.Equal(2, damageTarget.TraceCount);
        Assert.Equal(1, damageTarget.TakeDamageCount);
        Assert.Equal(50f, damageTarget.LastDamage.Damage);
        Assert.Equal(new Vector3(0f, 0f, 100f), damageTarget.LastDamage.DamageForce);
        Assert.Equal(0x3234, damageTarget.LastDamage.DamageType);
    }

    [Fact]
    public void FireBulletsReseedsSpreadForEachSourceShot()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        var weapon = new JoltHitscanWeapon();
        weapon.Initialize(host);
        var recording = new WeaponRecording();
        weapon.Recording = recording;
        var info = new SourceFireBulletsInfo(3, Vector3.Zero, Vector3.UnitZ,
            new Vector3(0.1f, 0.1f, 0f), 10f, 1, Damage: 1f);

        var results = weapon.FireBullets(in info, 47);
        var manipulator = new SourceShotManipulator(info.Direction);
        for (var shot = 0; shot < results.Count; shot++)
        {
            var random = new SourceUniformRandomStream((47 + shot) & 255);
            var expected = manipulator.ApplySpread(info.Spread, 0f, 0f, 0f, random.RandomFloat);
            Assert.Equal(expected.X, results[shot].Direction.X, 6);
            Assert.Equal(expected.Y, results[shot].Direction.Y, 6);
            Assert.Equal(expected.Z, results[shot].Direction.Z, 6);
        }

        Assert.Equal(new[] { 47, 48, 49 }, recording.Frames.Select(frame => frame.RandomSeed));

        var repeatedSeed = new SourceUniformRandomStream(47);
        var repeated = manipulator.ApplySpread(info.Spread, 0f, 0f, 0f, repeatedSeed.RandomFloat);
        Assert.NotEqual(repeated, results[1].Direction);
    }

    [Fact]
    public void FireBulletsUsesTitleAmmoDefinitionForPlayerDamageAndDamageType()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 3f),
            JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        var target = new DamageTargetProbe();
        var weapon = new JoltHitscanWeapon();
        weapon.Initialize(host);
        weapon.IsPlayerTarget = _ => true;
        weapon.DamageTargetResolver = _ => target;
        weapon.AmmoDefinitionResolver = _ => new SourceAmmoDefinition(0x10,
            SourceAmmoFlags.InterpretPlayerDamageAsDamageToPlayer | SourceAmmoFlags.ForceDropIfCarried, 33);
        HitscanHit? forceDropHit = null;
        weapon.ForceDropIfCarried = hit => forceDropHit = hit;
        var impacts = new List<SourceFireBulletsImpact>();
        weapon.Impact += impacts.Add;

        var info = new SourceFireBulletsInfo(1, Vector3.Zero, Vector3.UnitZ,
            Vector3.Zero, 10f, 4, Damage: 0f, PlayerDamage: 0, DamageType: 0x1);
        weapon.FireBullets(in info, 3);

        var impact = Assert.Single(impacts);
        Assert.Equal(33f, impact.Damage);
        Assert.Equal(0x2010, impact.DamageType);
        Assert.Equal(33f, target.LastDamage.Damage);
        Assert.Equal(0x2010, target.LastDamage.DamageType);
        Assert.NotNull(forceDropHit);
    }

    [Fact]
    public void FireBulletsReportsShotResponsiveTriggersBeforeTheFirstSolid()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        var trigger = host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 1.5f),
            JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.Trigger,
            new SourceRigidBodyProfile { ContentsMask = SourceContents.MaskShot });
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 5.5f),
            JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.Trigger,
            new SourceRigidBodyProfile { ContentsMask = SourceContents.MaskShot });
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 4f),
            JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        var weapon = new JoltHitscanWeapon();
        weapon.Initialize(host);
        var triggerHits = new List<HitscanHit>();
        var eventOrder = new List<string>();
        weapon.TriggerHit += hit => { triggerHits.Add(hit); eventOrder.Add("trigger"); };
        weapon.Impact += _ => eventOrder.Add("impact");

        var info = new SourceFireBulletsInfo(1, Vector3.Zero, Vector3.UnitZ, Vector3.Zero, 10f, 1, Damage: 5f);
        weapon.FireBullets(in info, 11);

        Assert.Equal(trigger.ID, unchecked((uint)Assert.Single(triggerHits).BodyId));
        Assert.Equal(new[] { "trigger", "impact" }, eventOrder);
    }

    [Fact]
    public void SourceHitboxCatalogRefinesPhysicsHitIntoAuthoredStudioMetadata()
    {
        var catalog = new SourceHitboxCatalog();
        catalog.Register(7, Matrix4x4.Identity, new[]
        {
            SourceHitboxDefinition.FromSourceUnits(3, SourceHitGroup.Head, 2,
                new(-8, -8, 10), new(8, 8, 26), Matrix4x4.Identity)
        });
        var physicsHit = new HitscanHit(new(0, 0, 0.4f), -Vector3.UnitZ, 7, 9, 0.5f);

        Assert.True(catalog.TryResolve(physicsHit, new(0, 0, 0), Vector3.UnitZ, 1f, out var hit));
        Assert.Equal(SourceHitGroup.Head, hit.HitGroup);
        Assert.Equal(3, hit.Hitbox);
        Assert.Equal(2, hit.PhysicsBone);
        Assert.InRange(hit.Position.Z, SourceUnits.ToMeters(9.99f), SourceUnits.ToMeters(10.01f));
        Assert.Equal(9, hit.SurfaceId);
    }

    [Fact]
    public void SourceHitboxCatalogManifestRoundTripsTransformsAndMetadata()
    {
        var catalog = new SourceHitboxCatalog();
        catalog.Register(4, Matrix4x4.CreateTranslation(1f, 2f, 3f), new[]
        {
            new SourceHitboxDefinition(2, SourceHitGroup.Stomach, 6,
                new(-0.2f, -0.3f, -0.4f), new(0.2f, 0.3f, 0.4f), Matrix4x4.Identity)
        });

        var restored = SourceHitboxCatalog.FromJson(catalog.ToJson());
        var body = Assert.Single(restored.CaptureManifest());
        Assert.Equal(4, body.BodyId);
        Assert.Equal(new Vector3(1f, 2f, 3f), body.BodyToWorld.Translation);
        Assert.Equal(SourceHitGroup.Stomach, Assert.Single(body.Definitions).HitGroup);
        Assert.Equal(6, body.Definitions[0].PhysicsBone);
    }

    [Fact]
    public void SourceHitboxCatalogUpdatesAnimatedBoneTransformWithoutDroppingMetadata()
    {
        var catalog = new SourceHitboxCatalog();
        catalog.Register(8, Matrix4x4.Identity, new[]
        {
            new SourceHitboxDefinition(5, SourceHitGroup.Chest, 3,
                new(-0.2f, -0.2f, -0.2f), new(0.2f, 0.2f, 0.2f), Matrix4x4.Identity)
        });
        Assert.True(catalog.UpdateHitboxTransform(8, 5, Matrix4x4.CreateTranslation(0f, 0f, 2f)));
        var physicsHit = new HitscanHit(Vector3.Zero, -Vector3.UnitZ, 8, 0, 0f);

        Assert.True(catalog.TryResolve(physicsHit, new(0f, 0f, 0f), Vector3.UnitZ, 5f, out var hit));
        Assert.Equal(SourceHitGroup.Chest, hit.HitGroup);
        Assert.Equal(5, hit.Hitbox);
        Assert.Equal(3, hit.PhysicsBone);
        Assert.InRange(hit.Position.Z, 1.79f, 1.81f);
    }

    [Fact]
    public void SourceHitboxCatalogPublishesAnAtomicCompletePose()
    {
        var catalog = new SourceHitboxCatalog();
        catalog.Register(12, Matrix4x4.Identity, new[]
        {
            new SourceHitboxDefinition(1, SourceHitGroup.Chest, 2, new(-1f), new(1f), Matrix4x4.Identity),
            new SourceHitboxDefinition(2, SourceHitGroup.Head, 3, new(-1f), new(1f), Matrix4x4.Identity)
        });

        Assert.False(catalog.UpdatePose(12, Matrix4x4.Identity,
            new Dictionary<int, Matrix4x4> { [1] = Matrix4x4.Identity }));
        Assert.True(catalog.UpdatePose(12, Matrix4x4.CreateTranslation(2f, 0f, 0f),
            new Dictionary<int, Matrix4x4>
            {
                [1] = Matrix4x4.CreateTranslation(1f, 0f, 0f),
                [2] = Matrix4x4.CreateTranslation(0f, 1f, 0f)
            }));

        var manifest = catalog.CaptureManifest();
        Assert.Equal(new Vector3(2f, 0f, 0f), manifest[0].BodyToWorld.Translation);
        Assert.Equal(new Vector3(1f, 0f, 0f), manifest[0].Definitions[0].BoneToBodyMeters.Translation);
        Assert.Equal(new Vector3(0f, 1f, 0f), manifest[0].Definitions[1].BoneToBodyMeters.Translation);
    }

    [Fact]
    public void HitscanWeaponRefinesJoltBodyHitBeforeFireBulletsMetadataDispatch()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        var body = host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 3f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);
        var catalog = new SourceHitboxCatalog();
        catalog.Register(unchecked((int)body.ID), Matrix4x4.CreateTranslation(0f, 0f, 3f), new[]
        {
            new SourceHitboxDefinition(4, SourceHitGroup.Chest, 1,
                new(-0.5f, -0.5f, -0.1f), new(0.5f, 0.5f, 0.1f), Matrix4x4.Identity)
        });
        var weapon = new JoltHitscanWeapon { HitboxCatalog = catalog };
        weapon.Initialize(host);
        var impacts = new List<SourceFireBulletsImpact>();
        weapon.Impact += impacts.Add;
        var info = new SourceFireBulletsInfo(1, Vector3.Zero, Vector3.UnitZ, Vector3.Zero, 10f, 1, Damage: 10f);

        weapon.FireBullets(in info, 1);

        Assert.Equal(SourceHitGroup.Chest, Assert.Single(impacts).Metadata.HitGroup);
        Assert.Equal(4, impacts[0].Metadata.Hitbox);
        Assert.Equal(1, impacts[0].Metadata.PhysicsBone);
    }

    [Fact]
    public void FireBulletsHonorsUnderwaterDamageAndSurfaceImpactFlags()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.05f), new(0f, 0f, 2f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, new SourceRigidBodyProfile { ContentsMask = SourceContents.Water });
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 5f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);
        var weapon = new JoltHitscanWeapon();
        weapon.Initialize(host);
        var impacts = new List<SourceFireBulletsImpact>();
        weapon.Impact += impacts.Add;

        var suppressed = new SourceFireBulletsInfo(1, Vector3.Zero, Vector3.UnitZ, Vector3.Zero, 10f, 1,
            Damage: 10f, Flags: SourceFireBulletsFlags.DontHitUnderwater);
        weapon.FireBullets(in suppressed, 1);
        Assert.Equal(0f, Assert.Single(impacts).Damage);
        Assert.True(impacts[0].HitWater);
        Assert.True(impacts[0].DamageSuppressed);
        Assert.True(impacts[0].SuppressSurfaceImpact);
        Assert.InRange(impacts[0].TracerDestination.Z, 1.9f, 2.1f);

        impacts.Clear();
        var allowed = suppressed with { Flags = SourceFireBulletsFlags.AllowWaterSurfaceImpacts };
        weapon.FireBullets(in allowed, 2);
        Assert.Equal(10f, Assert.Single(impacts).Damage);
        Assert.True(impacts[0].HitWater);
        Assert.False(impacts[0].SuppressSurfaceImpact);
        Assert.InRange(impacts[0].TracerDestination.Z, 1.9f, 2.1f);
    }

    [Fact]
    public void FireBulletsTracerCadenceAdvancesAcrossMissedShots()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 3f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);
        var weapon = new JoltHitscanWeapon();
        weapon.Initialize(host);
        var impacts = new List<SourceFireBulletsImpact>();
        weapon.Impact += impacts.Add;
        var info = new SourceFireBulletsInfo(1, Vector3.Zero, Vector3.UnitZ, Vector3.Zero, 10f, 1,
            TracerFrequency: 2, Damage: 1f);

        weapon.FireBullets(in info, 1);
        var firstTracer = Assert.Single(impacts).IsTracer;
        impacts.Clear();
        var miss = info with { Direction = Vector3.UnitX };
        Assert.False(Assert.Single(weapon.FireBullets(in miss, 2)).Hit);
        Assert.Empty(impacts);
        weapon.FireBullets(in info, 3);
        Assert.Equal(firstTracer, Assert.Single(impacts).IsTracer);
    }

    [Fact]
    public void HitscanCastAllReturnsOrderedJoltRayIntersections()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 2f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, surfaceId: 11);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 5f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World, surfaceId: 12);
        using var queries = new JoltHitscanQueries(host);

        var hits = queries.CastAll(Vector3.Zero, Vector3.UnitZ, 10f);

        Assert.Equal(2, hits.Count);
        Assert.Equal(11, hits[0].SurfaceId);
        Assert.Equal(12, hits[1].SurfaceId);
        Assert.True(hits[0].Fraction < hits[1].Fraction);
    }

    [Fact]
    public void GenericSourceGlassTraceUsesSixteenUnitProbeAndRefiresBehindTheExit()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.05f), new(0f, 0f, 2f),
            JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World,
            new SourceRigidBodyProfile { ContentsMask = SourceContents.Window });
        var behind = host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 4f),
            JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        using var queries = new JoltHitscanQueries(host);

        var result = queries.CastSourceGlass(Vector3.Zero, Vector3.UnitZ, 10f,
            hit => (hit.Contents & SourceContents.Window) != 0);

        Assert.True(result.PassedThrough);
        Assert.True(result.Exit.Position.Z > result.Entry.Position.Z);
        Assert.Equal(behind.ID, unchecked((uint)result.ContinuationHit!.Value.BodyId));
    }

    [Fact]
    public void CounterStrikeHitscanPairsJoltEntryAndExitAndContinuesUntilPowerEnds()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 2f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);
        host.CreateBoxBody(new(1f, 1f, 0.1f), new(0f, 0f, 5f), JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.World);
        using var queries = new JoltHitscanQueries(host);

        var trace = queries.CastCounterStrikePenetrating(Vector3.Zero, Vector3.UnitZ, 10f,
            "BULLET_PLAYER_50AE", 100f, 1f, 1,
            _ => SourceBulletMaterial.Wood);

        Assert.Equal(3, trace.Impacts.Count);
        Assert.False(trace.Impacts[0].Exit);
        Assert.True(trace.Impacts[1].Exit);
        Assert.False(trace.Impacts[2].Exit);
        Assert.Equal(60f, trace.Impacts[1].Damage, 4);
        Assert.Equal(0, trace.State.PenetrationsRemaining);
        Assert.True(trace.StoppedByPenetration);
    }

    [Fact]
    public void LiveJoltCeilingBlocksStandingHullButAllowsCrouchedHull()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(2048, 0, 2048, 256);
        host.CreateBoxBody(new(2, 0.05f, 2), new(0, -0.05f, 0), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        host.CreateBoxBody(new(2, 0.1f, 2), new(0, 1.1f, 0), JoltPhysicsSharp.MotionType.Static, SourceObjectLayer.World);
        using var queries = new JoltMovementQueries(host);
        Assert.True(queries.IsEmpty(Vector3.Zero, true));
        Assert.False(queries.IsEmpty(Vector3.Zero, false));
        var motor = new SourceMovementMotor(new SourceMovementProfile { DuckTransitionSeconds = 0.1f }, queries, Vector3.Zero);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Duck), 1f / 66f);
        Assert.True(motor.State.Ducking);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        Assert.True(motor.State.Ducking);
    }

    [Fact]
    public void OptInRigidBodyDragUsesExplicitExponentialVelocityLaw()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile
            {
                GravityFactor = 0f, LinearDampingPerSecond = 0f, AngularDampingPerSecond = 0f,
                EnableDrag = true, DragCoefficientPerSecond = 2f, RollingDragCoefficientPerSecond = 3f
            });
        host.Bodies.SetLinearVelocity(body, Vector3.UnitX);
        host.Bodies.SetAngularVelocity(body, Vector3.UnitY);
        host.Step();
        var expectedLinear = MathF.Exp(-2f * host.FixedStepSeconds);
        var expectedAngular = MathF.Exp(-3f * host.FixedStepSeconds);
        Assert.InRange(host.Bodies.GetLinearVelocity(body).X, expectedLinear - 0.002f, expectedLinear + 0.002f);
        Assert.InRange(host.Bodies.GetAngularVelocity(body).Y, expectedAngular - 0.002f, expectedAngular + 0.002f);
    }

    [Fact]
    public void GroundFrictionUsesStopSpeedControlTerm()
    {
        var queries = new FlatGroundQueries();
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        var before = motor.State with { Velocity = new Vector3(SourceUnits.ToMeters(50f), 0, 0), Ground = GroundState.Grounded };
        motor.LoadState(before);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        Assert.InRange(SourceUnits.ToSource(motor.State.Velocity.X), 43f, 45f);
    }

    [Fact]
    public void AirAccelerationCapsWishSpeedButUsesOriginalAccelerationSpeed()
    {
        var queries = new FlatGroundQueries { State = new(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1, false, false) };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);
        motor.Tick(new SourceInput(new Vector2(0, 1), Buttons.None), 1f / 66f);
        Assert.True(motor.State.Velocity.X > 0f);
        Assert.True(SourceUnits.ToSource(motor.State.Velocity.X) < 320f);
    }

    [Fact]
    public void FlyMoveTypeDoesNotFallThroughToWalkingGravity()
    {
        var queries = new FlatGroundQueries();
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new Vector3(0f, 1f, 0f));
        motor.Tick(new SourceInput(new(0, 1), Buttons.None, MoveType: SourceMoveType.Fly), 1f / 66f);
        Assert.Equal(SourceMoveType.Fly, motor.State.MoveType);
        Assert.Equal(GroundState.Airborne, motor.State.Ground);
        var expected = 10f * (1f / 66f) * 320f;
        Assert.Equal(expected, SourceUnits.ToSource(motor.State.Velocity.X), 3);
        Assert.Equal(0f, motor.State.Velocity.Y);
    }

    [Fact]
    public void FlyGravityMoveTypeAppliesGravityAndUsesFlyDispatch()
    {
        var queries = new FlatGroundQueries();
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new(0, 10, 0));
        motor.Tick(new SourceInput(new(0, 1), Buttons.None, MoveType: SourceMoveType.FlyGravity), 1f / 66f);
        Assert.Equal(SourceMoveType.FlyGravity, motor.State.MoveType);
        Assert.True(motor.State.Velocity.Y < 0f);
        Assert.True(motor.State.Position.X > 0f);
    }

    [Fact]
    public void FlyUsesSingleTossCollisionAndStopsOnDefaultGroundContact()
    {
        var motor = new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero);

        motor.Tick(new SourceInput(new(0f, 1f), Buttons.None, MoveType: SourceMoveType.Fly), 1f / 66f);

        Assert.Equal(GroundState.Grounded, motor.State.Ground);
        Assert.Equal(Vector3.Zero, motor.State.Velocity);
    }

    [Fact]
    public void FlyClampsVelocityBeforeTheSingleTossSweep()
    {
        var profile = new SourceMovementProfile { MaxVelocitySourceUnitsPerSecond = 100f };
        var motor = new SourceMovementMotor(profile, new FlatGroundQueries(), new Vector3(0f, 1f, 0f));
        motor.LoadState(new(new Vector3(0f, 1f, 0f), new Vector3(10f, 0f, 0f), GroundState.Airborne,
            Vector3.UnitY, -1, 1f, false, false, MoveType: SourceMoveType.Fly));
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None, MoveType: SourceMoveType.Fly), 1f);
        Assert.Equal(profile.MaxVelocity, motor.State.Velocity.X, 5);
        Assert.Equal(profile.MaxVelocity, motor.State.Position.X, 5);
    }

    [Theory]
    [InlineData(SourceMoveType.Fly)]
    [InlineData(SourceMoveType.FlyGravity)]
    public void StationaryGroundedTossMoveReturnsBeforeGravity(SourceMoveType moveType)
    {
        var motor = new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(),
            new Vector3(0f, 0.02f, 0f));
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None, MoveType: moveType), 1f / 66f);
        Assert.Equal(new Vector3(0f, 0.02f, 0f), motor.State.Position);
        Assert.Equal(Vector3.Zero, motor.State.Velocity);
        Assert.Equal(GroundState.Grounded, motor.State.Ground);
    }

    [Fact]
    public void BackwardWishSpeedUsesProfileScale()
    {
        var profile = new SourceMovementProfile { BackwardSpeedScale = 0.5f };
        var queries = new FlatGroundQueries();
        var motor = new SourceMovementMotor(profile, queries, Vector3.Zero);
        motor.Tick(new SourceInput(new(0, -1), Buttons.None), 1f / 66f);
        Assert.InRange(SourceUnits.ToSource(MathF.Abs(motor.State.Velocity.X)), 23.5f, 25.5f);
    }

    [Fact]
    public void ContactImpulsePolicyIsAppliedAtSourceMovementContact()
    {
        var queries = new FlatGroundQueries { ContactBodyId = 42 };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new(0f, 0.1f, 0f))
        {
            ContactImpulsePolicy = (_, _) => new Vector3(2f, 0f, 0f)
        };
        motor.LoadState(motor.State with { Position = Vector3.Zero, Ground = GroundState.Grounded });

        motor.Tick(new SourceInput(new(0f, 1f), Buttons.None), 1f / 66f);

        Assert.True(queries.ImpulseCount > 0);
        Assert.Equal(42, queries.LastImpulseBodyId);
        Assert.Equal(new Vector3(2f, 0f, 0f), queries.LastImpulse);
    }

    [Fact]
    public void DuckTransitionUsesSourceDuckAndUnduckTimings()
    {
        var profile = new SourceMovementProfile { DuckTransitionSeconds = 1f };
        var motor = new SourceMovementMotor(profile, new FlatGroundQueries(), Vector3.Zero);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Duck), 0.5f);
        Assert.True(motor.State.Ducking);
        Assert.Equal(28f, motor.State.ViewHeightSourceUnits);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 0.5f);
        Assert.False(motor.State.Ducking);
        Assert.InRange(motor.State.ViewHeightSourceUnits, 63.9f, 64.1f);
    }

    [Fact]
    public void GroundDuckAndUnduckKeepTheSourceHullUntilEachTransitionFinishes()
    {
        var motor = new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero);
        motor.LoadState(motor.State with { Ground = GroundState.Grounded });

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Duck), 1f / 66f);
        Assert.False(motor.State.Ducking);

        for (var tick = 1; tick < 27; tick++)
            motor.Tick(new SourceInput(Vector2.Zero, Buttons.Duck), 1f / 66f);
        Assert.True(motor.State.Ducking);

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        Assert.True(motor.State.Ducking);
        for (var tick = 1; tick < 14; tick++)
            motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        Assert.False(motor.State.Ducking);
    }

    [Fact]
    public void AirborneCrouchAppliesSourceHullOriginOffsetAndRestoresIt()
    {
        var profile = new SourceMovementProfile();
        var motor = new SourceMovementMotor(profile, new FlatGroundQueries(), new Vector3(0f, 1f, 0f));
        motor.LoadState(motor.State with { Ground = GroundState.Airborne });

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Duck, MoveType: SourceMoveType.Noclip), 1f / 66f);
        Assert.Equal(1f + SourceUnits.ToMeters(18f), motor.State.Position.Y, 5);

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None, MoveType: SourceMoveType.Noclip), 1f / 66f);
        Assert.Equal(1f, motor.State.Position.Y, 5);
    }

    [Fact]
    public void GroundSurfaceFactorsAffectSpeedAndJumpSeparatelyFromFriction()
    {
        var profile = new SourceMovementProfile();
        var queries = new FlatGroundQueries { GroundSurface = new SourceSurface("mud", 0.8f, 0.001f, MaxSpeedFactor: 0.5f, JumpFactor: 0.5f) };
        var motor = new SourceMovementMotor(profile, queries, Vector3.Zero);
        motor.LoadState(new(Vector3.Zero, Vector3.Zero, GroundState.Grounded, Vector3.UnitY, -1, 1f, false, false));
        motor.Tick(new SourceInput(new(0, 1), Buttons.None), 1f / 66f);
        Assert.Equal(0.5f, motor.State.SurfaceMaxSpeedFactor);
        Assert.InRange(SourceUnits.ToSource(motor.State.Velocity.X), 23.5f, 25f);
        motor.LoadState(motor.State with { Ground = GroundState.Grounded, Velocity = Vector3.Zero });
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Jump), 1f / 66f);
        Assert.InRange(SourceUnits.ToSource(motor.State.Velocity.Y), 127f, 129f);
    }

    [Fact]
    public void GroundSurfaceFrictionUsesSourcePhysicsScaleAndCap()
    {
        var queries = new FlatGroundQueries
        {
            GroundSurface = new SourceSurface("low-friction", Friction: 0.4f)
        };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);

        // CategorizeGroundSurface multiplies the VPhysics friction by 1.25
        // and clamps the player value to [0, 1]. The contact's normalized
        // friction field must not replace that Source surface-data rule.
        Assert.Equal(0.5f, motor.State.SurfaceFriction, 5);
    }

    [Fact]
    public void AirborneUpwardMotionUsesSourceQuarterFrictionFallback()
    {
        var queries = new FlatGroundQueries();
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);
        motor.LoadState(motor.State with
        {
            Position = new Vector3(0f, 1f, 0f),
            Ground = GroundState.Airborne,
            Velocity = new Vector3(0f, SourceUnits.ToMeters(100f), 0f)
        });

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);

        Assert.Equal(GroundState.Airborne, motor.State.Ground);
        Assert.Equal(0.25f, motor.State.SurfaceFriction, 5);
    }

    [Fact]
    public void ClearSweepStillRejectsAStationaryHullThatEndsEmbedded()
    {
        var queries = new FlatGroundQueries { Empty = false };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new Vector3(0f, 1f, 0f));
        motor.Tick(new SourceInput(new Vector2(0f, 1f), Buttons.None,
            MoveType: SourceMoveType.Walk), 1f / 66f);

        Assert.Equal(0f, motor.State.Velocity.X);
    }

    [Fact]
    public void AllSolidMovementStopsWithoutApplyingARecoveryLift()
    {
        var queries = new FlatGroundQueries { AllSolid = true };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new Vector3(0f, 2f, 0f));
        motor.LoadState(new(new Vector3(0f, 2f, 0f), Vector3.UnitX,
            GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false));
        motor.Tick(new SourceInput(new(0f, 1f), Buttons.None), 1f / 66f);
        Assert.Equal(new Vector3(0f, 2f, 0f), motor.State.Position);
        Assert.Equal(0f, motor.State.Velocity.X);
        Assert.Equal(0f, motor.State.Velocity.Z);
        Assert.Equal(GroundState.Stuck, motor.State.Ground);
    }

    [Fact]
    public void MovementSnapshotRestoresJumpEdgeHistoryForPrediction()
    {
        var motor = new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero);
        motor.LoadState(new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Grounded, Vector3.UnitY, -1,
            1f, false, false, PreviousJumpDown: true));

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Jump), 1f / 66f);
        Assert.False(motor.State.Jumped);
        Assert.True(motor.State.PreviousJumpDown);

        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        motor.LoadState(motor.State with { Ground = GroundState.Grounded, PreviousJumpDown = false });
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Jump), 1f / 66f);
        Assert.True(motor.State.Jumped);
    }

    [Fact]
    public void SurfaceRegistryPreservesPhysicsGameplayAndAudioMetadata()
    {
        var surface = new SourceSurface("metal", 0.4f, 0.2f, 7800f, 0.03f, 0.8f, 1.1f, false,
            0.125f, 0.7f, 0.9f, 0.2f, "metal_hard", "metal_soft", "metal_scrape", "metal_smooth", "metal_bullet", "metal_step");
        var registry = new SourceSurfaceRegistry();
        registry.Register(44, surface);
        var restored = registry.Get(44);
        Assert.Equal(7800f, restored.DensityKgPerM3);
        Assert.Equal(0.125f, restored.ThicknessInches);
        Assert.Equal("metal_bullet", restored.BulletImpactSound);
        Assert.Equal("metal_step", restored.FootstepSound);
    }

    [Fact]
    public void SurfaceRegistryManifestRoundTripsAuthoredSurfaceData()
    {
        var registry = new SourceSurfaceRegistry();
        registry.Register(44, new SourceSurface("metal", 0.4f, 0.2f, 7800f, 0.03f, 0.8f, 1.1f,
            false, 0.125f, 0.7f, 0.9f, 0.2f, "hard", "soft", "rough", "smooth", "bullet", "step"));

        var restored = SourceSurfaceRegistry.FromJson(registry.ToJson()).Get(44);

        Assert.Equal("metal", restored.Name);
        Assert.Equal(7800f, restored.DensityKgPerM3);
        Assert.Equal(0.125f, restored.ThicknessInches);
        Assert.Equal("bullet", restored.BulletImpactSound);
    }

    [Fact]
    public void ContactMaterialPolicyCanInstallTitleSpecificCombineRules()
    {
        var policy = new SourceContactMaterialPolicy
        {
            CombineFriction = static (first, second) => first.Friction + second.Friction,
            CombineRestitution = static (first, second) => first.Elasticity * second.Elasticity
        };
        var first = new SourceSurface("first", Friction: 0.25f, Elasticity: 0.4f);
        var second = new SourceSurface("second", Friction: 0.5f, Elasticity: 0.5f);

        Assert.Equal(0.75f, policy.GetFriction(first, second));
        Assert.Equal(0.2f, policy.GetRestitution(first, second));
    }

    [Fact]
    public void ContactMaterialPolicyPreservesSourceMaterialManagerClamps()
    {
        var policy = new SourceContactMaterialPolicy
        {
            CombineFriction = static (_, _) => 4f,
            CombineRestitution = static (_, _) => 3f
        };
        var surface = new SourceSurface("surface");

        Assert.Equal(1f, policy.GetFriction(surface, surface));
        Assert.Equal(1f, policy.GetRestitution(surface, surface));
    }

    [Fact]
    public void SourceShotManipulatorPreservesAccurateFirstShotAndInjectedSpread()
    {
        var manipulator = new SourceShotManipulator(Vector3.UnitZ);
        var random = new Queue<float>(new[] { 0f, 0f, 0f, 0f });
        var spread = manipulator.ApplySpread(new(0.1f, 0.2f, 0f), 1f, 0f, 1f,
            (_, _) => random.Dequeue());

        Assert.Equal(Vector3.UnitZ, manipulator.ShotDirection);
        Assert.Equal(Vector3.UnitX, manipulator.Right);
        Assert.Equal(Vector3.UnitY, manipulator.Up);
        Assert.Equal(Vector3.UnitZ, spread);
    }

    [Fact]
    public void SourceUniformRandomStreamIsSeedDeterministicAndSupportsSourceRanges()
    {
        var first = new SourceUniformRandomStream(12345);
        var second = new SourceUniformRandomStream(12345);
        var different = new SourceUniformRandomStream(12346);

        var firstValues = Enumerable.Range(0, 8).Select(_ => first.RandomFloat(-1f, 1f)).ToArray();
        var secondValues = Enumerable.Range(0, 8).Select(_ => second.RandomFloat(-1f, 1f)).ToArray();

        Assert.Equal(firstValues, secondValues);
        Assert.All(firstValues, value => Assert.InRange(value, -1f, 1f));
        Assert.NotEqual(firstValues[0], different.RandomFloat(-1f, 1f));
        Assert.InRange(new SourceUniformRandomStream(7).RandomInt(3, 3), 3, 3);
    }

    [Fact]
    public void CounterStrikePenetrationPolicyMatchesReferencedTablesAndStateLaw()
    {
        Assert.Equal(new SourceBulletTypeParameters(39f, 5000f, 2400f),
            SourceCounterStrikePenetration.GetBulletTypeParameters("BULLET_PLAYER_762MM"));
        Assert.Equal((0.5f, 0.3f),
            SourceCounterStrikePenetration.GetMaterialParameters(SourceBulletMaterial.Metal));

        var state = new SourcePenetrationState(100f, 30f, 0f, 1);
        var result = SourceCounterStrikePenetration.TryPenetrate(in state,
            SourceBulletMaterial.Wood, SourceBulletMaterial.Wood, 10f, false, 1f);

        Assert.True(result.Success);
        Assert.Equal(60f, result.State.Damage, 4);
        Assert.Equal(25f, result.State.PenetrationPower, 4);
        Assert.Equal(10f, result.State.CurrentDistance, 4);
        Assert.Equal(0, result.State.PenetrationsRemaining);

        var tooThick = SourceCounterStrikePenetration.TryPenetrate(in state,
            SourceBulletMaterial.Concrete, SourceBulletMaterial.Concrete, 100f, false, 1f);
        Assert.False(tooThick.Success);
    }

    [Fact]
    public void WeaponRecordingRoundTripsAndComparesSeededShotData()
    {
        var recording = new WeaponRecording();
        recording.Capture(4, 0, 19, Vector3.Zero, Vector3.UnitZ, true,
            new(Vector3.UnitZ, Vector3.UnitY, 7, 3, 0.25f, SourceHitGroup.Head, 4, 2,
                SourceContents.Hitbox));
        var restored = WeaponRecording.FromJson(recording.ToJson());
        var comparison = WeaponParityComparator.Compare(recording.Frames, restored.Frames);

        Assert.True(comparison.Passes(0.000001f, 0.000001f));
        Assert.Equal(19, restored.Frames[0].RandomSeed);
        Assert.Equal(3, restored.Frames[0].HitData.SurfaceId);
        Assert.Equal(SourceHitGroup.Head, restored.Frames[0].HitData.HitGroup);
        Assert.Equal(SourceContents.Hitbox, restored.Frames[0].HitData.Contents);
    }

    [Fact]
    public void ProjectileRecordingRoundTripsStateAndImpactMetadata()
    {
        var recording = new ProjectileRecording();
        var state = new ProjectileState(new(1, 2, 3), Vector3.UnitZ, true, 1, 2, 4.5f);
        recording.Capture(8, state, new(new(1, 2, 3), Vector3.UnitY, 9, 0.25f, 4, 2f));
        var restored = ProjectileRecording.FromJson(recording.ToJson());
        var comparison = ProjectileParityComparator.Compare(recording.Frames, restored.Frames);

        Assert.True(comparison.Passes(0.000001f, 0.000001f));
        Assert.Equal(4.5f, restored.Frames[0].State.PenetrationPowerRemaining);
        Assert.Equal(2f, restored.Frames[0].Hit!.Value.ThicknessInches);
    }

    [Fact]
    public void AuthoredVolumesClassifyWaterAndLadders()
    {
        var volumes = new SourceMovementVolumes();
        volumes.AddWater(new SourceWaterVolume(new SourceAabb(new(-2, -2, -2), new(2, 2, 2)), 2,
            Current: SourceWaterCurrent.Current0 | SourceWaterCurrent.CurrentUp));
        volumes.AddWaterJump(new SourceWaterJumpVolume(new SourceAabb(new(-1, -1, -1), new(1, 1, 1)), new(100, 200, 0), 0.51f));
        volumes.AddLadder(new SourceLadderVolume(new SourceAabb(new(3, -2, -2), new(4, 2, 2)), -Vector3.UnitX, 17));
        Assert.Equal(SourceWaterLevel.Eyes, volumes.GetWaterLevel(Vector3.Zero, false));
        Assert.Equal(SourceUnits.ToMeters(new Vector3(100f, 100f, 0f)),
            volumes.GetWaterBaseVelocity(Vector3.Zero, SourceWaterLevel.Waist));
        var lowCeilingWater = new SourceMovementVolumes();
        lowCeilingWater.AddWater(new SourceWaterVolume(
            new SourceAabb(new(-2f, -2f, -2f), new(2f, 1f, 2f)), 2f));
        Assert.Equal(SourceWaterLevel.Waist, lowCeilingWater.GetWaterLevel(Vector3.Zero, false));
        var customProfile = new SourceMovementProfile { StandingEyeSourceUnits = 40f };
        var customWater = new SourceMovementVolumes(movementProfile: customProfile);
        customWater.AddWater(new SourceWaterVolume(
            new SourceAabb(new(-2f, -2f, -2f), new(2f, 2f, 2f)), 1.1f));
        Assert.Equal(SourceWaterLevel.Eyes, customWater.GetWaterLevel(Vector3.Zero, false));
        var currentWater = new SourceMovementVolumes(movementProfile: new SourceMovementProfile());
        currentWater.AddWater(new SourceWaterVolume(new SourceAabb(new(-2f, -2f, -2f), new(2f, 2f, 2f)), 1.1f,
            Current: SourceWaterCurrent.Current0));
        Assert.Equal(SourceUnits.ToMeters(100f), currentWater.GetWaterBaseVelocity(Vector3.Zero,
            currentWater.GetWaterLevel(Vector3.Zero, false), false).X);
        Assert.Equal(SourceUnits.ToMeters(150f), currentWater.GetWaterBaseVelocity(Vector3.Zero,
            currentWater.GetWaterLevel(Vector3.Zero, true), true).X);
        Assert.True(volumes.TryWaterJump(Vector3.Zero, Vector3.UnitX, out var waterJumpVelocity, out var waterJumpDuration));
        Assert.Equal(SourceUnits.ToMeters(new Vector3(100, 200, 0)), waterJumpVelocity);
        Assert.Equal(0.51f, waterJumpDuration);
        Assert.True(volumes.TryLadder(new Vector3(3.5f, 0, 0), Vector3.UnitX, out var normal, out var bodyId));
        Assert.Equal(-Vector3.UnitX, normal);
        Assert.Equal(17, bodyId);
    }

    [Fact]
    public void LiveJoltMovementQueryPreservesAuthoredCrouchedWaterCurrent()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(256, 0, 256, 128);
        using var queries = new JoltMovementQueries(host);
        queries.Volumes.AddWater(new SourceWaterVolume(
            new SourceAabb(new(-2f, -2f, -2f), new(2f, 2f, 2f)), 1.1f,
            Current: SourceWaterCurrent.Current0));

        var standingLevel = queries.GetWaterLevel(Vector3.Zero, false);
        var crouchedLevel = queries.GetWaterLevel(Vector3.Zero, true);
        Assert.Equal(SourceWaterLevel.Waist, standingLevel);
        Assert.Equal(SourceWaterLevel.Eyes, crouchedLevel);
        Assert.Equal(SourceUnits.ToMeters(100f),
            queries.GetWaterBaseVelocity(Vector3.Zero, standingLevel, false).X);
        Assert.Equal(SourceUnits.ToMeters(150f),
            queries.GetWaterBaseVelocity(Vector3.Zero, crouchedLevel, true).X);
    }

    [Fact]
    public void LadderUsesSourceClimbSpeedAndPlaneVelocity()
    {
        var queries = new FlatGroundQueries { LadderActive = true, LadderNormal = -Vector3.UnitX, LadderBodyId = 17 };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);
        motor.LoadState(new(Vector3.Zero, Vector3.UnitX * 1f, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false,
            MoveType: SourceMoveType.Ladder));
        motor.Tick(new SourceInput(new(0, 1), Buttons.None, MoveType: SourceMoveType.Ladder), 1f / 66f);
        Assert.Equal(0f, motor.State.Velocity.X, 5);
        Assert.True(motor.State.Velocity.Y > 0f);
        Assert.Equal(200f, SourceUnits.ToSource(motor.State.Velocity.Y), 4);
        Assert.Equal(17, motor.State.GroundBodyId);
    }

    [Fact]
    public void LadderJumpUsesSourceTwoHundredSeventyUnitNormalImpulse()
    {
        var queries = new FlatGroundQueries { LadderActive = true, LadderNormal = -Vector3.UnitX, LadderBodyId = 17 };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, new Vector3(0f, 1f, 0f));
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Jump, MoveType: SourceMoveType.Ladder), 1f / 66f);

        Assert.Equal(SourceMoveType.Walk, motor.State.MoveType);
        Assert.Equal(-270f, SourceUnits.ToSource(motor.State.Velocity.X), 4);
        Assert.Equal(0f, motor.State.Velocity.Y, 5);
    }

    [Fact]
    public void WaterJumpDispatchConsumesConfiguredVelocityAndDuration()
    {
        var queries = new FlatGroundQueries
        {
            WaterLevel = SourceWaterLevel.Waist,
            WaterJumpActive = true,
            WaterJumpVelocity = new(1, 2, 3),
            WaterJumpDuration = 0.2f
        };
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.Jump), 1f / 66f);
        Assert.True(motor.State.Jumped);
        Assert.InRange(motor.State.WaterJumpTime, 0.18f, 0.2f);
        Assert.True(motor.State.Velocity.Y > 1.9f);
        motor.Tick(new SourceInput(Vector2.Zero, Buttons.None), 0.1f);
        Assert.InRange(motor.State.WaterJumpTime, 0.08f, 0.1f);
        Assert.Equal(2f, motor.State.Velocity.Y, 5);
    }

    [Fact]
    public void ProjectileUsesContinuousSweepAndStopsAfterConfiguredBounceBudget()
    {
        var queries = new PlaneProjectileQueries();
        var projectile = new SourceProjectileMotor(new SourceProjectileProfile { MaximumBounces = 0 }, queries, new(0, 1, 0), new(0, -1, 0));
        var hit = projectile.Tick(1f);
        Assert.True(hit.HasValue);
        Assert.False(projectile.State.Active);
        Assert.Equal(Vector3.Zero, projectile.State.Velocity);
    }

    [Fact]
    public void ProjectileUsesConfiguredGravityRatherThanHiddenConstant()
    {
        var queries = new NoHitProjectileQueries();
        var projectile = new SourceProjectileMotor(new SourceProjectileProfile
        {
            GravitySourceUnitsPerSecondSquared = 400f,
            GravityScale = 1f
        }, queries, Vector3.Zero, Vector3.Zero);
        projectile.Tick(0.1f);
        Assert.InRange(SourceUnits.ToSource(projectile.State.Velocity.Y), -40.1f, -39.9f);
    }

    [Fact]
    public void ProjectilePenetrationUsesExplicitQueryAndTracksBudget()
    {
        var projectile = new SourceProjectileMotor(new SourceProjectileProfile
        {
            GravityScale = 0f, PenetrationPower = 10f, MaximumPenetrations = 1
        }, new PenetratingProjectileQueries(), Vector3.Zero, Vector3.UnitX);
        var hit = projectile.Tick(0.1f);
        Assert.True(hit.HasValue);
        Assert.Equal(1, projectile.State.Penetrations);
        Assert.True(projectile.State.Active);
        Assert.Equal(new Vector3(2, 0, 0), projectile.State.Position);
        Assert.Equal(8f, projectile.State.PenetrationPowerRemaining);
    }

    [Fact]
    public void MovementRecordingsRoundTripAndCompareDeterministically()
    {
        var recording = new MovementRecording();
        recording.Capture(0, new(Vector3.Zero, Vector3.UnitX, GroundState.Grounded, Vector3.UnitY, -1, 1f, false, false));
        recording.Capture(1, new(Vector3.UnitX, Vector3.UnitX, GroundState.Grounded, Vector3.UnitY, -1, 1f, false, false));
        var roundTrip = MovementRecording.FromJson(recording.ToJson());
        var comparison = ParityComparator.Compare(recording.Frames, roundTrip.Frames);
        Assert.True(comparison.Passes(0.000001f, 0.000001f));
        Assert.Equal(0f, comparison.PositionMaximum);
    }

    [Fact]
    public void MovementComparatorUsesNumericTolerancesForPoseDrift()
    {
        var state = new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Grounded, Vector3.UnitY, -1, 1f, false, false);
        var expected = new MovementRecording();
        expected.Capture(0, state);
        var actual = new MovementRecording();
        actual.Capture(0, state with { Position = new(0.0001f, 0f, 0f) });

        var comparison = ParityComparator.Compare(expected.Frames, actual.Frames);

        Assert.Contains(comparison.Errors, error => error.Field == "position");
        Assert.True(comparison.Passes(0.001f, 0.001f));
        Assert.False(comparison.Passes(0.00001f, 0.001f));
    }

    [Fact]
    public void MovementComparatorUsesNumericToleranceForGroundNormalDrift()
    {
        var expected = new MovementRecording();
        expected.Capture(0, new(Vector3.Zero, Vector3.Zero, GroundState.Grounded, Vector3.UnitY, -1, 1f, false, false));
        var actual = new MovementRecording();
        actual.Capture(0, new(Vector3.Zero, Vector3.Zero, GroundState.Grounded,
            Vector3.Normalize(new(0f, 0.99999f, 0.001f)), -1, 1f, false, false));

        var comparison = ParityComparator.Compare(expected.Frames, actual.Frames);

        Assert.True(comparison.GroundNormalMaximum > 0f);
        Assert.True(comparison.Passes(0.001f, 0.001f, groundNormalTolerance: 0.01f));
        Assert.False(comparison.Passes(0.001f, 0.001f, groundNormalTolerance: 0.000001f));
    }

    [Fact]
    public void MovementRecordingPreservesCommandPoseContactsAndImpulses()
    {
        var recording = new MovementRecording();
        var contact = new MovementContact(new(1, 2, 3), Vector3.UnitY, 0.25f, 7, 0.8f, 0.1f, SurfaceId: 4);
        recording.Capture(8, new(Vector3.One, Vector3.UnitX, GroundState.Grounded, Vector3.UnitY, 7, 0.8f, false, false),
            new(new(0.5f, 0.25f), Buttons.Jump, ViewYawRadians: 1.2f, UpMove: 3.5f, ViewPitchRadians: -0.35f), Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.4f),
            new(0, 1, 0), new[] { contact }, new[] { new Vector3(2, 0, 0) });
        var restored = MovementRecording.FromJson(recording.ToJson()).Frames[0];
        Assert.Equal(Buttons.Jump, restored.Buttons);
        Assert.Equal(new Vector2(0.5f, 0.25f), restored.Move);
        Assert.Equal(3.5f, restored.UpMove);
        Assert.Equal(1.2f, restored.ViewYawRadians);
        Assert.Equal(-0.35f, restored.ViewPitchRadians);
        var restoredContacts = Assert.Single(restored.Contacts!);
        Assert.Equal(4, restoredContacts.SurfaceId);
        Assert.Equal(new Vector3(2, 0, 0), restored.Impulses![0]);
    }

    [Fact]
    public void ParityComparatorRejectsMissingOrReorderedTicks()
    {
        var state = new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false);
        var expected = new MovementRecording();
        expected.Capture(1, state);
        expected.Capture(2, state);
        var actual = new MovementRecording();
        actual.Capture(2, state);
        var comparison = ParityComparator.Compare(expected.Frames, actual.Frames);
        Assert.True(comparison.TimingMismatchCount > 0);
        Assert.False(comparison.Passes(0.001f, 0.001f));
    }

    [Fact]
    public void ParityComparatorRejectsMovementStateDivergenceWithoutPoseDrift()
    {
        var state = new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Grounded, Vector3.UnitY, 12, 0.5f, false, false);
        var expected = new MovementRecording();
        expected.Capture(1, state);
        var actual = new MovementRecording();
        actual.Capture(1, state with { Ducking = true, GroundBodyId = 13, SurfaceFriction = 0.25f });

        var comparison = ParityComparator.Compare(expected.Frames, actual.Frames);

        Assert.False(comparison.Passes(0.001f, 0.001f));
        Assert.Contains(comparison.Errors, error => error.Field == "ducking");
        Assert.Contains(comparison.Errors, error => error.Field == "ground-body");
        Assert.Contains(comparison.Errors, error => error.Field == "surface-state");
    }

    [Fact]
    public void ReferenceCourseExecutesDeterministicallyAndRoundTrips()
    {
        var course = new SourceReferenceCourse
        {
            Name = "acceleration-runway",
            Commands = Enumerable.Range(0, 12)
                .Select(tick => new SourceCommand(tick, new(new(0, 1), Buttons.None)))
                .ToList()
        };
        var restored = SourceReferenceCourse.FromJson(course.ToJson());
        var first = course.Execute(new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero));
        var second = restored.Execute(new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero));
        var comparison = ParityComparator.Compare(first.Frames, second.Frames);
        Assert.Equal(course.Commands.Count, restored.Commands.Count);
        Assert.True(comparison.Passes(0.000001f, 0.000001f));
        Assert.Equal(12, first.Frames.Count);
    }

    [Fact]
    public void PredictionBufferReplaysCommandsAndBoundsHistory()
    {
        var buffer = new SourcePredictionBuffer(2);
        for (var tick = 0; tick < 4; tick++) buffer.AddCommand(new SourceCommand(tick, new(Vector2.UnitY, Buttons.None)));
        buffer.AddSnapshot(new SourceSnapshot(3, new(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false)));
        Assert.False(buffer.TryGetCommand(1, out _));
        Assert.True(buffer.TryGetCommand(3, out var command));
        Assert.Equal(3, command.Tick);
        Assert.True(buffer.TryGetSnapshot(3, out _));
        Assert.Equal(2, buffer.CommandsFrom(2).Count());
    }

    [Fact]
    public void PredictionBufferReplaysAuthoritativeSnapshotThroughCommands()
    {
        var queries = new FlatGroundQueries();
        var motor = new SourceMovementMotor(new SourceMovementProfile(), queries, Vector3.Zero);
        var buffer = new SourcePredictionBuffer(8);
        buffer.AddSnapshot(new SourceSnapshot(0, new(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false)));
        buffer.AddCommand(new SourceCommand(1, new(new(0, 1), Buttons.None)));
        buffer.AddCommand(new SourceCommand(2, new(new(0, 1), Buttons.None)));
        Assert.True(buffer.ReplayFrom(motor, 0, 1f / 66f, out var replayed));
        Assert.True(replayed.Position.X > 0f);
        Assert.Equal(SourceMoveType.Walk, motor.State.MoveType);
    }

    [Fact]
    public void PredictionBufferSerializesWorldCorrectionState()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, new(1f, 2f, 3f), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        var buffer = new SourcePredictionBuffer();
        var world = host.CaptureState(5);
        buffer.AddSnapshot(new SourceSnapshot(5,
            new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false), world));

        var restored = SourcePredictionBuffer.FromJson(buffer.ToJson());
        Assert.True(restored.TryGetSnapshot(5, out var snapshot));
        Assert.Equal(body.ID, Assert.Single(snapshot.WorldState!.Bodies).BodyId);
        Assert.Equal(5, snapshot.WorldState.Tick);
    }

    [Fact]
    public void PredictionBufferSerializesCommandsAndSnapshotsForCorrectionLogs()
    {
        var buffer = new SourcePredictionBuffer(8);
        var state = new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false);
        buffer.AddSnapshot(new SourceSnapshot(4, state));
        buffer.AddCommand(new SourceCommand(5, new(new(0, 1), Buttons.Jump, 0.5f)));
        var restored = SourcePredictionBuffer.FromJson(buffer.ToJson(), 8);
        Assert.True(restored.TryGetSnapshot(4, out _));
        Assert.True(restored.TryGetCommand(5, out var command));
        Assert.Equal(Buttons.Jump, command.Input.Buttons);
        Assert.Equal(0.5f, command.Input.ViewYawRadians);
    }

    [Fact]
    public void LagCompensationRewindsAndRestoresAuthoritativeJoltWorldState()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize(1024, 0, 1024, 256);
        var body = host.CreateBoxBody(Vector3.One, new(1f, 0f, 0f), JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic, new SourceRigidBodyProfile { GravityFactor = 0f });
        var history = new SourceLagCompensationHistory(2);
        history.Record(host, 10);

        var moved = JoltRigidBodyState.Capture(host, body) with { Position = new(9f, 0f, 0f) };
        host.RestoreState(new SourcePhysicsWorldState(11,
            new[] { new SourcePhysicsBodySnapshot(body.ID, moved) }));

        Assert.True(history.TryBeginRewind(host, 10, out var rewind));
        using (rewind)
        {
            Assert.Equal(1f, JoltRigidBodyState.Capture(host, body).Position.X);
        }
        Assert.Equal(9f, JoltRigidBodyState.Capture(host, body).Position.X);
    }

    [Fact]
    public void PredictionCommandsCarryStableTransportSequenceAndRejectInvalidInput()
    {
        var buffer = new SourcePredictionBuffer(8);
        buffer.AddCommand(new SourceCommand(7, new(new(0.25f, 1f), Buttons.Forward, 0.5f), 42));

        var restored = SourcePredictionBuffer.FromJson(buffer.ToJson(), 8);
        Assert.True(restored.TryGetCommand(7, out var command));
        Assert.Equal(42, command.Sequence);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            buffer.AddCommand(new SourceCommand(8, default, -1)));
        Assert.Throws<ArgumentException>(() =>
            buffer.AddCommand(new SourceCommand(9, new(new(float.NaN, 0f), Buttons.None))));
        Assert.Throws<ArgumentException>(() =>
            buffer.AddSnapshot(new SourceSnapshot(4, default,
                new SourcePhysicsWorldState(3, Array.Empty<SourcePhysicsBodySnapshot>()))));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            buffer.ApplyCorrection(new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero),
                new SourceAuthoritativeCorrection(0, -1, default), 1f / 66f, out _));
    }

    [Fact]
    public void PredictionBufferAppliesAuthoritativeCorrectionAndReplaysOnlyLaterTicks()
    {
        var motor = new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero);
        var buffer = new SourcePredictionBuffer(8);
        buffer.AddCommand(new SourceCommand(1, new(new(0, 1), Buttons.None), 10));
        buffer.AddCommand(new SourceCommand(2, new(new(0, 1), Buttons.None), 11));
        var correction = new SourceAuthoritativeCorrection(1, 10,
            new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false));

        Assert.True(buffer.ApplyCorrection(motor, correction, 1f / 66f, out var state));
        Assert.Single(buffer.CommandsFrom(correction.Tick + 1));
        Assert.True(state.Position.X > 0f);
        Assert.True(buffer.TryGetSnapshot(1, out var snapshot));
        Assert.Equal(1, snapshot.Tick);
    }

    [Fact]
    public void SourceMultiDamageMatchesTraceThenAggregateCommitOrdering()
    {
        var target = new DamageTargetProbe();
        var accumulator = new SourceMultiDamageAccumulator();
        var first = new SourceDamageInfo(10f, 10f, Vector3.UnitX, Vector3.Zero, Vector3.One, 0x2, 3);
        var second = new SourceDamageInfo(7f, 7f, Vector3.UnitY, Vector3.One, Vector3.UnitZ, 0x8, 4);

        accumulator.DispatchTraceAttack(12, target, first, Vector3.UnitZ, default);
        accumulator.DispatchTraceAttack(12, target, second, Vector3.UnitZ, default);
        Assert.Equal(2, target.TraceCount);
        Assert.Equal(0, target.TakeDamageCount);

        accumulator.ApplyMultiDamage();
        Assert.Equal(1, target.TakeDamageCount);
        Assert.Equal(17f, target.LastDamage.Damage);
        Assert.Equal(10f, target.LastDamage.MaxDamage);
        Assert.Equal(Vector3.UnitX + Vector3.UnitY, target.LastDamage.DamageForce);
        Assert.Equal(0xA, target.LastDamage.DamageType);
        Assert.Equal(4, target.LastDamage.AmmoType);
        Assert.Equal(Vector3.One, target.LastDamage.DamagePosition);
        Assert.Equal(Vector3.UnitZ, target.LastDamage.ReportedPosition);
    }

    [Fact]
    public void EvidenceManifestLoadsTypedMovementProfile()
    {
        var json = File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "source-profile.json"));
        var profile = SourceProfileLoader.LoadMovement(json);
        Assert.Equal(800f, profile.GravitySourceUnitsPerSecondSquared);
        Assert.Equal(0.510f, profile.JumpTimingSeconds);
        Assert.Equal(21f, profile.JumpHeightSourceUnits);
        Assert.Equal(12f, profile.WaterViewDistanceSourceUnits);
        Assert.Equal(0f, profile.BounceMultiplier);
        Assert.Equal(3500f, profile.MaxVelocitySourceUnitsPerSecond);
        Assert.Equal(4f, profile.GroundFriction);
        Assert.Equal(4, profile.MaxBumps);
        Assert.Equal(0.8f, profile.WaterWishSpeedScale);
        Assert.Equal(1f / 3f, profile.CrouchSpeedScale);
        Assert.Equal(5f, profile.NoclipSpeedFactor);
        Assert.Equal(3f, profile.ObserverSpeedFactor);
        Assert.Equal(5f, profile.NoclipAcceleration);
        Assert.Equal(5f, profile.ObserverAcceleration);
        Assert.Equal(30f, profile.AirWishSpeedCapSourceUnitsPerSecond);
        Assert.Equal(140f, profile.GroundCategorizationUpwardSpeedSourceUnitsPerSecond);
        Assert.Equal(-0.707f, profile.LadderFacingDotThreshold);
        Assert.Equal(0.2f, profile.LadderPerpendicularDamping);
        Assert.Equal(200f, profile.LadderSpeedSourceUnitsPerSecond);
        Assert.Equal(270f, profile.LadderJumpSpeedSourceUnitsPerSecond);
        Assert.Equal(0.4f, profile.DuckDownTransitionSeconds);
        Assert.Equal(0.2f, profile.DuckUpTransitionSeconds);
        Assert.Equal(0.08f, profile.QueryRecoveryDistanceSourceUnits);
        Assert.Equal(64f, profile.StandingEyeSourceUnits);
        Assert.Equal(28f, profile.DuckEyeSourceUnits);
        Assert.Equal(new Vector3(16f, 36f, 16f), profile.StandingHalfExtentsSourceUnits);
        Assert.Equal(new Vector3(16f, 18f, 16f), profile.CrouchedHalfExtentsSourceUnits);
    }

    [Fact]
    public void NoclipUsesSourceAccelerationFrictionAndSpeedButtonRules()
    {
        var profile = new SourceMovementProfile();
        var accelerated = new SourceMovementMotor(profile, new FlatGroundQueries(), Vector3.Zero);
        accelerated.Tick(new SourceInput(new(0f, 1f), Buttons.None, MoveType: SourceMoveType.Noclip), 1f / 66f);

        var speedHeld = new SourceMovementMotor(profile, new FlatGroundQueries(), Vector3.Zero);
        speedHeld.Tick(new SourceInput(new(0f, 1f), Buttons.Speed, MoveType: SourceMoveType.Noclip), 1f / 66f);

        var maxSpeed = profile.MaxSpeed * profile.NoclipSpeedFactor;
        var firstAcceleration = profile.NoclipAcceleration * (1f / 66f) * maxSpeed;
        var frictionDrop = (maxSpeed / 4f) * profile.GroundFriction * (1f / 66f);
        var expected = firstAcceleration - frictionDrop;
        Assert.Equal(expected, accelerated.State.Velocity.X, 5);
        Assert.True(accelerated.State.Velocity.X > speedHeld.State.Velocity.X);
        Assert.Equal(firstAcceleration * 0.5f - frictionDrop, speedHeld.State.Velocity.X, 5);
    }

    [Fact]
    public void WaterMoveUsesSourceIdleDriftJumpWishAndScalarAcceleration()
    {
        var profile = new SourceMovementProfile();
        var idleQueries = new FlatGroundQueries { WaterLevel = SourceWaterLevel.Waist };
        var idle = new SourceMovementMotor(profile, idleQueries, new Vector3(0f, 1f, 0f));
        idle.Tick(new SourceInput(Vector2.Zero, Buttons.None), 1f / 66f);
        Assert.True(idle.State.Velocity.Y < 0f);

        var jumpQueries = new FlatGroundQueries { WaterLevel = SourceWaterLevel.Waist };
        var jump = new SourceMovementMotor(profile, jumpQueries, new Vector3(0f, 1f, 0f));
        jump.Tick(new SourceInput(Vector2.Zero, Buttons.Jump), 1f / 66f);
        Assert.True(jump.State.Velocity.Y > 0f);
        Assert.True(jump.State.Velocity.Y < profile.MaxSpeed);
    }

    [Fact]
    public void ObserverDispatchPreservesSourceModesAndTargetFollowing()
    {
        var profile = new SourceMovementProfile();
        var fixedCamera = new SourceMovementMotor(profile, new FlatGroundQueries(), Vector3.Zero);
        fixedCamera.LoadState(new(Vector3.Zero, new(1f, 0f, 0f), GroundState.Airborne, Vector3.UnitY, -1, 1f, false, false));
        fixedCamera.Tick(new SourceInput(new(0f, 1f), Buttons.None, MoveType: SourceMoveType.Observer,
            ObserverMode: SourceObserverMode.Fixed), 1f / 66f);
        Assert.Equal(Vector3.Zero, fixedCamera.State.Position);
        Assert.Equal(new Vector3(1f, 0f, 0f), fixedCamera.State.Velocity);

        var queries = new FlatGroundQueries
        {
            ObserverTarget = new MovementState(new(4f, 5f, 6f), new(7f, 8f, 9f),
                GroundState.Grounded, Vector3.UnitY, 3, 1f, false, false)
        };
        var inEye = new SourceMovementMotor(profile, queries, Vector3.Zero);
        inEye.Tick(new SourceInput(Vector2.Zero, Buttons.None, MoveType: SourceMoveType.Observer,
            ObserverMode: SourceObserverMode.InEye), 1f / 66f);
        Assert.Equal(new Vector3(4f, 5f, 6f), inEye.State.Position);
        Assert.Equal(new Vector3(7f, 8f, 9f), inEye.State.Velocity);

        var roaming = new SourceMovementMotor(profile, new FlatGroundQueries(), Vector3.Zero);
        roaming.Tick(new SourceInput(new(0f, 1f), Buttons.None, MoveType: SourceMoveType.Observer,
            ObserverMode: SourceObserverMode.Roaming, ObserverNoClip: true), 1f / 66f);
        Assert.True(roaming.State.Position.LengthSquared() > 0f);
    }

    [Fact]
    public void MovementRecordingPreservesObserverInputState()
    {
        var recording = new MovementRecording();
        var state = new MovementState(Vector3.Zero, Vector3.Zero, GroundState.Airborne,
            Vector3.UnitY, -1, 1f, false, false, MoveType: SourceMoveType.Observer);
        recording.Capture(0, state, new SourceInput(Vector2.Zero, Buttons.None,
            MoveType: SourceMoveType.Observer, ObserverMode: SourceObserverMode.InEye, ObserverNoClip: false));
        var restored = MovementRecording.FromJson(recording.ToJson());
        Assert.Equal(SourceObserverMode.InEye, restored.Frames[0].ObserverMode);
        Assert.False(restored.Frames[0].ObserverNoClip);
    }

    [Fact]
    public void EvidenceManifestLoadsTypedPushawayProfile()
    {
        var json = File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "source-profile.json"));
        var profile = SourceProfileLoader.LoadPushaway(json);

        Assert.Equal(30000f, profile.PropForce);
        Assert.Equal(75f, profile.MinimumPlayerSpeedSourceUnitsPerSecond);
        Assert.Equal(1000f, profile.MaximumPropForce);
        Assert.Equal(200000f, profile.PlayerForce);
        Assert.Equal(10000f, profile.MaximumPlayerForce);
        Assert.Equal(10f, profile.MinimumPushMassKilograms);
        Assert.Equal(30f, profile.MaximumPushMassKilograms);
        Assert.Equal(5f, profile.MaximumPushawayDistanceSourceUnits);
        Assert.Equal(0.25f, profile.RotatingDoorForceScale);
    }

    [Fact]
    public void MovementProfileRejectsNonFiniteOrInvalidAuthoredValues()
    {
        var invalid = new SourceMovementProfile { MaxSpeedSourceUnitsPerSecond = float.NaN };
        Assert.Throws<InvalidDataException>(invalid.Validate);

        invalid = new SourceMovementProfile { MaxBumps = 0 };
        Assert.Throws<InvalidDataException>(invalid.Validate);

        invalid = new SourceMovementProfile { StandingHalfExtentsSourceUnits = new Vector3(16f, 0f, 16f) };
        Assert.Throws<InvalidDataException>(invalid.Validate);

        invalid = new SourceMovementProfile { CrouchedHalfExtentsSourceUnits = new Vector3(float.NaN, 18f, 16f) };
        Assert.Throws<InvalidDataException>(invalid.Validate);
    }

    [Fact]
    public void PhysicsHostRetainsAuthoritativeMovementProfile()
    {
        var profile = new SourceMovementProfile { StepHeightSourceUnits = 22f };
        using var host = new JoltPhysicsHost(profile);

        Assert.Same(profile, host.MovementProfile);
        Assert.Equal(22f, host.MovementProfile.StepHeightSourceUnits);
    }

    [Fact]
    public void SourceVehicleContractPreservesAuthoredWheelAndAxleShape()
    {
        var profile = new SourceVehicleProfile
        {
            AxleCount = 2,
            WheelsPerAxle = 2,
            Axles = new[]
            {
                new SourceVehicleAxleProfile
                {
                    Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 14f },
                    TorqueFactor = 0.5f,
                    BrakeFactor = 0.5f
                },
                new SourceVehicleAxleProfile
                {
                    Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 15f },
                    TorqueFactor = 0.5f,
                    BrakeFactor = 0.5f
                }
            },
            Engine = new SourceVehicleEngineProfile { GearRatios = new[] { 2.5f, 1.8f, 1.2f } }
        };

        profile.Validate();

        Assert.Equal(4, profile.WheelCount);
        Assert.Equal(14f, profile.Axles[0].Wheels.RadiusSourceUnits);
        Assert.Equal(3, profile.Engine.GearRatios.Count);
    }

    [Fact]
    public void SourceVehicleContractRejectsUnmatchedOrInvalidAuthoredData()
    {
        var profile = new SourceVehicleProfile
        {
            AxleCount = 2,
            WheelsPerAxle = 2,
            Axles = new[] { new SourceVehicleAxleProfile { Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 14f } } }
        };
        Assert.Throws<InvalidDataException>(profile.Validate);

        profile = new SourceVehicleProfile
        {
            AxleCount = 1,
            WheelsPerAxle = 2,
            Axles = new[] { new SourceVehicleAxleProfile { Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = float.NaN } } }
        };
        Assert.Throws<InvalidDataException>(profile.Validate);

        profile = new SourceVehicleProfile
        {
            AxleCount = 1,
            WheelsPerAxle = 1,
            Axles = new[] { new SourceVehicleAxleProfile { Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 14f } } },
            Steering = new SourceVehicleSteeringProfile { SteeringExponent = float.PositiveInfinity }
        };
        Assert.Throws<InvalidDataException>(profile.Validate);
    }

    [Fact]
    public void SourcePushawayPolicyMatchesSourceForceClampAndSpeedGate()
    {
        var profile = new SourcePushawayProfile();
        var blocked = SourcePushawayPolicy.ComputeObstacleForce(profile, new(2f, 10f, 0f), Vector3.Zero,
            74f, 20f, multiplayerSolid: true, rotatingDoor: false);
        Assert.Equal(Vector3.Zero, blocked);

        var force = SourcePushawayPolicy.ComputeObstacleForce(profile, new(2f, 10f, 0f), Vector3.Zero,
            75f, 20f, multiplayerSolid: true, rotatingDoor: false);
        Assert.Equal(1000f, force.X);
        Assert.Equal(0f, force.Y);
        Assert.Equal(0f, force.Z);
    }

    [Fact]
    public void SourcePushawayPolicyMatchesPlayerMassClampAndDoorScale()
    {
        var profile = new SourcePushawayProfile();
        var force = SourcePushawayPolicy.ComputePlayerCommandPush(profile,
            new(1f, 0f, 0f), Vector3.Zero, Vector3.Zero, new(1f, 0f, 0f),
            30f, nearestPropPointInsidePlayerBounds: true, rotatingDoor: true);

        Assert.Equal(2500f, force.X);
        Assert.Equal(0f, force.Y);
        Assert.Equal(0f, force.Z);
    }

    [Fact]
    public void SourcePhysicsImpulseConversionUsesExactSourceInchScale()
    {
        Assert.Equal(new Vector3(0.0254f, -0.0508f, 0.0762f),
            SourcePhysicsImpulseConversion.ToJolt(new Vector3(1f, -2f, 3f)));
    }

    [Fact]
    public void SourceFluidProfilePreservesAuthoredPlaneAndControllerParameters()
    {
        var profile = new SourceFluidProfile
        {
            SurfacePlane = new Vector4(0f, 1f, 0f, 0f),
            CurrentVelocitySourceUnitsPerSecond = new Vector3(4f, 0f, 2f),
            DensityKgPerM3 = 1000f,
            Damping = 1f,
            TorqueFactor = 0.01f,
            ViscosityFactor = 0.1f,
            BuoyancyForceNewtons = 12f
        };
        profile.Validate();

        Assert.Equal(Vector3.UnitY, profile.SurfaceNormal);
        Assert.Equal(1000f, profile.DensityKgPerM3);
        Assert.Equal(12f, profile.BuoyancyForceNewtons);
        Assert.Equal(SourceUnits.ToMeters(new Vector3(4f, 0f, 2f)),
            SourceUnits.ToMeters(profile.CurrentVelocitySourceUnitsPerSecond));
    }

    [Fact]
    public void JoltFluidControllerTracksSensorFluidStartAndEndTouch()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize();
        var fluid = host.CreateBoxBody(new(5f), Vector3.Zero, JoltPhysicsSharp.MotionType.Static,
            SourceObjectLayer.Trigger);
        var body = host.CreateBoxBody(new(0.1f), Vector3.Zero, JoltPhysicsSharp.MotionType.Dynamic,
            SourceObjectLayer.Dynamic);
        using var controller = new JoltFluidController(host);
        controller.RegisterFluidBody(fluid, new SourceFluidProfile
        {
            SurfacePlane = new Vector4(0f, 1f, 0f, 0f),
            DensityKgPerM3 = 1000f,
            Damping = 1f,
            TorqueFactor = 0.01f,
            ViscosityFactor = 0.1f
        });
        controller.RegisterFixedStep();

        host.Step();
        Assert.True(controller.ActiveContactCount > 0);

        JoltPhysicsSharp.RVector3 outside = new(100f, 100f, 100f);
        host.Bodies.SetRPosition(in body, in outside, JoltPhysicsSharp.Activation.Activate);
        host.Step();
        Assert.Equal(0, controller.ActiveContactCount);
    }

    [Fact]
    public void SourceNetworkSerializationPreservesCommandsAndCorrections()
    {
        var command = new SourceCommand(12, new SourceInput(new(0.25f, -1f), Buttons.Jump,
            0.5f, 2f), 44);
        var restoredCommand = SourceNetworkSerialization.DeserializeCommand(
            SourceNetworkSerialization.SerializeCommand(command));
        Assert.Equal(command, restoredCommand);

        var correction = new SourceAuthoritativeCorrection(12, 44,
            new MovementState(new(1f, 2f, 3f), new(4f, 5f, 6f), GroundState.Grounded,
                Vector3.UnitY, 7, 1f, false, false));
        var restoredCorrection = SourceNetworkSerialization.DeserializeCorrection(
            SourceNetworkSerialization.SerializeCorrection(correction));
        Assert.Equal(correction, restoredCorrection);
    }

    [Fact]
    public void SourceImpairedTransportPreservesReliablePacketsUntilAcknowledged()
    {
        var transport = new SourceImpairedTransport(new SourceNetworkImpairment(
            LossProbability: 0.5f, MinimumLatencyTicks: 2, MaximumLatencyTicks: 2, Seed: 2));
        var packet = transport.Send(SourceNetworkEndpoint.Client, new byte[] { 1, 2, 3 }, reliable: true, currentTick: 0);

        var delivered = new List<SourceNetworkPacket>();
        for (var tick = 0; tick < 20 && delivered.Count == 0; tick++)
        {
            delivered.AddRange(transport.Receive(SourceNetworkEndpoint.Server, tick));
            transport.RetransmitUnacknowledged(tick);
        }

        Assert.NotEmpty(delivered);
        Assert.All(delivered, value => Assert.Equal(packet.Sequence, value.Sequence));
        transport.Acknowledge(SourceNetworkEndpoint.Client, packet.Sequence);
        Assert.Equal(0, transport.UnacknowledgedReliableCount);
    }

    [Fact]
    public void SourceImpairedTransportAppliesLatencyAndPayloadLimit()
    {
        var transport = new SourceImpairedTransport(new SourceNetworkImpairment(
            MinimumLatencyTicks: 3, MaximumLatencyTicks: 3, Seed: 9));
        transport.Send(SourceNetworkEndpoint.Client, new byte[] { 7 }, reliable: false, currentTick: 0);
        Assert.Empty(transport.Receive(SourceNetworkEndpoint.Server, 2));
        Assert.Single(transport.Receive(SourceNetworkEndpoint.Server, 3));
        Assert.Throws<ArgumentException>(() => transport.Send(SourceNetworkEndpoint.Client,
            new byte[SourceNetworkPacket.MaximumDatagramPayload + 1], false, 4));
    }

    [Fact]
    public void SourceUdpTransportRoundTripsCommandAndCorrectionOnLoopback()
    {
        using var server = new SourceUdpTransport(
            new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 0),
            new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 1));
        using var client = new SourceUdpTransport(
            new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 0), server.LocalEndPoint);
        server.SetRemote(client.LocalEndPoint);

        var command = new SourceCommand(3, new SourceInput(new(1f, 0f), Buttons.Jump), Sequence: 8);
        var sentCommand = client.SendCommand(command, 3);
        var receivedCommand = WaitForCommand(server);
        Assert.Equal(sentCommand.Sequence, receivedCommand.packet.Sequence);
        Assert.Equal(command, receivedCommand.command);

        var correction = new SourceAuthoritativeCorrection(3, command.Sequence,
            new MovementState(Vector3.One, Vector3.Zero, GroundState.Airborne, Vector3.UnitY,
                -1, 1f, false, false));
        var sentCorrection = server.SendCorrection(correction, 3, sentCommand.Sequence);
        var receivedCorrection = WaitForCorrection(client);
        Assert.Equal(sentCorrection.Sequence, receivedCorrection.packet.Sequence);
        Assert.Equal(correction, receivedCorrection.correction);
    }

    [Fact]
    public void SourceAuthoritativeSessionOrdersCommandsAndProducesWorldCorrection()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize();
        var motor = new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero);
        var session = new SourceAuthoritativePhysicsSession(host, motor,
            _ => new SourceInput(Vector2.Zero, Buttons.None));
        var corrections = new List<SourceAuthoritativeCorrection>();
        session.CorrectionProduced += corrections.Add;

        Assert.True(session.AcceptCommand(new SourceCommand(0,
            new SourceInput(new(0f, 1f), Buttons.None), Sequence: 5)));
        Assert.False(session.AcceptCommand(new SourceCommand(0,
            new SourceInput(Vector2.Zero, Buttons.None), Sequence: 6)));
        Assert.False(session.AcceptCommand(new SourceCommand(0,
            new SourceInput(Vector2.Zero, Buttons.None), Sequence: 4)));

        Assert.Equal(1, session.Advance(host.FixedStepSeconds));
        var correction = Assert.Single(corrections);
        Assert.Equal(0, correction.Tick);
        Assert.Equal(5, correction.LastProcessedSequence);
        Assert.NotNull(correction.WorldState);
        Assert.Equal(1, session.SimulationTick);
    }

    [Fact]
    public void SourceUdpCommandReachesAuthorityAndReturnsCorrection()
    {
        using var serverTransport = new SourceUdpTransport(
            new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 0),
            new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 1));
        using var clientTransport = new SourceUdpTransport(
            new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 0), serverTransport.LocalEndPoint);
        serverTransport.SetRemote(clientTransport.LocalEndPoint);
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize();
        var session = new SourceAuthoritativePhysicsSession(host,
            new SourceMovementMotor(new SourceMovementProfile(), new FlatGroundQueries(), Vector3.Zero),
            _ => new SourceInput(Vector2.Zero, Buttons.None));
        session.CorrectionProduced += correction => session.SendProducedCorrection(serverTransport, correction, correction.Tick);

        clientTransport.SendCommand(new SourceCommand(0,
            new SourceInput(Vector2.Zero, Buttons.None), Sequence: 9), 0);
        Assert.Equal(1, session.ProcessUdpCommands(serverTransport));
        session.Advance(host.FixedStepSeconds);

        var result = WaitForCorrection(clientTransport);
        Assert.Equal(0, result.correction.Tick);
        Assert.Equal(9, result.correction.LastProcessedSequence);
    }

    [Fact]
    public void SourceNetworkConnectionTrackerMatchesTimeoutAndReconnectStates()
    {
        var tracker = new SourceNetworkConnectionTracker(1f,
            new SourceNetworkTimeoutProfile(ConnectionProblemSeconds: 15f, SignOnTimeoutSeconds: 120f));
        tracker.Connect(0);
        Assert.Equal(SourceNetworkConnectionState.Connected, tracker.State);
        tracker.ObservePacket(1);
        Assert.Equal(SourceNetworkConnectionState.TimingOut, tracker.Advance(16));
        Assert.Equal(SourceNetworkConnectionState.TimedOut, tracker.Advance(121));

        tracker.ObservePacket(122);
        Assert.Equal(SourceNetworkConnectionState.Connected, tracker.State);
        Assert.Equal(0f, tracker.TimeSinceLastReceivedSeconds(122));
        tracker.Disconnect();
        Assert.Equal(SourceNetworkConnectionState.Disconnected, tracker.State);
    }

    [Fact]
    public void PhysicsPerformanceRecorderEnforcesAuthoredBudgets()
    {
        var recorder = new SourcePhysicsPerformanceRecorder();
        recorder.Capture(0, new PhysicsStepMetrics(1, 2, 1f / 66f, 0.5d, 4));
        recorder.Capture(1, new PhysicsStepMetrics(2, 2, 1f / 66f, 2.0d, 5));

        var report = recorder.Evaluate(new SourcePhysicsPerformanceBudget
        {
            MaximumStepMilliseconds = 1d,
            MaximumActiveBodies = 4,
            MaximumCollisionSteps = 1,
            MaximumIntegrationSubSteps = 2
        });

        Assert.False(report.Passes);
        Assert.Equal(2, report.SampleCount);
        Assert.Contains("step-time:1", report.Violations);
        Assert.Contains("active-bodies:1", report.Violations);
        Assert.Contains("collision-steps:1", report.Violations);
    }

    private static (SourceNetworkPacket packet, SourceCommand command) WaitForCommand(SourceUdpTransport transport)
    {
        for (var attempt = 0; attempt < 100; attempt++)
        {
            if (transport.TryReceiveCommand(out var packet, out var command)) return (packet, command);
            Thread.Sleep(1);
        }
        throw new Xunit.Sdk.XunitException("Timed out waiting for UDP command.");
    }

    private static (SourceNetworkPacket packet, SourceAuthoritativeCorrection correction) WaitForCorrection(SourceUdpTransport transport)
    {
        for (var attempt = 0; attempt < 100; attempt++)
        {
            if (transport.TryReceiveCorrection(out var packet, out var correction)) return (packet, correction);
            Thread.Sleep(1);
        }
        throw new Xunit.Sdk.XunitException("Timed out waiting for UDP correction.");
    }

    [Fact]
    public void JoltPushawayControllerRunsOnFixedStepAndRecordsConvertedImpulse()
    {
        using var host = new JoltPhysicsHost(new SourceMovementProfile());
        host.Initialize();
        var body = host.CreateBoxBody(new Vector3(0.1f), new Vector3(0.1f, 0f, 0f),
            JoltPhysicsSharp.MotionType.Dynamic, SourceObjectLayer.Dynamic,
            new SourceRigidBodyProfile { MassKg = 1f });
        var impulses = new List<SourcePhysicsImpulseEvent>();
        host.ImpulseApplied += impulses.Add;
        using var controller = new JoltPushawayController(host, new SourcePushawayProfile(),
            force => force * 0.001f,
            () => new[] { body });
        controller.SetPlayerState(Vector3.Zero, 100f);
        controller.RegisterFixedStep();

        host.Step();

        var impulse = Assert.Single(impulses, value => value.BodyId == body.ID);
        Assert.True(impulse.Impulse.X > 0f);
        Assert.Equal(Vector3.Zero, impulse.WorldPoint);
    }

    private sealed class FlatGroundQueries : ISourceMovementQueries
    {
        public MovementState State { get; set; }
        public SourceSurface GroundSurface { get; set; } = new("fallback");
        public bool LadderActive { get; set; }
        public Vector3 LadderNormal { get; set; } = -Vector3.UnitX;
        public int LadderBodyId { get; set; } = -1;
        public SourceWaterLevel WaterLevel { get; set; } = SourceWaterLevel.Dry;
        public bool Empty { get; set; } = true;
        public bool StartSolid { get; set; }
        public bool AllSolid { get; set; }
        public bool WaterJumpActive { get; set; }
        public Vector3 WaterJumpVelocity { get; set; }
        public float WaterJumpDuration { get; set; }
        public int ContactBodyId { get; set; } = -1;
        public int ImpulseCount { get; private set; }
        public int LastImpulseBodyId { get; private set; } = -1;
        public Vector3 LastImpulse { get; private set; }
        public MovementState? ObserverTarget { get; set; }
        public MovementContact SweepPlayer(Vector3 start, Vector3 end, bool crouched)
        {
            if (StartSolid || AllSolid)
                return new(start, -Vector3.UnitX, 0f, ContactBodyId, 1f, 0f, StartSolid, AllSolid);
            if (end.Y <= 0 && start.Y >= 0) return new(new(end.X, 0, end.Z), Vector3.UnitY, 0.5f, ContactBodyId, 1, 0);
            return new(end, Vector3.UnitY, 1f, -1, 1, 0);
        }
        public bool IsEmpty(Vector3 position, bool crouched) => Empty;
        public Vector3 GetBodyPointVelocity(int bodyId, Vector3 worldPoint) => Vector3.Zero;
        public void ApplyCharacterImpulse(int bodyId, Vector3 point, Vector3 impulse)
        {
            ImpulseCount++;
            LastImpulseBodyId = bodyId;
            LastImpulse = impulse;
        }
        public SourceWaterLevel GetWaterLevel(Vector3 position, bool crouched) => WaterLevel;
        public bool TryGetObserverTarget(out MovementState target)
        {
            if (ObserverTarget is { } value) { target = value; return true; }
            target = default; return false;
        }
        public bool TryLadder(Vector3 position, Vector3 direction, out Vector3 normal, out int bodyId)
        {
            normal = LadderNormal; bodyId = LadderBodyId; return LadderActive;
        }
        public SourceSurface GetSurface(int surfaceId) => GroundSurface;
        public bool TryWaterJump(Vector3 position, Vector3 direction, out Vector3 velocity, out float durationSeconds)
        {
            velocity = WaterJumpVelocity; durationSeconds = WaterJumpDuration; return WaterJumpActive;
        }
    }

    private static JoltPhysicsSharp.IndexedTriangle Triangle(uint a, uint b, uint c)
    {
        return new JoltPhysicsSharp.IndexedTriangle(in a, in b, in c, 0, 0);
    }

    private sealed class PlaneProjectileQueries : IProjectileQueries
    {
        public bool Sweep(Vector3 start, Vector3 end, out ProjectileHit hit)
        {
            if (start.Y >= 0 && end.Y <= 0)
            {
                var fraction = start.Y / (start.Y - end.Y);
                hit = new(Vector3.Lerp(start, end, fraction), Vector3.UnitY, 1, 0f);
                return true;
            }
            hit = default; return false;
        }
    }

    private sealed class NoHitProjectileQueries : IProjectileQueries
    {
        public bool Sweep(Vector3 start, Vector3 end, out ProjectileHit hit)
        {
            hit = default;
            return false;
        }
    }

    private sealed class PenetratingProjectileQueries : IProjectileQueries, IProjectilePenetrationQueries
    {
        public bool Sweep(Vector3 start, Vector3 end, out ProjectileHit hit)
        {
            hit = new(new(0.5f, 0, 0), -Vector3.UnitX, 5, 0f, ThicknessInches: 1f);
            return true;
        }

        public bool TryPenetrate(Vector3 entryPosition, in ProjectileHit entryHit, Vector3 incomingVelocity,
            float availablePower, out Vector3 exitPosition, out Vector3 exitVelocity, out float consumedPower)
        {
            exitPosition = new(2, 0, 0);
            exitVelocity = incomingVelocity;
            consumedPower = 2f;
            return true;
        }
    }

    private sealed class DamageTargetProbe : ISourceDamageTarget
    {
        public int TraceCount { get; private set; }
        public int TakeDamageCount { get; private set; }
        public SourceDamageInfo LastDamage { get; private set; }
        public void TraceAttack(in SourceDamageInfo info, Vector3 direction, in HitscanHit hit) => TraceCount++;
        public void TakeDamage(in SourceDamageInfo info) { TakeDamageCount++; LastDamage = info; }
    }
}
