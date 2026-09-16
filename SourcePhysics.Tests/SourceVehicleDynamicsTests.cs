using System.Numerics;
using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class SourceVehicleDynamicsTests
{
    [Fact]
    public void VehicleSpeedBoundaryUsesSourceMphConversion()
    {
        var sourceSpeed = SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(60f);

        Assert.Equal(60f, SourceVehicleDynamics.SourceUnitsPerSecondToMilesPerHour(sourceSpeed), 4);
    }

    [Fact]
    public void SteeringMatchesSourceSpeedRemapAndDigitalExponent()
    {
        var profile = Profile() with
        {
            Steering = new SourceVehicleSteeringProfile
            {
                DegreesSlow = 30f, DegreesFast = 15f, DegreesBoost = 10f,
                SpeedSlowMilesPerHour = 0f, SpeedFastMilesPerHour = 60f,
                SteeringExponent = 2f
            },
            Engine = new SourceVehicleEngineProfile { MaxSpeedMilesPerHour = 100f, BoostMaxSpeedMilesPerHour = 120f }
        };

        var angle = SourceVehicleDynamics.CalculateSteeringAngleDegrees(profile,
            SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(30f), 0.5f, analog: false);

        Assert.Equal(5.625f, angle, 4);
    }

    [Fact]
    public void AutomaticTransmissionUsesSourceWheelRpmFormulaAndShiftRules()
    {
        var profile = Profile() with
        {
            Engine = new SourceVehicleEngineProfile
            {
                GearRatios = new[] { 2f, 1f }, AxleRatio = 2f,
                ShiftUpRpm = 1000f, ShiftDownRpm = 200f, IsAutomaticTransmission = true
            }
        };
        var result = SourceVehicleDynamics.CalculateEngineTransmission(profile,
            new[] { MathF.PI * 10f, MathF.PI * 10f }, 0, 1f);

        Assert.Equal(1, result.Gear);
        Assert.Equal(600f, result.EngineRpm, 3);
    }

    [Fact]
    public void DriveAndBrakeTorqueUseAuthoredWheelAndAxleFactors()
    {
        var profile = Profile() with
        {
            Engine = new SourceVehicleEngineProfile
            {
                Horsepower = 100f, MaxRpm = 6000f, AxleRatio = 2f,
                GearRatios = new[] { 2f }
            },
            Axles = new[]
            {
                new SourceVehicleAxleProfile
                {
                    Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 10f },
                    TorqueFactor = 1f, BrakeFactor = 1f
                }
            }
        };
        var drive = SourceVehicleDynamics.ComputeDriveTorqueNewtonMeters(profile, 0, 1f,
            10f, 1000f, 0f, 0f, false, false, 0);
        var brake = SourceVehicleDynamics.ComputeBrakeTorqueNewtonMeters(profile, 0.5f, 1f,
            9.81f, 1000f, 10f, 10f, 0);

        Assert.True(drive > 0f);
        Assert.True(brake < 0f);
    }

    [Fact]
    public void SpeedGovernorMatchesSourcePcAutobrakeAndReverseRules()
    {
        var profile = Profile() with
        {
            Engine = new SourceVehicleEngineProfile
            {
                MaxSpeedMilesPerHour = 100f, MaxReverseSpeedMilesPerHour = 50f,
                AutoBrakeSpeedGain = 1.1f, AutoBrakeSpeedFactor = 0.5f
            }
        };
        var overSpeed = SourceVehicleDynamics.ApplySpeedGovernor(profile, 1f, 0.2f,
            SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(120f), false, 2, true);
        Assert.Equal(0f, overSpeed.Throttle);
        Assert.Equal(0.1f, overSpeed.Brake, 5);

        var airborne = SourceVehicleDynamics.ApplySpeedGovernor(profile, 1f, 0.2f,
            SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(120f), false, 0, true);
        Assert.Equal(0f, airborne.Brake);

        var reverse = SourceVehicleDynamics.ApplySpeedGovernor(profile, -1f, 0f,
            SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(60f), false, 2, true);
        Assert.Equal(-0.1f, reverse.Throttle, 5);
    }

    [Fact]
    public void SpeedGovernorMatchesSourceConsoleThrottleReductionAndBoostBranch()
    {
        var profile = Profile() with
        {
            Engine = new SourceVehicleEngineProfile
            {
                MaxSpeedMilesPerHour = 100f, BoostMaxSpeedMilesPerHour = 140f,
                MaxReverseSpeedMilesPerHour = 50f
            }
        };
        var normal = SourceVehicleDynamics.ApplySpeedGovernor(profile, 1f, 0f,
            SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(110f), false, 2, false);
        Assert.Equal(0.1f, normal.Throttle, 5);

        var boost = SourceVehicleDynamics.ApplySpeedGovernor(profile, 1f, 0f,
            SourceVehicleDynamics.MilesPerHourToSourceUnitsPerSecond(110f), true, 2, false);
        Assert.Equal(1f, boost.Throttle);
    }

    [Fact]
    public void WheelContactOutsideTheSourceFifteenDegreeConeHasZeroFriction()
    {
        Assert.Equal(0f, SourceVehicleDynamics.OverrideWheelContactFriction(0.8f,
            new System.Numerics.Vector3(0.3f, 0f, 0f)));
        Assert.Equal(0.8f, SourceVehicleDynamics.OverrideWheelContactFriction(0.8f,
            new System.Numerics.Vector3(0.2f, 0f, 0f)));
    }

    [Fact]
    public void HandbrakeMatchesSourceContactAndOpposingThrottleRules()
    {
        Assert.True(SourceVehicleDynamics.ResolveHandbrake(false, false, -1f, 6f, true));
        Assert.False(SourceVehicleDynamics.ResolveHandbrake(true, false, 0f, 6f, false));
        Assert.True(SourceVehicleDynamics.ResolveHandbrake(true, true, 0f, 0f, true));
    }

    [Fact]
    public void SkidStateSelectsFastestContactAndLocksToSpeedWhenHandbraking()
    {
        var profile = Profile() with
        {
            Steering = new SourceVehicleSteeringProfile { IsSkidAllowed = true }
        };
        var state = SourceVehicleDynamics.CalculateSkidState(profile, 40f, new[]
        {
            new SourceVehicleWheelSkidSample(true, new System.Numerics.Vector3(2f, 0f, 0f), 3),
            new SourceVehicleWheelSkidSample(false, default, 0)
        }, handbrake: true);

        Assert.Equal(40f, state.SkidSpeedSourceUnitsPerSecond);
        Assert.Equal(3, state.SkidSurfaceId);
        Assert.Equal(1, state.WheelsInContact);
        Assert.Equal(1, state.WheelsNotInContact);
    }

    [Fact]
    public void ExtraForcesUseSourceUprightGateAndAngularVelocityCap()
    {
        Assert.Equal(20f, SourceVehicleDynamics.ComputeUprightDownforce(0.04f, 2f, 10f, 1f));
        Assert.Equal(0f, SourceVehicleDynamics.ComputeUprightDownforce(0.05f, 2f, 10f, 1f));
        var capped = SourceVehicleDynamics.ClampAngularVelocity(new System.Numerics.Vector3(0f, 6f, 8f), 5f);
        Assert.Equal(3f, capped.Y, 3);
        Assert.Equal(4f, capped.Z, 3);
    }

    [Fact]
    public void SuspensionUsesSourceCompressionRelaxationAndNormalProjectionRules()
    {
        var result = SourceVehicleDynamics.ComputeSuspensionForce(
            raycastDistance: 0.5f, raycastLength: 1f, springConstant: 10f,
            springDampingRelax: 2f, springDampingCompression: 4f,
            inverseNormalDotDirection: 4f,
            projectedSurfaceVelocity: default, surfaceVelocity: new System.Numerics.Vector3(0f, 1f, 0f),
            raycastDirection: new System.Numerics.Vector3(0f, 1f, 0f), deltaSeconds: 0.1f);

        Assert.Equal(19f, result.Force, 3);
        Assert.Equal(1.9f, result.Impulse, 3);
        Assert.Equal((0f, 0f), SourceVehicleDynamics.ComputeSuspensionForce(
            1f, 1f, 10f, 2f, 4f, 1f, default, default,
            new System.Numerics.Vector3(0f, 1f, 0f), 0.1f));
    }

    [Fact]
    public void VehicleRecordingRoundTripsAndReportsNumericAndDiscreteDivergence()
    {
        var recording = new SourceVehicleRecording();
        recording.Capture(0, new SourceVehicleControl { Throttle = 1f },
            new SourceVehicleOperatingState { SpeedSourceUnitsPerSecond = 10f, Gear = 1 },
            new[] { new SourceVehicleWheelContact(true, Vector3.Zero, Vector3.UnitY, 0.5f, 3, 0.8f) },
            new[] { new SourceVehicleWheelSkidSample(true, Vector3.UnitX, 3) });

        var restored = SourceVehicleRecording.FromJson(recording.ToJson());
        var same = SourceVehicleRecordingComparator.Compare(recording, restored);
        Assert.True(same.Passes(0f), string.Join(",", same.Errors));

        var altered = SourceVehicleRecording.FromJson(recording.ToJson());
        altered.Frames[0] = altered.Frames[0] with
        {
            OperatingState = altered.Frames[0].OperatingState with { SpeedSourceUnitsPerSecond = 11f }
        };
        var comparison = SourceVehicleRecordingComparator.Compare(recording, altered, 0.1f);
        Assert.False(comparison.Passes(0.1f));
        Assert.Contains(comparison.Errors, error => error.Field == "speed");
        Assert.True(comparison.NumericMaximum > 0.9f);
    }

    private static SourceVehicleProfile Profile() => new()
    {
        AxleCount = 1,
        WheelsPerAxle = 2,
        Axles = new[] { new SourceVehicleAxleProfile { Wheels = new SourceVehicleWheelProfile { RadiusSourceUnits = 10f } } }
    };
}
