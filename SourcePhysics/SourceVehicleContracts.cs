using System.Numerics;

namespace SourcePhysics;

public enum SourceVehicleType
{
    CarWheels = 1 << 0,
    CarRaycast = 1 << 1,
    JetskiRaycast = 1 << 2,
    AirboatRaycast = 1 << 3
}

public enum SourceVehicleTireType
{
    Normal,
    Braking,
    Powerslide
}

public sealed record SourceVehicleControl
{
    public float Throttle { get; init; }
    public float Steering { get; init; }
    public float Brake { get; init; }
    public float Boost { get; init; }
    public bool Handbrake { get; init; }
    public bool HandbrakeLeft { get; init; }
    public bool HandbrakeRight { get; init; }
    public bool BrakePedal { get; init; }
    public bool HasBrakePedal { get; init; }
    public bool AnalogSteering { get; init; }
}

public sealed record SourceVehicleOperatingState
{
    public float SpeedSourceUnitsPerSecond { get; init; }
    public float EngineRpm { get; init; }
    public int Gear { get; init; }
    public float BoostDelaySeconds { get; init; }
    public int BoostTimeLeftMilliseconds { get; init; }
    public float SkidSpeedSourceUnitsPerSecond { get; init; }
    public int SkidSurfaceId { get; init; }
    public float SteeringAngleDegrees { get; init; }
    public int WheelsNotInContact { get; init; }
    public int WheelsInContact { get; init; }
    public bool IsTorqueBoosting { get; init; }
}

public sealed record SourceVehicleBodyProfile
{
    public Vector3 MassCenterOverrideSourceUnits { get; init; }
    public float MassOverrideKilograms { get; init; }
    public float AddGravitySourceUnitsPerSecondSquared { get; init; }
    public float TiltForce { get; init; }
    public float TiltForceHeightSourceUnits { get; init; }
    public float CounterTorqueFactor { get; init; }
    public float KeepUprightTorque { get; init; }
    public float MaxAngularVelocityRadiansPerSecond { get; init; }
}

public sealed record SourceVehicleWheelProfile
{
    public float RadiusSourceUnits { get; init; }
    public float MassKilograms { get; init; }
    public float Inertia { get; init; }
    public float Damping { get; init; }
    public float RotationalDamping { get; init; }
    public float FrictionScale { get; init; }
    public int MaterialId { get; init; }
    public int BrakeMaterialId { get; init; } = -1;
    public int SkidMaterialId { get; init; } = -1;
    public float SpringAdditionalLengthSourceUnits { get; init; }
}

public sealed record SourceVehicleSuspensionProfile
{
    public float SpringConstant { get; init; }
    public float SpringDamping { get; init; }
    public float StabilizerConstant { get; init; }
    public float SpringDampingCompression { get; init; }
    public float MaxBodyForce { get; init; }
}

public sealed record SourceVehicleAxleProfile
{
    public Vector3 OffsetSourceUnits { get; init; }
    public Vector3 WheelOffsetSourceUnits { get; init; }
    public Vector3 RaytraceCenterOffsetSourceUnits { get; init; }
    public Vector3 RaytraceOffsetSourceUnits { get; init; }
    public SourceVehicleWheelProfile Wheels { get; init; } = new();
    public SourceVehicleSuspensionProfile Suspension { get; init; } = new();
    public float TorqueFactor { get; init; }
    public float BrakeFactor { get; init; }
}

public sealed record SourceVehicleSteeringProfile
{
    public float DegreesSlow { get; init; }
    public float DegreesFast { get; init; }
    public float DegreesBoost { get; init; }
    public float SteeringRateSlow { get; init; }
    public float SteeringRateFast { get; init; }
    public float SteeringRestRateSlow { get; init; }
    public float SteeringRestRateFast { get; init; }
    /// Source vehicle files author these thresholds in miles per hour.
    public float SpeedSlowMilesPerHour { get; init; }
    public float SpeedFastMilesPerHour { get; init; }
    [Obsolete("Source vehicle steering speed thresholds are authored in MPH; use SpeedSlowMilesPerHour.")]
    public float SpeedSlowSourceUnitsPerSecond { get => SpeedSlowMilesPerHour; init => SpeedSlowMilesPerHour = value; }
    [Obsolete("Source vehicle steering speed thresholds are authored in MPH; use SpeedFastMilesPerHour.")]
    public float SpeedFastSourceUnitsPerSecond { get => SpeedFastMilesPerHour; init => SpeedFastMilesPerHour = value; }
    public float TurnThrottleReduceSlow { get; init; }
    public float TurnThrottleReduceFast { get; init; }
    public float BrakeSteeringRateFactor { get; init; }
    public float ThrottleSteeringRestRateFactor { get; init; }
    public float PowerslideAcceleration { get; init; }
    public float BoostSteeringRestRateFactor { get; init; }
    public float BoostSteeringRateFactor { get; init; }
    public float SteeringExponent { get; init; }
    public bool IsSkidAllowed { get; init; }
    public bool DustCloud { get; init; }
}

public sealed record SourceVehicleEngineProfile
{
    public float Horsepower { get; init; }
    /// Source vehicle files author these limits in miles per hour.
    public float MaxSpeedMilesPerHour { get; init; }
    public float MaxReverseSpeedMilesPerHour { get; init; }
    [Obsolete("Source vehicle speed limits are authored in MPH; use MaxSpeedMilesPerHour.")]
    public float MaxSpeedSourceUnitsPerSecond { get => MaxSpeedMilesPerHour; init => MaxSpeedMilesPerHour = value; }
    [Obsolete("Source vehicle speed limits are authored in MPH; use MaxReverseSpeedMilesPerHour.")]
    public float MaxReverseSpeedSourceUnitsPerSecond { get => MaxReverseSpeedMilesPerHour; init => MaxReverseSpeedMilesPerHour = value; }
    public float MaxRpm { get; init; }
    public float AxleRatio { get; init; }
    public float ThrottleTimeSeconds { get; init; }
    public IReadOnlyList<float> GearRatios { get; init; } = Array.Empty<float>();
    public float ShiftUpRpm { get; init; }
    public float ShiftDownRpm { get; init; }
    public float BoostForce { get; init; }
    public float BoostDurationSeconds { get; init; }
    public float BoostDelaySeconds { get; init; }
    public float BoostMaxSpeedMilesPerHour { get; init; }
    [Obsolete("Source vehicle boost speed limits are authored in MPH; use BoostMaxSpeedMilesPerHour.")]
    public float BoostMaxSpeedSourceUnitsPerSecond { get => BoostMaxSpeedMilesPerHour; init => BoostMaxSpeedMilesPerHour = value; }
    public float AutoBrakeSpeedGain { get; init; }
    public float AutoBrakeSpeedFactor { get; init; }
    public bool TorqueBoost { get; init; }
    public bool IsAutomaticTransmission { get; init; }
}

public sealed record SourceVehicleProfile
{
    public SourceVehicleType Type { get; init; } = SourceVehicleType.CarRaycast;
    public int AxleCount { get; init; }
    public int WheelsPerAxle { get; init; }
    public SourceVehicleBodyProfile Body { get; init; } = new();
    public IReadOnlyList<SourceVehicleAxleProfile> Axles { get; init; } = Array.Empty<SourceVehicleAxleProfile>();
    public SourceVehicleEngineProfile Engine { get; init; } = new();
    public SourceVehicleSteeringProfile Steering { get; init; } = new();

    public int WheelCount => checked(AxleCount * WheelsPerAxle);

    public void Validate()
    {
        if (!Enum.IsDefined(Type)) throw new InvalidDataException("Vehicle type is not defined by the Source contract.");
        if (AxleCount is < 1 or > 4) throw new InvalidDataException("Vehicle axle count must be 1..4.");
        if (WheelsPerAxle is < 1 or > 2) throw new InvalidDataException("Vehicle wheels per axle must be 1..2.");
        if (Axles.Count != AxleCount) throw new InvalidDataException("Vehicle axle data does not match axle count.");
        if (WheelCount > 8) throw new InvalidDataException("Vehicle wheel count exceeds the Source contract.");
        if (Engine.GearRatios.Count > 6) throw new InvalidDataException("Vehicle gear count exceeds the Source contract.");
        if (Axles.Any(axle => axle.Wheels.RadiusSourceUnits <= 0f || !float.IsFinite(axle.Wheels.RadiusSourceUnits)))
            throw new InvalidDataException("Every vehicle wheel must have a finite positive radius.");
        if (Axles.Any(axle => !float.IsFinite(axle.TorqueFactor) || !float.IsFinite(axle.BrakeFactor)))
            throw new InvalidDataException("Vehicle axle factors must be finite.");
        if (Engine.GearRatios.Any(ratio => !float.IsFinite(ratio)))
            throw new InvalidDataException("Vehicle gear ratios must be finite.");

        var bodyScalars = new[]
        {
            Body.MassOverrideKilograms, Body.AddGravitySourceUnitsPerSecondSquared,
            Body.TiltForce, Body.TiltForceHeightSourceUnits, Body.CounterTorqueFactor,
            Body.KeepUprightTorque, Body.MaxAngularVelocityRadiansPerSecond
        };
        if (bodyScalars.Any(value => !float.IsFinite(value)))
            throw new InvalidDataException("Vehicle body values must be finite.");
        RequireNonNegative(Body.MassOverrideKilograms, "vehicle mass override");
        RequireNonNegative(Body.MaxAngularVelocityRadiansPerSecond, "vehicle maximum angular velocity");
        ValidateVector(Body.MassCenterOverrideSourceUnits, "vehicle mass-center override");

        var engineScalars = new[]
        {
            Engine.Horsepower, Engine.MaxSpeedMilesPerHour, Engine.MaxReverseSpeedMilesPerHour,
            Engine.MaxRpm, Engine.AxleRatio, Engine.ThrottleTimeSeconds, Engine.ShiftUpRpm,
            Engine.ShiftDownRpm, Engine.BoostForce, Engine.BoostDurationSeconds, Engine.BoostDelaySeconds,
            Engine.BoostMaxSpeedMilesPerHour, Engine.AutoBrakeSpeedGain, Engine.AutoBrakeSpeedFactor
        };
        if (engineScalars.Any(value => !float.IsFinite(value)))
            throw new InvalidDataException("Vehicle engine values must be finite.");
        RequireNonNegative(Engine.Horsepower, "vehicle horsepower");
        RequireNonNegative(Engine.MaxSpeedMilesPerHour, "vehicle maximum speed");
        RequireNonNegative(Engine.MaxReverseSpeedMilesPerHour, "vehicle maximum reverse speed");
        RequireNonNegative(Engine.MaxRpm, "vehicle maximum RPM");
        RequireNonNegative(Engine.AxleRatio, "vehicle axle ratio");
        RequireNonNegative(Engine.ThrottleTimeSeconds, "vehicle throttle time");
        RequireNonNegative(Engine.ShiftUpRpm, "vehicle shift-up RPM");
        RequireNonNegative(Engine.ShiftDownRpm, "vehicle shift-down RPM");
        RequireNonNegative(Engine.BoostForce, "vehicle boost force");
        RequireNonNegative(Engine.BoostDurationSeconds, "vehicle boost duration");
        RequireNonNegative(Engine.BoostDelaySeconds, "vehicle boost delay");
        RequireNonNegative(Engine.BoostMaxSpeedMilesPerHour, "vehicle boost maximum speed");
        RequireNonNegative(Engine.AutoBrakeSpeedGain, "vehicle auto-brake speed gain");
        RequireNonNegative(Engine.AutoBrakeSpeedFactor, "vehicle auto-brake speed factor");

        var steeringScalars = new[]
        {
            Steering.DegreesSlow, Steering.DegreesFast, Steering.DegreesBoost,
            Steering.SteeringRateSlow, Steering.SteeringRateFast, Steering.SteeringRestRateSlow,
            Steering.SteeringRestRateFast, Steering.SpeedSlowMilesPerHour,
            Steering.SpeedFastMilesPerHour, Steering.TurnThrottleReduceSlow,
            Steering.TurnThrottleReduceFast, Steering.BrakeSteeringRateFactor,
            Steering.ThrottleSteeringRestRateFactor, Steering.PowerslideAcceleration,
            Steering.BoostSteeringRestRateFactor, Steering.BoostSteeringRateFactor,
            Steering.SteeringExponent
        };
        if (steeringScalars.Any(value => !float.IsFinite(value)))
            throw new InvalidDataException("Vehicle steering values must be finite.");
        foreach (var value in steeringScalars)
            RequireNonNegative(value, "vehicle steering parameter");

        foreach (var axle in Axles)
        {
            ValidateVector(axle.OffsetSourceUnits, "vehicle axle offset");
            ValidateVector(axle.WheelOffsetSourceUnits, "vehicle wheel offset");
            ValidateVector(axle.RaytraceCenterOffsetSourceUnits, "vehicle raytrace center offset");
            ValidateVector(axle.RaytraceOffsetSourceUnits, "vehicle raytrace offset");
            var wheel = axle.Wheels;
            var wheelScalars = new[]
            {
                wheel.RadiusSourceUnits, wheel.MassKilograms, wheel.Inertia, wheel.Damping,
                wheel.RotationalDamping, wheel.FrictionScale, wheel.SpringAdditionalLengthSourceUnits
            };
            if (wheelScalars.Any(value => !float.IsFinite(value)))
                throw new InvalidDataException("Vehicle wheel values must be finite.");
            RequireNonNegative(wheel.MassKilograms, "vehicle wheel mass");
            RequireNonNegative(wheel.Inertia, "vehicle wheel inertia");
            RequireNonNegative(wheel.Damping, "vehicle wheel damping");
            RequireNonNegative(wheel.RotationalDamping, "vehicle wheel rotational damping");
            RequireNonNegative(wheel.FrictionScale, "vehicle wheel friction scale");
            RequireNonNegative(wheel.SpringAdditionalLengthSourceUnits, "vehicle spring additional length");
            var suspensionScalars = new[]
            {
                axle.Suspension.SpringConstant, axle.Suspension.SpringDamping,
                axle.Suspension.StabilizerConstant, axle.Suspension.SpringDampingCompression,
                axle.Suspension.MaxBodyForce, axle.TorqueFactor, axle.BrakeFactor
            };
            if (suspensionScalars.Any(value => !float.IsFinite(value)))
                throw new InvalidDataException("Vehicle suspension and axle values must be finite.");
            RequireNonNegative(axle.Suspension.SpringConstant, "vehicle spring constant");
            RequireNonNegative(axle.Suspension.SpringDamping, "vehicle spring damping");
            RequireNonNegative(axle.Suspension.StabilizerConstant, "vehicle stabilizer constant");
            RequireNonNegative(axle.Suspension.SpringDampingCompression, "vehicle compression damping");
            RequireNonNegative(axle.Suspension.MaxBodyForce, "vehicle maximum body force");
            RequireNonNegative(axle.TorqueFactor, "vehicle torque factor");
            RequireNonNegative(axle.BrakeFactor, "vehicle brake factor");
        }
    }

    private static void RequireNonNegative(float value, string name)
    {
        if (value < 0f) throw new InvalidDataException($"{name} cannot be negative.");
    }

    private static void ValidateVector(Vector3 value, string name)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new InvalidDataException($"{name} must be finite.");
    }
}

public readonly record struct SourceVehicleWheelContact(
    bool InContact,
    Vector3 ContactPointMeters,
    Vector3 ContactNormal,
    float SuspensionLengthMeters,
    int SurfaceId,
    float SurfaceFriction,
    /// Jolt body identity for moving-platform velocity and exact contact routing.
    int BodyId = -1,
    /// Contacted-body point velocity in Jolt metres per second.
    Vector3 SurfaceVelocityMetersPerSecond = default);

public readonly record struct SourceVehicleWheelSkidSample(
    bool InContact,
    Vector3 ContactPointVelocitySourceUnitsPerSecond,
    int SurfaceId);

public readonly record struct SourceVehicleSkidState(
    float SkidSpeedSourceUnitsPerSecond,
    int SkidSurfaceId,
    int WheelsInContact,
    int WheelsNotInContact);
