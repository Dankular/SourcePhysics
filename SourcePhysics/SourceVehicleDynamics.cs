using System.Numerics;

namespace SourcePhysics;

/// Pure Source vehicle-controller equations extracted from physics_vehicle.cpp.
/// The result is engine-independent and can be applied to Jolt wheel/body
/// impulses only after the caller supplies the title's wheel-contact data.
public static class SourceVehicleDynamics
{
    public const float ThrottleOpposingForceEpsilonSourceUnitsPerSecond = 5f;
    public const float PowerslideSpeedThresholdSourceUnitsPerSecond = 18f;
    public const float WheelContactConeSin15Degrees = 0.2588f;
    public const float MilesPerHourToMetersPerSecond = 0.44707f;
    public const float WattsPerHorsepower = 745f;
    public const float SecondsPerMinute = 60f;

    public readonly record struct SpeedGovernorResult(float Throttle, float Brake);
    public readonly record struct PreparedControl(SourceVehicleControl Control, bool Powerslide);

    /// Exact control preprocessing performed by CVehicleController::Update
    /// before steering, engine, handbrake and skid dispatch. Physics-system
    /// integration consumes this result; no Jolt policy is introduced here.
    public static PreparedControl PrepareControl(SourceVehicleControl control,
        float speedSourceUnitsPerSecond, bool isBoosting)
    {
        ArgumentNullException.ThrowIfNull(control);
        if (!float.IsFinite(speedSourceUnitsPerSecond))
            throw new ArgumentOutOfRangeException(nameof(speedSourceUnitsPerSecond));
        if (!float.IsFinite(control.Throttle) || !float.IsFinite(control.Brake) ||
            !float.IsFinite(control.Boost))
            throw new ArgumentOutOfRangeException(nameof(control));

        var absoluteSpeed = MathF.Abs(speedSourceUnitsPerSecond);
        var throttle = control.Throttle;
        var brake = control.Brake;
        var powerslide = control.Handbrake &&
            absoluteSpeed > PowerslideSpeedThresholdSourceUnitsPerSecond;

        if (control.Handbrake)
            throttle = 0f;
        if (isBoosting)
        {
            throttle = throttle < 0f ? -1f : 1f;
            control = control with { Boost = 1f };
        }
        if (throttle == 0f && brake == 0f && !control.Handbrake)
            brake = 0.1f;

        return new(control with { Throttle = throttle, Brake = brake }, powerslide);
    }

    /// Exact CVehicleController::CalcEngine speed-governor branch from
    /// physics_vehicle.cpp. Source has separate PC and console rules, so the
    /// caller must provide the title's platform branch.
    public static SpeedGovernorResult ApplySpeedGovernor(SourceVehicleProfile profile,
        float throttle, float brake, float speedSourceUnitsPerSecond, bool torqueBoost,
        int wheelsInContact, bool isPc)
    {
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        if (!float.IsFinite(throttle) || !float.IsFinite(brake) ||
            !float.IsFinite(speedSourceUnitsPerSecond))
            throw new ArgumentOutOfRangeException(nameof(speedSourceUnitsPerSecond));

        var absoluteSpeed = MathF.Abs(SourceUnitsPerSecondToMilesPerHour(speedSourceUnitsPerSecond));
        if (isPc)
        {
            var maxSpeed = MathF.Max(1f, torqueBoost
                ? profile.Engine.BoostMaxSpeedMilesPerHour
                : profile.Engine.MaxSpeedMilesPerHour);
            if (throttle > 0f && absoluteSpeed > maxSpeed)
            {
                var fraction = absoluteSpeed / maxSpeed;
                if (fraction > profile.Engine.AutoBrakeSpeedGain)
                {
                    throttle = 0f;
                    brake = (fraction - 1f) * profile.Engine.AutoBrakeSpeedFactor;
                    if (wheelsInContact == 0) brake = 0f;
                }
                throttle *= 0.1f;
            }
        }
        else if (throttle > 0f &&
                 ((!torqueBoost && absoluteSpeed > profile.Engine.MaxSpeedMilesPerHour * throttle) ||
                  (torqueBoost && absoluteSpeed > profile.Engine.BoostMaxSpeedMilesPerHour)))
        {
            throttle *= 0.1f;
        }

        if (throttle < 0f && !torqueBoost &&
            absoluteSpeed > profile.Engine.MaxReverseSpeedMilesPerHour)
            throttle *= 0.1f;

        return new(throttle, brake);
    }

    /// Exact physics_vehicle.cpp wheel-contact material override. The normal is
    /// expressed in wheel space, where X is the wheel's lateral axis.
    public static float OverrideWheelContactFriction(float friction, Vector3 wheelSpaceContactNormal)
    {
        if (wheelSpaceContactNormal.LengthSquared() > 0f &&
            MathF.Abs(wheelSpaceContactNormal.X) > WheelContactConeSin15Degrees)
            return 0f;
        return friction;
    }

    /// Exact UpdateHandbrake policy, including the opposing-throttle gravity
    /// escape and the requirement for at least one contacted wheel.
    public static bool ResolveHandbrake(bool requested, bool powerslide, float throttle,
        float speedSourceUnitsPerSecond, bool anyWheelContact)
    {
        var handbrake = requested;
        if (!powerslide &&
            ((throttle < 0f && speedSourceUnitsPerSecond > ThrottleOpposingForceEpsilonSourceUnitsPerSecond) ||
             (throttle > 0f && speedSourceUnitsPerSecond < -ThrottleOpposingForceEpsilonSourceUnitsPerSecond)))
            handbrake = true;
        return handbrake && anyWheelContact;
    }

    /// Exact UpdateSkidding reduction from the wheel contact-point velocities.
    public static SourceVehicleSkidState CalculateSkidState(SourceVehicleProfile profile,
        float speedSourceUnitsPerSecond, IReadOnlyList<SourceVehicleWheelSkidSample> wheels,
        bool handbrake)
    {
        ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(wheels);
        profile.Validate();
        if (wheels.Count != profile.WheelCount)
            throw new ArgumentException("Wheel skid sample count must match the vehicle wheel count.", nameof(wheels));
        if (!profile.Steering.IsSkidAllowed)
            return new SourceVehicleSkidState(0f, 0, profile.WheelCount, 0);

        var skidSpeed = 0f;
        var skidSurface = 0;
        var inContact = 0;
        foreach (var wheel in wheels)
        {
            if (!wheel.InContact) continue;
            inContact++;
            var speed = wheel.ContactPointVelocitySourceUnitsPerSecond.Length();
            if (speed > skidSpeed || skidSpeed <= 0f)
            {
                skidSpeed = speed;
                skidSurface = wheel.SurfaceId;
            }
        }
        if (handbrake && MathF.Abs(speedSourceUnitsPerSecond) > 30f)
            skidSpeed = MathF.Abs(speedSourceUnitsPerSecond);
        return new SourceVehicleSkidState(skidSpeed, skidSurface, inContact, wheels.Count - inContact);
    }

    /// Source UpdateExtraForces: tilt downforce is active only while the body
    /// cache's world-up matrix element is within the upright threshold.
    public static float ComputeUprightDownforce(float bodyUpY, float tiltForce,
        float gravityLength, float bodyMass) =>
        MathF.Abs(bodyUpY) < 0.05f ? tiltForce * gravityLength * bodyMass : 0f;

    /// Source's post-controller angular velocity limit, preserving direction.
    public static Vector3 ClampAngularVelocity(Vector3 angularVelocity,
        float maximumAngularVelocityRadiansPerSecond)
    {
        if (maximumAngularVelocityRadiansPerSecond > 0f &&
            angularVelocity.Length() > maximumAngularVelocityRadiansPerSecond)
            return Vector3.Normalize(angularVelocity) * maximumAngularVelocityRadiansPerSecond;
        return angularVelocity;
    }

    /// CPhysics_Airboat::DoSimulationPontoonsGround, expressed as a pure
    /// force/impulse calculation. Distances and spring constants use the
    /// units authored by the Source vehicle controller; the caller supplies
    /// the already-projected surface and ray directions from the trace.
    public static (float Force, float Impulse) ComputeSuspensionForce(
        float raycastDistance, float raycastLength, float springConstant,
        float springDampingRelax, float springDampingCompression,
        float inverseNormalDotDirection, Vector3 projectedSurfaceVelocity,
        Vector3 surfaceVelocity, Vector3 raycastDirection, float deltaSeconds)
    {
        if (!float.IsFinite(deltaSeconds) || deltaSeconds < 0f)
            throw new ArgumentOutOfRangeException(nameof(deltaSeconds));
        var difference = raycastDistance - raycastLength;
        if (difference >= 0f) return (0f, 0f);
        var force = -difference * springConstant * Math.Clamp(inverseNormalDotDirection, 0f, 3f);
        var speed = Vector3.Dot(projectedSurfaceVelocity - surfaceVelocity, raycastDirection);
        force -= speed > 0f ? springDampingRelax * speed : springDampingCompression * speed;
        force = MathF.Max(0f, force);
        return (force, force * deltaSeconds);
    }

    public static float SourceUnitsPerSecondToMilesPerHour(float sourceUnitsPerSecond) =>
        SourceUnits.ToMeters(sourceUnitsPerSecond) / MilesPerHourToMetersPerSecond;

    public static float MilesPerHourToSourceUnitsPerSecond(float milesPerHour) =>
        SourceUnits.ToSource(milesPerHour * MilesPerHourToMetersPerSecond);

    public static float CalculateSteeringAngleDegrees(SourceVehicleProfile profile,
        float speedSourceUnitsPerSecond, float steering, bool analog)
    {
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        var speedMph = MathF.Abs(SourceUnitsPerSecondToMilesPerHour(speedSourceUnitsPerSecond));
        var degrees = RemapClamped(speedMph, profile.Steering.SpeedSlowMilesPerHour,
            profile.Steering.SpeedFastMilesPerHour, profile.Steering.DegreesSlow, profile.Steering.DegreesFast);
        var speedGame = MilesPerHourToSourceUnitsPerSecond(speedMph);
        if (speedGame > MilesPerHourToSourceUnitsPerSecond(profile.Engine.MaxSpeedMilesPerHour))
        {
            degrees = RemapClamped(speedGame,
                MilesPerHourToSourceUnitsPerSecond(profile.Engine.MaxSpeedMilesPerHour),
                MilesPerHourToSourceUnitsPerSecond(profile.Engine.BoostMaxSpeedMilesPerHour),
                profile.Steering.DegreesFast, profile.Steering.DegreesBoost);
        }
        if (profile.Steering.SteeringExponent == 0f) return steering * degrees;
        var sign = steering < 0f ? -1f : 1f;
        var absolute = MathF.Abs(steering);
        var output = analog
            ? MathF.Pow(absolute, 2f) * sign * profile.Steering.DegreesSlow
            : MathF.Pow(absolute, profile.Steering.SteeringExponent) * sign * degrees;
        return Math.Clamp(output, -degrees, degrees);
    }

    public static (int Gear, float EngineRpm) CalculateEngineTransmission(
        SourceVehicleProfile profile, IReadOnlyList<float> wheelAngularVelocitiesRadiansPerSecond,
        int currentGear, float throttle)
    {
        ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(wheelAngularVelocitiesRadiansPerSecond);
        profile.Validate();
        if (wheelAngularVelocitiesRadiansPerSecond.Count != profile.WheelCount)
            throw new ArgumentException("Wheel angular velocity count must match the vehicle wheel count.", nameof(wheelAngularVelocitiesRadiansPerSecond));
        if (profile.Engine.GearRatios.Count == 0) return (currentGear, 0f);
        var gear = Math.Clamp(currentGear, 0, profile.Engine.GearRatios.Count - 1);
        var average = wheelAngularVelocitiesRadiansPerSecond.Sum(MathF.Abs) *
                      (0.5f / MathF.PI / wheelAngularVelocitiesRadiansPerSecond.Count);
        var rpm = EstimateRpm(profile, average, gear);
        if (profile.Engine.IsAutomaticTransmission)
        {
            if (throttle > 0f)
            {
                while (rpm > profile.Engine.ShiftUpRpm && gear < profile.Engine.GearRatios.Count - 1)
                {
                    gear++;
                    rpm = EstimateRpm(profile, average, gear);
                }
            }
            while (rpm < profile.Engine.ShiftDownRpm && gear > 0)
            {
                gear--;
                rpm = EstimateRpm(profile, average, gear);
            }
        }
        return (gear, rpm);
    }

    public static float ComputeDriveTorqueNewtonMeters(SourceVehicleProfile profile,
        int gear, float throttle, float wheelRadiusSourceUnits, float engineRpm,
        float vehicleSpeedSourceUnitsPerSecond, float steering, bool torqueBoost, bool boosting,
        int axleIndex)
    {
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        if (axleIndex < 0 || axleIndex >= profile.Axles.Count) throw new ArgumentOutOfRangeException(nameof(axleIndex));
        if (!float.IsFinite(wheelRadiusSourceUnits) || wheelRadiusSourceUnits <= 0f)
            throw new ArgumentOutOfRangeException(nameof(wheelRadiusSourceUnits));
        gear = Math.Clamp(gear, 0, Math.Max(0, profile.Engine.GearRatios.Count - 1));
        if (profile.Engine.GearRatios.Count == 0 || engineRpm >= profile.Engine.MaxRpm) return 0f;
        var radiusMeters = SourceUnits.ToMeters(wheelRadiusSourceUnits);
        var wheelForce = throttle * profile.Engine.Horsepower * (WattsPerHorsepower * SecondsPerMinute) *
            profile.Engine.GearRatios[gear] * profile.Engine.AxleRatio /
            (profile.Engine.MaxRpm * radiusMeters * (2f * MathF.PI));
        var boostFactor = 0.5f;
        if (torqueBoost && boosting)
        {
            var speed = MathF.Abs(vehicleSpeedSourceUnitsPerSecond);
            var maxSpeed = MilesPerHourToSourceUnitsPerSecond(profile.Engine.MaxSpeedMilesPerHour);
            var speedFactor = RemapClamped(speed, 0f, maxSpeed, 0.1f, 1f);
            var turnFactor = 1f - MathF.Abs(steering) * 0.95f;
            boostFactor = MathF.Max(boostFactor, profile.Engine.BoostForce * speedFactor * turnFactor);
        }
        var torqueScale = profile.Axles.Sum(axle => axle.TorqueFactor);
        if (torqueScale > 0f) torqueScale = 1f / torqueScale;
        var axleTorque = boostFactor * wheelForce * profile.Axles[axleIndex].TorqueFactor * torqueScale * radiusMeters;
        return axleTorque;
    }

    public static float ComputeBrakeTorqueNewtonMeters(SourceVehicleProfile profile,
        float brake, float signedSpeedSourceUnitsPerSecond, float gravityMetersPerSecondSquared,
        float bodyMassKg, float totalWheelMassKg, float wheelRadiusSourceUnits, int axleIndex)
    {
        ArgumentNullException.ThrowIfNull(profile);
        profile.Validate();
        if (axleIndex < 0 || axleIndex >= profile.Axles.Count) throw new ArgumentOutOfRangeException(nameof(axleIndex));
        var sign = signedSpeedSourceUnitsPerSecond >= 0f ? -1f : 1f;
        var radiusMeters = SourceUnits.ToMeters(wheelRadiusSourceUnits);
        return 0.5f * sign * gravityMetersPerSecondSquared * (bodyMassKg + totalWheelMassKg) *
            brake * profile.Axles[axleIndex].BrakeFactor * radiusMeters;
    }

    private static float EstimateRpm(SourceVehicleProfile profile, float averageWheelRevolutionsPerSecond, int gear) =>
        averageWheelRevolutionsPerSecond * profile.Engine.AxleRatio * profile.Engine.GearRatios[gear] * SecondsPerMinute;

    private static float RemapClamped(float value, float fromMin, float fromMax, float toMin, float toMax)
    {
        if (fromMax == fromMin) return value >= fromMax ? toMax : toMin;
        var t = Math.Clamp((value - fromMin) / (fromMax - fromMin), 0f, 1f);
        return toMin + (toMax - toMin) * t;
    }
}
