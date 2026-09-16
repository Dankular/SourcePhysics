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
    public const float AirboatBuoyancyScalar = 1.6f;
    public const float AirboatPontoonAreaSquareMeters = 2.8f;
    public const float AirboatPontoonHeightSourceUnits = 0.41f;
    public const float AirboatPontoonCount = 4f;
    public const float AirboatWaterDragLeftRight = 0.6f;
    public const float AirboatWaterDragForwardBack = 0.005f;
    public const float AirboatWaterDragUpDown = 0.0025f;
    public const float AirboatGroundDragLeftRight = 2f;
    public const float AirboatGroundDragForwardBack = 1f;
    public const float AirboatGroundDragUpDown = 0.8f;
    public const float AirboatDryFrictionScale = 0.6f;
    public const float AirboatGravity = 9.81f;
    public const float AirboatSteeringRateMin = 0.00045f;
    public const float AirboatSteeringRateMax = 0.00225f;
    public const float AirboatSteeringInterval = 0.5f;
    public const float AirboatRotationalDrag = 0.00004f;
    public const float AirboatRotationalDamping = 0.001f;
    public const float AirboatUprightReferenceAngleRadians = 0.17453292f;
    public const float MilesPerHourToMetersPerSecond = 0.44707f;
    public const float WattsPerHorsepower = 745f;
    public const float SecondsPerMinute = 60f;

    public readonly record struct SpeedGovernorResult(float Throttle, float Brake);
    public readonly record struct PreparedControl(SourceVehicleControl Control, bool Powerslide);
    public readonly record struct PowerslideResult(SourceVehicleTireType TireType,
        float FrontAccelerationSourceUnitsPerSecondSquared,
        float RearAccelerationSourceUnitsPerSecondSquared);
    public readonly record struct AirboatSteeringResult(Vector3 RotationalImpulse,
        bool SteeringReversed, float SteerTime, float PreviousSteeringAngle);
    public readonly record struct AirboatUprightResult(Vector3 AngularImpulse, float Error);

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

    /// Exact CVehicleController::UpdatePowerslide result. The caller applies
    /// the returned front/rear accelerations to its Jolt body and resolves the
    /// selected tire material per wheel.
    public static PowerslideResult ResolvePowerslide(SourceVehicleProfile profile,
        SourceVehicleControl control, bool powerslide, float speedSourceUnitsPerSecond,
        bool occupied)
    {
        ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(control);
        profile.Validate();
        if (!float.IsFinite(speedSourceUnitsPerSecond))
            throw new ArgumentOutOfRangeException(nameof(speedSourceUnitsPerSecond));
        if (!profile.Steering.IsSkidAllowed || !occupied)
            return new(SourceVehicleTireType.Normal, 0f, 0f);

        var left = powerslide && control.HandbrakeLeft;
        var right = powerslide && control.HandbrakeRight;
        var tireType = left || right
            ? SourceVehicleTireType.Powerslide
            : powerslide ? SourceVehicleTireType.Braking : SourceVehicleTireType.Normal;
        if (speedSourceUnitsPerSecond <= 0f || left == right)
            return new(tireType, 0f, 0f);

        var powerSlide = RemapClamped(SourceUnitsPerSecondToMilesPerHour(speedSourceUnitsPerSecond),
            profile.Steering.SpeedSlowMilesPerHour,
            profile.Steering.SpeedFastMilesPerHour, 0f, 1f);
        var acceleration = profile.Steering.PowerslideAcceleration * powerSlide;
        return left
            ? new(tireType, acceleration, -acceleration)
            : new(tireType, -acceleration, acceleration);
    }

    public static int ResolveWheelMaterialIndex(SourceVehicleWheelProfile wheel,
        SourceVehicleTireType tireType)
    {
        ArgumentNullException.ThrowIfNull(wheel);
        return tireType switch
        {
            SourceVehicleTireType.Powerslide when wheel.SkidMaterialId != -1 => wheel.SkidMaterialId,
            SourceVehicleTireType.Braking when wheel.BrakeMaterialId != -1 => wheel.BrakeMaterialId,
            _ => wheel.MaterialId
        };
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

    /// Exact InitCarSystemBody extra-gravity force setup from
    /// physics_vehicle.cpp: addGravity is multiplied by the environment
    /// gravity magnitude and the vehicle body mass before being handed to
    /// the IVP car system.
    public static float ComputeExtraGravityForce(float addGravity,
        float gravityLengthMetersPerSecondSquared, float bodyMassKg)
    {
        if (!float.IsFinite(addGravity) || addGravity < 0f)
            throw new ArgumentOutOfRangeException(nameof(addGravity));
        if (!float.IsFinite(gravityLengthMetersPerSecondSquared) || gravityLengthMetersPerSecondSquared < 0f)
            throw new ArgumentOutOfRangeException(nameof(gravityLengthMetersPerSecondSquared));
        if (!float.IsFinite(bodyMassKg) || bodyMassKg < 0f)
            throw new ArgumentOutOfRangeException(nameof(bodyMassKg));
        return addGravity * gravityLengthMetersPerSecondSquared * bodyMassKg;
    }

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

    /// Literal CPhysics_Airboat::DoSimulationPontoonsWater law from
    /// physics_airboat.cpp. Source obtains flDepth from a 1000-unit upward
    /// water trace, clamps it to PONTOON_HEIGHT (0.41), converts the clamped
    /// value with 0.0254, and distributes the buoyancy over four pontoons.
    /// The caller applies the returned impulse along the authored world-up
    /// direction at the pontoon impact point.
    public static (float Force, float Impulse) ComputeAirboatPontoonBuoyancy(
        float depthSourceUnits, float bodyMassKg, float deltaSeconds)
    {
        if (!float.IsFinite(depthSourceUnits) || depthSourceUnits < 0f)
            throw new ArgumentOutOfRangeException(nameof(depthSourceUnits));
        if (!float.IsFinite(bodyMassKg) || bodyMassKg < 0f)
            throw new ArgumentOutOfRangeException(nameof(bodyMassKg));
        if (!float.IsFinite(deltaSeconds) || deltaSeconds < 0f)
            throw new ArgumentOutOfRangeException(nameof(deltaSeconds));

        var depth = Math.Clamp(depthSourceUnits, 0f, AirboatPontoonHeightSourceUnits);
        var submergedVolume = AirboatPontoonAreaSquareMeters * depth * 0.0254f;
        var force = AirboatBuoyancyScalar * (1f / AirboatPontoonCount) * bodyMassKg *
            submergedVolume * 1000f;
        return (force, force * deltaSeconds);
    }

    /// Exact local-space water drag impulse from CPhysics_Airboat::DoSimulationDrag.
    /// The source code intentionally leaves the averaged water dampening unused;
    /// this method preserves that behavior.
    public static Vector3 ComputeAirboatWaterDragImpulse(Vector3 localVelocity,
        float speedMetersPerSecond, float bodyMassKg, float deltaSeconds)
    {
        ValidateAirboatDragInputs(localVelocity, speedMetersPerSecond, bodyMassKg, deltaSeconds);
        var negative = -localVelocity;
        var directionalDrag = new Vector3(
            AirboatWaterDragLeftRight * negative.X,
            AirboatWaterDragUpDown * negative.Y,
            AirboatWaterDragForwardBack * negative.Z);
        return directionalDrag * (speedMetersPerSecond * bodyMassKg * deltaSeconds);
    }

    /// Exact local-space ground friction drag impulse from
    /// CPhysics_Airboat::DoSimulationDrag. Source divides by speed before
    /// applying the directional ground-drag coefficients.
    public static Vector3 ComputeAirboatGroundDragImpulse(Vector3 localVelocity,
        float speedMetersPerSecond, float bodyMassKg, float averageGroundFriction,
        float deltaSeconds)
    {
        ValidateAirboatDragInputs(localVelocity, speedMetersPerSecond, bodyMassKg, deltaSeconds);
        if (!float.IsFinite(averageGroundFriction) || averageGroundFriction < 0f)
            throw new ArgumentOutOfRangeException(nameof(averageGroundFriction));
        if (speedMetersPerSecond <= 0f) return Vector3.Zero;
        var frictionDrag = bodyMassKg * AirboatGravity * AirboatDryFrictionScale * averageGroundFriction /
            speedMetersPerSecond;
        var negative = -localVelocity;
        var directionalDrag = new Vector3(
            AirboatGroundDragLeftRight * negative.X,
            AirboatGroundDragUpDown * negative.Y,
            AirboatGroundDragForwardBack * negative.Z);
        return directionalDrag * (frictionDrag * deltaSeconds);
    }

    /// Exact CPhysics_Airboat::DoSimulationTurbine impulse law. The supplied
    /// forward vector is the world-space core Z column; it is intentionally
    /// not normalized or reconstructed by this contract.
    public static Vector3 ComputeAirboatThrustImpulse(Vector3 forwardWorld,
        float thrust, bool weakJump, bool airborne, float bodyMassKg, float deltaSeconds)
    {
        if (!float.IsFinite(forwardWorld.X) || !float.IsFinite(forwardWorld.Y) ||
            !float.IsFinite(forwardWorld.Z))
            throw new ArgumentOutOfRangeException(nameof(forwardWorld));
        if (!float.IsFinite(thrust)) throw new ArgumentOutOfRangeException(nameof(thrust));
        if (!float.IsFinite(bodyMassKg) || bodyMassKg < 0f)
            throw new ArgumentOutOfRangeException(nameof(bodyMassKg));
        if (!float.IsFinite(deltaSeconds) || deltaSeconds < 0f)
            throw new ArgumentOutOfRangeException(nameof(deltaSeconds));

        var effectiveThrust = thrust;
        if (weakJump || (airborne && effectiveThrust < 0f)) effectiveThrust *= 0.5f;
        if (forwardWorld.Y < -0.5f && effectiveThrust > 0f)
            effectiveThrust *= 1f + forwardWorld.Y;
        else if (forwardWorld.Y > 0.5f && effectiveThrust < 0f)
            effectiveThrust *= 1f - forwardWorld.Y;
        return forwardWorld * (effectiveThrust * bodyMassKg * deltaSeconds);
    }

    /// Exact CPhysics_Airboat::DoSimulationSteering state and rotational
    /// impulse update for the non-X360 constants used by the supplied Source
    /// reference. The result is expressed in the core's local coordinates.
    public static AirboatSteeringResult ComputeAirboatSteering(
        float steeringAngle, float thrust, float localForwardVelocity,
        bool analogSteering, bool steeringReversed, float previousSteeringAngle,
        float steerTime, float rotationalSpeedY, float bodyMassKg, float deltaSeconds)
    {
        if (!float.IsFinite(steeringAngle) || !float.IsFinite(thrust) ||
            !float.IsFinite(localForwardVelocity) || !float.IsFinite(previousSteeringAngle) ||
            !float.IsFinite(steerTime) || !float.IsFinite(rotationalSpeedY) ||
            !float.IsFinite(bodyMassKg) || !float.IsFinite(deltaSeconds))
            throw new ArgumentOutOfRangeException(nameof(steeringAngle));
        if (steerTime < 0f || bodyMassKg < 0f || deltaSeconds < 0f)
            throw new ArgumentOutOfRangeException(nameof(steerTime));

        if (steeringAngle == 0f || thrust != 0f)
        {
            if (!analogSteering)
            {
                if (thrust < 0f) steeringReversed = true;
                else if (thrust > 0f || localForwardVelocity > 0f) steeringReversed = false;
            }
            else
            {
                if (thrust < -2f) steeringReversed = true;
                else if (thrust > 2f || localForwardVelocity > 0f) steeringReversed = false;
            }
        }

        var steeringForce = 0f;
        if (MathF.Abs(steeringAngle) > 0.01f)
        {
            var steeringSign = steeringAngle < 0f ? -1f : 1f;
            if (steeringReversed) steeringSign *= -1f;
            var previousSign = previousSteeringAngle < 0f ? -1f : 1f;
            if (MathF.Abs(previousSteeringAngle) < 0.01f || steeringSign != previousSign)
                steerTime = 0f;

            var steerScale = analogSteering
                ? RemapClamped(MathF.Abs(steeringAngle), 0f, AirboatSteeringInterval,
                    AirboatSteeringRateMin, AirboatSteeringRateMax)
                : RemapClamped(steerTime, 0f, AirboatSteeringInterval,
                    AirboatSteeringRateMin, AirboatSteeringRateMax);
            steeringForce = steerScale * bodyMassKg * deltaSeconds * -steeringSign;
            steerTime += deltaSeconds;
        }

        var rotationalSign = rotationalSpeedY < 0f ? -1f : 1f;
        var rotationalDrag = AirboatRotationalDrag * rotationalSpeedY * rotationalSpeedY *
            bodyMassKg * deltaSeconds * rotationalSign;
        var rotationalDamping = AirboatRotationalDamping * MathF.Abs(rotationalSpeedY) *
            bodyMassKg * deltaSeconds * rotationalSign;
        var rotationalForce = steeringForce + rotationalDrag + rotationalDamping;
        return new(new Vector3(0f, -rotationalForce, 0f), steeringReversed,
            steerTime, steeringAngle * (steeringReversed ? -1f : 1f));
    }

    /// Shared core-space implementation of Source's
    /// DoSimulationKeepUprightPitch/DoSimulationKeepUprightRoll controllers.
    /// `goalAxisCore` is the world-down axis transformed into core space.
    public static AirboatUprightResult ComputeAirboatUprightImpulse(
        Vector3 goalAxisCore, bool rollController, bool weakJump,
        bool hasSurfaceContact, float previousError, float deltaSeconds,
        float bodyMassKg)
    {
        if (!IsFinite(goalAxisCore) || !float.IsFinite(previousError) ||
            !float.IsFinite(deltaSeconds) || !float.IsFinite(bodyMassKg))
            throw new ArgumentOutOfRangeException(nameof(goalAxisCore));
        if (goalAxisCore.LengthSquared() < 1e-12f)
            throw new ArgumentOutOfRangeException(nameof(goalAxisCore));
        if (deltaSeconds < 0f || bodyMassKg < 0f)
            throw new ArgumentOutOfRangeException(nameof(deltaSeconds));
        if (!rollController && weakJump)
            return new(Vector3.Zero, previousError);

        var reference = new Vector3(0f,
            -MathF.Cos(AirboatUprightReferenceAngleRadians),
            MathF.Sin(AirboatUprightReferenceAngleRadians));
        var goal = Vector3.Normalize(goalAxisCore);
        if (rollController) goal.Y = reference.Y;
        else goal.X = reference.X;
        goal = Vector3.Normalize(goal);

        var rotationAxis = Vector3.Cross(reference, goal);
        var sine = rotationAxis.Length();
        if (sine > 1e-12f) rotationAxis /= sine;
        var angle = MathF.Atan2(sine, Vector3.Dot(reference, goal));
        if (hasSurfaceContact || (rollController && MathF.Abs(angle) <
            MathF.PI / 18f))
            return new(Vector3.Zero, angle);

        var impulseMagnitude = bodyMassKg * (rollController
            ? 0.2f * angle + 0.3f * deltaSeconds * (angle - previousError)
            : 0.1f * angle + 0.04f * deltaSeconds * (angle - previousError));
        var impulse = rotationAxis * impulseMagnitude;
        var maximum = bodyMassKg * (rollController
            ? 2f * MathF.PI / 180f
            : 1.5f * MathF.PI / 180f);
        var length = impulse.Length();
        if (length > maximum && length > 1e-12f) impulse *= maximum / length;
        return new(impulse, angle);
    }

    private static void ValidateAirboatDragInputs(Vector3 localVelocity,
        float speedMetersPerSecond, float bodyMassKg, float deltaSeconds)
    {
        if (!float.IsFinite(localVelocity.X) || !float.IsFinite(localVelocity.Y) ||
            !float.IsFinite(localVelocity.Z))
            throw new ArgumentOutOfRangeException(nameof(localVelocity));
        if (!float.IsFinite(speedMetersPerSecond) || speedMetersPerSecond < 0f)
            throw new ArgumentOutOfRangeException(nameof(speedMetersPerSecond));
        if (!float.IsFinite(bodyMassKg) || bodyMassKg < 0f)
            throw new ArgumentOutOfRangeException(nameof(bodyMassKg));
        if (!float.IsFinite(deltaSeconds) || deltaSeconds < 0f)
            throw new ArgumentOutOfRangeException(nameof(deltaSeconds));
    }

    private static bool IsFinite(Vector3 value) =>
        float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

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
