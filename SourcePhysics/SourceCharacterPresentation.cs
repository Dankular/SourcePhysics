using System.Numerics;

namespace SourcePhysics;

/// <summary>
/// Authoritative fixed-tick presentation input for Stride animation, camera,
/// root-motion and audio consumers. It carries physics state only; title
/// presentation policy remains in the consuming system.
/// </summary>
public readonly record struct SourceCharacterPresentationFrame(
    int Tick,
    SourceInput Command,
    MovementState Movement,
    Vector3 PositionMeters,
    Quaternion Orientation,
    float ViewHeightMeters)
{
    public void Validate()
    {
        if (Tick < 0) throw new ArgumentOutOfRangeException(nameof(Tick));
        ValidateFinite(Command.Move, nameof(Command.Move));
        if (!float.IsFinite(Command.ViewYawRadians) || !float.IsFinite(Command.ViewPitchRadians) || !float.IsFinite(Command.UpMove))
            throw new InvalidDataException("Presentation command contains a non-finite scalar.");
        ValidateFinite(Movement.Position, nameof(Movement.Position));
        ValidateFinite(Movement.Velocity, nameof(Movement.Velocity));
        ValidateFinite(PositionMeters, nameof(PositionMeters));
        ValidateFinite(Orientation, nameof(Orientation));
        if (!float.IsFinite(ViewHeightMeters) || ViewHeightMeters < 0f)
            throw new InvalidDataException("Presentation view height must be finite and non-negative.");
    }

    private static void ValidateFinite(Vector2 value, string name)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y))
            throw new InvalidDataException($"Presentation {name} must be finite.");
    }

    private static void ValidateFinite(Vector3 value, string name)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new InvalidDataException($"Presentation {name} must be finite.");
    }

    private static void ValidateFinite(Quaternion value, string name)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z) || !float.IsFinite(value.W))
            throw new InvalidDataException($"Presentation {name} must be finite.");
    }
}
