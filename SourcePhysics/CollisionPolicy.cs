namespace SourcePhysics;

/// Authored Source collision-set policy. Configure it before the Jolt host is initialized.
public sealed class SourceCollisionPolicy
{
    private readonly bool[,] collisions;

    public SourceCollisionPolicy()
    {
        var count = Enum.GetValues<SourceObjectLayer>().Length;
        collisions = new bool[count, count];
        for (var a = 0; a < count; a++)
        for (var b = 0; b < count; b++)
            collisions[a, b] = SourceCollisionRules.CanCollide((SourceObjectLayer)a, (SourceObjectLayer)b);
    }

    public bool CanCollide(SourceObjectLayer a, SourceObjectLayer b) => collisions[(int)a, (int)b];

    public void SetCollision(SourceObjectLayer a, SourceObjectLayer b, bool enabled)
    {
        collisions[(int)a, (int)b] = enabled;
        collisions[(int)b, (int)a] = enabled;
    }
}
