using JoltPhysicsSharp;

namespace SourcePhysics;

/// Applies the Source collision-group matrix at the solver's per-body pair
/// boundary. Object layers remain the broadphase coarse filter; this filter
/// preserves the authored Source group identity for bodies sharing a layer.
internal sealed class SourceSimShapeFilter : SimShapeFilter
{
    private readonly Func<BodyID, SourceCollisionGroup> groupResolver;

    public SourceSimShapeFilter(Func<BodyID, SourceCollisionGroup> groupResolver)
    {
        this.groupResolver = groupResolver ?? throw new ArgumentNullException(nameof(groupResolver));
    }

    protected override bool ShouldCollide(Body body1, Shape shape1, in SubShapeID subShapeIDOfShape1,
        Body inBody2, Shape shape2, in SubShapeID subShapeIDOfShape2) =>
        SourceCollisionRules.ShouldCollide(groupResolver(body1.ID), groupResolver(inBody2.ID));
}
