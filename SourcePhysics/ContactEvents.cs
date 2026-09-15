using JoltPhysicsSharp;
using System.Numerics;
using System.Collections.Concurrent;

namespace SourcePhysics;

public readonly record struct SourceContactEvent(uint BodyA, uint BodyB, Vector3 ContactPoint, bool Persisted,
    Vector3 Normal = default, float PenetrationDepth = 0f);
public readonly record struct SourceContactRemovedEvent(uint BodyA, uint BodyB);
public readonly record struct SourceTriggerEvent(uint TriggerBody, uint OtherBody, Vector3 ContactPoint, bool Persisted,
    Vector3 Normal = default);
public readonly record struct SourceTriggerRemovedEvent(uint TriggerBody, uint OtherBody);

public sealed class SourceContactRouter
{
    private readonly Func<BodyID, SourceSurface> surfaceResolver;
    private readonly SourceContactMaterialPolicy materialPolicy;
    private readonly Func<BodyID, bool> sensorResolver;
    public event Action<SourceContactEvent>? ContactAdded;
    public event Action<SourceContactEvent>? ContactPersisted;
    public event Action<SourceContactRemovedEvent>? ContactRemoved;
    public event Action<SourceTriggerEvent>? TriggerEntered;
    public event Action<SourceTriggerEvent>? TriggerStayed;
    public event Action<SourceTriggerRemovedEvent>? TriggerExited;
    private readonly ConcurrentQueue<SourceContactEvent> added = new();
    private readonly ConcurrentQueue<SourceContactEvent> persisted = new();
    private readonly ConcurrentQueue<SourceContactRemovedEvent> removed = new();
    private readonly ConcurrentQueue<SourceTriggerEvent> triggerEntered = new();
    private readonly ConcurrentQueue<SourceTriggerEvent> triggerStayed = new();
    private readonly ConcurrentQueue<SourceTriggerRemovedEvent> triggerExited = new();

    public SourceContactRouter(Func<BodyID, SourceSurface> surfaceResolver,
        SourceContactMaterialPolicy? materialPolicy = null, Func<BodyID, bool>? sensorResolver = null)
    {
        this.surfaceResolver = surfaceResolver;
        this.materialPolicy = materialPolicy ?? new SourceContactMaterialPolicy();
        this.sensorResolver = sensorResolver ?? (static _ => false);
    }

    internal void OnAdded(PhysicsSystem system, in Body a, in Body b, in ContactManifold manifold, ref ContactSettings settings)
    {
        ApplySourceCombine(a.ID, b.ID, ref settings);
        var contact = CreateEvent(a, b, manifold, false);
        added.Enqueue(contact);
        QueueTrigger(contact);
    }

    internal void OnPersisted(PhysicsSystem system, in Body a, in Body b, in ContactManifold manifold, ref ContactSettings settings)
    {
        ApplySourceCombine(a.ID, b.ID, ref settings);
        var contact = CreateEvent(a, b, manifold, true);
        persisted.Enqueue(contact);
        QueueTrigger(contact);
    }

    private static SourceContactEvent CreateEvent(in Body a, in Body b, in ContactManifold manifold, bool persisted)
    {
        var point = manifold.PointCount > 0 ? manifold.GetWorldSpaceContactPointOn1(0) : (Vector3)a.RPosition;
        return new(a.ID.ID, b.ID.ID, point, persisted, manifold.WorldSpaceNormal, manifold.PenetrationDepth);
    }

    private void ApplySourceCombine(BodyID a, BodyID b, ref ContactSettings settings)
    {
        var first = surfaceResolver(a); var second = surfaceResolver(b);
        settings.CombinedFriction = materialPolicy.GetFriction(first, second);
        settings.CombinedRestitution = materialPolicy.GetRestitution(first, second);
    }
    internal void OnRemoved(PhysicsSystem _, ref SubShapeIDPair pair)
    {
        removed.Enqueue(new(pair.Body1ID.ID, pair.Body2ID.ID));
        if (sensorResolver(pair.Body1ID)) triggerExited.Enqueue(new(pair.Body1ID.ID, pair.Body2ID.ID));
        else if (sensorResolver(pair.Body2ID)) triggerExited.Enqueue(new(pair.Body2ID.ID, pair.Body1ID.ID));
    }

    private void QueueTrigger(in SourceContactEvent contact)
    {
        if (sensorResolver(new BodyID(contact.BodyA)))
        {
            var eventValue = new SourceTriggerEvent(contact.BodyA, contact.BodyB, contact.ContactPoint, contact.Persisted, contact.Normal);
            if (contact.Persisted) triggerStayed.Enqueue(eventValue); else triggerEntered.Enqueue(eventValue);
        }
        else if (sensorResolver(new BodyID(contact.BodyB)))
        {
            var eventValue = new SourceTriggerEvent(contact.BodyB, contact.BodyA, contact.ContactPoint, contact.Persisted, -contact.Normal);
            if (contact.Persisted) triggerStayed.Enqueue(eventValue); else triggerEntered.Enqueue(eventValue);
        }
    }

    internal void Flush()
    {
        while (added.TryDequeue(out var value)) ContactAdded?.Invoke(value);
        while (persisted.TryDequeue(out var value)) ContactPersisted?.Invoke(value);
        while (removed.TryDequeue(out var value)) ContactRemoved?.Invoke(value);
        while (triggerEntered.TryDequeue(out var value)) TriggerEntered?.Invoke(value);
        while (triggerStayed.TryDequeue(out var value)) TriggerStayed?.Invoke(value);
        while (triggerExited.TryDequeue(out var value)) TriggerExited?.Invoke(value);
    }
}
