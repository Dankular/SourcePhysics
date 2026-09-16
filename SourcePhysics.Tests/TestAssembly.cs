using Xunit;

// Jolt's native Foundation and PhysicsSystem lifetime is process-global. The
// tests already exercise concurrency explicitly where it is part of the
// contract, but independent fixture collections must not initialize and shut
// down the native foundation concurrently.
[assembly: CollectionBehavior(DisableTestParallelization = true)]
