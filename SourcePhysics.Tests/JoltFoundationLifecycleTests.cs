using SourcePhysics;
using Xunit;

namespace SourcePhysics.Tests;

public sealed class JoltFoundationLifecycleTests
{
    [Fact]
    public async Task ConcurrentHostsShareFoundationLifetimeWithoutNativeShutdownRace()
    {
        var tasks = Enumerable.Range(0, 8).Select(_ => Task.Run(() =>
        {
            using var host = new JoltPhysicsHost(new SourceMovementProfile());
            host.Initialize(256, 0, 256, 128);
            Assert.NotNull(host.System);
        }));

        await Task.WhenAll(tasks);
    }

    [Fact]
    public void RepeatedHostTeardownDisposesTheNativePhysicsSystemBeforeFoundation()
    {
        for (var iteration = 0; iteration < 16; iteration++)
        {
            using var host = new JoltPhysicsHost(new SourceMovementProfile());
            host.Initialize(256, 0, 256, 128);
        }
    }
}
