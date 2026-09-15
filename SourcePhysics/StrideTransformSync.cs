using Stride.Core.Mathematics;
using Stride.Engine;
using NumericsMatrix4x4 = System.Numerics.Matrix4x4;
using NumericsQuaternion = System.Numerics.Quaternion;
using NumericsVector3 = System.Numerics.Vector3;

namespace SourcePhysics;

/// Keeps Jolt's world-space authoritative pose aligned with Stride's
/// parent-relative TransformComponent properties.
public static class StrideTransformSync
{
    public static void SetWorldPose(TransformComponent transform, NumericsVector3 worldPosition,
        NumericsQuaternion worldRotation)
    {
        ArgumentNullException.ThrowIfNull(transform);
        if (transform.Parent is null)
        {
            transform.Position = ToStride(worldPosition);
            transform.Rotation = ToStride(worldRotation);
            return;
        }

        transform.Parent.UpdateWorldMatrix();
        var parent = ToNumerics(transform.Parent.WorldMatrix);
        if (!NumericsMatrix4x4.Invert(parent, out var inverse) ||
            !NumericsMatrix4x4.Decompose(parent, out _, out var parentRotation, out _))
            throw new InvalidOperationException("Stride parent transform is not invertible or decomposable.");

        var localPosition = NumericsVector3.Transform(worldPosition, inverse);
        var localRotation = NumericsQuaternion.Normalize(NumericsQuaternion.Conjugate(parentRotation) * worldRotation);
        transform.Position = ToStride(localPosition);
        transform.Rotation = ToStride(localRotation);
    }

    private static NumericsMatrix4x4 ToNumerics(Matrix value) => new(
        value.M11, value.M12, value.M13, value.M14,
        value.M21, value.M22, value.M23, value.M24,
        value.M31, value.M32, value.M33, value.M34,
        value.M41, value.M42, value.M43, value.M44);

    private static Stride.Core.Mathematics.Vector3 ToStride(NumericsVector3 value) => new(value.X, value.Y, value.Z);
    private static Stride.Core.Mathematics.Quaternion ToStride(NumericsQuaternion value) =>
        new(value.X, value.Y, value.Z, value.W);
}
