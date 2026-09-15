using Stride.Engine;
using System.Numerics;

namespace SourcePhysics;

/// Consumes the authoritative Source presentation frame through Stride's
/// AnimationComponent. Clip names are deliberately authored by the title;
/// the physics port never guesses model-specific animation assets.
public sealed class JoltCharacterAnimation : SyncScript
{
    public JoltCharacterMovement? Movement { get; set; }
    public string IdleClip { get; set; } = string.Empty;
    public string MoveClip { get; set; } = string.Empty;
    public string CrouchIdleClip { get; set; } = string.Empty;
    public string CrouchMoveClip { get; set; } = string.Empty;
    public string JumpClip { get; set; } = string.Empty;
    public string FallClip { get; set; } = string.Empty;
    public float CrossfadeSeconds { get; set; } = 0.08f;
    public float MoveThresholdMetersPerSecond { get; set; } = 0.01f;

    private AnimationComponent? animation;
    private string currentClip = string.Empty;

    public override void Start()
    {
        if (!float.IsFinite(CrossfadeSeconds) || CrossfadeSeconds < 0f)
            throw new InvalidOperationException("CrossfadeSeconds must be finite and non-negative.");
        if (!float.IsFinite(MoveThresholdMetersPerSecond) || MoveThresholdMetersPerSecond < 0f)
            throw new InvalidOperationException("MoveThresholdMetersPerSecond must be finite and non-negative.");

        animation = Entity.Get<AnimationComponent>();
        Movement ??= Entity.Get<JoltCharacterMovement>();
        if (animation is not null && Movement is not null)
            Movement.PresentationUpdated += OnPresentation;
    }

    public override void Update() { }

    public override void Cancel()
    {
        if (Movement is not null) Movement.PresentationUpdated -= OnPresentation;
        animation = null;
    }

    private void OnPresentation(SourceCharacterPresentationFrame frame)
    {
        if (animation is null) return;
        var clip = SelectClip(frame.Movement);
        if (clip.Length == 0 || clip == currentClip) return;
        if (CrossfadeSeconds == 0f) animation.Play(clip);
        else animation.Crossfade(clip, TimeSpan.FromSeconds(CrossfadeSeconds));
        currentClip = clip;
    }

    private string SelectClip(in MovementState state)
    {
        var horizontalSpeed = new Vector2(state.Velocity.X, state.Velocity.Z).Length();
        var moving = horizontalSpeed > MoveThresholdMetersPerSecond;
        if (state.Ground != GroundState.Grounded)
            return state.Velocity.Y >= 0f ? JumpClip : FallClip;
        if (state.Ducking)
            return moving ? CrouchMoveClip : CrouchIdleClip;
        return moving ? MoveClip : IdleClip;
    }
}
