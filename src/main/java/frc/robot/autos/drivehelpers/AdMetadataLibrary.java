package frc.robot.autos.drivehelpers;

public final class AdMetadataLibrary {

    //
    //  SHOOT MODE HELPERS
    //
    //  Speaker tags for 2024: 4, 7, 8
    //  (You can adjust these if your vision system uses a different mapping)
    //
    private static final int[] SPEAKER_TAGS = { 4, 7, 8 };

    //
    // SHOOT_AUTO_AIM
    // - Rotation helper
    // - Activates when heading error exceeds threshold
    // - Deactivates with hysteresis
    //
    public static final ADHelperMetadata SHOOT_AIM =
        new ADHelperMetadata(
            SPEAKER_TAGS,
            ADHelperMetadata.TagSelectionMode.NEAREST,
            2.0,   // activate heading error (deg)
            0.0,   // activate distance error (unused)
            0.0,   // activate lateral error (unused)
            1.0,   // deactivate heading error (deg)
            0.0,   // deactivate distance error
            0.0,   // deactivate lateral error
            ADAction.ADActionType.ROTATION
        );

    //
    // SHOOT_AUTO_RANGE
    // - Translation helper (forward/back)
    // - Activates when distance error exceeds threshold
    //
    public static final ADHelperMetadata SHOOT_RANGE =
        new ADHelperMetadata(
            SPEAKER_TAGS,
            ADHelperMetadata.TagSelectionMode.NEAREST,
            0.0,   // activate heading error (unused)
            0.30,  // activate distance error (meters)
            0.0,   // activate lateral error (unused)
            0.0,
            0.20,  // deactivate distance error (meters)
            0.0,
            ADAction.ADActionType.TRANSLATION
        );

    //
    // SHOOT_AUTO_STRAFE
    // - Translation helper (left/right)
    // - Activates when lateral error exceeds threshold
    //
    public static final ADHelperMetadata SHOOT_STRAFE =
        new ADHelperMetadata(
            SPEAKER_TAGS,
            ADHelperMetadata.TagSelectionMode.NEAREST,
            0.0,
            0.0,
            0.15,  // activate lateral error (meters)
            0.0,
            0.0,
            0.05,  // deactivate lateral error (meters)
            ADAction.ADActionType.TRANSLATION
        );

    //
    // Prevent instantiation
    //
    private AdMetadataLibrary() {}
}