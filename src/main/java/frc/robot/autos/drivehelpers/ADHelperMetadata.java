// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

public class ADHelperMetadata {

    // Which AprilTags this helper cares about
    public final int[] requiredTagIds;

    // Whether to use nearest, farthest, or first valid tag
    public final TagSelectionMode tagSelectionMode;

    // Error thresholds for activation
    public final double activateHeadingErrorDeg;
    public final double activateDistanceErrorMeters;
    public final double activateLateralErrorMeters;

    // Error thresholds for deactivation (hysteresis)
    public final double deactivateHeadingErrorDeg;
    public final double deactivateDistanceErrorMeters;
    public final double deactivateLateralErrorMeters;

    // Helper type (rotation, translation, full)
    public final ADAction.ADActionType helperType;

    public ADHelperMetadata(
        int[] requiredTagIds,
        TagSelectionMode tagSelectionMode,
        double activateHeadingErrorDeg,
        double activateDistanceErrorMeters,
        double activateLateralErrorMeters,
        double deactivateHeadingErrorDeg,
        double deactivateDistanceErrorMeters,
        double deactivateLateralErrorMeters,
        ADAction.ADActionType helperType
    ) {
        this.requiredTagIds = requiredTagIds;
        this.tagSelectionMode = tagSelectionMode;
        this.activateHeadingErrorDeg = activateHeadingErrorDeg;
        this.activateDistanceErrorMeters = activateDistanceErrorMeters;
        this.activateLateralErrorMeters = activateLateralErrorMeters;
        this.deactivateHeadingErrorDeg = deactivateHeadingErrorDeg;
        this.deactivateDistanceErrorMeters = deactivateDistanceErrorMeters;
        this.deactivateLateralErrorMeters = deactivateLateralErrorMeters;
        this.helperType = helperType;
    }

    public enum TagSelectionMode {
        NEAREST,
        FARTHEST,
        FIRST_VALID
    }
}