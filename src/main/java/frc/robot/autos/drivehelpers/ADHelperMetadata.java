// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

public class ADHelperMetadata {

    // Which AprilTags this helper cares about
    public final int[] requiredTagIds;

    // Whether to use nearest, farthest, or first valid tag
    public final TagSelectionMode tagSelectionMode;

    // PID control factors
    public final double kP;
    public final double kI;
    public final double kD;

    // Error thresholds for activation
    public final double activateHeadingErrorDeg;
    public final double activateDistanceErrorMeters;
    public final double activateLateralErrorMeters;

    // Error thresholds for deactivation (hysteresis)
    public final double finishHeadingErrorDeg;
    public final double finishDistanceErrorMeters;
    public final double finishLateralErrorMeters;

    // Helper type (rotation, translation, full)
    public final ADAction.ADActionType helperType;

    public ADHelperMetadata(
        int[] requiredTagIds,
        TagSelectionMode tagSelectionMode,
        double kP,
        double kI,
        double kD,
        double activateHeadingErrorDeg,
        double activateDistanceErrorMeters,
        double activateLateralErrorMeters,
        double finishHeadingErrorDeg,
        double finishDistanceErrorMeters,
        double finishLateralErrorMeters,
        ADAction.ADActionType helperType
    ) {
        this.requiredTagIds = requiredTagIds;
        this.tagSelectionMode = tagSelectionMode;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.activateHeadingErrorDeg = activateHeadingErrorDeg;
        this.activateDistanceErrorMeters = activateDistanceErrorMeters;
        this.activateLateralErrorMeters = activateLateralErrorMeters;
        this.finishHeadingErrorDeg = finishHeadingErrorDeg;
        this.finishDistanceErrorMeters = finishDistanceErrorMeters;
        this.finishLateralErrorMeters = finishLateralErrorMeters;
        this.helperType = helperType;
    }

    public enum TagSelectionMode {
        NEAREST,
        FARTHEST,
        FIRST_VALID
    }
}