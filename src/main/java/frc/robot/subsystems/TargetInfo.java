// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;

public class TargetInfo {

    public final int tagId;
    public final Pose3d robotPoseInTagSpace;           // Tag → Robot transform (from Limelight)
    public final double timestamp;                     // Timestamp of capture

    public TargetInfo(int tagId, Pose3d robotPoseInTagSpace, double timestamp) {
        this.tagId = tagId;
        this.robotPoseInTagSpace = robotPoseInTagSpace;
        this.timestamp = timestamp;
    }

    //
    // --- Derived helper-friendly values ---
    //

    /** Horizontal angle to tag (degrees). Positive = tag is to the right. */
    // The aim helper’s job is: “Rotate until the tag is centered in the camera.”
    // That means the heading error IS the measurement. For a PID The correct call is:
    // double omega = m_pid.calculate(headingError, 0.0);
    // which seems weird passing in the pre-computed error, but with a target of
    // 0 degrees (centered) it works: headingError = currentHeading - desiredHeading
    // For a tag at yaw angle θ relative to the robot:
    // If the tag is to the left → yaw is negative
    // If the tag is to the right → yaw is positive
    // If the tag is centered → yaw ≈ 0
    // So: headingError = tagYawDegrees
    public double headingErrorDeg() {
        // Rotation around Z axis is yaw
        return robotPoseInTagSpace.getRotation().getZ();
    }

    /** Forward/back distance to tag (meters). Positive = tag is in front. */
    public double distanceErrorMeters() {
        return robotPoseInTagSpace.getTranslation().getX();
    }

    /** Left/right offset to tag (meters). Positive = tag is to the left. */
    public double lateralErrorMeters() {
        return robotPoseInTagSpace.getTranslation().getY();
    }
}