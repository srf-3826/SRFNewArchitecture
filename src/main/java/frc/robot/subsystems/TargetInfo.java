// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;

public class TargetInfo {
    public final int tagId;
    public final Pose3d robotPoseInTagSpace;
    public final double timestamp;

    public TargetInfo(int tagId, Pose3d robotPoseInTagSpace, double timestamp) {
        this.tagId = tagId;
        this.robotPoseInTagSpace = robotPoseInTagSpace;
        this.timestamp = timestamp;
    }
}