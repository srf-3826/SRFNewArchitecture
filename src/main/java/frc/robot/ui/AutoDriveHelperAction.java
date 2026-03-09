package frc.robot.ui;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface AutoDriveHelperAction extends ContinuousAction {
    ChassisSpeeds getSpeeds();
}