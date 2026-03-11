package frc.robot.ui;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

// This class extends ContinuousAction because it is only used by the drive subsystem
// Doing it this way instead of including getSpeeds() in ContinuousAction 
// means other subsystems can use ContinuousAction instances without having
// to implement a dummy, do nothing getSpeeds() method.
public interface AutoDriveHelperAction extends ContinuousAction {
    ChassisSpeeds getSpeeds();
}