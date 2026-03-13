package frc.robot.autos.drivehelpers;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.TargetInfo;

public interface ContinuousAction {
    void start();
    void update(Optional<TargetInfo> tag, boolean isAutoDriveEnabled);
    void stop();
    ChassisSpeeds getSpeeds();
    boolean isFinished(ADContext adCtx,
                       Optional<TargetInfo> tag,
                       boolean autoDriveEnabled);
    boolean shouldActivate(ADContext adCtx, 
                           Optional<TargetInfo> tag,
                           boolean autoDriveEnabled);
}