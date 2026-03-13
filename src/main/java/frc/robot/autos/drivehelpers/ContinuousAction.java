package frc.robot.autos.drivehelpers;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.TargetInfo;

public interface ContinuousAction {
    void start();
    void update(Optional<TargetInfo> tag, boolean isAutoDriveEnabled);
    void stop();
    ChassisSpeeds getSpeeds();
    default boolean isFinished() { return false; }
    default boolean shouldActivate( ADContext adCtx, 
                                    boolean autoDriveEnabled, 
                                    Optional<TargetInfo> tag) { return true; }
}