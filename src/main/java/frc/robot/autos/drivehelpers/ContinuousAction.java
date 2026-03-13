package frc.robot.autos.drivehelpers;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.TargetInfo;

public interface ContinuousAction {
    void start();
    void update(Optional<TargetInfo> tag);
    void stop();
    ChassisSpeeds getSpeeds();
    default boolean isFinished() { return false; }
    default boolean shouldActivate(ADContext adCtx) { return true; }
}