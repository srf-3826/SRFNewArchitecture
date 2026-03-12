package frc.robot.autos.drivehelpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ContinuousAction { 
    void start();
    void update();
    void stop();
    ChassisSpeeds getSpeeds();
    
    default boolean isFinished() {
        return false;
    }
}