// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoC;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveToPoseHelper {

    private final SwerveSubsystem swerve;
    private Pose2d targetPose;

    private final HolonomicDriveController controller;

    public AutoDriveToPoseHelper(SwerveSubsystem swerve) {
        this.swerve = swerve;

        // Students can tune these easily
        PIDController xPID = new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, AutoC.KD_X_CONTROLLER);
        PIDController yPID = new PIDController(AutoC.KP_Y_CONTROLLER, AutoC.KI_Y_CONTROLLER, AutoC.KD_Y_CONTROLLER);
        ProfiledPIDController thetaPID = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER, 
                                                                   AutoC.KI_THETA_CONTROLLER, 
                                                                   AutoC.KD_THETA_CONTROLLER,
                                                                   AutoC.K_THETA_CONTROLLER_TP_CONSTRAINTS);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        controller = new HolonomicDriveController(xPID, yPID, thetaPID);

        // Default target pose (can be overwritten)
        targetPose = new Pose2d();
    }

    /** Set the target pose for this helper */
    public void setTargetPose(Pose2d pose) {
        this.targetPose = pose;
    }

    /** Compute auto chassis speeds for the default drive command */
    public ChassisSpeeds calculate() {
        Pose2d current = swerve.getPose();

        // Desired linear velocity at the target = 0 (stop there)
        return controller.calculate(current,
                                    targetPose,
                                    0.0,
                                    targetPose.getRotation());
    }
}