// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ui;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TargetInfo;
import frc.robot.subsystems.VisionSubsystem;

public final class HelperContext {

    // --- Drive-related subsystems and suppliers ---
    public final SwerveSubsystem drive;
    public final Supplier<Pose2d> poseSupplier;
    public final Supplier<Rotation2d> headingSupplier;
    public final Supplier<ChassisSpeeds> autoDriveChassisSpeedsSupplier;

    // --- Vision / targeting ---
    public final VisionSubsystem vision;
    public final Supplier<Optional<TargetInfo>> targetSupplier;

    // --- Intake / feeder ---
    public final IntakeSubsystem intake;

    // --- Shooter ---
    public final ShooterSubsystem shooter;

    // --- Climber ---
    public final ClimberSubsystem climber;

    // --- ModeManager ---
    public final ModeManager modeManager;

    // --- Constructor: RobotContainer builds this once ---
    public HelperContext(SwerveSubsystem drive,
                         Supplier<Pose2d> poseSupplier,
                         Supplier<Rotation2d> headingSupplier,
                         Supplier<ChassisSpeeds> autoDriveChassisSpeedsSupplier,
                         VisionSubsystem vision,
                         Supplier<Optional<TargetInfo>> targetSupplier,
                         IntakeSubsystem intake,
                         ShooterSubsystem shooter,
                         ClimberSubsystem climber,
                         ModeManager modeManager) {
        this.drive                          = drive;
        this.poseSupplier                   = drive::getPose;
        this.headingSupplier                = drive::getYaw2d;
        this.autoDriveChassisSpeedsSupplier = drive::getAutoDriveAssistSpeeds;
        this.vision                         = vision;
        this.targetSupplier                 = targetSupplier;
        this.intake                         = intake;
        this.shooter                        = shooter;
        this.climber                        = climber;
        this.modeManager                    = modeManager;
    }
}
