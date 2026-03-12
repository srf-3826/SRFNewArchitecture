package frc.robot.autos.drivehelpers;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TargetInfo;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.ui.ModeManager;

/*
 * This class provides context for processing Auto Drive Assist Actions
 */
public final class ADContext {
    
    // --- Drive-related subsystems and suppliers ---
    public final SwerveSubsystem swerve;
    public final Supplier<Pose2d> poseSupplier;
    public final Supplier<Rotation2d> headingSupplier;
    public final Supplier<ChassisSpeeds> autoDriveChassisSpeedsSupplier;

    // --- Vision / targeting ---
    public final VisionSubsystem vision;
    public final Supplier<Optional<TargetInfo>> targetSupplier;

    // ModeManager
    public final ModeManager modeManager;

    // --- Constructor: RobotContainer builds this once ---
    public ADContext(   SwerveSubsystem swerve,
                        Supplier<Pose2d> poseSupplier,
                        Supplier<Rotation2d> headingSupplier,
                        Supplier<ChassisSpeeds> autoDriveChassisSpeedsSupplier,
                        VisionSubsystem vision,
                        Supplier<Optional<TargetInfo>> targetSupplier,
                        ModeManager modeManager ) {
        this.swerve                         = swerve;
        this.poseSupplier                   = poseSupplier;
        this.headingSupplier                = headingSupplier;
        this.autoDriveChassisSpeedsSupplier = autoDriveChassisSpeedsSupplier;
        this.vision                         = vision;
        this.targetSupplier                 = targetSupplier;
        this.modeManager                    = modeManager;
    }

    public boolean inShootMode() {
        return modeManager.isShootMode();
    }

    public boolean isShootAutoDriveEnabled() {
        return swerve.isShootAutoDriveHelpersEnabled();
    }
}