package frc.robot.autos.drivehelpers;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.ui.ModeManager;

/*
 * This class provides context for processing Auto Drive Assist Actions
 */
public final class ADContext {
    
    // --- Drive-related subsystems and suppliers ---
    public final SwerveSubsystem swerve;
    public final VisionSubsystem vision;
    public final ModeManager modeManager;

    // --- Constructor: RobotContainer builds this once ---
    public ADContext(   SwerveSubsystem swerve,
                        VisionSubsystem vision,
                        ModeManager modeManager ) {
        this.swerve                         = swerve;
        this.vision                         = vision;
        this.modeManager                    = modeManager;
    }

    public boolean inShootMode() {
        return modeManager.isShootMode();
    }
}