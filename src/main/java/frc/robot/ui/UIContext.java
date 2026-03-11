package frc.robot.ui;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class UIContext {

    public final SwerveSubsystem swerve;
    public final IntakeSubsystem intake;
    public final ShooterSubsystem shooter;
    public final ClimberSubsystem climber;

    public final ModeManager modeManager;
    public final SystemActionManager systemActionManager;

    // --- Constructor: RobotContainer builds this once ---
    public UIContext(   SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ClimberSubsystem climber,
                        ModeManager modeManager,
                        SystemActionManager systemActionManager) {
        this.swerve                         = swerve;
        this.intake                         = intake;
        this.shooter                        = shooter;
        this.climber                        = climber;
        this.modeManager                    = modeManager;
        this.systemActionManager            = systemActionManager;
    }

    // This is a helper method to enable ALT-X : EMIT_RESET_ALL
    public void resetAllMechanisms() {
        systemActionManager.emit(Action.PIVOT_TO_HOLD);
        systemActionManager.emit(Action.STOP_SHOOTER);
        systemActionManager.emit(Action.STOW_ELEVATOR);
        systemActionManager.emit(Action.STOW_WINCH);
    }
}
