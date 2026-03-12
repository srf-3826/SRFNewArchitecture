package frc.robot.ui;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveParkCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class UIContext {

    public final SwerveSubsystem swerve;
    public final IntakeSubsystem intake;
    public final ShooterSubsystem shooter;
    public final ClimberSubsystem climb;
    public final SystemActionManager m_systemActionManager;
    public final SwerveParkCmd m_swerveParkCmd;

    // --- Constructor: RobotContainer builds this once ---
    public UIContext(   SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        ShooterSubsystem shooter,
                        ClimberSubsystem climb,
                        SystemActionManager systemActionManager,
                        SwerveParkCmd swerveParkCmd ) {

        this.swerve                         = swerve;
        this.intake                         = intake;
        this.shooter                        = shooter;
        this.climb                          = climb;
        this.m_systemActionManager          = systemActionManager;
        this.m_swerveParkCmd                = swerveParkCmd;
    } 

    // This is a helper method to enable ALT-X : EMIT_RESET_ALL
    public void resetAllMechanisms() {
        m_systemActionManager.emit(Action.PIVOT_TO_HOLD);
        m_systemActionManager.emit(Action.STOP_SHOOTER);
        m_systemActionManager.emit(Action.STOW_ELEVATOR);
        m_systemActionManager.emit(Action.STOW_WINCH);
    }

    // This is a helper method to schedule a park command
    public void schedulePark() {
        CommandScheduler.getInstance().schedule(m_swerveParkCmd);
    }
}
