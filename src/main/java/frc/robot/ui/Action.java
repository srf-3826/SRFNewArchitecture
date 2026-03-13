package frc.robot.ui;

import java.util.function.Consumer;

public enum Action {

    // --- Swerve / drive ---
    FL_COR(
        ctx -> ctx.swerve.setFLCenOfRotation(),
        ctx -> ctx.swerve.resetCenOfRotation()
    ),

    FR_COR(
        ctx -> ctx.swerve.setFRCenOfRotation(),
        ctx -> ctx.swerve.resetCenOfRotation()
    ),

    BL_COR(
        ctx -> ctx.swerve.setBLCenOfRotation(),
        ctx -> ctx.swerve.resetCenOfRotation()
    ),

    BR_COR(
        ctx -> ctx.swerve.setBRCenOfRotation(),
        ctx -> ctx.swerve.resetCenOfRotation()
    ),

    GO_SLOW(
        ctx -> ctx.swerve.setVarMaxOutputFactor(0.5),
        ctx -> ctx.swerve.setVarMaxOutputFactor(1.0)
    ),

    GO_SUPER_SLOW(
        ctx -> ctx.swerve.setVarMaxOutputFactor(0.2),
        ctx -> ctx.swerve.setVarMaxOutputFactor(1.0)
    ),

    AUTO_DRIVE_ON(
        ctx -> ctx.swerve.getAutoDriveAgent().enableAutoDriveHelpers(),
        null
    ),

    AUTO_DRIVE_OFF(
        ctx -> ctx.swerve.getAutoDriveAgent().disableAutoDriveHelpers(),
        null
    ),

    PARK(
        ctx -> ctx.schedulePark(),
        null
    ),

    DEFENSE_GO_SLOW(
        ctx -> ctx.swerve.setVarMaxOutputFactor(0.5),
        null
    ),

    DEFENSE_GO_FAST(
        ctx -> ctx.swerve.setVarMaxOutputFactor(1.0),
        null
    ),

    // --- Intake ---
    START_INTAKE(
        ctx -> ctx.intake.startIntake(),
        null
    ),

    BED_ROLL_IN(
        ctx -> ctx.intake.startBedRollersIn(),
        null
    ),

    BED_ROLL_STOP(
        ctx -> ctx.intake.stopBedRollers(),
        null
    ),

    DUMP_FUEL(
        ctx -> ctx.intake.dumpFuel(),
        null
    ),

    PIVOT_TO_HOLD(
        ctx -> ctx.intake.stopAndPivotToHold(),
        null
    ),

    RETRACT_INTAKE(
        ctx -> ctx.intake.retractIntake(),
        null
    ),

    // --- Shooter ---
    SHOOT_NEAR(
        ctx -> ctx.shooter.spinUpFlywheelClose(),
        null
    ),

    SHOOT_FAR(
        ctx -> ctx.shooter.spinUpFlywheelFar(),
        null
    ),

    INC_FLYWHEEL_VEL(
        ctx -> ctx.shooter.incrementFlywheelVel(),
        null
    ),

    DEC_FLYWHEEL_VEL(
        ctx -> ctx.shooter.decrementFlywheelVel(),
        null
    ),

    FIRE_ONE(
        ctx -> ctx.shooter.singleShot(),
        null
    ),

    FIRE_CONTINUOUS(
        ctx -> ctx.shooter.shootContinuous(),
        ctx -> ctx.shooter.stopShooting()
    ),

    STOP_SHOOTER(
        ctx -> ctx.shooter.shutdownShooter(),
        null
    ),

    // --- Climb ---
    ELEVATOR_UP(
        ctx -> ctx.climb.raiseElevator(),
        ctx -> ctx.climb.stopElevator()
    ),

    ELEVATOR_DOWN(
        ctx -> ctx.climb.lowerElevator(),
        ctx -> ctx.climb.stopElevator()
    ),

    WINCH_UP(
        ctx -> ctx.climb.winchUp(),
        ctx -> ctx.climb.stopWinch()
    ),

    WINCH_DOWN(
        ctx -> ctx.climb.winchDown(),
        ctx -> ctx.climb.stopWinch()
    ),

    STOW_ELEVATOR(
        ctx -> ctx.climb.stowElevator(),
        null
    ),

    STOW_WINCH(
        ctx -> ctx.climb.stowWinch(),
        null
    ),

    EMIT_RESET_ALL(
        ctx -> ctx.resetAllMechanisms(),
        null
    );

    public final Consumer<UIContext> onRise;
    public final Consumer<UIContext> onFall;

    Action(Consumer<UIContext> onRise, Consumer<UIContext> onFall) {
        this.onRise = onRise;
        this.onFall = onFall;
    }
}
