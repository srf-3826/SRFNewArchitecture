package frc.robot.ui;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;

public enum Action {

    // ===========================
    // Default Action
    // ===========================

    NONE(
        UI_Mode.ANY,
        Buttons.NONE,
        0,
        null,        // helperFactory
        Set.of(),
        null,               // onRise
        null),              // onFall

    // ============================
    // Actions handled at a System level 
    // either entirely within SwerveMnager in 
    // support of AutoDriveHelpers,
    // or actions that affect more than one
    // subsystem. allowing cross subsystem
    // communication/syncronization
    //=============================
    AUTO_AIM(
        UI_Mode.SHOOTING,
        Buttons.NONE,
        0,
        null,
        Set.of(),
        null,
        null),

    AUTO_RANGE(
        UI_Mode.SHOOTING,
        Buttons.NONE,
        0,
        null,
        Set.of(),
        null,
        null),

    AUTO_STRAFE(
        UI_Mode.SHOOTING,
        Buttons.NONE,
        0,
        null,
        Set.of(),
        null,
        null),

    AUTO_ALIGN_FEED_STATION(
        UI_Mode.INTAKING,                           // Actually only used for dumping support, not intake
        Buttons.NONE,
        0,
        null,
        Set.of(),
        null,
        null),

    STOW_ALL_MECHANISMS(                           // Triggered by mode changes
        UI_Mode.ANY,
        Buttons.NONE,                             
        11,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.retractIntake(),        // Dump first? Stop rollers, fully retract),
        null),

    START_BED_ROLLERS_IN(                         // Triggered in both Intaking and shooting modes
        UI_Mode.ANY,
        Buttons.NONE,
        15,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.startBedRollersIn(),
        null),

    START_BED_ROLLERS_OUT(
        UI_Mode.ANY,
        Buttons.NONE,
        16,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.startBedRollersOut(),
        null),

    STOP_BED_ROLLERS(
        UI_Mode.ANY,
        Buttons.NONE,
        14,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.stopBedRollers(),
        null),

    // ============================
    // INTAKING MODE ACTIONS
    // ============================

    START_SLOW_DRIVE_INTAKING(
        UI_Mode.INTAKING,
        Buttons.RB,                             // while held
        12,
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.setVarMaxOutputFactor(0.5),
        null),

    CANCEL_SLOW_DRIVE_INTAKING(
        UI_Mode.INTAKING,
        Buttons.RB,                             // upon release
        13,
        null,
        Set.of(SubsystemType.DRIVE),
        null,
        ctx -> ctx.drive.setVarMaxOutputFactor(1.0)),

    INTAKE_STOP_AND_HOLD(
        UI_Mode.INTAKING,
        Buttons.A,                                  // momentary press
        28,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.stopAndPivotToHold(),
        null),

    DUMP_FUEL(                                    // Extend and reverse rollers to dump fuel
        UI_Mode.INTAKING,
        Buttons.ALT | Buttons.RT,                 // while held
        32,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.dumpFuel(),
        null),

    STOP_FUEL_DUMPING(                            // Extend and reverse rollers to dump fuel
        UI_Mode.INTAKING,
        Buttons.ALT | Buttons.RT,                 // upon release
        31,
        null,
        Set.of(SubsystemType.INTAKE),
        null,
        ctx -> ctx.intake.stopAndPivotToHold()),

    INTAKE_START(                                   // Extend and start rollers for intake
        UI_Mode.INTAKING,
        Buttons.Y,                                  // momentary press
        30,
        null,
        Set.of(SubsystemType.INTAKE),
        ctx -> ctx.intake.startIntake(),
        null),

    // ============================
    // SHOOTING MODE ACTIONS
    // ============================

    ENABLE_AUTO_DRIVE_SHOOT_ASSIST(
        UI_Mode.SHOOTING,
        Buttons.B,                              // Enable auto assist with Button B
        10,
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.setShootingAutoDriveOK(),
        null),

    DISABLE_AUTO_DRIVE_SHOOT_ASSIST(
        UI_Mode.SHOOTING,
        Buttons.ALT | Buttons.B,                // Disable Auto Assist with ALT - B
        11,
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.clearShootingAutoDriveOK(),
        null),

    START_SLOW_DRIVE_SHOOTING(
        UI_Mode.SHOOTING,
        Buttons.RB,                             // while held
        12,
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.setVarMaxOutputFactor(0.4),
        null),

    CANCEL_SLOW_DRIVE_SHOOTING(
        UI_Mode.SHOOTING,
        Buttons.RB,                             // upon release
        13,
        null,
        Set.of(SubsystemType.DRIVE),
        null,
        ctx -> ctx.drive.setVarMaxOutputFactor(1.0)),

    SHOOT_ONE(
        UI_Mode.SHOOTING,
        Buttons.LT,                             // momentary press
        20,
        null,
        Set.of(SubsystemType.SHOOTER, SubsystemType.INTAKE),
        ctx -> ctx.shooter.singleShot(),
        null),
        
    SHOOT_CONTINUOUS(
        UI_Mode.SHOOTING,
        Buttons.RT,                             // while held
        37,
        null,
        Set.of(SubsystemType.SHOOTER, SubsystemType.INTAKE),
        ctx -> ctx.shooter.shootContinuous(),
        null),

    STOP_SHOOTING(
        UI_Mode.SHOOTING,
        Buttons.RT,                             // RT release
        31,
        null,
        Set.of(SubsystemType.SHOOTER, SubsystemType.INTAKE),
        null,
        ctx -> ctx.shooter.stopShooting()),      // On release, stop feed rollers, maybe bed rollers

   SET_FAR_SHOT(
        UI_Mode.SHOOTING,
        Buttons.Y,                               // momentary press (optional)
        38,
        null,
        Set.of(SubsystemType.SHOOTER),
        ctx -> ctx.shooter.spinUpFlywheelFar(),
        null),
        
    SET_NEAR_SHOT(
        UI_Mode.SHOOTING,
        Buttons.A,                              // momentary press (optional)
        39,
        null,
        Set.of(SubsystemType.SHOOTER),
        ctx -> ctx.shooter.spinUpFlywheelClose(),
        null),

    INCREMENT_SHOOTER_VEL(
        UI_Mode.SHOOTING,
        Buttons.ALT | Buttons.Y,               // momentary press (optional)
        38,
        null,
        Set.of(SubsystemType.SHOOTER),
        ctx -> ctx.shooter.incrementFlywheelVel(),
        null),

    DECREMENT_SHOOTER_VEL(
        UI_Mode.SHOOTING,
        Buttons.ALT | Buttons.A,               // momentary press (optional)
        39,
        null,
        Set.of(SubsystemType.SHOOTER),
        ctx -> ctx.shooter.decrementFlywheelVel(),      // 100 RPM per press
        null),
    
    // ============================
    // NAVIGATING MODE ACTIONS
    // ============================

    START_PRECISION_SLOW_DRIVE_NAV(
        UI_Mode.NAVIGATING,
        Buttons.RB,                             // while held
        12,
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.setVarMaxOutputFactor(0.4),        // Go slow
        null),

    CANCEL_PRECISION_SLOW_DRIVE_NAV(
        UI_Mode.NAVIGATING,
        Buttons.RB,                             // upon release
        13,
        null,
        Set.of(SubsystemType.DRIVE),
        null,
        ctx -> ctx.drive.setVarMaxOutputFactor(1.0)),       // Full speed

    // ============================
    // DEFENSE MODE ACTIONS
    // ============================

    FR_COR(
        UI_Mode.DEFENSE,
        Buttons.RT,                     // while held
        51,
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.setFRCenOfRotation(),
        null),

    FR_COR_EXIT(
        UI_Mode.DEFENSE,
        Buttons.RT,                     // on release
        51,
        null,
        Set.of(SubsystemType.DRIVE),
        null,
        ctx -> ctx.drive.resetCenOfRotation()),
 
    FL_COR(
        UI_Mode.DEFENSE,
        Buttons.LT,                     // while held
        53, 
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.setFLCenOfRotation(),
        null),

    FL_COR_EXIT(
        UI_Mode.DEFENSE,
        Buttons.LT,                     //on release
        53, 
        null,
        Set.of(SubsystemType.DRIVE),
        ctx -> ctx.drive.resetCenOfRotation(),
        null),

    BR_COR(
        UI_Mode.DEFENSE,
        Buttons.RB,                     // while held
        55,
        null,
        Set.of(SubsystemType.DRIVE),
        null,
        null),

    BL_COR(
        UI_Mode.DEFENSE,
        Buttons.LB,                     // while held
        57,
        null,
        Set.of(SubsystemType.DRIVE),
        null,
        null);

    // ============================
    // CLIMBING MODE ACTIONS
    // ============================

    // TBD
 
    // ============================================================
    // ENUM FIELDS
    // ============================================================

    public final UI_Mode requiredMode;
    public final int requiredMask;
    public final int priority;
    public final Function<HelperContext, ContinuousAction> helperFactory;
    public final Set<SubsystemType> subsystems;
    public final Consumer<HelperContext> onRise;
    public final Consumer<HelperContext> onFall;

    Action(UI_Mode mode,
           int mask,
           int priority,
           Function<HelperContext, ContinuousAction> helperFactory,
           Set<SubsystemType> subsystems,
           Consumer<HelperContext> onRise,
           Consumer<HelperContext> onFall) {

        this.requiredMode = mode;
        this.requiredMask = mask;
        this.priority = priority;
        this.helperFactory = helperFactory;
        this.subsystems = subsystems;
        this.onRise = onRise;
        this.onFall = onFall;
    }
}