package frc.robot.ui;

//
// This class has 2 purposes:
// 1. Keep track of current mode, and support Mode change
//    (including any System level Action housekeeping per transition,
//    emitting (pushing) as many as needed to the systemActionManager 
//    in an unbounded queue, with no edge detection).
//    ActionManager retrieves the queue and empties it per loop, 
//    processing in that same loop as many System Actions as are present.  
// 2. Poll the game controller for pressed buttons, once per loop via the
//    pollButtons() method, called from RobotContainer. The first found will
//    be mapped to an Action, which is emitted (pushed) to buttonActionManager,
//    which in turn detects Action edges. This ensures at most one Action per
//    risingEdge and one action per fallingEdge, which Actions are
//    provided to ActionManager.
//
public class ModeManager {
    private final ButtonReader          m_buttonReader;
    private final ButtonActionManager   m_buttonActions;
    private final SystemActionManager   m_systemActions;

    
    private UI_Mode m_mode = UI_Mode.DRIVE;

    public ModeManager( ButtonReader m_buttonReader,
                        ButtonActionManager buttonActions,
                        SystemActionManager systemActions) {
        this.m_buttonReader = m_buttonReader;
        this.m_buttonActions = buttonActions;
        this.m_systemActions = systemActions;
    }
/*
 * getters for current UI_Mode
 */
    public UI_Mode getMode()       { return m_mode; }
    public boolean isIntakeMode()  { return m_mode == UI_Mode.INTAKE; }
    public boolean isShootMode()   { return m_mode == UI_Mode.SHOOT; }
    public boolean isClimbMode()   { return m_mode == UI_Mode.CLIMB; }
    public boolean isDefenseMode() { return m_mode == UI_Mode.DEFENSE; }

    /*
     * setMode is the single way to change Modes. That makes it a central point
     * for ensureing all new Modes get logged, but it also allows it to
     * do any convenient housekeeping for New modes, such as stowing mechanisms
     * that mmight still be extended, but which are inappropriate to the
     * new Mode, or in the case of leving DEFENSE, where SLOW mode is a toggle
     * driven by a pair of buttons (instead of a while held Action), ensure that
     * it does not leave in the SLOW state. 
     */
    public void setMode(UI_Mode newMode) {
        if (newMode == m_mode) return;
            
        // If leaving DEFENSE, ensure robot is not going to be left in GO_SLOW state
        if (m_mode == UI_Mode.DEFENSE) {
            m_systemActions.emit(Action.DEFENSE_GO_FAST);
        }

        m_mode = newMode;

        switch (newMode) {
            case INTAKE:
                // Intake Prep
                m_systemActions.emit(Action.START_INTAKE);
                m_systemActions.emit(Action.STOW_ELEVATOR);
                m_systemActions.emit(Action.STOW_WINCH);
                break;

            case SHOOT:
                // Shooter Prep
                m_systemActions.emit(Action.PIVOT_TO_HOLD);
                m_systemActions.emit(Action.STOW_WINCH);
                m_systemActions.emit(Action.STOW_ELEVATOR);
                break;

            case CLIMB:
                // Climb Prep
                m_systemActions.emit(Action.RETRACT_INTAKE);
                m_systemActions.emit(Action.STOP_SHOOTER);
                break;

            case DEFENSE:
                // Defense Prep
                m_systemActions.emit(Action.RETRACT_INTAKE);
                m_systemActions.emit(Action.STOP_SHOOTER);
                m_systemActions.emit(Action.STOW_WINCH);
                m_systemActions.emit(Action.STOW_ELEVATOR);
                break;

            case DRIVE:
            default:
                // No special prep; could add SHOOTER idle, etc., if desired
                break;
        }
    }

/*
 * pollButtons is the main UI reader for operator intent. The only other place
 * where buttons are handled in in RobotContainer, where ButtonBindings
 * handles the few buttons that are reserved as triggers, rather then being
 * polled here. This is best because those buttons apply in ALL Modes, Always,
 * regardless of season.
 */
    public void pollButtons() {
        
        boolean alt = m_buttonReader.LB();  // Modifier/shift button while held (in all modes but DEFENSE)
        Action a;                           // temp var, convenient to hold Action after applying Alt filter

        // --- Global: AUTO_DRIVE_ON / OFF (all modes, including DEFENSE) ---
        // COuld be moved to configureButtonBindings!
        if (m_buttonReader.B()) {
            a = alt ? Action.AUTO_DRIVE_OFF : Action.AUTO_DRIVE_ON;
            m_buttonActions.emit(a);
        }

        // --- DEFENSE mode overrides most UI ---
        if (m_mode == UI_Mode.DEFENSE) {
            // ALT is ignored in defense (i.e. LB has no modifier meaning here)
            if (m_buttonReader.A()) { m_buttonActions.emit(Action.DEFENSE_GO_SLOW); }
            if (m_buttonReader.Y()) { m_buttonActions.emit(Action.DEFENSE_GO_FAST); }
            if (m_buttonReader.X()) { m_buttonActions.emit(Action.PARK); }
            // COR in defense
            if (m_buttonReader.LT()) { m_buttonActions.emit(Action.FL_COR); }
            if (m_buttonReader.RT()) { m_buttonActions.emit(Action.FR_COR); }
            if (m_buttonReader.LB()) { m_buttonActions.emit(Action.BL_COR); }
            if (m_buttonReader.RB()) { m_buttonActions.emit(Action.BR_COR); }
            return;
        }

        /*
         *  Semi Global UI:
         */
        // Button RB - Slow driving - all modes except DEFENSE
        if (m_buttonReader.RB()) {
            a = alt ? Action.GO_SUPER_SLOW : Action.GO_SLOW;
            m_buttonActions.emit(a);
        }

        // Buttons LT and RT, plus Alt - COR shifts - all Modes except DEFENSE and SHOOT
        if (m_mode != UI_Mode.SHOOT) {
            if (m_buttonReader.LT()) {
                a = alt ? Action.BL_COR : Action.FL_COR;
                m_buttonActions.emit(a);
            }
            if (m_buttonReader.RT()) {
                a = alt ? Action.BR_COR : Action.FR_COR;
                m_buttonActions.emit(a);
            }
        }

        // --- Mode-specific face buttons ---
        switch (m_mode) {

            case INTAKE:
                // Button A
                if (m_buttonReader.A()) {
                    a = alt ? Action.RETRACT_INTAKE : Action.PIVOT_TO_HOLD;
                    m_buttonActions.emit(a);
                }
                // Button X
                if (m_buttonReader.X()) {
                    a = alt ? Action.EMIT_RESET_ALL : Action.PARK;
                    m_buttonActions.emit(a);
                }
                // Button Y
                if (m_buttonReader.Y()) {
                    a = alt ? Action.DUMP_FUEL : Action.START_INTAKE;
                    m_buttonActions.emit(a);
                }
                // Button B - handled globally for autodriveAssist
                // Buttons LT and RT - handled semi-globally for COR shifts
                break;

            case SHOOT:
                // Button A
                if (m_buttonReader.A()) {
                    a = alt ? Action.DEC_FLYWHEEL_VEL : Action.SHOOT_NEAR;
                    m_buttonActions.emit(a);
                }
                // Button X
                if (m_buttonReader.X()) {
                    a = alt ? Action.EMIT_RESET_ALL : Action.STOP_SHOOTER;
                    m_buttonActions.emit(a);
                }
                if (m_buttonReader.Y()) {
                    // Button Y
                    a = alt ? Action.INC_FLYWHEEL_VEL : Action.SHOOT_FAR;
                    m_buttonActions.emit(a);
                }
                // Button B - handled globally for autodriveAssist
                // Buttons LT and RT - no Alt action
                if (m_buttonReader.LT()) { m_buttonActions.emit(Action.FIRE_ONE); }
                if (m_buttonReader.RT()) { m_buttonActions.emit(Action.FIRE_CONTINUOUS);
                }
                break;

            case CLIMB:
                // Button A
                if (m_buttonReader.A()) {
                    a = alt ? Action.ELEVATOR_DOWN : Action.ELEVATOR_UP;
                    m_buttonActions.emit(a);
                }
                // Button X
                if (m_buttonReader.X()) {
                    a = alt ? Action.EMIT_RESET_ALL : Action.STOW_ELEVATOR;
                    m_buttonActions.emit(a);
                }
                // Button Y
                if (m_buttonReader.Y()) {
                    a = alt ? Action.WINCH_DOWN : Action.WINCH_UP;
                    m_buttonActions.emit(a);
                }
                // Button B - handled globally for autodriveAssist
                // Buttons LT and RT - handled semi-globally for COR shifts
                break;

            case DRIVE:
            default:
                // DRIVE mode: only globals and COR apply
                break;
        }
    }
}
