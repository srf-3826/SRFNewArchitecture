// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ui;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
    There are two ModeManager functions:
    1. Change and keep track of UI modes
    2. Read Xbox controller inputs, and map those to an Action in an enum set of Actions.
    To do the later, it reads the current button(s) and forms those that are pressed 
    into a bit Mask. It checks that mask against all Actions registered in the Action enum 
    with both a matching required bit mask, and a requiredMode that matches the active Mode.
    It returns either Action.NONE, or a single Action that is the 
    highest priority Action within the set of matching Actions, if more than one. 
    It is up to the ActionManager to sort out the semantics, state, rising and 
    falling edges of any Actions returned. ModeManager getAction() is just a mapping funtion.
    The only other function of ModeManager is to accept Mode changes. This is the only place
    to do that. When the mode is changed, it should ensure machanisms are correctly
    stowed (retracted) for the new context (via the SystemActionManager). However, it will always 
    remain up to the drive team to explicitly command all mechanism deployments as needed. 
*/

public class ModeManager {
    private CommandXboxController m_xbox;
    private UI_Mode m_mode;

    public ModeManager(CommandXboxController xboxController) {
        m_xbox = xboxController;
        m_mode = UI_Mode.NAVIGATING;
    }
    
    // Mode setter and getters
    public void setMode(UI_Mode mode) {
        if (m_mode != mode) {

            // When changing modes, it is convenient to automatically take care of
            // mechanism housekeeping.
            if (mode == UI_Mode.DEFENSE) {
                // When entering DEFENSE mode, stow everything in safe positions
                // systemActionManager.emit(Action.STOW_ALL_MECHANISMS);
            } else if (m_mode == UI_Mode.INTAKING) {
                // Current mode INTAKING implies the intake is extended. Pull it back
                // when entering any other mode.
                // systemActionManager.emit(Action.STOP_AND_PIVOT_TO_HOLD);
            }
            // Add mode change logging here. 
            m_mode = mode;
        }
     }

    public UI_Mode getMode() {
        return m_mode;
    }

    // isShootingMode is provided for swerveSubsystem to help decide whether 
    // to start or stop AutoDriveAssistHelpers. 
    public boolean isShootingMode() {
        return m_mode == UI_Mode.SHOOTING;
    }

    public boolean isIntakingMode() {
        return m_mode == UI_Mode.INTAKING;
    }

    /*
     *  Handle butotn polling: Pure mapping: button(s) → semantic action
     *  Return only one Action enum per cycle. Returns Action.NONE if
     *  no button is pressed.
     */
    public Action getAction() {
        Action best = null;

        // To poll buttons, form a bitmap of all currently pressed. Note that 
        // m_xbox.someButton().getAsBoolean() always returns true if the button is 
        // pressed. Unlike triggers, rising or falling edges are immaterial here.
        // (keep in mind that while actionManager will track rising and falling 
        // edges and the state, it does so for each returned Action, not for the
        // buttons which generated those Actions. And for architectural intents
        // and purposes, ModeManager knows nothing about ActionManager).
        int mask = 0;
        if (m_xbox.a().getAsBoolean())              mask |= Buttons.A;
        if (m_xbox.b().getAsBoolean())              mask |= Buttons.B;
        if (m_xbox.x().getAsBoolean())              mask |= Buttons.X;
        if (m_xbox.y().getAsBoolean())              mask |= Buttons.Y;
        if (m_xbox.leftTrigger().getAsBoolean())    mask |= Buttons.LT;         // default is true if > 50% pressed
        if (m_xbox.rightTrigger().getAsBoolean())   mask |= Buttons.RT;         // default is true if > 50% pressed
        if (m_xbox.leftBumper().getAsBoolean())     mask |= Buttons.ALT;        // LB
        if (m_xbox.rightBumper().getAsBoolean())    mask |= Buttons.RB;
        if (m_xbox.start().getAsBoolean())          mask |= Buttons.START;

        if (mask == Buttons.NONE) {         // None pressed might be the most likely case? Either way, 
                                            // can return now for slightly better efficiency. Returning 
                                            // now also avoids any confusion with Actions having Buttons.NONE 
                                            // requiredMask entries in the Action enum, which now 
                                            // represent System level Actions.
            return Action.NONE;
        }
/*
        Now scan the Action enum for all possible actions, and return the highest priority 
        Action whose requiredMask matches the just read button mask, and whose requiredMode 
        matches the current mode. 
        For the algorithm used here, any prior active Action will change to 
        Action.NONE when released. Additional button(s) pressed will result in a different
        mask, so that too will result in changing either to Action.NONE, or to a new Action
        matching the different mask.
        AactionManager will not be able to tell why an Action changed, and doesn't need to care,
        It just identifies it as a release (falling edge) of any prior action. 
        In both such cases the new Action will be returned.
*/
        for (Action a : Action.values()) {
            if ((a.requiredMode != m_mode) && (a.requiredMode != UI_Mode.ANY)) continue;
            if ((mask & a.requiredMask) != a.requiredMask) continue;

            if (best == null || a.priority < best.priority) {       // Lower numbers are higher priority
                best = a;
            }
        }

        return best;
    }
}