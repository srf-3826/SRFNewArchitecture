package frc.robot.ui;

import java.util.EnumMap;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ActionableSubsystem;

public class ActionManager {

    private final HelperContext m_ctx;
    private final RobotContainer m_rContainer;

    private final EnumMap<Action, Boolean> lastPressed = new EnumMap<>(Action.class);

    public ActionManager(HelperContext ctx, RobotContainer container) {
        this.m_ctx = ctx;
        this.m_rContainer = container;

        for (Action a : Action.values()) {
            lastPressed.put(a, false);
        }
    }

    public void process(Action action) {

        boolean isPressed = action != Action.NONE;
        boolean wasPressed = lastPressed.get(action);

        boolean rising = !wasPressed && isPressed;
        boolean falling = wasPressed && !isPressed;

        // TODO: POTENTIAL BUG
        // Before overwriting lastPressed, need to check to see
        // if it was some prior Action, or Action.NONE. If the former, 
        // it may need a handleFalling(lastPressed) first, before doing
        // handleRising(action)
        lastPressed.put(action, isPressed);

        if (rising) handleRising(action);
        if (falling) handleFalling(action);
    }

    private void handleRising(Action action) {

        if (action.onRise != null) {
            action.onRise.accept(m_ctx);
            return;
        }

        if (action.helperFactory != null) {
            var helper = action.helperFactory.apply(m_ctx);

            for (var type : action.subsystems) {
                ActionableSubsystem subsystem = m_rContainer.subsystemFor(type);
                subsystem.startContinuous(action, helper);
            }
        }
    }

    private void handleFalling(Action action) {
        if (action.onFall != null) {
            action.onFall.accept(m_ctx);
            return;                     // TODO: potential bug. A return here rules out
                                        // Actions with simultaneous falling edge AND
                                        // continuous properties. On the other hand, 
                                        // such a continuous action, essentially a 
                                        // whileheld action, will have its own stopContinuous 
                                        // action, so the rule could easily be exclusionary
                                        // letting the return stand.
        }

        for (var type : action.subsystems) {
            ActionableSubsystem subsystem = m_rContainer.subsystemFor(type);
            subsystem.stopContinuous(action);
            subsystem.cancelOneShot(action);        // TODO: eliminate cancelOneShot - now redundant
                                                    // given the change to onRise / onFall enum fields
        }
    }

    /** Called every loop to allow mechanism helpers to auto-terminate. */
    public void update() {
        for (ActionableSubsystem subsystem : m_rContainer.allActionableSubsystems()) {
            subsystem.updateContinuous();
        }
    }
}