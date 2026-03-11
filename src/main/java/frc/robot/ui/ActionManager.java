package frc.robot.ui;

import java.util.List;

/**
 * ActionManager executes both button-driven and system-driven Actions.
 * 
 * Responsibilities:
 *  - Drain all pending actions (no edge detection) per loop
 *  - Drain any button actions (with edge detection), max one per loop
 *  - Process falling edges first
 *  - Process rising edges second
 *  - Call onRise/onFall on each Action
 *  - Pass UIContext to each action
 */

public class ActionManager {

    private final UIContext context;
    private final ButtonActionManager buttonActions;
    private final SystemActionManager systemActions;

    public ActionManager(
        UIContext context,
        ButtonActionManager buttonActions,
        SystemActionManager systemActions
    ) {
        this.context = context;
        this.buttonActions = buttonActions;
        this.systemActions = systemActions;
    }

    public void update() {

        // 1. System actions (no edge detection)
        List<Action> sys = systemActions.drain();
        for (Action a : sys) {
            if (a.onRise != null) {
                a.onRise.accept(context);
            }
        }

        // 2. Button falling edge
        Action falling = buttonActions.getFallingEdge();
        if (falling != null && falling.onFall != null) {
            falling.onFall.accept(context);
        }

        // 3. Button rising edge
        Action rising = buttonActions.getRisingEdge();
        if (rising != null && rising.onRise != null) {
            rising.onRise.accept(context);
        }

        // 4. Prepare for next cycle
        buttonActions.endCycle();
    }
}


