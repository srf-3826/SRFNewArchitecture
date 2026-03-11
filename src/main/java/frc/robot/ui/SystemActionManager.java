package frc.robot.ui;

import java.util.ArrayList;
import java.util.List;

/**
 * SystemActionManager is a simple queue for system-level Actions.
 *
 * These actions:
 *  - are NOT triggered by buttons
 *  - bypass edge detection
 *  - run their onRise immediately when drained by ActionManager
 *  - may have onFall (rare, but supported)
 *  - are consumed once per cycle
 */

public class SystemActionManager {

    private final List<Action> pending = new ArrayList<>();

    public void emit(Action action) {
        if (action != null) {
            pending.add(action);
        }
    }

    public List<Action> drain() {
        List<Action> out = new ArrayList<>(pending);
        pending.clear();
        return out;
    }
}
