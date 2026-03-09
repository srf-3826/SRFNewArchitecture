package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ui.Action;
import frc.robot.ui.ContinuousAction;

/** 
 * Creates a new MechanismSubsystem, but it is not meant for
 * stand alone use. It is instead a new base class which should be
 * extended by any ActionableSubsystem (instead of extending SubsystemBase
 * directly).
 * It will provide a subsystem with the necessary hooks for polled UI actions.
*/
public abstract class ActionableSubsystem extends SubsystemBase {

    protected ContinuousAction m_currentHelper = null;
    protected Action m_currentAction = Action.NONE;

    /** Called when a continuous helper begins. */
    public void startContinuous(Action action, ContinuousAction helper) {
        m_currentHelper = helper;
        m_currentAction = action;
        helper.start();
    }

    /** Called every loop by ActionManager to allow auto-termination. */
    public void updateContinuous() {
        if (m_currentHelper != null && m_currentHelper.isFinished()) {
            m_currentHelper.stop();
            m_currentHelper = null;
            m_currentAction = Action.NONE;
        }
    }

    /** Called when the action ends (button released). */
    public void stopContinuous(Action action) {
        if (m_currentHelper != null && m_currentAction == action) {
            m_currentHelper.stop();
            m_currentHelper = null;
            m_currentAction = Action.NONE;
        }
    }

    /** Optional override for subsystems that need to cancel one-shot actions. */
    public void cancelOneShot(Action action) {
        // default: do nothing
    }

    public boolean clearOnAutoTerminate(Action action) {
      return true;
    }
}
