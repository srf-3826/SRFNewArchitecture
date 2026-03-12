package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autos.drivehelpers.ADAction;
import frc.robot.autos.drivehelpers.ContinuousAction;

/** 
 * Creates a new MechanismSubsystem, but it is not meant for
 * stand alone use. It is instead a new base class which should be
 * extended by any ActionableSubsystem (instead of extending SubsystemBase
 * directly).
 * It will provide a subsystem with the necessary hooks for auto Drive Assist actions.
*/
public abstract class ActionableSubsystem extends SubsystemBase {

    protected ContinuousAction m_currentHelper = null;
    protected ADAction m_currentAction = ADAction.NONE;

    /** Called when a continuous helper begins. */
    public void startContinuous(ADAction action, ContinuousAction helper) {
        m_currentHelper = helper;
        m_currentAction = action;
        helper.start();
    }

    /** Called every loop by ActionManager to allow auto-termination. */
    public void updateContinuous() {
        if (m_currentHelper != null && m_currentHelper.isFinished()) {
            m_currentHelper.stop();
            m_currentHelper = null;
            m_currentAction = ADAction.NONE;
        }
    }

    /** Called when the action ends (button released). */
    public void stopContinuous(ADAction action) {
        if (m_currentHelper != null && m_currentAction == action) {
            m_currentHelper.stop();
            m_currentHelper = null;
            m_currentAction = ADAction.NONE;
        }
    }
}
