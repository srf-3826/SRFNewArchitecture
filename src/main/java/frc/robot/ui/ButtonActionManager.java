package frc.robot.ui;

/**
 * ButtonActionManager receives button-driven Actions from ModeManager
 * and detects rising/falling edges for ActionManager to execute.
 */

public class ButtonActionManager {

    private Action m_previous = null;
    private Action m_current  = null;

    // emit is a consumer method for button Actions pushed by ModeManager
    public void emit(Action action) {
        m_current = action;
    }

    // Any new Actions are obtained by ActionManager on the rising edge
    public Action getRisingEdge() {
        if (m_current != m_previous && m_current != null) {
            System.out.println("Button "+""+" pressed");
            return m_current;
        }
        return null;
    }

    // Terminating Actions are not obtained by ActionManager until the falling edge
    public Action getFallingEdge() {
        if (m_current != m_previous && m_previous != null) {
            System.out.println("Button "+""+" released");
            return m_previous;
        }
        return null;
    }

    public void endCycle() {
        m_previous = m_current;
        m_current  = null;
    }
}
