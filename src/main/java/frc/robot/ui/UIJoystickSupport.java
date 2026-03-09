package frc.robot.ui;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.*;

public class UIJoystickSupport {

    private UIJoystickSupport() {}  // prevent any instances from being created
                        
    public static double smoothJoystick( double joystickValue, double deadBand ) {
        joystickValue = (MathUtil.applyDeadband(joystickValue, deadBand));
        return ((joystickValue * joystickValue) * Math.signum(joystickValue));
    }

    public static double smoothJoystick( double joystickValue ) {
        joystickValue = (MathUtil.applyDeadband(joystickValue, UIC.JOYSTICK_DEADBAND));
        return ((joystickValue * joystickValue) * Math.signum(joystickValue));
    }

    public static double clampRange( double value, double range_min, double range_max ) {
        if (value < range_min) {
            //SmartDashboard.putNumber("Clamp min applied", value);
            value = range_min;
        } else if (value > range_max) {
            //SmartDashboard.putNumber("Clamp max applied", value);
            value = range_max;
        }
        return value;
    }

    public static double clampMax( double varValue, double varMax ) {
        if (Math.abs(varValue) > varMax ) {
            varValue = varMax * Math.signum(varValue);
        }
        return varValue;
    }
}
