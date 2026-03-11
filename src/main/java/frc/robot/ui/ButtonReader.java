package frc.robot.ui;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * ButtonReader provides raw access to the driver's controller.
 * It contains NO mode logic, NO edge detection, and NO action mapping.
 */

public class ButtonReader {

    private final CommandXboxController controller;

    public ButtonReader(CommandXboxController controller) {
        this.controller = controller;
    }

    public boolean A()  { return controller.a().getAsBoolean(); }
    public boolean B()  { return controller.b().getAsBoolean(); }
    public boolean X()  { return controller.x().getAsBoolean(); }
    public boolean Y()  { return controller.y().getAsBoolean(); }

    public boolean LB() { return controller.leftBumper().getAsBoolean(); }
    public boolean RB() { return controller.rightBumper().getAsBoolean(); }

    public boolean LT() { return controller.leftTrigger().getAsBoolean(); }
    public boolean RT() { return controller.rightTrigger().getAsBoolean(); }
}
