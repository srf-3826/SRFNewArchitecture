package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.autos.drivehelpers.DriveBlender;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ui.UIJoystickSupport;

// Uncomment if slew rate limiting is desired
//  import edu.wpi.first.math.filter.SlewRateLimiter;

public class DefaultDriveCmd extends Command {
    private CommandXboxController           m_xbox;
    private DriveBlender                    m_driveBlender = new DriveBlender(); 
    private SwerveSubsystem                 m_swerveDrive;
    // Optional: uncomment here and below if slew rate limiting is desired
    // private SlewRateLimiter                 m_translateSRLimiter;
    // private SlewRateLimiter                 m_strafeSRLimiter;
    // private SlewRateLimiter                 m_rotateSRLimiter;

    public DefaultDriveCmd( CommandXboxController xbox,
                            SwerveSubsystem swerveDriveSubsys) {
        m_xbox = xbox;
        m_swerveDrive = swerveDriveSubsys;
        addRequirements(swerveDriveSubsys);

        // If slew rate llimiting is desired, uncomment and tune the rate limits here
        // m_translateSRLimiter = new SlewRateLimiter(0.5);
        // m_strafeSRLimiter = new SlewRateLimiter(0.5);
        // m_rotateSRLimiter = new SlewRateLimiter(0.5);
    }

    @Override
    public void execute() {
        // Get manual chassis speed components, then apply smoothing
        // SmoothJoystick applies Deadband, then squares the input values, 
        // preserving sign. 
        double translateVal = UIJoystickSupport.smoothJoystick(-m_xbox.getLeftY());
        double strafeVal = UIJoystickSupport.smoothJoystick(-m_xbox.getLeftX());
        double rotateVal = UIJoystickSupport.smoothJoystick(-m_xbox.getRightX());
        // Optionally, apply slewRateLimiters?
        // translateVal = m_translateSRLimiter.calculate(translateVal);
        // strafeVal = m_strafeSRLimiter.calculate(strafeVal);
        // rotateVal = m_rotateSRLimiter.calculate(rotateVal);

        // Assenble into mannualSpeeds, in meters and radians per sec:
        ChassisSpeeds manualSpeeds = new ChassisSpeeds(translateVal * SDC.MAX_ROBOT_SPEED_M_PER_SEC,
                                                       strafeVal * SDC.MAX_ROBOT_SPEED_M_PER_SEC,
                                                       rotateVal * SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC);
    
        // Fetch any autoSpeeds that may be active (again meters and radians per sec, will be 0.0 if none)
        ChassisSpeeds autoSpeeds = m_swerveDrive.getAutoDriveAssistSpeeds();

        // Blend the manual and auto speeds (all robot relative at this point)
        // If field relative speeds are needed the conversion will be done by swerve.drive() 
        ChassisSpeeds blendedSpeeds = m_driveBlender.blend(manualSpeeds, autoSpeeds);

        // Finally, pass the RO blended speeds to swerve.drive()
        // Because this command is at its core manual (even with autoDrive helpers) the control
        // output that it generates is always processed as "openLoop" by the swerve subsystem
        m_swerveDrive.drive(blendedSpeeds, true);
    }
}