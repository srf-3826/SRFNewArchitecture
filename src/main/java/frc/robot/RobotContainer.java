package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.autos.drivehelpers.ADContext;

// Uncomment of any autos actually used
// import frc.robot.autos.*;

import frc.lib.sensors.GyroIO;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.ui.UI_Mode;
import frc.robot.ui.SystemActionManager;
import frc.robot.ui.UIContext;
import frc.robot.ui.ModeManager;
import frc.robot.ui.ActionManager;
import frc.robot.ui.ButtonActionManager;
import frc.robot.ui.ButtonReader;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private CANBus swerveCanbus = new CANBus(Constants.CAN_BUS_FOR_SWERVE);
    private CANBus allElseCanbus = new CANBus(Constants.CAN_BUS_FOR_EVERYTHING_ELSE);

    // Declare subsystem and supporting object handles
    private GyroIO                      m_gyroIO;
    private SwerveDrivePoseEstimator    m_swerveDrivePoseEstimator;
 
    private SwerveSubsystem             m_swerveSubsystem;
    private IntakeSubsystem             m_intakeSubsystem;
    private ShooterSubsystem            m_shooterSubsystem;
    private ClimberSubsystem            m_climberSubsystem;   
    private VisionSubsystem             m_visionSubsystem;

    // Declare UI Support managers and members
    private UIContext                                   m_ctx;
    private ButtonReader                                m_buttonReader;
    private ModeManager                                 m_modeManager;
    private ButtonActionManager                         m_buttonActionManager;
    private SystemActionManager                         m_systemActionManager;
    private ActionManager                               m_actionManager;

    // Declear auto Drive assist components
    private ADContext                                   m_adCtx;

    // Declare choosable autonomous Commands and any other Commands used with ButtonBindings
    private DoNothingCmd                m_doNothingCmd;
    private SwerveParkCmd               m_parkCmd;

    // Create sendable choosers for starting position and desired Auto routine
    private static SendableChooser<Command> m_autoRoutineChooser = new SendableChooser<>();

    // Declare CommandXboxController
    private static CommandXboxController m_xbox = new CommandXboxController(0);

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        // Context for ADAction enum, swerveSubsystem, and auto drive helpers 
        m_adCtx = new ADContext( 
            m_swerveSubsystem,
            () -> m_swerveSubsystem.getPose(),
            () -> m_swerveSubsystem.getYaw2d(),
            () -> m_swerveSubsystem.getAutoDriveAssistSpeeds(),
            m_visionSubsystem,
            () -> m_visionSubsystem.getNearestTargetInfo(),
            m_modeManager
        );

        m_gyroIO = new GyroIO(swerveCanbus, GC.PIGEON_2_CANID, GC.INVERT_GYRO);
        m_swerveSubsystem = new SwerveSubsystem(swerveCanbus,
                                                m_gyroIO,
                                                m_visionSubsystem,
                                                m_modeManager,
                                                m_adCtx);
        m_intakeSubsystem = new IntakeSubsystem(allElseCanbus);
        m_shooterSubsystem = new ShooterSubsystem(allElseCanbus);
        m_climberSubsystem = new ClimberSubsystem(allElseCanbus);

        m_swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SDC.SWERVE_KINEMATICS, 
                                                                  m_gyroIO.getRotation2d(),
                                                                  m_swerveSubsystem.getModulePositions(),
                                                                  m_swerveSubsystem.getPose());
        m_visionSubsystem = new VisionSubsystem(m_swerveDrivePoseEstimator, m_swerveSubsystem);

        // Commands
        m_swerveSubsystem.setDefaultCommand(new DefaultDriveCmd(m_xbox, 
                                                m_swerveSubsystem));
        // DoNothing Cmd is a placeholder for Auto routines
        m_doNothingCmd = new DoNothingCmd();
 
        // Park Cmd exits on any joystick input, so need to pass it all joystick input lambdas
        m_parkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                      () -> -m_xbox.getLeftY(),
                                      () -> -m_xbox.getLeftX(),
                                      () -> -m_xbox.getRightX());
        /*
            Polling UI Support
        */
        m_buttonReader = new ButtonReader(m_xbox);
        m_buttonActionManager = new ButtonActionManager();
        m_systemActionManager = new SystemActionManager();
        m_modeManager = new ModeManager(m_buttonReader, m_buttonActionManager, m_systemActionManager);
        
        m_ctx = new UIContext( 
            m_swerveSubsystem,
            m_intakeSubsystem,
            m_shooterSubsystem,
            m_climberSubsystem,
            m_systemActionManager,
            m_parkCmd
        );

        m_actionManager = new ActionManager(m_ctx, m_buttonActionManager, m_systemActionManager);

          m_autoRoutineChooser.setDefaultOption("Do nothing", m_doNothingCmd);
        SmartDashboard.putData("Autonomous Selection:", m_autoRoutineChooser);

        configureButtonBindings();
    }

    /**************************************************************
     * Getters for useful objects
     **************************************************************/

     // Mostly the gyro class object is passed to systems that need it
     // but just in case, a (so far unused) getter is provided here.
    public GyroIO getGyroIO() {
        return m_gyroIO;
    }

    // The raw HID XboxCtrl is needed only by the RumbleCmd 
    // Since that Cmd is a type of utility that can be invoked from
    // any subsystem (or other Cmd) it is most efficient to let it
    // fetch the single Xbox object in the system from RobotContainer 
    // via this getter:
    public static XboxController getHidXboxCtrl() {
        return m_xbox.getHID();
    }

    /***********************************************
     * Update() drives the UI Polling every loop.
     * It also ensures all Phoenix6 StatusSignals 
     * in the various subsystems and gyroIO class 
     * are kept refreshed.
     * update() is called from robot.periodic(), 
     * before CommandScheduler.run()
     ************************************************/
    public void update() {
        // If ENABLED, poll all global mode sensitive UI Actions, 
        // and handle lifeCycle of Continuous Actions
        if (DriverStation.isTeleopEnabled()) {
            m_modeManager.pollButtons();
            m_actionManager.update();        
        }

        // Now ensure all StatusSignals stay up to date.
        m_gyroIO.update();
        m_swerveSubsystem.update();
        m_intakeSubsystem.update();
        m_shooterSubsystem.update();
        m_climberSubsystem.update();

        // This would be a good place to log any RobotContainer data of interest
    }

    private void configureButtonBindings() {
        // The assignments that govern the XboxController UI for the Rebuilt Season
        // can mostly be found in ModeManager, where a polling architecture is implemented.
        // But the ButtonBindings here create a hybrid UI, where the following pemanently assigned 
        // and (almost) never ALT modified Actions are bound here:
    
        //    L Joystick Button     => Set Field Oriented drive. If already FO, ignore.
        //    R Joystick Button     => Set Robot Oriented drive. If already RO, ignore.
        //    Back                  => Zero the Gyro
        //    POV_UP                => SCORING Mode
        //    POV_DOWN              => DEFENSE Mode
        //    POV_LEFT              => INTAKING Mode
        //    POV_RIGHT             => NAVIGATING Mode
        //    ALT + POV RIGHT       => (possibly) CLIMBING Mode

        // Left joystick button sets field oriented driving
        m_xbox.leftStick().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        // Right joystick button sets robot oriented driving
        m_xbox.rightStick().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        
        // Back button Zeros the Gyro
        m_xbox.back().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

        //
        // D-Pad ordinal buttons set the currently active Mode
        //
    // If there are more than 4 modes, uncomment the following line:
        // Trigger ALT = m_xbox.rightBumper();
    // and insert as many lines as needed for the additional Modes, substituting desired ordinal
    // direction(s) and Mode name(s) as needed, using this example:
        // ALT.and(m_xbox.povRight()).onTrue(new InstantCommand(()-> m_modeManager.setMode(Mode.CLIMB)));
    // Finally, fix the original 4 Modes to match the following format:
        // m_xbox.povRight().and(ALT.negate()).onTrue(new InstantCommand(()-> m_modeManager.setMode(Mode.DRIVE)));

        m_xbox.povLeft().onTrue( new InstantCommand(()-> m_modeManager.setMode(UI_Mode.INTAKE)));
        m_xbox.povUp().onTrue(   new InstantCommand(()-> m_modeManager.setMode(UI_Mode.SHOOT)));
        m_xbox.povRight().onTrue(new InstantCommand(()-> m_modeManager.setMode(UI_Mode.DRIVE)));
        m_xbox.povDown().onTrue( new InstantCommand(()-> m_modeManager.setMode(UI_Mode.DEFENSE)));   
     }

    /*
     * getSelectedAutoCommand is called from Robot.AutonomousInit(),
     */
    public Command getSelectedAutoCommand() {
/*
        Command selectedAuto = m_autoRoutineChooser.getSelected();

        if (selectedAuto == null) {
            selectedAuto = new DoNothingCmd();
        }

        return selectedAuto;
*/
        return null;
    }
}