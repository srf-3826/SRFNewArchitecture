package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.lib.sensors.GyroIO;
import frc.lib.sensors.MotionEstimator;
import frc.robot.Constants.*;
import frc.robot.autos.drivehelpers.AimHelper;
import frc.robot.autos.drivehelpers.RangeHelper;
import frc.robot.autos.drivehelpers.StrafeAlignHelper;
import frc.robot.ui.Action;
import frc.robot.ui.AutoDriveHelperAction;
import frc.robot.ui.ContinuousAction;
import frc.robot.ui.HelperContext;
import frc.robot.ui.ModeManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.lib.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveSubsystem extends ActionableSubsystem {

    private double              m_now;

    private SwerveDriveOdometry m_swerveOdometry;
    private MotionEstimator     m_motionEstimator;
    private SwerveModule[]      m_swerveMods;
    private SwerveModuleState[] m_states = new SwerveModuleState[4];

    private SwerveDrivePoseEstimator  m_swerveDrivePoseEstimator;

    private final CANBus              m_swerveCanbus;
    private final GyroIO              m_gyro;
    private final VisionSubsystem     m_visionSubsystem;
    private final ModeManager         m_modeManager;

    private Rotation2d          m_currentHeading2d;

    private Translation2d       m_cenOfRotationOffset = SDC.REL_POS2D_CEN;
    private String              m_cenRotIdString;
    private boolean             m_isFieldOriented = true;       // default is Field Oriented on start

    // The following is a temporary driver settable speed reduction factor, normally
    // triggered (by Right Bumper, when held). When triggered, speed will be slower, for both 
    // translate and strafing, as well as rotation. Starts out as full speed.
    private static double       m_varMaxOutputFactor = 1.0;

    // The following are fixed (i.e. changable via re-compile only) reductions in the max speeds
    // allowed, to both increase driver control and reduce chance of damage, independent of
    // m_varMaxOutputFactor. The final throttle limits are the product of each with m_varOutputLimit.
    public double m_fixedMaxTranslationOutput  = SDC.OUTPUT_DRIVE_LIMIT_FACTOR;                  
    public double m_fixedMaxRotationOutput     = SDC.OUTPUT_ROTATE_LIMIT_FACTOR;

    private final HelperContext       m_ctx;

    private boolean m_shootingDriveHelpersEnabled = false;
    private boolean m_shootingAutoDriveHelpersOK = false;
  
    public class OwnedHelper {
        public final Action m_owner;
        public final ContinuousAction m_helper;

        public OwnedHelper(Action owner, ContinuousAction helper) {
            this.m_owner = owner;
            this.m_helper = helper;
        }
    }

    public List<OwnedHelper> m_activeHelpers = new ArrayList<>();
    
    private GenericEntry        m_isFieldOrientedEntry;
    public  GenericEntry        m_odometryPoseXEntry;
    public  GenericEntry        m_odometryPoseYEntry;
    private GenericEntry        m_odometryHeadingEntry;
    private GenericEntry        m_cenOfRotEntry;
    private GenericEntry        m_maxOutputFactorEntry;
    private GenericEntry        m_odometrySpeedEntry;
    private GenericEntry        m_odometryAngVelEntry;
    private GenericEntry        m_maxSpeedEntry;
    private GenericEntry        m_maxAngVelEntry;
    private GenericEntry        m_maxAccelEntry;
    private GenericEntry        m_maxAngAccelEntry;

    // Magic numbers ahead (publishing interval of 10 hz, and out of phasefactor). May want to move to
    // constants.java. Swerve modules will publish at the same rate as SwerveSubsystem, but out of 
    // phase by half the interval. 
    private static final double PUBLISH_INTERVAL = 0.01;            // Was .1, changed to Temporarily publish every loop
    private double              m_lastSwerveSubsystemPubTime = 0.0;
    private double              m_lastSwerveModulesPubTime = PUBLISH_INTERVAL / 2.0;

    public SwerveSubsystem(CANBus swerveCanbus,
                           GyroIO gyro,
                           VisionSubsystem visionSubsystem,
                           ModeManager modeManager,
                           HelperContext ctx ) {
        m_swerveCanbus = swerveCanbus;
        m_gyro = gyro;
        m_visionSubsystem = visionSubsystem;
        m_modeManager = modeManager;
        m_ctx = ctx;

        CommandScheduler.getInstance().registerSubsystem(this);

        m_currentHeading2d = getYaw2d();

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, SDC.FL_Mod0.MODULE_CONSTANTS, m_swerveCanbus),
            new SwerveModule(1, SDC.FR_Mod1.MODULE_CONSTANTS, m_swerveCanbus),
            new SwerveModule(2, SDC.BL_Mod2.MODULE_CONSTANTS, m_swerveCanbus),
            new SwerveModule(3, SDC.BR_Mod3.MODULE_CONSTANTS, m_swerveCanbus)
        };

        disableShootingDriveHelpers();

        // By pausing init for a second before setting module offsets, we avoid 
        // a bug with inverting motors. This call is thread blocking, but is only 
        // called once at startup, so ignore.
        // See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        Timer.delay(1.0);
        // Update 1/2024: also added waitForCANcoder method in SwerveModule, 
        // which does not hurt but according to above thread the delay is still 
        // needed, so restored that.
        resetModulesToAbsolute();

        resetCenOfRotation();

        m_swerveDrivePoseEstimator.resetPosition(m_currentHeading2d,
                                                 getModulePositions(),
                                                 getPose());
        m_motionEstimator = new MotionEstimator();

        setupPublishing();
    }

    // The following two methods enable and disable AutoDriveHelpersOK when in
    // SHOOTING mode. If not in shooting mode (should never happen because
    // ActionManager only processes Actions whose Enum mode matches the active mode)
    // but to be sure there are no loose ends, just ensure the flag is cleared.
    // Called from Polled action processing.
    public void setShootingAutoDriveOK() {
        if (m_modeManager.isShootingMode()) {
            m_shootingAutoDriveHelpersOK = true;
        }
        else if (m_shootingAutoDriveHelpersOK) {
            clearShootingAutoDriveOK();
        }
    }

    public void clearShootingAutoDriveOK() {
        m_shootingAutoDriveHelpersOK = true;        
    }

    // The following two methods enable and disable AutoDriveHelpers when in
    // SHOOTING mode. If not in shooting mode, just ensure the helpers are disabled.
    public void enableShootingDriveHelpers() {
        if (m_modeManager.isShootingMode() && m_shootingAutoDriveHelpersOK) {
            m_shootingDriveHelpersEnabled = true;
        } else if (m_shootingDriveHelpersEnabled) {
            disableShootingDriveHelpers();
        }
    }

    public void disableShootingDriveHelpers() {
        m_shootingDriveHelpersEnabled = false;
        m_activeHelpers.clear();
    }

    // The following method handles autoDriveAssist helpers in shooting mode
     private void updateAutoHelpers() {
        // Question - remove the following line if continuous PID assistance is
        // better (leaving the PID active will more quickly react to collisions with
        // defendeers)
        m_activeHelpers.removeIf(h -> h.m_helper.isFinished());

        if (!m_modeManager.isShootingMode() || !m_shootingDriveHelpersEnabled) {
            return;
        }

        if (m_visionSubsystem.hasValidTag() && !hasHelperOfType(AimHelper.class)) {
            m_activeHelpers.add(new OwnedHelper(Action.AUTO_AIM, new AimHelper(m_ctx)));
        }

        if (m_visionSubsystem.distanceErrorTooLarge() && !hasHelperOfType(RangeHelper.class)) {
            m_activeHelpers.add(new OwnedHelper(Action.AUTO_RANGE, new RangeHelper(m_ctx)));
        }

        if (m_visionSubsystem.lateralErrorTooLarge() && !hasHelperOfType(StrafeAlignHelper.class)) {
            m_activeHelpers.add(new OwnedHelper(Action.AUTO_STRAFE, new StrafeAlignHelper(m_ctx)));
        }
    }

    private boolean hasHelperOfType(Class<?> clazz) {
        return m_activeHelpers.stream().anyMatch(h -> clazz.isInstance(h.m_helper));
    }

    public ChassisSpeeds getAutoDriveAssistSpeeds() {
        double vx = 0, vy = 0, omega = 0;
        for (OwnedHelper oh : m_activeHelpers) {
            if (oh.m_helper instanceof AutoDriveHelperAction dh) {
                ChassisSpeeds s = dh.getSpeeds();
                vx    += s.vxMetersPerSecond;
                vy    += s.vyMetersPerSecond;
                omega += s.omegaRadiansPerSecond;
            }
        }

        return new ChassisSpeeds(vx, vy, omega);
    }     

    // The following five methods establish the center of rotation, initially or
    // on the fly, to either the center of the robot (default) or to one of the 
    // four module wheels.
    public void setFLCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_FL;
        m_cenRotIdString = "FL";
    }

    public void setFRCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_FR;
        m_cenRotIdString = "FR";
    }

    public void setBLCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_BL;
        m_cenRotIdString = "BL";
    }

    public void setBRCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_BR;
        m_cenRotIdString = "BR";
    }

    public void resetCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_CEN;
        m_cenRotIdString = "Cen";
    }

    // drive() is the handler for teleop joystick driving, typically called from 
    // DefaultDriveCmd with isOpenLoop set to false.
    // It can also be called from PID controllers or other Commands as needed, 
    // typically with isOpenLoop set to false.
    public void drive(ChassisSpeeds chassisSpeeds,
                      boolean isOpenLoop) {
        chassisSpeeds = chassisSpeeds.times(m_varMaxOutputFactor * m_fixedMaxTranslationOutput);

        SwerveModuleState[] swerveModuleStates =
            SDC.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds,
                                                       m_cenOfRotationOffset);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 
                                                    SDC.MAX_ROBOT_SPEED_M_PER_SEC);
        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_modNum], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SDC.MAX_ROBOT_SPEED_M_PER_SEC);
        
        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(desiredStates[mod.m_modNum], false);
        }
    }    

    public boolean isFieldOriented() {
        return m_isFieldOriented;
    }

    public void setFieldOriented( boolean fieldOrientedSetting ) {
        m_isFieldOriented = fieldOrientedSetting;
        // to reset the max vel and accel data, just change to either robot or field oriented. 
        // Then switch back to leave current field oriented setting unchanged 
        m_motionEstimator.resetMax();
    }

    public void setVarMaxOutputFactor(double maxOutputFactor) {
        if (maxOutputFactor < .1) {
            maxOutputFactor = .1;
        }
        if (maxOutputFactor > 1.0) {
            maxOutputFactor = 1.0;
        };
        m_varMaxOutputFactor = maxOutputFactor;
    }

    public static double getVarMaxOutputFactor() {
        return m_varMaxOutputFactor;
    } 

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose2d) {
        m_swerveOdometry.resetPosition(m_currentHeading2d, getModulePositions(), pose2d);
    }

    public double getRobotTranslateVel() {
        m_states = getModuleStates();
        return (SDC.SWERVE_KINEMATICS
                .toChassisSpeeds(m_states).vxMetersPerSecond);
    }
    
    public double getRobotStrafeVel() {
        m_states = getModuleStates();
        return (SDC.SWERVE_KINEMATICS
                .toChassisSpeeds(m_states).vyMetersPerSecond);
    }
    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.m_modNum] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
            positions[mod.m_modNum] = mod.getModulePosition();
        }
        return positions;
    }

    public void zeroGyro() {
        m_gyro.zeroGyro();
        m_currentHeading2d = getYaw2d();
    }

    public Rotation2d getYaw2d() {
        return m_gyro.getRotation2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : m_swerveMods){
            mod.resetToAbsolute();
        }
    }

    public void stop() {
        for(SwerveModule mod : m_swerveMods) {
            mod.stop();
        }            
    }

    public double getCurrentPose() {
        return m_swerveOdometry.getPoseMeters().getY();
    }

    /*
        periodic is called on every loop instance. 
    */
    @Override
    public void periodic() {
        // This method will be called by the command scheduler once per loop, 
        // Question: only when robot is enabled?
        updateAutoHelpers();

        m_now = Timer.getFPGATimestamp();
        // The following methods are separate with separate decimators to reduce publish frequency
        publishSwerveSubsystemData(); 
        publishSwerveModuleData();
    }

    /*
    update is called from robot.periodic() (transitively through robotContaine)
    once per loop, and prior to periodic(), whether enabled or not.
    It's purpose is to keep all Phoenix6 StstusSignals owned by swerveDrive components
    refdreshed, as well as to keep odometry and motion estimation up to date.
    */
    public void update() {
        m_currentHeading2d = getYaw2d();            // cache the current gyro heading
                                                    // whetehr ENABLED or not
        if (DriverStation.isEnabled()) {            // and if ENABLED, update odometry and motion estimation
            m_swerveOdometry.update(m_currentHeading2d, getModulePositions()); 
            m_motionEstimator.update(m_swerveOdometry.getPoseMeters());
        }
    
        // Allow SwerveModules to all refresh their StatusSignals
        // For referencce, gyro StatusSignals are refreshed by RobotContainer before calling
        // swerve.update(), because other subsystems are also dependent on gyro output.
        for(SwerveModule mod : m_swerveMods) {
            mod.update();
        }    
    }

    public void setupPublishing() {
        ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");
        if (sbt == null) {
            SmartDashboard.putString("SwerveDrive Tab", "getTab() Error occured");
        } else {
            // Initialize column for Robot Data
            ShuffleboardLayout sl =  sbt.getLayout("RobotData", BuiltInLayouts.kGrid)
                                        .withPosition(0, 0)
                                        .withSize(1, SDC.SWERVE_MOD_LIST_HGT)
                                        .withProperties(Map.of("Number of Columns", 1,
                                                               "Number of Rows", 12, 
                                                               "Label position", "LEFT"));
            if (sl == null) {
                SmartDashboard.putString("RobotData Layout", "getLayout() Error occured");
            } else {
                m_isFieldOrientedEntry  = sl.add("Field Or", "Yes")
                                            .withPosition(0, 0)
                                            .getEntry();
                m_odometryPoseXEntry    = sl.add("Xpos m", F.df2.format(0.0))
                                            .withPosition(0, 1)
                                            .getEntry();
                m_odometryPoseYEntry    = sl.add("Ypos m", F.df2.format(0.0))
                                            .withPosition(0, 2)
                                            .getEntry();
                m_odometryHeadingEntry  = sl.add("Hdg D", F.df1.format(0.0))
                                            .withPosition(0, 3)
                                            .getEntry();
                m_cenOfRotEntry         = sl.add("C rot", m_cenRotIdString)
                                            .withPosition(0, 4)
                                            .getEntry();
                m_maxOutputFactorEntry  = sl.add("M out", F.df2.format(0.0))
                                            .withPosition(0, 5)
                                            .getEntry();
                m_odometrySpeedEntry    = sl.add("Spd", F.df2.format(0.0))
                                            .withPosition(0, 6)
                                            .getEntry();
                m_odometryAngVelEntry   = sl.add("@ Vel", F.df2.format(0.0))
                                            .withPosition(0, 7)
                                            .getEntry();
                m_maxSpeedEntry         = sl.add("M spd", F.df2.format(0.0))
                                            .withPosition(0, 8)
                                            .getEntry();
                m_maxAngVelEntry        = sl.add("M @vel", F.df2.format(0.0))
                                            .withPosition(0, 9)
                                            .getEntry();
                m_maxAccelEntry         = sl.add("Maccel", F.df2.format(0.0))
                                            .withPosition(0, 10)
                                            .getEntry();
                m_maxAngAccelEntry      = sl.add("M@accel", F.df2.format(0.0))
                                            .withPosition(0, 11)
                                            .getEntry();

                if ((m_isFieldOrientedEntry == null)
                    ||(m_odometryPoseXEntry == null)
                    ||(m_odometryPoseYEntry == null)
                    ||(m_odometryHeadingEntry == null)
                    ||(m_cenOfRotEntry == null)
                    ||(m_maxOutputFactorEntry == null)
                    ||(m_odometrySpeedEntry == null)
                    ||(m_odometryAngVelEntry == null)
                    ||(m_maxSpeedEntry == null)
                    ||(m_maxAngVelEntry == null)
                    ||(m_maxAccelEntry == null)
                    ||(m_maxAngAccelEntry == null)) {
                    SmartDashboard.putString("RobotData List Entries", "Null Entry handles(s) encountered");
                }
                // Initalize a column holding units for the adjacent Swerve Module Data Columns
                ShuffleboardLayout s2 =  sbt.getLayout("Units", BuiltInLayouts.kGrid)
                                            .withPosition(SDC.FIRST_SWERVE_MOD_LIST_COL + 4, 0)
                                            .withSize(2, SDC.SWERVE_MOD_LIST_HGT)
                                            .withProperties(Map.of("Number of Columns", 2,
                                                                    "Number of Rows", 12, 
                                                                    "Label position", "RIGHT"));
                // No need to cache the entries here - they only get written once in this setup method
                // Titles must be unique within this grid widget. Use variable len strings composed of
                // zero width non visible spaces. Default values are strings with the correct digits for 
                // adjacent Swerve Module data sets.
                s2.add(" "+makeInvisibleTitleOfLen(0), "DM  SM  CC")
                  .withPosition(0, 0);
                s2.add(" "+makeInvisibleTitleOfLen(1), "Deg - offsets")
                  .withPosition(0, 1);
                s2.add(" "+makeInvisibleTitleOfLen(2), "Deg - CC enc")
                  .withPosition(0, 2);
                s2.add(" "+makeInvisibleTitleOfLen(3), "Deg - SM enc")
                  .withPosition(0, 3);
                s2.add(" "+makeInvisibleTitleOfLen(4), "Deg - Setpoint")
                  .withPosition(0, 4);
                s2.add(" "+makeInvisibleTitleOfLen(5), "SM - PID out")
                  .withPosition(0, 5);
                s2.add(" "+makeInvisibleTitleOfLen(6), "Amps")
                  .withPosition(0, 6);
                s2.add(" "+makeInvisibleTitleOfLen(7), "Celcius")
                  .withPosition(0, 7);
                s2.add(" "+makeInvisibleTitleOfLen(8), "m/sec")
                  .withPosition(0, 8);
                s2.add(" "+makeInvisibleTitleOfLen(9), "meters")
                  .withPosition(0, 9);
                s2.add(" "+makeInvisibleTitleOfLen(10), "Amps")
                  .withPosition(0, 10);
                s2.add(" "+makeInvisibleTitleOfLen(11), "Celcius")
                  .withPosition(0, 11);
            }
        }
    }

    private static String makeInvisibleTitleOfLen(int index) {
        // "\u2008" is a non visible zero width Unicode space.
        return "\u200B".repeat(index + 1);
    }

    /*
     * This is where all SwerveSubsystem related data publishing gets scheduled and performed
     */
    public void publishSwerveSubsystemData() {
        // m_now is always set by periodic(), first thing
        if (m_now - m_lastSwerveSubsystemPubTime < PUBLISH_INTERVAL) return;  // 10 Hz
    
        // If this code is reached, it is itme to publish
        m_lastSwerveSubsystemPubTime = m_now;

        // class variable m_currentHeading2d is refreshed in periodic().
        // doing this should avoid making CAN bus data requests too frequently for
        // the Pigeon2 to keep up.
        m_isFieldOrientedEntry.setString(m_isFieldOriented ? "Yes" : "No");
        // Insert UI Mode display here, drop display of AngVelOdom and AngAccelOdom. Add linear jerk
        Pose2d location2d = getPose();
        m_odometryPoseXEntry.setString(F.df2.format(location2d.getX()));
        m_odometryPoseYEntry.setString(F.df2.format(location2d.getY()));           
        m_odometryHeadingEntry.setString(F.df1.format(location2d.getRotation().getDegrees()));
        m_cenOfRotEntry.setString(m_cenRotIdString);
        m_maxOutputFactorEntry.setString(F.df2.format(m_varMaxOutputFactor * m_fixedMaxTranslationOutput));
        m_odometrySpeedEntry.setString(F.df2.format(m_motionEstimator.getVelocity()));
        m_odometryAngVelEntry.setString(F.df2.format(m_motionEstimator.getAngularVelocity())); 
        m_maxSpeedEntry.setString(F.df2.format(m_motionEstimator.getMaxVelocity())); 
        m_maxAngVelEntry.setString(F.df2.format(m_motionEstimator.getMaxAngularVelocity())); 
        m_maxAccelEntry.setString(F.df2.format(m_motionEstimator.getMaxAcceleration())); 
        m_maxAngAccelEntry.setString(F.df2.format(m_motionEstimator.getMaxAngularAcceleration())); 
    }

    /*
     * This is where all SwerveModule related data publishing gets scheduled and performed
     */
    public void publishSwerveModuleData() {
        // m_now is always set by periodic(), first thing
        if (m_now - m_lastSwerveModulesPubTime < PUBLISH_INTERVAL) return;
        for(SwerveModule mod : m_swerveMods) {
            mod.publishModuleData();
        }
        m_lastSwerveModulesPubTime = m_now;
    } 

    // This is a test routine, designed to rotate all modules
    // synchronously to an identical specified heading in degrees
    public void rotateModulesToAngle(double angleDeg) {
        for(SwerveModule mod : m_swerveMods) {
            mod.setAngle(angleDeg);
        }    
    }

    // This is a method to cause all modules to be rotated to 
    // angles contained in an array passed as an argument, the
    // specified angles being in FL, FR, BL, and BR order.
    // Primarily used to set all modules to their PARK positions, 
    // but could be useful for other purposes.
    public void rotateModulesToAngles( double angleDeg[] ) {
        for(SwerveModule mod : m_swerveMods) {
            mod.setAngle(angleDeg[mod.m_modNum] );
        }
    }
}