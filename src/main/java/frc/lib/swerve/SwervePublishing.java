// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.sensors.MotionEstimator;
import frc.robot.Constants.F;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class SwervePublishing {
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

    private SwerveSubsystem     m_swerveDrive;
    private MotionEstimator     m_motionEstimator;

    public SwervePublishing( SwerveSubsystem swerveDrive, MotionEstimator motionEstimator) {
        m_swerveDrive = swerveDrive;
        m_motionEstimator = motionEstimator;
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
                m_cenOfRotEntry         = sl.add("C rot", "Cen")
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
    public void publishSwerveSubsystemData(double m_now,
                                           boolean isFieldOriented,
                                           String cenRotIdString,
                                           double varMaxOutputFactor,
                                           double fixedMaxTranslationOutput 
                                           ) {
        // m_now is always set by periodic(), first thing
        if (m_now - m_lastSwerveSubsystemPubTime < PUBLISH_INTERVAL) return;  // 10 Hz
    
        // If this code is reached, it is itme to publish
        m_lastSwerveSubsystemPubTime = m_now;

        // class variable m_currentHeading2d is refreshed in periodic().
        // doing this should avoid making CAN bus data requests too frequently for
        // the Pigeon2 to keep up.
        m_isFieldOrientedEntry.setString(isFieldOriented ? "Yes" : "No");
        // Insert UI Mode display here, drop display of AngVelOdom and AngAccelOdom. Add linear jerk
        Pose2d location2d = m_swerveDrive.getPose();
        m_odometryPoseXEntry.setString(F.df2.format(location2d.getX()));
        m_odometryPoseYEntry.setString(F.df2.format(location2d.getY()));           
        m_odometryHeadingEntry.setString(F.df1.format(location2d.getRotation().getDegrees()));
        m_cenOfRotEntry.setString(cenRotIdString);
        m_maxOutputFactorEntry.setString(F.df2.format(varMaxOutputFactor * fixedMaxTranslationOutput));
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
    public void publishSwerveModulesData(double now, SwerveModule[] swerveMods) {
        // m_now is always set by periodic(), first thing
        if (now - m_lastSwerveModulesPubTime < PUBLISH_INTERVAL) return;
        for(SwerveModule mod : swerveMods) {
            mod.publishModuleData();
        }
        m_lastSwerveModulesPubTime = now;
    } 
}
