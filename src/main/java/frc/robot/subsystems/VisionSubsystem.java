package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.*;

public class VisionSubsystem extends ActionableSubsystem {
    private LimelightResults         m_limelightResults;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private SwerveSubsystem          m_drivetrain;
    // private LimelightHelpers.IMUData imuData;
    // private PoseClass                poses;
    private AprilTagFieldLayout      m_aprilTagFieldLayout;

    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimator, 
                           SwerveSubsystem drivetrain) {
        // m_limelightResults = new LimelightResults();
        m_poseEstimator = poseEstimator;
        m_drivetrain = drivetrain;
        // imuData = new LimelightHelpers.IMUData();
        // poses = new PoseClass();

        Pose3d camPose = new Pose3d();
        setCameraPos(camPose,
                     camPose.getRotation());
        m_aprilTagFieldLayout = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();
    }

    public void setCameraPos(Pose3d position, Rotation3d rotation)
    {
        LimelightHelpers.setCameraPose_RobotSpace(
            VC.LIMELIGHT_NAME,
            position.getX(),  // back to front
            position.getY(),  // right to left
            position.getZ(),  // down to up
            rotation.getX(),  // Roll
            rotation.getY(),  // Pitch
            rotation.getZ()); // Yaw
    }

    @Override
    public void periodic() 
    {
        LimelightHelpers.SetRobotOrientation(VC.LIMELIGHT_NAME,
                                             m_drivetrain.getYaw2d().getDegrees(), 0,
                                             0, 0,
                                             0, 0);
        // m_poseEstimator.update(m_drivetrain.getYaw2d(), m_drivetrain.getModulePositions());
        // Instead of the above, call nearestTarget = getNearestTargetInfo, and if valid 
        // (i.e. not .empty), call the following: this.addVisionMeasurement(nearestTarget);
        // 
    }

    // Call this to get nearest AprilTag TargetInfo
    public Optional<TargetInfo> getNearestTargetInfo() {

        m_limelightResults = LimelightHelpers.getLatestResults(VC.LIMELIGHT_NAME);

        if (!m_limelightResults.valid || m_limelightResults.targets_Fiducials.length == 0) {
            return Optional.empty();
        }

        var best = m_limelightResults.targets_Fiducials[0];

        return Optional.of(new TargetInfo((int)best.fiducialID,
                                          best.getRobotPose_TargetSpace(),      // a robotToTag Pose3d
                                          m_limelightResults.timestamp_RIOFPGA_capture));
    }

    public List<TargetInfo> getAllTargetInfo() {

        m_limelightResults = LimelightHelpers.getLatestResults(VC.LIMELIGHT_NAME);

        if (!m_limelightResults.valid || m_limelightResults.targets_Fiducials.length == 0) {
            return List.of();
        }

        List<TargetInfo> list = new ArrayList<>();

        for (var t : m_limelightResults.targets_Fiducials) {
            list.add(new TargetInfo((int)t.fiducialID,
                                    t.getRobotPose_TargetSpace(),
                                    m_limelightResults.timestamp_RIOFPGA_capture
                                   )
                    );
        }
        return list;
    }

    // Call this from periodic() whenever valid TargetInfo data is available
    public void addVisionMeasurement(TargetInfo info) {
        Pose3d fieldToTag = m_aprilTagFieldLayout.getTagPose(info.tagId).get();

        // Convert Pose3d (tag → robot) into a Transform3d
        Transform3d tagToRobot = new Transform3d(info.robotPoseInTagSpace.getTranslation(),
                                                info.robotPoseInTagSpace.getRotation());
        Pose3d fieldToRobot = fieldToTag.transformBy(tagToRobot);

        m_poseEstimator.addVisionMeasurement(fieldToRobot.toPose2d(), info.timestamp);
    }

    // returns true on success, false on fail
    public boolean tryLocateBot()
    {
        LimelightHelpers.SetRobotOrientation(VC.LIMELIGHT_NAME, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VC.LIMELIGHT_NAME);
        boolean doRejectUpdate = false;
        // imuData = LimelightHelpers.getIMUData(VC.LIMELIGHT_NAME);
        
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        // Find out what that translates to. This is incomplete
        /*
        if(Math.abs(imuData.accelX) > 360)
        {
            doRejectUpdate = true;
        }
        */
        if(megatag2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        /*
         * The vision noise matrix:
            VecBuilder.fill(xStdDev, yStdDev, thetaStdDev); // Units = meters, meters, radians
            tells the estimator how uncertain each vision measurement is.
                • Smaller numbers → trust vision more
                • Larger numbers → trust odometry more
            For LL2 hardware, the sweet spot is usually moderate trust because LL2 has:
                Good but not perfect AprilTag accuracy, no pose ambiguity output, and 
                Slightly noisier depth estimation than LL3/LL4
            These starting values work well for most LL2 robots:
             VecBuilder.fill(0.35, 0.35, Units.degreesToRadians(8.0));
            This means you trust vision to within ~35 cm in X/Y, and heading to within ~8°
            These are conservative enough to avoid snapping, but tight enough to correct drift.
            How to tune vision noise (step-by-step)
            1. Start with the recommended values, run the robot and observe behavior.
            2. If the pose "snaps" or jumps when tags appear this means vision is weighted too strongly,
            so increase noise:
            VecBuilder.fill(0.45, 0.45, Units.degreesToRadians(12))
            Symptoms of too‑strong vision: Robot pose jumps sideways when a tag enters view, Auto 
            paths jerk when passing near tags, Heading suddenly rotates a few degrees
            3. If the pose drifts even when tags are visible, this means vision is weighted too weakly,
            so decrease noise:
            VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(5))
            Symptoms of too‑weak vision: Robot slowly drifts off its true position, Auto routines 
            start correct but end several inches off, Tag alignment commands feel "sloppy"
            4. If heading is stable but X/Y are noisy, tune the heading separately:
            VecBuilder.fill(0.35, 0.35, Units.degreesToRadians(4))
            LL2 heading estimation is often better than distance estimation.
            5. If the robot oscillates between odometry and vision this means the two sources 
            disagree strongly. Increase both process noise and vision noise slightly:
            // Odometry noise
            VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1.5))
            // Vision noise
            VecBuilder.fill(0.40, 0.40, Units.degreesToRadians(10))
            This tells the estimator: 
           "Odometry is a bit less Certain", "Vision is also a bit less certain", 
            and the estimator blends them more smoothly
         */
        {
            // was: m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); 
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.35,.35, Units.degreesToRadians(8.0))); 
            m_poseEstimator.addVisionMeasurement(
                megatag2.pose,
                megatag2.timestampSeconds);
        }
        return !doRejectUpdate;
    }

    public void setTargetFilter(int[] ids)
    {
        LimelightHelpers.SetFiducialIDFiltersOverride(VC.LIMELIGHT_NAME, ids);
    }

    // The following 4 methods support SwerveSubsystem autoDriveAssist helpers
    public boolean hasValidTag() {
        return getNearestTargetInfo().isPresent();
    }

    public double computeDesiredRangeToTag(int tagId) {

        Optional<Pose3d> tagPoseOpt = m_aprilTagFieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) return SSC.SHOOTER_OPTIMAL_RANGE_METERS;

        Pose3d fieldToTag = tagPoseOpt.get();
        Pose2d fieldToRobot = m_drivetrain.getPose();

        // Horizontal distance from robot to tag
        double dx = fieldToTag.getX() - fieldToRobot.getX();
        double dy = fieldToTag.getY() - fieldToRobot.getY();
        double distanceRobotToTag = Math.hypot(dx, dy);

        // Convert tag distance to goal distance
        // (subtract the known tag-to-goal offset)
        double tagToGoalOffset = 0.50;  // meters; adjust once field is built

        double distanceRobotToGoal = distanceRobotToTag - tagToGoalOffset;

        // Convert robot-center distance to shooter-exit distance
        double shooterToGoal = distanceRobotToGoal - SSC.SHOOTER_OFFSET_FORWARD;

        // The desired range is the difference between current and optimal
        return shooterToGoal;
    }

    public boolean distanceErrorTooLarge() {
        var opt = getNearestTargetInfo();
        if (opt.isEmpty()) return false;

        var info = opt.get();
        double range = info.robotPoseInTagSpace.getTranslation().getNorm();
        double desired = computeDesiredRangeToTag(info.tagId);

        return Math.abs(range - desired) > 0.10;  // 10 cm threshold
    }

    public boolean lateralErrorTooLarge() {
        var opt = getNearestTargetInfo();
        if (opt.isEmpty()) return false;

        var info = opt.get();
        double ty = info.robotPoseInTagSpace.getTranslation().getY();

        return Math.abs(ty) > 0.05;  // 5 cm threshold
    }

    public Pose2d getBotPose() {
        return m_poseEstimator.getEstimatedPosition();
    }
    
    /*
     * From https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
     * 

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight") {
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if(mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      }
    */
}