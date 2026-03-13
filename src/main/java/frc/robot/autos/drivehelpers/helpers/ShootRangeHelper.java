package frc.robot.autos.drivehelpers.helpers;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.drivehelpers.ADContext;
import frc.robot.autos.drivehelpers.ADHelperMetadata;
import frc.robot.autos.drivehelpers.AdMetadataLibrary;
import frc.robot.autos.drivehelpers.ContinuousAction;
import frc.robot.subsystems.TargetInfo;

public class ShootRangeHelper implements ContinuousAction {

    private final ADContext m_ctx;
    private final ADHelperMetadata metadata = AdMetadataLibrary.SHOOT_RANGE;

    private final PIDController m_pid =
        new PIDController(1.2, 0.0, 0.05);  // tune as needed

    private ChassisSpeeds m_latestSpeeds = new ChassisSpeeds();

    public ShootRangeHelper(ADContext ctx) {
        this.m_ctx = ctx;
    }

    @Override
    public void start() {
        m_pid.reset();
    }

    @Override
    public void update(Optional<TargetInfo> tagOpt, boolean autoDriveEnabled) {

        if (!autoDriveEnabled || tagOpt.isEmpty()) {
            m_latestSpeeds = new ChassisSpeeds();
            return;
        }

        TargetInfo tag = tagOpt.get();
        double distError = tag.distanceErrorMeters();

        double vx = m_pid.calculate(distError, 0.0);

        m_latestSpeeds = new ChassisSpeeds(vx, 0, 0);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return m_latestSpeeds;
    }

    @Override
    public boolean shouldActivate(ADContext adCtx) {

        if (!adCtx.inShootMode() || !adCtx.isShootAutoDriveEnabled())
            return false;

        Optional<TargetInfo> tag = adCtx.autoDriveAgent.selectTagFor(metadata);
        if (tag.isEmpty()) return false;

        double distError = tag.get().distanceErrorMeters();

        return Math.abs(distError) > metadata.activateDistanceErrorMeters;
    }

    @Override
    public boolean isFinished() {
        return !m_ctx.inShootMode() || !m_ctx.isShootAutoDriveEnabled();
    }

    @Override
    public void stop() {
        m_latestSpeeds = new ChassisSpeeds();
    }
}
