package frc.robot.autos.drivehelpers.helpers;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.drivehelpers.ADContext;
import frc.robot.autos.drivehelpers.ADHelperMetadata;
import frc.robot.autos.drivehelpers.AdMetadataLibrary;
import frc.robot.autos.drivehelpers.ContinuousAction;
import frc.robot.subsystems.TargetInfo;

public class ShootStrafeHelper implements ContinuousAction {

    private final ADContext m_ctx;
    private final ADHelperMetadata metadata = AdMetadataLibrary.SHOOT_STRAFE;

    private final PIDController m_pid =
        new PIDController(1.0, 0.0, 0.05);  // tune as needed

    private ChassisSpeeds m_latestSpeeds = new ChassisSpeeds();

    public ShootStrafeHelper(ADContext ctx) {
        this.m_ctx = ctx;
    }

    @Override
    public void start() {
        m_pid.reset();
    }

    @Override
    public void update(Optional<TargetInfo> tagOpt) {

        if (tagOpt.isEmpty()) {
            m_latestSpeeds = new ChassisSpeeds(0, 0, 0);
            return;
        }

        TargetInfo tag = tagOpt.get();
        double latError = tag.lateralErrorMeters();

        double vy = m_pid.calculate(latError, 0.0);

        m_latestSpeeds = new ChassisSpeeds(0, vy, 0);
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

        double latError = tag.get().lateralErrorMeters();

        return Math.abs(latError) > metadata.activateLateralErrorMeters;
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