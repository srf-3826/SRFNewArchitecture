package frc.robot.autos.drivehelpers.helpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.autos.drivehelpers.ADContext;
import frc.robot.autos.drivehelpers.ADHelperMetadata;
import frc.robot.autos.drivehelpers.AdMetadataLibrary;
import frc.robot.autos.drivehelpers.ContinuousAction;
import frc.robot.subsystems.TargetInfo;

public class ShootAimHelper implements ContinuousAction {

    private final ADContext m_ctx;
    private final ADHelperMetadata metadata = AdMetadataLibrary.SHOOT_AIM;

    private final PIDController m_pid =
        new PIDController(0.03, 0.0, 0.001);  // tune as needed

    private ChassisSpeeds m_latestSpeeds = new ChassisSpeeds();

    public ShootAimHelper(ADContext ctx) {
        this.m_ctx = ctx;
        m_pid.enableContinuousInput(-180, 180);
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
        double headingError = tag.headingErrorDeg();

        double omega = m_pid.calculate(headingError, 0.0);

        m_latestSpeeds = new ChassisSpeeds(0, 0, omega);
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

        double headingError = tag.get().headingErrorDeg();
        return Math.abs(headingError) > metadata.activateHeadingErrorDeg;
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
