package frc.robot.autos.drivehelpers.helpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.autos.drivehelpers.ADContext;
import frc.robot.autos.drivehelpers.ADHelperMetadata;
import frc.robot.autos.drivehelpers.ContinuousAction;
import frc.robot.subsystems.TargetInfo;

public class ShootAimHelper implements ContinuousAction {

    @SuppressWarnings("unused")
    private final ADContext m_adCtx;                // Does this need to be here?
    private final ADHelperMetadata m_adMetadata;
    private final PIDController pid;
    private ChassisSpeeds m_latestSpeeds = new ChassisSpeeds();

    public ShootAimHelper(ADContext ctx, ADHelperMetadata metadata) {
        this.m_adCtx = ctx;
        this.m_adMetadata = metadata;
        this.pid = new PIDController(m_adMetadata.kP, m_adMetadata.kI, m_adMetadata.kD);
    }

    @Override
    public boolean shouldActivate(ADContext adCtx,
                                  Optional<TargetInfo> tag,
                                  boolean enabled) {

        if (!enabled || !adCtx.inShootMode() || tag.isEmpty())
            return false;

        return Math.abs(tag.get().headingErrorDeg()) > m_adMetadata.activateHeadingErrorDeg;
    }

    @Override
    public void update(Optional<TargetInfo> tag, boolean enabled) {
        if (!enabled || tag.isEmpty()) {
            m_latestSpeeds = new ChassisSpeeds();
            return;
        }

        double error = tag.get().headingErrorDeg();
        double omega = pid.calculate(error, 0.0);
        m_latestSpeeds = new ChassisSpeeds(0, 0, omega);
    }

    @Override
    public boolean isFinished(ADContext adCtx,
                              Optional<TargetInfo> tag,
                              boolean enabled) {
        if (!enabled || tag.isEmpty() || !adCtx.inShootMode()) return true;
        return Math.abs(tag.get().headingErrorDeg()) < m_adMetadata.finishHeadingErrorDeg;
    }

    @Override public void start() {}
    @Override public void stop() { m_latestSpeeds = new ChassisSpeeds(); }
    @Override public ChassisSpeeds getSpeeds() { return m_latestSpeeds; }
}
