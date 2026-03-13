package frc.robot.autos.drivehelpers.helpers;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.drivehelpers.ADContext;
import frc.robot.autos.drivehelpers.ADHelperMetadata;
import frc.robot.autos.drivehelpers.ContinuousAction;
import frc.robot.subsystems.TargetInfo;

public class ShootRangeHelper implements ContinuousAction {
    @SuppressWarnings("unused")
    private final ADContext m_adCtx;
    private final ADHelperMetadata m_adMetadata;
    private final PIDController m_pid;
    private ChassisSpeeds m_latestSpeeds = new ChassisSpeeds();

    public ShootRangeHelper(ADContext ctx, ADHelperMetadata adMetadata) {
        this.m_adCtx = ctx;
        this.m_adMetadata = adMetadata;
        this.m_pid = new PIDController(m_adMetadata.kP, m_adMetadata.kI, m_adMetadata.kD);
    }

    @Override
    public boolean shouldActivate(ADContext adCtx,
                                  Optional<TargetInfo> tag,
                                  boolean enabled) {
        if (!enabled || !adCtx.inShootMode() || tag.isEmpty()) return false;
        return Math.abs(tag.get().distanceErrorMeters()) >  m_adMetadata.activateDistanceErrorMeters;
    }

    @Override
    public void update(Optional<TargetInfo> tag, boolean enabled) {
        if (!enabled || tag.isEmpty()) {
            m_latestSpeeds = new ChassisSpeeds();
            return;
        }
        double error = tag.get().distanceErrorMeters();
        double vx = m_pid.calculate(error, 0.0);
        m_latestSpeeds = new ChassisSpeeds(vx, 0, 0);
    }

    @Override
    public boolean isFinished(ADContext adCtx,
                              Optional<TargetInfo> tag,
                              boolean enabled) {
        if (!enabled || tag.isEmpty()) return true;
        return Math.abs(tag.get().distanceErrorMeters()) < m_adMetadata.finishDistanceErrorMeters;
    }

    @Override public void start() {}
    @Override public void stop() { m_latestSpeeds = new ChassisSpeeds(); }
    @Override public ChassisSpeeds getSpeeds() { return m_latestSpeeds; }
}