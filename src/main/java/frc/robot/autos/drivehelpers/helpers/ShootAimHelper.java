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

    private final ADHelperMetadata m_adMetadata;
    private final PIDController pid;
    private ChassisSpeeds latest = new ChassisSpeeds();

    public ShootAimHelper(ADContext ctx, ADHelperMetadata metadata) {
        this.m_adMetadata = metadata;
        this.pid = new PIDController(m_adMetadata.kP, m_adMetadata.kI, m_adMetadata.kD);
    }

    @Override
    public boolean shouldActivate(ADContext ctx,
                                  Optional<TargetInfo> tag,
                                  boolean enabled) {

        if (!enabled || !ctx.inShootMode() || tag.isEmpty())
            return false;

        return Math.abs(tag.get().headingErrorDeg()) > m_adMetadata.activateHeadingErrorDeg;
    }

    @Override
    public void update(Optional<TargetInfo> tag, boolean enabled) {

        if (!enabled || tag.isEmpty()) {
            latest = new ChassisSpeeds();
            return;
        }

        double error = tag.get().headingErrorDeg();
        double omega = pid.calculate(error, 0.0);

        latest = new ChassisSpeeds(0, 0, omega);
    }

    @Override
    public boolean isFinished(ADContext ctx,
                              Optional<TargetInfo> tag,
                              boolean enabled) {

        if (!enabled || tag.isEmpty())
            return true;

        return Math.abs(tag.get().headingErrorDeg()) < m_adMetadata.finishHeadingErrorDeg;
    }

    @Override public void start() {}
    @Override public void stop() { latest = new ChassisSpeeds(); }
    @Override public ChassisSpeeds getSpeeds() { return latest; }
}
