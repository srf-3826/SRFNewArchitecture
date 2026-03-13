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

    private final ADHelperMetadata metadata;
    private final PIDController pid;
    private ChassisSpeeds latest = new ChassisSpeeds();

    public ShootRangeHelper(ADContext ctx, ADHelperMetadata metadata) {
        this.metadata = metadata;
        this.pid = new PIDController(metadata.kP, metadata.kI, metadata.kD);
    }

    @Override
    public boolean shouldActivate(ADContext ctx,
                                  Optional<TargetInfo> tag,
                                  boolean enabled) {

        if (!enabled || !ctx.inShootMode() || tag.isEmpty())
            return false;

        return Math.abs(tag.get().distanceErrorMeters()) >
               metadata.activateDistanceErrorMeters;
    }

    @Override
    public void update(Optional<TargetInfo> tag, boolean enabled) {

        if (!enabled || tag.isEmpty()) {
            latest = new ChassisSpeeds();
            return;
        }

        double error = tag.get().distanceErrorMeters();
        double vx = pid.calculate(error, 0.0);

        latest = new ChassisSpeeds(vx, 0, 0);
    }

    @Override
    public boolean isFinished(ADContext ctx,
                              Optional<TargetInfo> tag,
                              boolean enabled) {

        if (!enabled || tag.isEmpty())
            return true;

        return Math.abs(tag.get().distanceErrorMeters()) <
               metadata.finishDistanceErrorMeters;
    }

    @Override public void start() {}
    @Override public void stop() { latest = new ChassisSpeeds(); }
    @Override public ChassisSpeeds getSpeeds() { return latest; }
}