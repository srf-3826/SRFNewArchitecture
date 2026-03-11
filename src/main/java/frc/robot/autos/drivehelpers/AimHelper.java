// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ui.AutoDriveHelperAction;
import frc.robot.ui.UIContext;

public class AimHelper implements AutoDriveHelperAction {

    private final UIContext m_ctx;
    private final PIDController m_headingPID;
    private ChassisSpeeds m_latestSpeeds = new ChassisSpeeds();

    public AimHelper(UIContext ctx) {
        this.m_ctx = ctx;

        m_headingPID = new PIDController(3.5, 0.0, 0.2);
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
        m_headingPID.setTolerance(Math.toRadians(1.5));  // ~1.5° tolerance
    }

    @Override
    public void start() {}

    @Override
    public void update() {

        var opt = m_ctx.vision.getNearestTargetInfo();
        if (opt.isEmpty()) {
            m_latestSpeeds = new ChassisSpeeds(0, 0, 0);
            return;
        }

        var info = opt.get();
        var robotToTag = info.robotPoseInTagSpace;

        double tx = robotToTag.getTranslation().getX();
        double ty = robotToTag.getTranslation().getY();

        double desiredHeading = Math.atan2(ty, tx);
        double currentHeading = m_ctx.swerve.getPose().getRotation().getRadians();

        double omega = m_headingPID.calculate(currentHeading, desiredHeading);

        m_latestSpeeds = new ChassisSpeeds(0, 0, omega);
    }

    public ChassisSpeeds getSpeeds() {
        return m_latestSpeeds;
    }

    @Override
    public boolean isFinished() {
        if (isShootMode) && is
        return m_headingPID.atSetpoint();
    }

    @Override
    public void stop() {}
}
