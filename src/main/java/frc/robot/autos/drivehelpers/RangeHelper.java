// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ui.AutoDriveHelperAction;
import frc.robot.ui.HelperContext;

public class RangeHelper implements AutoDriveHelperAction {

    private final HelperContext m_ctx;
    private final PIDController m_distancePID;
    private ChassisSpeeds m_latestSpeeds;

    public RangeHelper(HelperContext ctx) {
        this.m_ctx = ctx;

        m_distancePID = new PIDController(1.2, 0.0, 0.1);
        m_distancePID.setTolerance(0.05);  // 5 cm tolerance
    }

    @Override
    public void start() {}

    @Override
    public void update() {

        var opt = m_ctx.vision.getNearestTargetInfo();
        if (opt.isEmpty()) {
            m_latestSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        var info = opt.get();
        var robotToTag = info.robotPoseInTagSpace;

        double tx = robotToTag.getTranslation().getX();
        double ty = robotToTag.getTranslation().getY();

        double range = Math.hypot(tx, ty);
        double desiredRange = m_ctx.vision.computeDesiredRangeToTag(info.tagId);

        double vx = m_distancePID.calculate(range, desiredRange);
        vx = MathUtil.clamp(vx, -1.5, 1.5);

        m_latestSpeeds = new ChassisSpeeds(vx, 0, 0);
    }

    public ChassisSpeeds getSpeeds() {
        return m_latestSpeeds;
    }

    @Override
    public boolean isFinished() {
        return m_distancePID.atSetpoint();
    }

    @Override
    public void stop() {}
}
