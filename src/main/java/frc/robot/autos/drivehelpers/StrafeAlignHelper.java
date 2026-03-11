// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ui.AutoDriveHelperAction;
import frc.robot.ui.UIContext;

public class StrafeAlignHelper implements AutoDriveHelperAction {

    private final UIContext m_ctx;
    private final PIDController m_strafePID;
    private ChassisSpeeds m_latestSpeeds;

    public StrafeAlignHelper(UIContext ctx) {
        this.m_ctx = ctx;

        m_strafePID = new PIDController(1.0, 0.0, 0.05);
        m_strafePID.setTolerance(0.03);  // 3 cm tolerance
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

        double lateral = robotToTag.getTranslation().getY();

        double vy = m_strafePID.calculate(lateral, 0.0);
        vy = MathUtil.clamp(vy, -1.0, 1.0);

        m_latestSpeeds = new ChassisSpeeds(0, vy, 0);
    }

    public ChassisSpeeds getSpeeds() {
        return m_latestSpeeds;
    }

    @Override
    public boolean isFinished() {
        return m_strafePID.atSetpoint();
    }

    @Override
    public void stop() {}
}