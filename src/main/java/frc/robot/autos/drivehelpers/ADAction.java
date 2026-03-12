// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

import java.util.function.Function;

public enum ADAction {

    NONE(
        null
    ),
        
    SHOOT_AUTO_AIM(
        m_ctx -> new ShootAimHelper(m_ctx)
    ),

    SHOOT_AUTO_RANGE(
        m_ctx -> new ShootRangeHelper(m_ctx)
    ),

    SHOOT_AUTO_STRAFE(
        m_ctx -> new ShootStrafeHelper(m_ctx)
    );

    public final Function<ADContext, ContinuousAction> m_helperFactory;

    ADAction(Function<ADContext, ContinuousAction> helperFactory) {
        this.m_helperFactory = helperFactory;
    }
}