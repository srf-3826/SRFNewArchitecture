// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.drivehelpers;

import java.util.function.Consumer;

import frc.robot.ui.UI_Mode;

public enum AutoDriveAction {

    public final UI_Mode m_mode;
    public final Consumer<ADContext> m_start;
    public final Consumer<ADContext> m_stop;

    AutoDriveAction(UI_Mode mode, Consumer<ADContext> onRise, Consumer<ADContext> onFall) {
        this.m_mode = mode;
        this.m_start = onRise;
        this.m_stop = onFall;
    }