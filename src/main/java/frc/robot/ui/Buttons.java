// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ui;

/*
    With 10 available buttons (not counting stick clicks and the D-Pad, reserved for fixed UI tasks)
    and with one of these buttons (ALT) used for a one level shift modifier, there are
    18 possible selectable actions PER MODE! The following Bitmap table is an arbitrary
    map of uniquely assigned bit masks, one per raw button. These can be very 
    useful when polling for which button is pressed, especially where more than one button
    is associated with a desired action (including, but not limited to, ALT + another button).
*/ 
public final class Buttons {
    public static final int NONE  = 0;
    public static final int A     = 1 << 0;
    public static final int B     = 1 << 1;
    public static final int X     = 1 << 2;
    public static final int Y     = 1 << 3;
    public static final int LT    = 1 << 4;
    public static final int RT    = 1 << 5;
    public static final int LB    = 1 << 6;
    public static final int ALT   = LB;            // Alias for LB
    public static final int RB    = 1 << 7;
    public static final int START = 1 << 8;
    public static final int BACK  = 1 << 9;
}

