// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  @SuppressWarnings("unused")
  private CANBus m_canBus;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(CANBus canBus) {
    m_canBus = canBus;
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public void raiseElevator() {};
  public void lowerElevator() {};
  public void stopElevator() {};
  public void winchUp() {};
  public void winchDown() {};
  public void stopWinch() {};
  public void stowElevator() {};
  public void stowWinch() {};

  @Override
  public void periodic() {
    // This method will be called by the command scheduler once per loop,
    // possibly only when the susytem is enabled
  }

  public void update() {
    // this method is called from robot.periodic() (transitively through robotContaine)
    // once per loop, and prior to periodic(), whether enabled or not
  }
}
