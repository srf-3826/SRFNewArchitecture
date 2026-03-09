// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;

public class IntakeSubsystem extends ActionableSubsystem {
  @SuppressWarnings("unused")
  private CANBus m_canBus;
  @SuppressWarnings("unused")
  private TalonFX m_pivotMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(CANBus canBus) {
    m_canBus = canBus;
    CommandScheduler.getInstance().registerSubsystem(this);
    m_pivotMotor = new TalonFX(ISC.INTAKE_PIVOT_MOTOR_ID, canBus);
  }

  public void startIntake() {}        // Extend pivot arm, start rollers
  public void stopAndPivotToHold() {} // Partial retraction of the pivot arm to hold fuel in Hopper
                                      // or just to protect the intake mechanism.
                                      // Used to terminate both intaking and dumping
  public void dumpFuel() {}           // Reverse intake and hopper bed rollers with Pivot arm extended
  public void retractIntake() {}      // Fully retract arm - for playing defense (should happen 
                                      // automatically when entering DEFENSE mode). But dangerous
                                      // if fuel is in hoppeer. Questions:
                                      // SHould all (any?) fuel be dumped first? Operator must decide - 
                                      // no sensors other that maybe monitoring of Pivot arm motor current 
                                      // with plan B = notify driveer to dump fuel, or Plan C, just 
                                      // retract to the hold position, but now there is higher risk of
                                      // damage from collisions. 
  public void startBedRollersIn() {}  // Supports intakeing and shooting
  public void startBedRollersOut() {} // Supports dumping fuel
  public void stopBedRollers() {}     // Return to idle state after intaking, dumping, or shooting
  public void increaseRpm() {}        // Optional - for tuning speed of intake rollers
  public void decreaseRpm() {}        // Ditto

  @SuppressWarnings("unused")
  private void stopIntakeRollers() {}  // Only used internally

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
