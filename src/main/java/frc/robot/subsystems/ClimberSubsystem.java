// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

// initialize devices on the rio can bus
final TalonFX m_climber = new TalonFX(31, "rio");

// users should reuse control requests when possible
final DutyCycleOut m_request = new DutyCycleOut(0.2);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {}
    // This method will be called once per scheduler run

  // set request to motor controller
  private void set(double speed){
    m_climber.setControl(m_request.withOutput(speed));
  }

  public Command upClimber(){
    return this.run(
    () -> {
      set(0.2);
    });
  }

  public Command downClimber(){
    return this.run(
    () -> {
      set(-0.2);
    });
  }
}
