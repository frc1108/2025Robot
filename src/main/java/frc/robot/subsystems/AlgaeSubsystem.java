// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;

@Logged
public class AlgaeSubsystem extends SubsystemBase {

final SparkMax m_algaeUpDown = new SparkMax(26, MotorType.kBrushless);
final SparkMax m_algaeSpin = new SparkMax(27, MotorType.kBrushless);


  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {}


  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
    private void setAlgaePower(double power) {
      m_algaeUpDown.set(power);
    }
  
  public Command upAlgae() {
  return this.startEnd(
    () -> this.setAlgaePower(0.4),() -> this.setAlgaePower(-0.025));
}
public Command downAlgae() {
  return this.startEnd(
    () -> this.setAlgaePower(-0.4),() -> this.setAlgaePower(-0.025));
}

private void setSpinPower(double power) {
  m_algaeSpin.set(power);
}

public Command inAlgaeRoller() {
  return this.startEnd(
    () -> this.setSpinPower(0.5),() -> this.setSpinPower(0.0));
}
public Command outAlgaeRoller() {
  return this.startEnd(
    () -> this.setSpinPower(-0.5),() -> this.setSpinPower(0.0));
}
  }