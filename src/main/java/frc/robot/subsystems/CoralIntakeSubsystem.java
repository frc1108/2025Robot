// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;

public class CoralIntakeSubsystem extends SubsystemBase {

  // Initialize intake SPARK.  We will use open loop control for this.
  private final SparkMax m_intakeMotor = new SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() {
    m_intakeMotor.configure(
      Configs.Intake.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  private void setIntakePower(double power) {
    m_intakeMotor.set(power);
  }
  public Command runIntakeCommand() {
    return this.startEnd(
      () -> this.setIntakePower(IntakeSetpoints.kForward),() -> this.setIntakePower(0.0));
  }
  /** Set the intake motor power in range of [-1,1] */
  public Command reverseIntakeCommand() {
    return this.startEnd(
      () -> this.setIntakePower(IntakeSetpoints.kReverse),() -> this.setIntakePower(0.0));
  }
  //slow coral out
  public Command reverseSlowIntakeCommand() {
    return this.startEnd(
      () -> this.setIntakePower(0.15),() -> this.setIntakePower(0.0));
  }
  public Command reverseFastIntakeCommand() {
    return this.startEnd(
      () -> this.setIntakePower(1.0),() -> this.setIntakePower(0.0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}