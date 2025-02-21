// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */

final SparkMax m_coralUpDown = new SparkMax(26, MotorType.kBrushless);
final SparkMax m_coralInOut = new SparkMax(27, MotorType.kBrushless);


  public CoralSubsystem() {}


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setInOut(double speed){
    m_coralInOut.set(speed);
  }
  private void set(double speed){
    m_coralUpDown.set(speed);
  }
public Command coralOut(){
      return this.run(
      () -> {
        setInOut(0.25);
      });
    }
    public Command coralIn(){
      return this.run(
      () -> {
        setInOut(-0.25);
      });
    }
    public Command coralStop(){
      return this.run(
      () -> {
        setInOut(0);
      });
    }
    public Command coralSpinIn(){
      return this.run(
      () -> {
        set(0.25);
      });
    }
    public Command coralSpinOut(){
      return this.run(
      () -> {
        set(-0.25);
      });
    }
    public Command coralSpinStop(){
      return this.run(
      () -> {
        set(0);
      });
    }
}
