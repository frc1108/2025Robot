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

final SparkMax m_coralUpDown = new SparkMax(41, MotorType.kBrushless);
final SparkMax m_coralInOut = new SparkMax(42, MotorType.kBrushless);
final SparkMax m_elevator = new SparkMax(21, MotorType.kBrushless);


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
  private void setElevator(double speed){
    m_elevator.set(speed);
  }
//coralOut=the green wheel 
public Command coralOut(){
      return this.run(
      () -> {
        setInOut(0.45);
      });
    }
    public Command coralIn(){
      return this.run(
      () -> {
        setInOut(-0.45);
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
        set(0.3);
      });
    }
    public Command coralSpinOut(){
      return this.run(
      () -> {
        set(-0.3);
      });
    }
    public Command coralSpinStop(){
      return this.run(
      () -> {
        set(0);
      });
    }
    public Command elevatorUp(){
      return this.run(
      () -> {
        setElevator(-0.45);
      });
    }
    public Command elevatorDown(){
      return this.run(
      () -> {
        setElevator(0.45);
      });
    }
    public Command elevatorStop(){
      return this.run(
      () -> {
        setElevator(0);
      });
    }
}
