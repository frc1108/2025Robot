// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

final SparkMax m_algaeUpDown = new SparkMax(26, MotorType.kBrushless);
final SparkMax m_algaeSpin = new SparkMax(27, MotorType.kBrushless);


  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    private void set(double speed){
      m_algaeUpDown.set(speed);
    }
  
    public Command upAlgae(){
      return this.run(
      () -> {
        set(-0.4);
      });
    }
  
    public Command downAlgae(){
      return this.run(
      () -> {
        set(0.3);
      });
    }
    
    public Command stopAlgae(){
      return this.run(
      () -> {
        set(-0.025);
      }); 
    }

    private void setSpin(double speed){
      m_algaeSpin.set(speed);
    }

    public Command inAlgaeRoller(){
      return this.run(
      () -> {
        setSpin(0.5);
      });
    }
    public Command outAlgaeRoller(){
      return this.run(
      () -> {
        setSpin(-0.5);
      });
    }
    public Command stopAlgaeRoller(){
      return this.run(
      () -> {
        setSpin(0);
      });
    }


  }