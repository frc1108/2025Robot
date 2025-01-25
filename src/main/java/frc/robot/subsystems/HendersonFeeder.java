// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HendersonConstants;

@Logged
public class HendersonFeeder extends SubsystemBase {
  private final SparkMax m_leftMotor = new SparkMax(HendersonConstants.kLeftFeederMotorCanId,MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(HendersonConstants.kRightFeederMotorCanId,MotorType.kBrushless);

  /** Creates a new HendersonFeeder. */
  public HendersonFeeder() {
    m_leftMotor.configure(Configs.Feeder.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(new SparkMaxConfig()
                            .follow(m_leftMotor,true),ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }
  public void set(double speed){
    m_leftMotor.set(speed);
  }

  @Logged(name = "Note Sensor Triggered")
  public boolean getBeamBreak(){
    return m_leftMotor.getForwardLimitSwitch().isPressed();
  }

  public Command stop() {
    return Commands.runOnce(()->set(0));
  }

  public Command runStopCommand(){
    return Commands.sequence(runOnce(()->set(0.8)),
             Commands.race(Commands.waitSeconds(5),
                           Commands.waitUntil(this::getBeamBreak)),
             runOnce(()->set(0)).withName("Beam Feeder"));
  }
}
