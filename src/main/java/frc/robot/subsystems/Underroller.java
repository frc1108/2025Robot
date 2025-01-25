// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.UnderrollerConstants;

@Logged
public class Underroller extends SubsystemBase {
  /** Creates a new Intake. */
  private final SparkMax m_underrollerFront = new SparkMax(UnderrollerConstants.kFrontCanId, MotorType.kBrushless);
  private final SparkMax m_underrollerRear = new SparkMax(UnderrollerConstants.kRearCanId, MotorType.kBrushless);
  public Underroller() {
    m_underrollerFront.configure(Configs.Underroller.underrollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_underrollerRear.configure(Configs.Underroller.underrollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void setUnderrollerspeed (double speed ){
  m_underrollerFront.set(speed );
  m_underrollerRear.set(speed );
}


  public Command runUnderroller(){
    return Commands.startEnd(()->setUnderrollerspeed(0.6),()->setUnderrollerspeed(0));
  }
  public Command reverseUnderroller(){
    return Commands.startEnd(()->setUnderrollerspeed(-0.35),()->setUnderrollerspeed(0));
  }
}
