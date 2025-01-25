// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HendersonConstants;
import frc.robot.Configs;

@Logged
public class HendersonLauncher extends SubsystemBase{
  private final SparkFlex m_leftMotor = new SparkFlex(HendersonConstants.kLeftLauncherMotorCanId,MotorType.kBrushless);
  private final SparkFlex m_rightMotor = new SparkFlex(HendersonConstants.kRightLauncherMotorCanId,MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pidController;
  private double m_goal = 0;
  private boolean m_enabled = false;

  /** Creates a new HendersonFeeder. */
  public HendersonLauncher() {
    m_leftMotor.configure(Configs.Launcher.launcherConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_rightMotor.configure(new SparkFlexConfig()
                          .follow(m_leftMotor,true), ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
   
    m_encoder = m_leftMotor.getEncoder();
    m_pidController = m_leftMotor.getClosedLoopController();

  }

  @Override
  public void periodic() {
     if (m_enabled) {
      m_pidController.setReference(m_goal, ControlType.kVelocity);
     }
     SmartDashboard.putNumber("Launcher Speed",m_encoder.getVelocity());

    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    disablePid();
    m_leftMotor.set(speed);
  }

  public Command run() {
    return Commands.runOnce(()->set(0.5));
  }

  public Command runReverse() {
    return Commands.runOnce(()->set(-0.35));
  }

  public Command stop() {
    disablePid();
    return Commands.runOnce(()->set(0));
  }

  public Command idle() {
    return Commands.runOnce(()->setRPM(HendersonConstants.kLauncherIdleRpm));
  }

  private void setRPM(double goal) {
    enablePid();
    m_goal = MathUtil.clamp(goal, 0, HendersonConstants.kMaxLauncherRpm);
  }

  public void enablePid() {
    m_enabled = true;
  }

  public void disablePid() {
    m_enabled = false;
    m_goal = 0;
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public double getGoal() {
    return m_goal;
  }
}
